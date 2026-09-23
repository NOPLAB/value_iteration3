// vi_lib::solvers::schur_gpu のデバイスカーネル。value_iteration_raw /
// action_cost_raw (u64・2^18 固定小数点・wrapping) の直訳 — CPU と同じ u64
// 打ち切り演算なので、下降途中で止めても値は上界のまま (契約は schur.rs)。
// パスループはホスト側にある: Windows の WDDM TDR (約 2 秒でカーネル強制
// リセット) をカーネル境界で回避するため、1 launch = 1 パス。

typedef unsigned long long u64;
typedef unsigned char u8;

#define MAX_COST 262144000000000ULL  /* 1e9 << 18 (params::MAX_COST) */
#define REACH_THRESH 262144000000ULL /* 1e6 << 18 (solvers::REACH_THRESH) */
#define TILE_INIT REACH_THRESH       /* schur::TILE_INIT — wrap 構造的不能の要 */
#define PROB_BASE_BIT 18

__device__ __forceinline__ u64 clamp_unreached(u64 v) {
    return v >= REACH_THRESH ? MAX_COST : v;
}

/* 値バッファ初期化: 全状態 TILE_INIT。釘付けは vi_pin が 0 を書く。 */
extern "C" __global__ void vi_init(long long total, int dom, u64* values) {
    long long i = (long long)blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= total) return;
    values[i] = TILE_INIT;
}

/* 釘付け: 問題 p のパッチ状態 pin_states[pin_off[p]..pin_off[p+1]] を 0 に。
   タイル内で値 0 を取るのは釘だけ (1 歩 ≥ 1 s) なので、vi_pass は V==0 を
   釘として飛ばす。 */
extern "C" __global__ void vi_pin(long long npin, int n_prob, int dom,
                                  const int* __restrict__ pin_off,
                                  const int* __restrict__ pin_states,
                                  u64* values) {
    long long k = (long long)blockIdx.x * blockDim.x + threadIdx.x;
    if (k >= npin) return;
    /* k が属する問題 p を pin_off から二分探索 */
    int lo = 0, hi = n_prob;
    while (hi - lo > 1) {
        int mid = (lo + hi) / 2;
        if (pin_off[mid] <= k) lo = mid; else hi = mid;
    }
    values[(long long)lo * dom + pin_states[k]] = 0ULL;
}

/* 1 パス: 問題 p = blockIdx.y の全セルを並列 in-place 更新 (chaotic
   relaxation)。各更新は「現在値 (≥ ν) に対する正確な Bellman」なので V ≥ ν は
   保たれ、収束先の最大不動点は訪問順に依らない (schur.rs の上界性スケッチ)。
   ライン GS (行内逐次) も試したが、行ストライドの読みが非 coalesced で
   1 パス 3 倍遅くなり、パス数半減と相殺して壁時計は同等だった — 単純で
   coalesced なセル並列を採る (どちらも u64 ALU 律速の平衡点)。

   停止判定用に clamp 後の**減少**だけを max_real[p] へ atomicMax する:
   真のミンプラス降下は単調減少で、上昇は未到達ポケット境界のクリープ
   (REACH 未満のまま毎パス微増し続ける) だけ。|Δ| 判定だとクリープが停止を
   永遠に阻む。上昇を無視しても膨張 (MAX→有限) と磨きは全て減少なので
   収束検知は落ちない。 */
extern "C" __global__ void vi_pass(int side, int nt, int dom, int n_actions,
                                   const int* __restrict__ trans_off,
                                   const int* __restrict__ trans_dat,
                                   const u8* __restrict__ frees,
                                   const u64* __restrict__ pens,
                                   const int* __restrict__ prob_slot,
                                   const u8* __restrict__ done,
                                   u64* values,
                                   u64* max_real) {
    int p = blockIdx.y;
    if (done[p]) return;
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= dom) return;
    int slot = prob_slot[p];
    const u8* fr = frees + (long long)slot * side * side;
    const u64* pn = pens + (long long)slot * side * side;
    u64* V = values + (long long)p * dom;

    /* to_index_raw: idx = it + ix*nt + iy*(nt*side) */
    int it = i % nt;
    int r = i / nt;
    int ix = r % side;
    int iy = r / side;
    if (!fr[iy * side + ix] || V[i] == 0ULL) return; /* 非 free / 釘 (パッチ) */

    u64 before = clamp_unreached(V[i]);
    u64 best = MAX_COST;
    for (int a = 0; a < n_actions; ++a) {
        int base = a * nt + it;
        int t1 = trans_off[base + 1];
        u64 cost = 0ULL;
        int ok = 1;
        for (int k = trans_off[base]; k < t1; ++k) {
            int jx = ix + trans_dat[4 * k];
            if (jx < 0 || jx >= side) { ok = 0; break; }
            int jy = iy + trans_dat[4 * k + 1];
            if (jy < 0 || jy >= side) { ok = 0; break; }
            int jt = (trans_dat[4 * k + 2] + nt) % nt;
            if (!fr[jy * side + jx]) { ok = 0; break; }
            u64 av = V[(long long)jt + (long long)jx * nt + (long long)jy * nt * side];
            /* C の unsigned 演算はラップする = wrapping_add / wrapping_mul */
            cost += (av + pn[jy * side + jx]) * (u64)trans_dat[4 * k + 3];
        }
        u64 c = ok ? (cost >> PROB_BASE_BIT) : MAX_COST;
        if (c < best) best = c;
    }
    V[i] = best;
    u64 after = clamp_unreached(best);
    u64 d = before > after ? before - after : 0ULL;
    if (d) atomicMax(&max_real[p], d);
}

/* W 列の回収: w_out[p * max_watch + k] = clamp(V[watch[k]])。問題 p の列は
   その slot (タイル) のポータル並び順 = TileEdges.portals の順。 */
extern "C" __global__ void vi_gather(long long total, int dom, int max_watch,
                                     const int* __restrict__ prob_slot,
                                     const int* __restrict__ watch_off,
                                     const int* __restrict__ watch_states,
                                     const u64* __restrict__ values,
                                     u64* w_out) {
    long long i = (long long)blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= total) return;
    int p = (int)(i / max_watch);
    int k = (int)(i % max_watch);
    int slot = prob_slot[p];
    int wo = watch_off[slot];
    if (k >= watch_off[slot + 1] - wo) return;
    w_out[i] = clamp_unreached(values[(long long)p * dom + watch_states[wo + k]]);
}
