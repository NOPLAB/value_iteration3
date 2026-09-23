// vi_lib::solvers::frontier_gpu のデバイスカーネル。frontier2d (pad モデル,
// `action_cost_pad`) の u64 直訳 — 2^18 固定小数点・同じ打ち切り・同じ
// wrapping。ラウンド内は全候補状態の並列 in-place 更新 (chaotic relaxation) で、
// MAX_COST 起点の単調降下なので収束先は frontier2d と同じ最大不動点。
// ラウンドループはホスト側 (1 launch = 1 ラウンド、WDDM TDR 回避)。
//
// レイアウトは **θ-major** (index = it·plane + col、col = iy·nx_pad + ix): warp の
// 隣接スレッドが「同じ θ の隣接列」を担当するので、同じ遷移オフセットで隣接
// アドレスを読む = coalesced。CPU の Padded (θ-fastest) のままだと隣接スレッドの
// 読みが nt·8 B 離れて帯域が 1/4 以下になる (house 実測 34M 更新/s、CPU 以下)。
// 遷移オフセットは列に対する相対 (dcol) + 着地 θ の絶対平面: off = nit·plane + dcol。

typedef unsigned long long u64;
typedef unsigned int u32;
typedef unsigned char u8;

#define MAX_COST 262144000000000ULL /* 1e9 << 18 (params::MAX_COST) */
#define PROB_BASE_BIT 18

/* ゴールごとの値リセット: final = 0、他 = MAX_COST。 */
extern "C" __global__ void vi_reset(long long n_pad, const u8* __restrict__ fin, u64* val) {
    long long p = (long long)blockIdx.x * blockDim.x + threadIdx.x;
    if (p >= n_pad) return;
    val[p] = fin[p] ? 0ULL : MAX_COST;
}

/* 動いた列 (chg_list) を遷移 reach (mx,my) で膨らませて候補列 (cand_list) を
   作る。列は pad 座標の列添字 (= pad 状態添字 / nt)。free θ を持たない列は
   候補にしない。消費した chg_flag は下ろす。 */
extern "C" __global__ void vi_dilate(const u32* __restrict__ chg_list, u32 chg_cnt,
                                     int nx_pad, int mx, int my,
                                     const u8* __restrict__ col_free,
                                     u32* chg_flag, u32* cand_flag,
                                     u32* cand_list, u32* cand_cnt) {
    u32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= chg_cnt) return;
    u32 c = chg_list[i];
    chg_flag[c] = 0;
    int cx = (int)(c % (u32)nx_pad);
    int cy = (int)(c / (u32)nx_pad);
    for (int dy = -my; dy <= my; ++dy) {
        int y = cy + dy;
        for (int dx = -mx; dx <= mx; ++dx) {
            int x = cx + dx;
            u32 c2 = (u32)(y * nx_pad + x); /* 膨張先は pad 内 (動く列は interior) */
            if (!col_free[c2]) continue;
            if (atomicExch(&cand_flag[c2], 1u) == 0) {
                u32 k = atomicAdd(cand_cnt, 1u);
                cand_list[k] = c2;
            }
        }
    }
}

/* 候補列 × θ の並列 Bellman 更新。`action_cost_pad` と同じ式:
   後継が非 free か MAX_COST なら行動無効、cost = Σ (V+pen)·prob >> 18。
   動いた状態は updates を数え、列を chg_list へ (dedupe は chg_flag)。
   θ=0 のスレッドが cand_flag を下ろす (次ラウンドの dilate 用)。 */
extern "C" __global__ void vi_update(const u32* __restrict__ cand_list, u32 cc, int nt,
                                     long long plane,
                                     u64* val, const u32* __restrict__ pen,
                                     const u8* __restrict__ free_, const u8* __restrict__ fin,
                                     const long long* __restrict__ pc_off,
                                     const u32* __restrict__ pc_prob,
                                     const u32* __restrict__ pc_start, int n_act,
                                     u32* cand_flag, u32* chg_flag,
                                     u32* chg_list, u32* chg_cnt, u64* updates) {
    /* 更新数はブロック内で集約して 1 回だけ atomicAdd (1 アドレスへの百万回
       atomic はラウンド内で直列化する)。スレッド割当は列が最速 (warp = 同 θ の
       連続候補列)。 */
    __shared__ u32 blk_updates;
    if (threadIdx.x == 0) blk_updates = 0;
    __syncthreads();
    /* 候補状態数 cc·nt は列数·nt (= n_pad) 以下 = 2^32 未満なので 32 bit で割る
       (64 bit 除算は GPU で 1 桁遅い)。 */
    u32 t = blockIdx.x * blockDim.x + threadIdx.x;
    u32 total = cc * (u32)nt;
    bool moved = false;
    if (t < total) {
        u32 ci = t % cc;
        int it = (int)(t / cc);
        u32 col = cand_list[ci];
        if (it == 0) cand_flag[col] = 0;
        long long p = (long long)it * plane + col;
        if (free_[p] && !fin[p]) {
            u64 min_cost = MAX_COST;
            for (int a = 0; a < n_act; ++a) {
                u32 s = pc_start[a * nt + it];
                u32 e = pc_start[a * nt + it + 1];
                u64 cost = 0;
                bool bad = false;
                for (u32 k = s; k < e; ++k) {
                    long long n = (long long)col + pc_off[k];
                    if (!free_[n]) { bad = true; break; }
                    u64 v = val[n];
                    if (v == MAX_COST) { bad = true; break; }
                    cost += (v + (u64)pen[n]) * (u64)pc_prob[k];
                }
                if (!bad) {
                    cost >>= PROB_BASE_BIT;
                    if (cost < min_cost) min_cost = cost;
                }
            }
            if (min_cost != val[p]) {
                val[p] = min_cost;
                moved = true;
                if (atomicExch(&chg_flag[col], 1u) == 0) {
                    u32 k = atomicAdd(chg_cnt, 1u);
                    chg_list[k] = col;
                }
            }
        }
    }
    if (moved) atomicAdd(&blk_updates, 1u);
    __syncthreads();
    if (threadIdx.x == 0 && blk_updates) atomicAdd(updates, (u64)blk_updates);
}

/* 収束後の方策: 各状態の argmin 行動 (最初の最小、CPU final_policy と同じ
   タイブレーク = 行動添字昇順で strict less)。-1 = 行動なし。 */
extern "C" __global__ void vi_policy(long long n_pad, int nt, long long plane,
                                     const u64* __restrict__ val, const u32* __restrict__ pen,
                                     const u8* __restrict__ free_, const u8* __restrict__ fin,
                                     const long long* __restrict__ pc_off,
                                     const u32* __restrict__ pc_prob,
                                     const u32* __restrict__ pc_start, int n_act,
                                     signed char* opt) {
    long long p = (long long)blockIdx.x * blockDim.x + threadIdx.x;
    if (p >= n_pad) return;
    opt[p] = -1;
    if (!free_[p] || fin[p]) return;
    int it = (int)(p / plane);
    long long col = p % plane;
    u64 min_cost = MAX_COST;
    int best = -1;
    for (int a = 0; a < n_act; ++a) {
        u32 s = pc_start[a * nt + it];
        u32 e = pc_start[a * nt + it + 1];
        u64 cost = 0;
        bool bad = false;
        for (u32 k = s; k < e; ++k) {
            long long n = col + pc_off[k];
            if (!free_[n]) { bad = true; break; }
            u64 v = val[n];
            if (v == MAX_COST) { bad = true; break; }
            cost += (v + (u64)pen[n]) * (u64)pc_prob[k];
        }
        if (!bad) {
            cost >>= PROB_BASE_BIT;
            if (cost < min_cost) { min_cost = cost; best = a; }
        }
    }
    opt[p] = (signed char)best;
}
