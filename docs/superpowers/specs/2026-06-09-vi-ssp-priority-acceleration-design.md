# VI を SSP として捉えた優先順序伝播による高速化 — 設計

- 日付: 2026-06-09
- 対象: `vi_rs/vi_reference`（本家 ROS1 `value_iteration` の u64 忠実移植）、`vi_compare`、`docs/research/`
- 関連:
  - [vi-u64-fast-solvers-design](2026-06-09-vi-u64-fast-solvers-design.md)（活性集合系の u64 高速ソルバ群、全て bit-exact）
  - [vi-reference-faithful-port-design](2026-06-08-vi-reference-faithful-port-design.md)（u64 忠実移植）
  - [vi-ros-compare-benchmark-design](2026-06-04-vi-ros-compare-benchmark-design.md)（比較ベンチ基盤）

## 1. 背景と目的

`vi_reference` の高速ソルバ群（frontier / block / pyramid）は**「どのセルを更新するか」**（活性集合）の工学的最適化であり、コスト数式は本家と不変ゆえ全て **bit-exact**（house.pgm で本家比 1.4〜17x、`report_u64.md`）。

本研究は**別軸**——「**固定点反復そのもの**を、SSP の単調構造を使った**優先順序伝播**（Dijkstra 流の優先キュー）に置き換える」という**アルゴリズムの根本変更**で高速化できるかを、数式的定式化と実験の両面から検証する。ユーザ意思決定（2026-06-09）:

- **精度**: 近似許容・精度を測定（本家固定点を ground truth に、速度×精度を実測）。
- **広さ**: 1〜2手法を深く（＝優先順序伝播を label-setting / label-correcting の2端で深掘り）。
- **TeX**: 本質モデル（SSP/Bellman）＋忠実な癖は remark 併記。

### 1.1 核心となる観察

本家 VI は**割引なし確率的最短経路（SSP, γ=1）**であり、`value_iteration_raw`/`action_cost_raw` の数式は不変のまま、状態を**値の昇順**に処理する優先順序伝播（Dijkstra 流）へ置換できる。

- 本家 Jacobi/Gauss-Seidel VI の仕事量は $O(\text{直径}\cdot N)$（house で 61 反復＝直径 ≈ 384/6）。
- frontier（活性集合）は $O(R\cdot N)$、$R$＝波面厚（house 実測 33.4M更新 / 8.85M状態 ≈ **3.8回/セル**）。
- 優先順序伝播は各到達セルを**実質1回確定**で $O(N\log N)$。直径非依存。

これが campus(14000×800×60, 直径 ≈ 2300 ホップ) で効くはずの「根本レバー」。

## 2. 数理モデルの定式化（`docs/research/` の TeX 本体）

状態 $s=(i_x,i_y,i_\theta)$、索引 $\mathrm{idx}(s)=i_\theta + i_x n_\theta + i_y n_\theta n_x$、状態数 $N=n_x n_y n_\theta$。

**Bellman 作用素**:

$$(\mathcal{T}V)(s)=\min_{a\in\mathcal{A}}A(s,a),\qquad
A(s,a)=\frac{1}{B}\sum_{(\delta,p)\in \tau_a(i_\theta)} p\big[V(s\oplus\delta)+g(s\oplus\delta)\big]$$

- $B=\text{PROB\_BASE}=2^{18}$、$\sum_{(\delta,p)\in\tau_a(i_\theta)}p=B$（$64^3$ サブセルサンプル集計）。
- $\delta=(d_{ix},d_{iy},d_{it})$、$d_{it}$ は**絶対**θ インデックス。$s\oplus\delta=(i_x+d_{ix},\,i_y+d_{iy},\,d_{it}\bmod n_\theta)$。
- 遷移先のいずれかが範囲外/非 free なら $A(s,a)=\text{MAX\_COST}$（行動全体が無効）。
- $g(s')=\text{penalty}(s')+\text{local\_penalty}(s')\ge B$（free セルの基準＝$B$＝1ステップコスト）。
- 固定点 $V^\*=\mathcal{T}V^\*$、ゴール（`final_state`）は吸収 $V=0$、非 free/未到達は $V=\text{MAX\_COST}$。

**癖は remark/脚注で併記**（本質モデルを汚さない）:
① $A$ 末尾の $\gg 18$ は u64 整数除算で切り捨て、② $\sum p[\cdots]$ は u64 wrapping 加算（未到達 $V=\text{MAX\_COST}$ 近傍は振動）、③ 本家は6種 sweep 順＝Gauss-Seidel の方向性で直径律速を緩和、④ $t_{\text{res}}=360/n_\theta$ の整数除算、⑤ `State::from_occupancy` の penalty 行跨ぎバグ、⑥ `value_function_writer` の $\text{total\_cost}/B$ 整数除算。

### 2.1 計算量と「根本から効く」根拠（TeX §計算量）

| 手法 | 仕事量 | house 実測 |
|---|---|---|
| 本家 Jacobi/GS-VI | $O(\text{直径}\cdot N)$ | 61反復 |
| frontier（活性集合） | $O(R\cdot N)$, $R$=波面厚 | 64反復, 33.4M更新 (R≈3.8) |
| **優先順序（本研究）** | $O(N\log N)$（ヒープ） | 目標: 各到達セル≈1回確定 |

### 2.2 単調性とその破れ（label-setting が近似になる理由）

純粋な Dijkstra（label-setting）が厳密になるのは「確定セルの値 ≥ それが採る行動の全 outcome の値」が成り立つときのみ。本問題では $A(s,a)$ が**全 outcome の確定を要する**ため、$s$ の最適行動が「**稀だが高値**の outcome（後退/横滑り）＋多数の低値前進 outcome」を含むと、低値 outcome が先に確定しても高値 outcome の確定が遅れ、その瞬間に前駆 $s$ のラベルが**確定走査位置より後方**に現れる＝単調性が破れる。

**重要な緩和**: 遷移 $\tau_a$ はサブセル離散化由来で**空間的に密集**（outcome は隣接1〜数セル＝値差は $O(\max|\delta|\cdot\text{step})$ で有界）。ゆえに単調性違反量は**遷移スプレッドで上から押さえられる小さな量**。

設計上の含意:
- **コア構造は二分ヒープ（優先キュー）**にして後方挿入を自然に処理する。
- **label-setting（A1）の近似源は settle-once（確定セルを再訪しない）に局所化**され、誤差は遷移スプレッドで有界 → 実測。
- **label-correcting（A2）は settle ガードを撤廃**して厳密・bit-exact。
- 単調性違反が実測で小さいことを利用する **Dial バケット（radix）化は後続最適化**（その違反量＝後方挿入距離の分布は §5 で測る実験結果でもある）。

## 3. アルゴリズム（実装する2ソルバ）

**不変条件**: コスト数式・状態グラフ・tie-break は本家と完全同一。`total_cost` を Dijkstra の tentative ラベルに流用（未確定＝`MAX_COST`＝∞、確定後は不変）。これにより既存 `action_cost_raw`/`value_iteration_raw` を**そのまま再利用**でき、追加するのは優先キューと（A1 用）`settled` ビットのみ。

### 3.1 逆方向隣接（前駆列挙）

cost-to-go では $V(s)$ が後続 $s\oplus\delta$ に依存するため、伝播は**逆向き**：ある $s^\*$ のラベルが下がると、$s^\*$ を outcome に持つ前駆 $s$ の $A(s,\cdot)$ が改善し得る。前駆は次で列挙する。

- 前計算 `rev_theta[it'] : Vec<(dix, diy, t_src)>` —— 全 (action $a$, source θ $t$, $\delta\in\tau_a(t)$) を走査し、$d_{it}\bmod n_\theta = it'$ となる $(d_{ix},d_{iy},t)$ を `it'` のリストへ追加。総エントリ数 ≤ 6 actions × 60 θ × (数個 outcome)。
- ラベル降下した $s^\*=(i_x^\*,i_y^\*,i_t^\*)$ の前駆候補: 各 $(d_{ix},d_{iy},t)\in$ `rev_theta[i_t^*]` について $s=(i_x^\*-d_{ix},\,i_y^\*-d_{iy},\,t)$。範囲内・free・非 final のみ採用（重複は許容＝過剰列挙は安全、候補が増えるだけ）。
- 採用した前駆 $s$ は `value_iteration_raw(s)` で**全行動 min を再評価**・書込（前駆は θ=t だが `value_iteration_raw` は当該セルの全行動を見るので安全）。

### 3.2 共通スケルトン（優先キュー）

```text
PQ = min-heap of (label = total_cost[s], idx s)   // Reverse で min-heap
seed:
  for s where total_cost[s] < MAX_COST (= final_state, V=0):
    push (0, s)                       // ゴールを種に
relax(s):                             // s を再評価し改善ならキューへ
  before = total_cost[s]
  value_iteration_raw(s)              // 現在ラベルで min_a A(s,a) を再評価・書込
  if total_cost[s] < before:
    push (total_cost[s], s); updates += 1; return true
  return false
main loop:
  (lab, s*) = PQ.pop_min(); iters += 1
  if lab != total_cost[s*]: continue  // stale（より小さいラベルで再挿入済み）→破棄
  <settle/relax 規則は A1/A2 で分岐 (§3.3 / §3.4)>
```

- `iters`＝pop 総数、`updates`＝ラベル改善回数（`U64SolveStats` 流用）。
- stale 破棄は遅延 decrease-key（同一セルが複数ラベルでキューに残る）。
- 種は final セルそのもの（`total_cost<MAX_COST`）を push。pop 時に label==0==total_cost ゆえ stale 扱いされず、前駆 relax の起点になる。

### 3.3 (A1) `prio_ls` — Priority Label-Setting（近似・最速）

settle 規則: pop した $s^\*$ を `settled` にし、**未 settled** な前駆のみ relax。確定セルは二度と触れない。

```text
  if settled[s*]: continue
  settled[s*] = true
  for s in predecessors(s*) where !settled[s] && free && !final: relax(s)
PQ 枯渇まで。converged=true。
```

- 各到達セルを実質1回確定 → 仕事量最小。
- **近似源**: §2.2 の通り settle-once のみ。誤差は遷移スプレッド有界。決定論遷移（単一 outcome）なら厳密（§4）。

### 3.4 (A2) `prio_lc` — Priority Label-Correcting（厳密・bit-exact）

settle 規則なし: pop した $s^\*$ の前駆を**無条件**（`free && !final`）に relax。改善は確定済みでも再 push。

```text
  // settled なし
  for s in predecessors(s*) where free && !final: relax(s)
PQ 枯渇まで。converged=true。
```

- 値は単調減少・下に有界（整数・≥0）ゆえ停止。停止時はどの relax も改善しない＝固定点。優先順序ゆえ frontier より少ない更新で本家と **bit-exact**。
- tie-break も `value_iteration_raw`（strict `<`、最初の行動勝ち）が本家と同一。各到達セルの**最終 relax は全 outcome 確定後**に起きるため、`optimal_action` は本家固定点と一致（label-correcting は successor 改善で前駆を必ず再 relax するので、最後の relax は確定値を見る）。

### 3.5 配置と公開 API

```text
vi_rs/vi_reference/src/solvers/
  priority.rs    # rev_theta / min-heap / relax / 共通スケルトン + (A1) prio_ls
  prio_lc.rs     # (A2) prio_lc（priority.rs の基盤を pub(crate) 再利用）
```

- `solvers/mod.rs`: `U64Solver` に `PriorityLabelSetting` / `PriorityLabelCorrecting` を追加、`from_name`（`"prio_ls"` / `"prio_lc"`）、`solve()` dispatch を追記。

## 4. 正しさの検証（TDD）

- **(A2) 厳密**: 既存 `solvers/mod.rs::test_support::parity_standard_maps`（empty / obstacle / sentinel の3マップ）に**合格必須**。到達セルで `total_cost` と `optimal_action` が Reference 固定点と bit 一致。
- **(A1) 近似**: 2本の near-parity テスト。
  1. **決定論遷移なら厳密**: 単一 outcome（prob=$B$）のみの合成 action 集合では (A1) も Reference と bit-exact（単調性違反が起き得ないため）。
  2. **到達セル RMSE 閾値**: 標準3マップで Reference 固定点との RMSE が小閾値以下、かつ方策一致率が下限以上（実測して閾値確定）。
- `cargo test -p vi_reference` を green 維持。ホスト実行は gitignore 済み `.cargo/config.toml` を避けるため `/tmp` から `--manifest-path` で実行（[[host-vi-rs-cargo-config-workaround]]）。

## 5. ベンチマーク

### 5.1 既存 house.pgm パイプライン

- `vi_u64_bench`（引数先頭 `<solver>`）は `U64Solver::from_name` 経由ゆえ**Rust 側の enum 追加だけで対応**。
- `vi_compare/u64/run_u64_bench.sh` の `SOLVERS` と `vi_compare/compare/make_u64_report.py` の `SOLVERS` に `prio_ls` / `prio_lc` を追加。
- `report_u64.md` に2行追加: elapsed / 反復(pop数) / updates / 本家比速度 / RMSE / 方策一致 / converged / bit-exact。**期待**: `prio_lc` は RMSE 0・方策 100%（bit-exact）、`prio_ls` は RMSE 小・方策高一致＋更新数最小。

### 5.2 直径レジームの実証 + 単調性違反の測定（合成ストリップ、ホスト完結）

house.pgm の直径（≈64）では優先順序の優位が出にくい。campus 比率（横長）で「本家/frontier ∝直径 vs 優先順序 ∝N」の分離を示すため、**ホスト完結の測定ハーネス**を追加（Docker/ROS 非依存）:

- `vi_reference/src/bin/vi_prio_measure.rs`（または `tests/` 計測）: 合成 free ストリップ（例 `512×64`, `1024×64`, `2048×64`, 全て `n_θ=60`）を生成し、ゴールを端に置く。各マップで `{reference, frontier2d, prio_ls, prio_lc}` を走らせ、以下を表で出力:
  1. **更新数（per-cell 平均）** と **elapsed**: frontier2d の R≈3.8 → 優先順序の ≈1 への低減、直径増大に対する本家/frontier の反復線形増 vs 優先順序の ∝N 頭打ち。
  2. **単調性違反の分布**: prio_lc で「確定済みセルが再 relax された回数」と「後方挿入距離（再 relax 時の `before-after` の step 単位）」のヒストグラム。§2.2 の「違反は遷移スプレッド有界」を実証し、Dial バケット化の可否を定量。
  3. **反直感の検証**: 「更新数↓でも wall-clock は random-access＋ヒープ支配で別物」（帯域律速の frontier と逆特性）か。これが研究の主結論。

## 6. 成果物

1. **`docs/research/2026-06-09-vi-ssp-acceleration.tex`** — §2 の数式定式化（本質モデル＋癖 remark）、§2.1/§2.2/§3 のアルゴリズムと近似解析、§5 の結果表・考察。latex 未インストールゆえ **.tex ソース納品**（`pdflatex` でビルド可、`docs/research/README.md` に手順）。実験後に結果セクションを実測値で更新。
2. **2ソルバ + テスト** — `priority.rs`(A1) / `prio_lc.rs`(A2) + enum/dispatch/from_name 配線 + parity/near-parity テスト。
3. **ベンチ/compare 配線** — `run_u64_bench.sh` / `make_u64_report.py` への追加、更新済み `report_u64.md`。
4. **直径レジーム測定** — `vi_prio_measure` + 結果表（更新数・wall-clock・違反分布、TeX §5 に反映）。

## 7. 段階デリバリ

1. **(A2) 厳密 + 基盤**: `priority.rs`（rev_theta/heap/relax）+ `prio_lc.rs` + parity テスト合格 + enum 配線。`cargo test -p vi_reference` green。
2. **(A1) 近似**: label-setting（settle-once）+ near-parity テスト（決定論一致 / RMSE 閾値）。
3. **直径レジーム測定**: `vi_prio_measure` 実装・実行、更新数・wall-clock・違反分布の表を取得。
4. **house.pgm ベンチ**: compare 配線、`report_u64.md` 更新（Docker 経路）。
5. **TeX 執筆**: 数式本体 + 計算量/単調性解析 + 実測結果・考察を `docs/research/` に。

各段階で parity green を維持。

## 8. リスクと留意点

- **label-setting の近似誤差が想定超**: 回転主体経路など単調性違反が多いマップで RMSE 大になり得る。→ (A2) 厳密が常に bit-exact の安全網。(A1) は「速度×精度トレードオフ点」として報告（失敗ではなく測定対象）。違反分布（§5.2-2）でどのマップで悪化するかを説明。
- **wall-clock が更新数に比例しない**: 優先キューの random-access＋ヒープ pop がキャッシュミスを増やし、帯域律速の frontier2d_pad/par（9.5〜17x）に wall-clock で負ける可能性。→ これ自体が研究結論。負ける場合は「更新数では下界に到達するが現行ハードでは帯域最適 Jacobi が有利」と明記。Dial バケット化（§2.2、違反が小なら有効）を follow-up として提示。
- **逆隣接の過剰列挙**: `value_iteration_raw` で全行動再評価するため候補が増えるが正しさ不変（コスト微増）。
- **未到達セルの wrapping 振動**: 優先順序は未到達セルに触れず `MAX_COST` 据置で安全。比較は到達セル（`REACH_THRESH`）のみ。
- **vi_reference のスコープ**: 「忠実移植＋活性集合ソルバ」に「優先順序ソルバ」を追加。コスト数式は不変ゆえ忠実性は保たれる。

## 9. 非目標

- bit-exact 系の更なる工学最適化（SoA/並列）—— 既存 frontier 系で達成済み。
- Dial バケット（radix）化 —— 単調性違反が小と実証できた場合の follow-up（本 spec では二分ヒープで正しさ優先）。
- (B) Eikonal/FMM 連続化・(C) 方策反復 —— 今回は (A) に集中（将来の別 spec）。
- campus 実マップ（14000×800）での実走 —— 合成ストリップで直径レジームを代理実証（実マップ投入は別途）。
- 優先順序ソルバのピーク性能最適化（正しさ＋速度×精度の実証が一次目的）。
