# Solver notes（ソルバ数式仕様）

`vi_lib` の u64 ソルバが実行する価値反復ロジックを論文形式で定式化した数式ノート。

| ファイル | 対象ソルバ | 内容 |
|---|---|---|
| `reference.tex` | `reference` | 全ソルバ共通の核：状態空間・固定小数点・サブセルサンプリング遷移・行動コスト・Bellman 作用素・ゴール処理・SSP 一意性・実装の癖 |
| `frontier2d_sparse.tex` | `frontier2d_sparse` | 高速化4層：活性集合（フロンティア）／ペナルティ融合 `cp`／非同期 Gauss–Seidel 並列／**θ マスク疎評価**、および各層の bit-exact 性 |

日本語を含むため **LuaLaTeX**（`ltjsarticle`、原ノ味フォント）でビルドする。依存パッケージは
amsmath / amssymb / amsthm / booktabs / graphicx / tikz / hyperref / geometry。図はすべて TikZ で
記述しており外部画像に依存しない。記法は原論文（Ueda et al., JRM 35(6), 2023）に合わせている。

## Docker でのビルド

```bash
cd docs/solvers
docker run --rm -v "$PWD":/work -w /work texlive/texlive:latest sh -c '
  for f in reference frontier2d_sparse; do
    lualatex -interaction=nonstopmode -halt-on-error "$f.tex"   # 1 回目
    lualatex -interaction=nonstopmode -halt-on-error "$f.tex"   # TOC/相互参照のため 2 回目
  done'
```

生成物 `reference.pdf` / `frontier2d_sparse.pdf`（コミット済み）。コンテナが root で書くため、
必要なら `docker run --rm -v "$PWD":/work -w /work texlive/texlive:latest chown $(id -u):$(id -g) *.pdf` で
所有者を戻す。
