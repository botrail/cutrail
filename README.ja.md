# cutrail

[English](README.md) | 日本語

Rust 製の切削シミュレーションコア。素材を三角形メッシュとして保持し、工具が通過した体積を堅牢なメッシュブーリアンで差し引く。3 軸・5 軸のフライス加工に対応し、STEP・STL・OBJ・glTF・3MF の入出力、設計モデルとの比較、Web（WASM）での実行ができる。


## 特徴

- **堅牢な材料除去。** 素材は [manifold-rust](https://crates.io/crates/manifold-rust)（Manifold の純 Rust 移植）のメッシュで、同じ深さの底面や同じ経路の側壁が何度重なっても差分が失敗しない。失敗は型付きのエラーで返り、状態は更新されない。
- **工具と移動。** フラット・ボール・ブルノーズエンドミル。直線、円弧・ヘリカル移動。工具軸は任意の向きで、区間内で軸が回る 5 軸の移動にも対応する。掃引体積は弦誤差で分割した凸包として生成する。
- **一定の処理時間。** 素材をタイルに分けて保持し、重なるタイルだけに差分を適用する。10 万区間を処理しても 1 区間の時間は履歴で増えない。
- **入出力。** 素材は直方体・円柱・メッシュ（STL）・STEP。出力は STL・OBJ・glTF・3MF。3 軸・フラットエンドミルの結果（円弧移動を含む）は平面と円柱の B-rep として STEP に書け、FreeCAD と OpenCASCADE で有効なソリッドとして読めることを確認している。
- **検証。** 設計モデルとの比較（削り残し・削りすぎの体積と領域）、シャンク・ホルダーと素材の干渉検出、除去体積と変更範囲の報告。
- **連携。** 時系列の姿勢を区間にまとめるアダプター、変更されたタイルだけを取り出す差分転送、Web Worker / Node 向けの WASM バインディング。

## クイックスタート

```toml
[dependencies]
cutrail = { path = "path/to/cutrail" }   # crates.io には未公開
```

```rust
use cutrail::{Engine, EngineOptions, ManifoldKernel, MotionSegment, Pose, StockShape, Tool};

fn main() -> cutrail::Result<()> {
    // 素材 60x40x20 mm を 10 mm のタイルで保持する
    let stock = StockShape::Box { min: [0.0, 0.0, 0.0], max: [60.0, 40.0, 20.0] };
    let options = EngineOptions::default().with_tile_size(10.0);
    let mut engine = Engine::new(ManifoldKernel, &[stock], options)?;

    // Ø6 のフラットエンドミルで溝を切り、穴を開ける
    let tool = Tool::flat_end_mill(6.0, 20.0);
    let slot = MotionSegment::linear(Pose::vertical([10.0, 10.0, 15.0]), Pose::vertical([50.0, 10.0, 15.0]));
    let plunge = MotionSegment::linear(Pose::vertical([30.0, 30.0, 20.0]), Pose::vertical([30.0, 30.0, 12.0]));
    for seg in [slot, plunge] {
        let report = engine.cut(&tool, &seg)?;
        println!("{:?}: removed {:.2} mm3 in {:?}", report.outcome, report.removed_volume, report.elapsed);
    }

    // 結果の出力
    engine.write_mesh_file("machined.glb")?;
    engine.write_step(std::fs::File::create("machined.step")?, "machined")?;
    Ok(())
}
```

5 軸の移動は始点と終点の姿勢（先端と軸）を与えるだけでよい。

```rust
let seg = MotionSegment::linear(
    Pose::tilted_about_y([2.0, 5.0, 7.0], -25.0),
    Pose::tilted_about_y([8.0, 5.0, 7.0], 25.0),
);
```

## 主な API

| 型・関数 | 役割 |
|---|---|
| `Engine` / `MachiningEngine<K>` | 素材の状態、差分、検証、履歴。`new`、`cut`、`cut_many`、`cut_batch`、`parts`、`mesh`、`changed_since`、`write_mesh_file`、`write_step`、`compare_with_design` |
| `EngineOptions` | 弦誤差、タイルの一辺、干渉検出などの設定 |
| `Tool`、`Cutter`、`Shank`、`Holder` | 工具。切削部とシャンク・ホルダーを区別する |
| `Pose`、`MotionSegment` | 先端と軸の向き、直線・円弧の区間、`pose_at(t)` |
| `CutReport`、`CutOutcome`、`Collision` | 1 回の加工の結果（`NoChange` / `Updated` / `Exhausted`、除去体積、変更範囲、干渉） |
| `TriMesh` | メッシュの読み書き（STL・OBJ 読込、STL・OBJ・glTF・3MF 書出）、頂点の併合 |
| `StockShape` | 直方体・円柱・メッシュの素材 |
| `step`（feature `stepio`） | STEP ファイルから素材・設計モデルを読む |
| `Brep` | 解析面に復元した B-rep と STEP 書出 |
| `DesignComparison` | 設計モデルとの差（削り残し・削りすぎ） |
| `SegmentBuilder` | 姿勢の列を距離・角度のしきい値と弦からの偏差の上限で区間にまとめる |
| `Kernel` | 形状演算カーネルの抽象。既定は `ManifoldKernel` |

## 精度と性能

- 円柱面は弦誤差 `sweep_tolerance`（既定 0.01 mm）で多角形にし、平均幅が直径に一致する半径を使う。除去体積の誤差は工具径 4 mm で 0.3% 程度。
- 回転を伴う区間は最遠点の弦誤差が許容差以下になるサブステップに分ける。
- 10 mm タイルで 1 mm 区間のポケット加工を 10 万区間処理したとき、1 区間あたりの中央値は 2 ms（最初の 100 区間）から 6 ms（最後の 100 区間）に収まり、メモリは 92 MB。5 軸の送りは 3 軸の約 1.7 倍、WASM（Node）は native の約 1.5 倍。
- 100 Hz で記録した 99 秒のロボット加工サイクル（botrail）は、弦からの偏差 0.01 mm で 710 区間にまとまり 1.8 秒で再生でき、サンプルごとに加工した基準との差は除去体積の 0.04% 以内。
- 詳細は [docs/accuracy-and-performance.md](docs/ja/accuracy-and-performance.md)。

## 制限

- 工具は回転体（フラット・ボール・ブルノーズ）。旋削、ねじ切り、カッター径補正は扱わない。
- 解析面の B-rep 出力は 3 軸・フラットエンドミルの水平送り・突込み・水平な円弧移動の結果に限られ、それ以外の面はファセットになる。
- 設計モデルとの比較の公差オフセットは球の多面体近似を使うため、小さい公差では分割数の調整が要る。
- `SegmentBuilder` は直線区間だけを作る。円弧は呼び出し側で `MotionSegment::arc` として渡す。

## ドキュメント

- [docs/ja/architecture.md](docs/ja/architecture.md)：設計の全体像と表現の選択理由
- [docs/ja/guide.md](docs/ja/guide.md)：使い方（設定、工具、移動、出力、STEP、比較、干渉検出、WASM）
- [docs/accuracy-and-performance.md](docs/ja/accuracy-and-performance.md)：精度の決め方と計測結果
- [docs/ja/brep-export.md](docs/ja/brep-export.md)：解析面の復元と STEP 出力

## 開発

```sh
cargo test --release                      # 単体・結合テスト
cargo test --release --features stepio    # STEP 読込を含む
cargo run --release --example slot -- out.stl
cargo run --release --example bench -- 10000 10 1 0 0 flat   # 区間数 タイル まとめ 簡略化 傾き 工具
cargo run --release --example step_export -- out/            # サンプル形状を STEP と STL に書き出す
cargo run --release --example pose_stream -- poses.csv --max-deviation 0.01 --tile 20   # 記録した姿勢ストリームの再生
cargo run --release --example compare_meshes -- a.stl b.stl  # 2 つのメッシュの削り残し・削りすぎ体積
wasm-pack build cutrail-wasm --target nodejs --release --out-dir pkg-node && node cutrail-wasm/test/node_test.cjs 1000 10 20
```

## 依存

- [manifold-rust](https://github.com/larsbrubaker/manifold-rust)（Apache-2.0）：メッシュブーリアン
- [truck](https://github.com/ricosjp/truck)（Apache-2.0、feature `stepio`）：STEP の読込とメッシュ化
- [zip](https://crates.io/crates/zip)：3MF、[web-time](https://crates.io/crates/web-time)：WASM での時刻、[thiserror](https://crates.io/crates/thiserror)

## ライセンス

Apache-2.0（[LICENSE](LICENSE)）
