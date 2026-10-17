# 使い方

[English](../guide.md) | 日本語

## エンジンの作成

```rust
use cutrail::{Engine, EngineOptions, ManifoldKernel, StockShape};

let stock = StockShape::Box { min: [0.0, 0.0, 0.0], max: [200.0, 100.0, 20.0] };
let options = EngineOptions { sweep_tolerance: 0.01, ..Default::default() }.with_tile_size(10.0);
let mut engine = Engine::new(ManifoldKernel, &[stock], options)?;
```

| `EngineOptions` | 既定 | 意味 |
|---|---|---|
| `sweep_tolerance` | 0.01 mm | 掃引体積の弦誤差。円の分割数と回転・円弧のサブステップ数を決める |
| `stock_tolerance` | 0.01 mm | 素材の円柱の弦誤差。メッシュ素材の頂点併合の距離（この 1/100） |
| `volume_epsilon` | 1e-9 | 保持単位の体積に対するこの比率以下の除去は `NoChange` |
| `tile_size` | `None` | 立方体タイルの一辺。`None` なら部品単位 |
| `simplify_threshold` / `simplify_tolerance` | `None` / 1e-3 | タイルの三角形数が閾値を超えたら簡略化（通常は不要） |
| `collision_check` / `collision_volume_epsilon` | `false` / 1e-6 mm³ | シャンク・ホルダーの干渉検出 |

素材は複数の `StockShape` を渡せる。`Box`、`Cylinder`（軸は +Z）、`Mesh`（閉じた三角形メッシュ。STL から読んだものや STEP から変換したもの）。

タイルの一辺は工具径の 1〜2 倍が目安。小さいほど 1 回の差分が速く、三角形も減る。準備時間は 3,200 タイルでも 0.2 秒程度。

## 工具

```rust
use cutrail::{Holder, Shank, Tool};

let flat = Tool::flat_end_mill(6.0, 20.0);                  // 直径、刃長
let ball = Tool::ball_end_mill(6.0, 20.0);                  // 先端は半径 3 の半球
let bull = Tool::bull_nose_end_mill(6.0, 1.0, 20.0);        // 角の半径 1
let with_holder = flat.clone()
    .with_shank(Shank { diameter: 6.0, length: 30.0 })
    .with_holder(Holder { diameter: 20.0, length: 40.0 });
```

刃長は先端（最下点）から切削部上端までの高さ。シャンクとホルダーは切削せず、`collision_check` が有効なときだけ素材との交差を調べる。

## 姿勢と区間

```rust
use cutrail::{MotionSegment, Pose};

let a = Pose::vertical([10.0, 10.0, 15.0]);              // 軸 +Z
let b = Pose::tilted_about_y([50.0, 10.0, 15.0], 20.0);  // 軸を Y 軸回りに 20° 傾ける
let c = Pose::new([50.0, 10.0, 15.0], [0.3, 0.0, 0.95]); // 任意の軸（正規化される）

let line = MotionSegment::linear(a, b);
let arc = MotionSegment::arc(
    Pose::vertical([30.0, 20.0, 15.0]),  // 始点
    Pose::vertical([20.0, 30.0, 15.0]),  // 終点
    [20.0, 20.0, 15.0],                  // 中心
    [0.0, 0.0, 1.0],                     // 法線（右手系で反時計回り）
);
let mid = arc.pose_at(0.5);
```

- 先端は直線または円弧の上を動き、軸は始点から終点へ球面線形補間で回る。区間内で軸が 150° 以上回る移動は拒否されるので、区間を分けて渡す。
- 円弧は始点と終点の平面上の位置が一致すれば一周。法線方向の高さの差は角度に比例して変わる（ヘリカル）。時計回りは法線を反転する。
- 座標は素材座標系、単位は mm。上位系の座標変換と単位変換は呼び出し側で行う。

## 切削

```rust
let report = engine.cut(&tool, &line)?;             // 1 区間
let reports = engine.cut_many(&tool, &segments)?;   // 順に処理。失敗した区間より前は確定済み
let report = engine.cut_batch(&tool, &segments)?;   // まとめて 1 回の更新・1 つのレポート
```

`CutReport`：

| フィールド | 内容 |
|---|---|
| `outcome` | `NoChange`（除去なし、状態は不変）、`Updated`、`Exhausted`（素材がすべてなくなった） |
| `revision` | 更新後のリビジョン。`NoChange` では据え置き |
| `removed_volume` | 除去体積 (mm³) |
| `changed_bounds` | 変更された範囲 |
| `cells_touched` | 差分を実行した保持単位の数 |
| `collision` | シャンク・ホルダーの干渉（体積と範囲） |
| `elapsed` | 処理時間 |
| `diagnostics` | 分割、消失、サブステップ数、干渉などの補足 |

失敗（`Err`）のときは状態が更新されない。`Error::Unsupported` は区間の分割などで対処でき、`Error::Kernel` / `Error::Validation` はカーネルの演算または結果の検証の失敗を表す。

## 状態の取得と表示

```rust
for part in engine.parts() { /* id, bounds, volume, triangles, revision */ }
let mesh = engine.mesh(part_id);                 // キャッシュされた表示メッシュ
let changed = engine.changed_since(last_revision); // 変更された保持単位だけを再送する
let total = engine.total_volume();
```

タイル単位では `PartId` はタイルの通し番号で、全量除去されたタイルは `parts()` から消える。部品単位では差分で分かれた部品に新しい番号が付き、最大の部品が元の番号を引き継ぐ。

## 出力

```rust
engine.write_stl(std::fs::File::create("out.stl")?)?;
engine.write_mesh_file("out.obj")?;   // 拡張子で stl / obj / glb / 3mf を選ぶ
engine.write_step(std::fs::File::create("out.step")?, "machined")?;   // 解析面の B-rep（brep-export.md）
let mesh = engine.combined_mesh();    // TriMesh：全保持単位の連結（表示用）
let solid = engine.merged_mesh()?;    // TriMesh：タイルを結合した 1 つの閉じたソリッド（出力・比較用）
```

## 素材の読込

```rust
use cutrail::TriMesh;
let mut mesh = TriMesh::read_stl_file("stock.stl")?;   // バイナリ・ASCII
mesh.merge_vertices(1e-4);                              // 取り込み時にも併合されるが、閉じているかの確認に使える
let engine = Engine::new(ManifoldKernel, &[StockShape::Mesh(mesh)], options)?;
```

STEP は feature `stepio` で読める（truck-stepio で B-rep を読み、メッシュ化して頂点を併合する）。

```rust
let shapes = cutrail::step::stock_from_step_file("stock.step", 0.01)?;   // シェルごとに StockShape::Mesh
let design = cutrail::step::meshes_from_step_file("design.step", 0.01)?; // 比較用のメッシュ
```

## 設計モデルとの比較

```rust
let design = TriMesh::read_stl_file("design.stl")?;
let cmp = engine.compare_with_design(&design, 0.05)?;   // 公差 0.05 mm（0 なら厳密）
println!("excess {:.3} mm3, gouge {:.3} mm3", cmp.excess_volume, cmp.gouge_volume);
cmp.excess.write_file("excess.glb")?;                   // 削り残しの領域
```

`excess` は設計の外に残った材料（削り残し）、`gouge` は設計の内側で削られた領域（削りすぎ）。公差を与えると、設計をその分だけ膨らませた形状・削った形状との差になる。オフセットは球の多面体近似を使うので、小さい公差では角が丸まる誤差がある。比較は検証用で、加工のたびに行う想定ではない。

## 干渉検出

```rust
let options = EngineOptions::default().with_tile_size(10.0).with_collision_check();
let report = engine.cut(&tool, &seg)?;
if let Some(c) = &report.collision {
    eprintln!("non-cutting part intersects {:.3} mm3 of stock within {:?}", c.volume, c.bounds);
}
```

切削部の除去を適用した後の素材に対して、シャンクとホルダーの掃引体積の交差を調べる。切削は通常どおり適用され、判断は呼び出し側に任せる。

## 姿勢ストリーム

物理周期や描画周期で送られてくる姿勢を区間にまとめる。

```rust
use cutrail::SegmentBuilder;
let mut builder = SegmentBuilder::new(5.0, 2.0)   // 区間の長さを 5 mm、軸の回転を 2° で区切る
    .with_max_deviation(0.01);                    // 経路が弦から 0.01 mm 離れる前に区間を切る
for pose in poses {
    if let Some(seg) = builder.push(pose) {
        engine.cut(&tool, &seg)?;
    }
}
if let Some(seg) = builder.flush() {
    engine.cut(&tool, &seg)?;
}
```

距離のしきい値だけでは角を弦で横切る（1 mm のしきい値で 90° の角が最大 0.5 mm 削れる）。`with_max_deviation` を使うと、飛ばしたサンプルがすべて出力する弦から許容差以内に収まるように区間を切るので、角はその角のサンプルで切れ、円弧は許容差の弦になる。`min_distance` は区間の長さの上限として働く。記録されたロボット経路（99 秒のサイクルを 100 Hz で約 9,900 姿勢）では、許容差 0.01 mm・上限 5 mm・2° で区間は約 700 本、基準の加工結果との差は全サンプルで 0.02 mm 以内、1 kHz のストリームからも同じ区間が得られる。計測は [accuracy-and-performance.md](accuracy-and-performance.md) を参照。

`examples/pose_stream.rs` はこのようなストリームを CSV（1 行に `t,x,y,z,ax,ay,az`、素材座標系。ヘッダ行 `# stock_box` と `# tool`）から再生し、しきい値とタイルサイズを変えて時間・区間数・基準メッシュとの差を報告する。

```sh
cargo run --release --example pose_stream -- poses.csv --max-deviation 0.01 --tile 20 --out out.obj
```

## WASM

`cutrail-wasm` が `WasmEngine` を公開する。区間は 1 区間あたり 12 個の数（始点の先端 xyz、始点の軸 xyz、終点の先端 xyz、終点の軸 xyz）を並べた `Float64Array` で渡し、メッシュは保持単位ごとに `Float32Array`（位置）と `Uint32Array`（添字）で受け取る。

```sh
wasm-pack build cutrail-wasm --target nodejs --release --out-dir pkg-node
node cutrail-wasm/test/node_test.cjs 1000 10 20        # 区間数 タイル 傾き
wasm-pack build cutrail-wasm --target web --release --out-dir www/pkg   # Web Worker（cutrail-wasm/www/）
```

WASM は単一スレッドで、処理時間は native の 1.5 倍程度。
