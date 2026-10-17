// Web Worker：メインスレッドから区間をまとめて受け取り、変更されたタイルのメッシュだけを返す。
// 事前に `wasm-pack build cutrail-wasm --target web --release --out-dir www/pkg` を実行する。
import init, { WasmEngine } from "./pkg/cutrail_wasm.js";

let engine = null;
let lastRevision = 0;

self.onmessage = async (ev) => {
  const msg = ev.data;
  if (msg.type === "init") {
    await init();
    engine = new WasmEngine(msg.min, msg.max, msg.toolDiameter, msg.fluteLength, msg.tileSize ?? 10, msg.sweepTolerance ?? 0.01);
    lastRevision = 0;
    self.postMessage({ type: "ready", parts: Array.from(engine.part_ids()) });
    return;
  }
  if (msg.type === "cut") {
    const removed = engine.cut_many(msg.segments); // Float64Array, 12 numbers per segment
    const changed = Array.from(engine.changed_since(lastRevision));
    const alive = new Set(engine.part_ids());
    lastRevision = engine.revision();
    const meshes = changed.map((id) => {
      const positions = engine.mesh_positions(id);
      const indices = engine.mesh_indices(id);
      return { id, positions, indices };
    });
    const transfer = meshes.flatMap((m) => [m.positions.buffer, m.indices.buffer]);
    self.postMessage(
      { type: "updated", removed, revision: lastRevision, totalVolume: engine.total_volume(), meshes, alive: Array.from(alive) },
      transfer
    );
  }
};
