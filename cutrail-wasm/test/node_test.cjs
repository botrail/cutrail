// Node での動作確認：1,000 区間のジグザグ（±20° の傾きあり）を実行し、時間と体積を表示する。
// 事前に `wasm-pack build cutrail-wasm --target nodejs --release --out-dir pkg-node` を実行する。
const path = require("node:path");
const { WasmEngine } = require(path.join(__dirname, "..", "pkg-node", "cutrail_wasm.js"));

const n = Number(process.argv[2] || 1000);
const tile = Number(process.argv[3] || 10);
const tilt = Number(process.argv[4] || 20);

const engine = new WasmEngine([0, 0, 0], [200, 100, 20], 6, 20, tile, 0.01);
const segments = [];
const pose = (p, k) => {
  const deg = tilt === 0 ? 0 : tilt * Math.sin((2 * Math.PI * k) / 40);
  const a = (deg * Math.PI) / 180;
  return [p[0], p[1], p[2], Math.sin(a), 0, Math.cos(a)];
};
let k = 0;
outer: for (let level = 0; ; level++) {
  const z = 18.5 - 1.5 * level;
  if (z < 1) break;
  for (let r = 0; r < 46; r++) {
    const y = 5 + 2 * r;
    const forward = (r + level) % 2 === 0;
    for (let i = 0; i < 190; i++) {
      const a = forward ? 5 + i : 195 - i;
      const b = forward ? a + 1 : a - 1;
      segments.push(...pose([a, y, z], k), ...pose([b, y, z], k + 1));
      k++;
      if (k === n) break outer;
    }
  }
}

const before = engine.total_volume();
const batch = 100;
const t0 = performance.now();
let removed = 0;
for (let i = 0; i < segments.length; i += batch * 12) {
  removed += engine.cut_many(Float64Array.from(segments.slice(i, i + batch * 12)));
}
const ms = performance.now() - t0;
const changed = engine.changed_since(0);
let verts = 0;
for (const id of changed) verts += engine.mesh_positions(id).length / 3;
console.log(
  JSON.stringify({
    segments: n,
    tile,
    tilt,
    total_ms: Math.round(ms),
    per_segment_ms: +(ms / n).toFixed(3),
    removed_mm3: +removed.toFixed(3),
    remaining_mm3: +engine.total_volume().toFixed(3),
    volume_check: Math.abs(before - removed - engine.total_volume()) < 1e-6,
    revision: engine.revision(),
    parts: engine.part_ids().length,
    changed_parts: changed.length,
    changed_vertices: verts,
    triangles: engine.total_triangles(),
  })
);
