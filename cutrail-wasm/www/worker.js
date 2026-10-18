// Web Worker: owns the engine. The page sends batches of segments or sampled poses and gets back the
// meshes of the tiles that changed, with every triangle labeled as stock skin or cut surface.
// Run `wasm-pack build cutrail-wasm --target web --release --out-dir www/pkg` first.
import init, { WasmEngine } from "./pkg/cutrail_wasm.js";

let engine = null;
let lastRevision = 0;
let initialized = false;

async function create(p) {
  if (!initialized) {
    await init();
    initialized = true;
  }
  if (engine) engine.free();
  engine = new WasmEngine(p.min, p.max, p.tool.diameter, p.tool.flute, p.tileSize, p.sweepTolerance);
  if (p.tool.kind === "ball") engine.set_ball_end_mill(p.tool.diameter, p.tool.flute);
  engine.set_stream(p.stream.minDistance, p.stream.minAngle, p.stream.maxDeviation);
  lastRevision = 0;
}

function snapshot(ids) {
  const meshes = [];
  for (const id of ids) {
    const m = engine.mesh_with_kinds(id);
    if (!m) continue;
    meshes.push({ id, positions: m.positions, indices: m.indices, kinds: m.kinds });
    m.free();
  }
  return meshes;
}

function reply(type, ids, removed, ms) {
  lastRevision = engine.revision();
  const meshes = snapshot(ids);
  const transfer = meshes.flatMap((m) => [m.positions.buffer, m.indices.buffer, m.kinds.buffer]);
  self.postMessage(
    {
      type,
      removed,
      ms,
      revision: lastRevision,
      totalVolume: engine.total_volume(),
      totalTriangles: engine.total_triangles(),
      segments: engine.segment_count(),
      alive: Array.from(engine.part_ids()),
      meshes,
    },
    transfer
  );
}

self.onmessage = async (ev) => {
  const msg = ev.data;
  try {
    if (msg.type === "init") {
      await create(msg);
      reply("ready", Array.from(engine.part_ids()), 0, 0);
      return;
    }
    const t0 = performance.now();
    let removed = 0;
    if (msg.type === "segments") removed = engine.cut_many(msg.data); // 12 numbers per segment
    else if (msg.type === "poses") removed = engine.push_poses(msg.data); // 6 numbers per pose
    else if (msg.type === "flush") removed = engine.flush();
    else return;
    const ms = performance.now() - t0;
    reply("updated", Array.from(engine.changed_since(lastRevision)), removed, ms);
  } catch (e) {
    self.postMessage({ type: "error", message: String(e) });
  }
};
