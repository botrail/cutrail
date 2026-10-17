//! Triangle meshes for display and output.

use crate::geom::{cross, norm, normalize, sub, Aabb, Point3};
use std::io::{self, Write};

/// A triangle mesh whose vertices carry positions only.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct TriMesh {
    /// Vertex positions
    pub positions: Vec<Point3>,
    /// Triangles (vertex indices, counterclockwise = outward)
    pub triangles: Vec<[u32; 3]>,
}

impl TriMesh {
    /// Number of triangles.
    pub fn triangle_count(&self) -> usize {
        self.triangles.len()
    }

    /// Bounding box.
    pub fn bounds(&self) -> Aabb {
        Aabb::from_points(self.positions.iter().copied())
    }

    /// Signed volume (positive for outward orientation, negative for an inward-facing closed surface).
    pub fn signed_volume(&self) -> f64 {
        let mut v = 0.0;
        for t in &self.triangles {
            let a = self.positions[t[0] as usize];
            let b = self.positions[t[1] as usize];
            let c = self.positions[t[2] as usize];
            v += crate::geom::dot(a, cross(b, c));
        }
        v / 6.0
    }

    /// Appends another mesh.
    pub fn append(&mut self, other: &TriMesh) {
        let offset = self.positions.len() as u32;
        self.positions.extend_from_slice(&other.positions);
        self.triangles
            .extend(other.triangles.iter().map(|t| [t[0] + offset, t[1] + offset, t[2] + offset]));
    }

    /// Writes binary STL.
    pub fn write_stl<W: Write>(&self, mut w: W) -> io::Result<()> {
        let mut header = [0u8; 80];
        let tag = b"cutrail binary STL";
        header[..tag.len()].copy_from_slice(tag);
        w.write_all(&header)?;
        w.write_all(&(self.triangles.len() as u32).to_le_bytes())?;
        for t in &self.triangles {
            let a = self.positions[t[0] as usize];
            let b = self.positions[t[1] as usize];
            let c = self.positions[t[2] as usize];
            let n = normalize(cross(sub(b, a), sub(c, a))).unwrap_or([0.0, 0.0, 0.0]);
            for v in [n, a, b, c] {
                for x in v {
                    w.write_all(&(x as f32).to_le_bytes())?;
                }
            }
            w.write_all(&0u16.to_le_bytes())?;
        }
        Ok(())
    }

    /// Writes a binary STL file.
    pub fn write_stl_file<P: AsRef<std::path::Path>>(&self, path: P) -> io::Result<()> {
        let file = std::fs::File::create(path)?;
        self.write_stl(io::BufWriter::new(file))
    }
}

impl TriMesh {
    /// Merges vertices within `tolerance` of each other and drops degenerate triangles. Returns the number of vertices removed.
    ///
    /// STL files and truck tessellations keep separate vertices per face, so this merge is needed before the mesh can be treated as a closed manifold.
    pub fn merge_vertices(&mut self, tolerance: f64) -> usize {
        use std::collections::HashMap;
        let cell = tolerance.max(1e-12);
        let mut lookup: HashMap<[i64; 3], u32> = HashMap::with_capacity(self.positions.len());
        let mut remap = Vec::with_capacity(self.positions.len());
        let mut positions = Vec::with_capacity(self.positions.len());
        for p in &self.positions {
            let key = [(p[0] / cell).round() as i64, (p[1] / cell).round() as i64, (p[2] / cell).round() as i64];
            let idx = *lookup.entry(key).or_insert_with(|| {
                positions.push(*p);
                (positions.len() - 1) as u32
            });
            remap.push(idx);
        }
        let before = self.positions.len();
        self.positions = positions;
        self.triangles = self
            .triangles
            .iter()
            .map(|t| [remap[t[0] as usize], remap[t[1] as usize], remap[t[2] as usize]])
            .filter(|t| t[0] != t[1] && t[1] != t[2] && t[0] != t[2])
            .collect();
        before - self.positions.len()
    }

    /// Length of the bounding-box diagonal.
    pub fn diagonal(&self) -> f64 {
        let b = self.bounds();
        if b.is_empty() {
            return 0.0;
        }
        norm(b.extent())
    }

    /// Reads STL (binary or ASCII). Vertices stay duplicated per facet; call `merge_vertices` if needed.
    pub fn read_stl<R: io::Read>(mut r: R) -> io::Result<TriMesh> {
        let mut data = Vec::new();
        r.read_to_end(&mut data)?;
        let looks_binary = data.len() >= 84 && {
            let n = u32::from_le_bytes([data[80], data[81], data[82], data[83]]) as usize;
            data.len() == 84 + 50 * n
        };
        let mut mesh = TriMesh::default();
        if looks_binary {
            let n = u32::from_le_bytes([data[80], data[81], data[82], data[83]]) as usize;
            for i in 0..n {
                let base = 84 + 50 * i + 12;
                let mut tri = [0u32; 3];
                for (k, slot) in tri.iter_mut().enumerate() {
                    let o = base + 12 * k;
                    let f = |j: usize| f32::from_le_bytes([data[o + 4 * j], data[o + 4 * j + 1], data[o + 4 * j + 2], data[o + 4 * j + 3]]) as f64;
                    mesh.positions.push([f(0), f(1), f(2)]);
                    *slot = (mesh.positions.len() - 1) as u32;
                }
                mesh.triangles.push(tri);
            }
            return Ok(mesh);
        }
        let text = String::from_utf8_lossy(&data);
        let mut current: Vec<Point3> = Vec::with_capacity(3);
        for line in text.lines() {
            let mut it = line.split_whitespace();
            if it.next() != Some("vertex") {
                continue;
            }
            let mut v = [0.0; 3];
            for slot in &mut v {
                *slot = it
                    .next()
                    .and_then(|s| s.parse().ok())
                    .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "bad STL vertex line"))?;
            }
            current.push(v);
            if current.len() == 3 {
                let base = mesh.positions.len() as u32;
                mesh.positions.extend_from_slice(&current);
                mesh.triangles.push([base, base + 1, base + 2]);
                current.clear();
            }
        }
        if mesh.triangles.is_empty() {
            return Err(io::Error::new(io::ErrorKind::InvalidData, "no triangles found in STL"));
        }
        Ok(mesh)
    }

    /// Reads an STL file.
    pub fn read_stl_file<P: AsRef<std::path::Path>>(path: P) -> io::Result<TriMesh> {
        TriMesh::read_stl(io::BufReader::new(std::fs::File::open(path)?))
    }

    /// Reads Wavefront OBJ (`v` and `f` records; polygons are fan-triangulated). Vertices are shared as in the file,
    /// and positions keep their full precision, unlike STL.
    pub fn read_obj<R: io::BufRead>(r: R) -> io::Result<TriMesh> {
        let bad = |line: usize, what: &str| io::Error::new(io::ErrorKind::InvalidData, format!("OBJ line {line}: {what}"));
        let mut mesh = TriMesh::default();
        for (n, line) in r.lines().enumerate() {
            let line = line?;
            let mut it = line.split_whitespace();
            match it.next() {
                Some("v") => {
                    let mut p = [0.0; 3];
                    for v in &mut p {
                        *v = it.next().and_then(|s| s.parse().ok()).ok_or_else(|| bad(n + 1, "vertex needs three numbers"))?;
                    }
                    mesh.positions.push(p);
                }
                Some("f") => {
                    let count = mesh.positions.len() as i64;
                    let corners: Vec<u32> = it
                        .map(|tok| {
                            let i: i64 = tok.split('/').next().unwrap_or("").parse().map_err(|_| bad(n + 1, "face index"))?;
                            let i = if i < 0 { count + i } else { i - 1 };
                            if i < 0 || i >= count {
                                return Err(bad(n + 1, "face index out of range"));
                            }
                            Ok(i as u32)
                        })
                        .collect::<io::Result<_>>()?;
                    if corners.len() < 3 {
                        return Err(bad(n + 1, "face needs three corners"));
                    }
                    for k in 1..corners.len() - 1 {
                        mesh.triangles.push([corners[0], corners[k], corners[k + 1]]);
                    }
                }
                _ => {}
            }
        }
        Ok(mesh)
    }

    /// Reads an OBJ file.
    pub fn read_obj_file<P: AsRef<std::path::Path>>(path: P) -> io::Result<TriMesh> {
        Self::read_obj(io::BufReader::new(std::fs::File::open(path)?))
    }

    /// Reads a file in the format given by its extension (stl / obj).
    pub fn read_file<P: AsRef<std::path::Path>>(path: P) -> io::Result<TriMesh> {
        let path = path.as_ref();
        match path.extension().and_then(|e| e.to_str()).map(|e| e.to_ascii_lowercase()).as_deref() {
            Some("stl") => Self::read_stl_file(path),
            Some("obj") => Self::read_obj_file(path),
            other => Err(io::Error::new(io::ErrorKind::InvalidInput, format!("unsupported mesh extension {other:?} (stl, obj)"))),
        }
    }

    /// Writes Wavefront OBJ.
    pub fn write_obj<W: Write>(&self, mut w: W) -> io::Result<()> {
        writeln!(w, "# cutrail")?;
        for p in &self.positions {
            writeln!(w, "v {} {} {}", p[0], p[1], p[2])?;
        }
        for t in &self.triangles {
            writeln!(w, "f {} {} {}", t[0] + 1, t[1] + 1, t[2] + 1)?;
        }
        Ok(())
    }

    /// Writes binary glTF (.glb) with f32 positions and u32 indices.
    pub fn write_glb<W: Write>(&self, mut w: W) -> io::Result<()> {
        let mut bin: Vec<u8> = Vec::with_capacity(self.triangles.len() * 12 + self.positions.len() * 12);
        for t in &self.triangles {
            for i in t {
                bin.extend_from_slice(&i.to_le_bytes());
            }
        }
        let index_len = bin.len();
        while bin.len() % 4 != 0 {
            bin.push(0);
        }
        let position_offset = bin.len();
        let b = self.bounds();
        for p in &self.positions {
            for x in p {
                bin.extend_from_slice(&(*x as f32).to_le_bytes());
            }
        }
        let position_len = bin.len() - position_offset;
        while bin.len() % 4 != 0 {
            bin.push(0);
        }
        let (min, max) = if b.is_empty() { ([0.0; 3], [0.0; 3]) } else { (b.min, b.max) };
        let json = format!(
            concat!(
                "{{\"asset\":{{\"version\":\"2.0\",\"generator\":\"cutrail\"}},",
                "\"buffers\":[{{\"byteLength\":{}}}],",
                "\"bufferViews\":[{{\"buffer\":0,\"byteOffset\":0,\"byteLength\":{},\"target\":34963}},",
                "{{\"buffer\":0,\"byteOffset\":{},\"byteLength\":{},\"target\":34962}}],",
                "\"accessors\":[{{\"bufferView\":0,\"componentType\":5125,\"count\":{},\"type\":\"SCALAR\"}},",
                "{{\"bufferView\":1,\"componentType\":5126,\"count\":{},\"type\":\"VEC3\",\"min\":[{},{},{}],\"max\":[{},{},{}]}}],",
                "\"meshes\":[{{\"primitives\":[{{\"attributes\":{{\"POSITION\":1}},\"indices\":0,\"mode\":4}}]}}],",
                "\"nodes\":[{{\"mesh\":0}}],\"scenes\":[{{\"nodes\":[0]}}],\"scene\":0}}"
            ),
            bin.len(),
            index_len,
            position_offset,
            position_len,
            self.triangles.len() * 3,
            self.positions.len(),
            min[0] as f32,
            min[1] as f32,
            min[2] as f32,
            max[0] as f32,
            max[1] as f32,
            max[2] as f32,
        );
        let mut json = json.into_bytes();
        while json.len() % 4 != 0 {
            json.push(b' ');
        }
        let total = 12 + 8 + json.len() + 8 + bin.len();
        w.write_all(&0x4654_6C67u32.to_le_bytes())?;
        w.write_all(&2u32.to_le_bytes())?;
        w.write_all(&(total as u32).to_le_bytes())?;
        w.write_all(&(json.len() as u32).to_le_bytes())?;
        w.write_all(&0x4E4F_534Au32.to_le_bytes())?;
        w.write_all(&json)?;
        w.write_all(&(bin.len() as u32).to_le_bytes())?;
        w.write_all(&0x004E_4942u32.to_le_bytes())?;
        w.write_all(&bin)?;
        Ok(())
    }

    /// Writes 3MF (millimeters, uncompressed).
    pub fn write_3mf<W: Write + io::Seek>(&self, w: W) -> io::Result<()> {
        use std::fmt::Write as _;
        let opts = zip::write::SimpleFileOptions::default().compression_method(zip::CompressionMethod::Stored);
        let mut z = zip::ZipWriter::new(w);
        let e = io::Error::other;
        z.start_file("[Content_Types].xml", opts).map_err(e)?;
        z.write_all(
            br#"<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
 <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
 <Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>
</Types>
"#,
        )?;
        z.start_file("_rels/.rels", opts).map_err(e)?;
        z.write_all(
            br#"<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
 <Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>
</Relationships>
"#,
        )?;
        let mut model = String::with_capacity(self.positions.len() * 40 + self.triangles.len() * 40);
        model.push_str(
            r#"<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xml:lang="en-US" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
 <resources>
  <object id="1" type="model">
   <mesh>
    <vertices>
"#,
        );
        for p in &self.positions {
            let _ = writeln!(model, "     <vertex x=\"{}\" y=\"{}\" z=\"{}\"/>", p[0], p[1], p[2]);
        }
        model.push_str("    </vertices>\n    <triangles>\n");
        for t in &self.triangles {
            let _ = writeln!(model, "     <triangle v1=\"{}\" v2=\"{}\" v3=\"{}\"/>", t[0], t[1], t[2]);
        }
        model.push_str(
            r#"    </triangles>
   </mesh>
  </object>
 </resources>
 <build>
  <item objectid="1"/>
 </build>
</model>
"#,
        );
        z.start_file("3D/3dmodel.model", opts).map_err(e)?;
        z.write_all(model.as_bytes())?;
        z.finish().map_err(e)?;
        Ok(())
    }

    /// Writes a file in the format given by its extension (stl / obj / glb / 3mf).
    pub fn write_file<P: AsRef<std::path::Path>>(&self, path: P) -> io::Result<()> {
        let path = path.as_ref();
        let ext = path.extension().and_then(|e| e.to_str()).map(|e| e.to_ascii_lowercase()).unwrap_or_default();
        let file = std::fs::File::create(path)?;
        let mut w = io::BufWriter::new(file);
        match ext.as_str() {
            "stl" => self.write_stl(&mut w),
            "obj" => self.write_obj(&mut w),
            "glb" => self.write_glb(&mut w),
            "3mf" => self.write_3mf(&mut w),
            other => Err(io::Error::new(io::ErrorKind::InvalidInput, format!("unsupported mesh format: {other}"))),
        }
    }
}
