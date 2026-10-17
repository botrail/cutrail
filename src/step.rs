//! Reading stock from STEP files (feature `stepio`).
//!
//! truck-stepio reads the B-rep, truck-meshalgo tessellates it, and vertices are merged into a closed mesh.
//! Not used for booleans; it is the input path for stock ([`StockShape::Mesh`]) and design models (for comparison).

use crate::error::{Error, Result};
use crate::mesh::TriMesh;
use crate::stock::StockShape;
use truck_meshalgo::prelude::*;
use truck_stepio::r#in::{ruststep, Table};

/// Converts a truck polygon mesh into a triangle mesh (quads and larger polygons are fan-triangulated).
fn to_trimesh(poly: &PolygonMesh) -> TriMesh {
    let positions = poly.positions().iter().map(|p| [p.x, p.y, p.z]).collect();
    let mut triangles = Vec::with_capacity(poly.faces().tri_faces().len());
    for f in poly.faces().tri_faces() {
        triangles.push([f[0].pos as u32, f[1].pos as u32, f[2].pos as u32]);
    }
    for f in poly.faces().quad_faces() {
        triangles.push([f[0].pos as u32, f[1].pos as u32, f[2].pos as u32]);
        triangles.push([f[0].pos as u32, f[2].pos as u32, f[3].pos as u32]);
    }
    for f in poly.faces().other_faces() {
        for i in 1..f.len() - 1 {
            triangles.push([f[0].pos as u32, f[i].pos as u32, f[i + 1].pos as u32]);
        }
    }
    TriMesh { positions, triangles }
}

/// Tessellates every shell in a STEP string. Vertices stay duplicated per face.
pub fn meshes_from_step_str(step: &str, chord_tolerance: f64) -> Result<Vec<TriMesh>> {
    let exchange = ruststep::parser::parse(step).map_err(|e| Error::InvalidInput(format!("STEP parse error: {e}")))?;
    let data = exchange.data.first().ok_or_else(|| Error::InvalidInput("STEP file has no data section".into()))?;
    let table = Table::from_data_section(data);
    let mut shells: Vec<_> = table.shell.iter().collect();
    shells.sort_by_key(|(id, _)| **id);
    let mut out = Vec::with_capacity(shells.len());
    for (id, shell) in shells {
        let cshell = table
            .to_compressed_shell(shell)
            .map_err(|e| Error::InvalidInput(format!("STEP shell #{id} conversion error: {e:?}")))?;
        let poly = cshell.robust_triangulation(chord_tolerance).to_polygon();
        out.push(to_trimesh(&poly));
    }
    if out.is_empty() {
        return Err(Error::InvalidInput("STEP file has no shells".into()));
    }
    Ok(out)
}

/// Tessellates every shell in a STEP file.
pub fn meshes_from_step_file<P: AsRef<std::path::Path>>(path: P, chord_tolerance: f64) -> Result<Vec<TriMesh>> {
    let text = std::fs::read_to_string(path).map_err(|e| Error::InvalidInput(format!("cannot read STEP file: {e}")))?;
    meshes_from_step_str(&text, chord_tolerance)
}

/// Returns every shell in a STEP string as a stock shape with merged vertices.
pub fn stock_from_step_str(step: &str, chord_tolerance: f64) -> Result<Vec<StockShape>> {
    Ok(meshes_from_step_str(step, chord_tolerance)?
        .into_iter()
        .map(|mut mesh| {
            mesh.merge_vertices(chord_tolerance * 0.01);
            StockShape::Mesh(mesh)
        })
        .collect())
}

/// Returns every shell in a STEP file as a stock shape.
pub fn stock_from_step_file<P: AsRef<std::path::Path>>(path: P, chord_tolerance: f64) -> Result<Vec<StockShape>> {
    let text = std::fs::read_to_string(path).map_err(|e| Error::InvalidInput(format!("cannot read STEP file: {e}")))?;
    stock_from_step_str(&text, chord_tolerance)
}
