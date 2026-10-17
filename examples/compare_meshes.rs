//! Compares a machined mesh with a reference mesh (another machined result, or a design model).
//!
//! ```sh
//! cargo run --release --example compare_meshes -- machined.obj reference.obj [tolerance]
//! ```
//!
//! Both files may be STL or OBJ (OBJ keeps full precision). `excess` is material present in the machined
//! mesh but outside the reference, `gouge` is material the reference keeps that the machined mesh lacks.
//! Both are volumes in mm³ with bounding boxes.

use std::error::Error;

use cutrail::{Engine, EngineOptions, ManifoldKernel, StockShape, TriMesh};

fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.len() < 2 {
        return Err("usage: compare_meshes <machined.(stl|obj)> <reference.(stl|obj)> [tolerance]".into());
    }
    let tolerance: f64 = args.get(2).map(|s| s.parse()).transpose()?.unwrap_or(0.0);
    let mut machined = TriMesh::read_file(&args[0])?;
    machined.merge_vertices(1e-6);
    let mut reference = TriMesh::read_file(&args[1])?;
    reference.merge_vertices(1e-6);
    let engine = Engine::new(ManifoldKernel, &[StockShape::Mesh(machined)], EngineOptions::default())?;
    let cmp = engine.compare_with_design(&reference, tolerance)?;
    println!(
        "machined {:.3} mm3, reference {:.3} mm3, tolerance {}: excess {:.4} mm3 {:?}, gouge {:.4} mm3 {:?}",
        engine.total_volume(),
        reference.signed_volume().abs(),
        tolerance,
        cmp.excess_volume,
        cmp.excess_bounds,
        cmp.gouge_volume,
        cmp.gouge_bounds
    );
    println!("RESULT excess_mm3={:.4} gouge_mm3={:.4}", cmp.excess_volume, cmp.gouge_volume);
    Ok(())
}
