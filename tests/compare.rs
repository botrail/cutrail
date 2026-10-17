//! Comparison against a design model: excess material, gouges, and tolerance.
mod common;
use common::{assert_close, slotted_cube};

#[test]
fn compare_with_design_reports_excess_and_gouge() {
    // Design: a through slot 4 wide and 4 deep (volume 840)
    let design = slotted_cube(4.0, 6.0).combined_mesh();
    // Result A only went 2 deep: excess 4*10*2 = 80
    let a = slotted_cube(4.0, 8.0);
    let ca = a.compare_with_design(&design, 0.0).unwrap();
    assert_close(ca.excess_volume, 80.0, 2e-2, "excess of shallow cut");
    assert!(ca.gouge_volume < 0.5, "gouge {}", ca.gouge_volume);
    assert!(ca.excess_bounds.is_some() && ca.excess.triangle_count() > 0);
    // Result B was cut 6 wide: gouge 2*10*4 = 80
    let b = slotted_cube(6.0, 6.0);
    let cb = b.compare_with_design(&design, 0.0).unwrap();
    assert_close(cb.gouge_volume, 80.0, 2e-2, "gouge of wide cut");
    assert!(cb.excess_volume < 0.5, "excess {}", cb.excess_volume);
    // Tolerance 0.5: the design dilated by 0.5 has its slot at x 3.5..6.5 with the floor at z 6.5, so A's excess drops to 3*1.5*10 = 45
    let ca_tol = a.compare_with_design(&design, 0.5).unwrap();
    assert!(
        ca_tol.excess_volume < ca.excess_volume,
        "{} vs {}",
        ca_tol.excess_volume,
        ca.excess_volume
    );
    assert_close(ca_tol.excess_volume, 45.0, 5e-2, "excess beyond tolerance");
    // Identical shapes give zero for both
    let same = slotted_cube(4.0, 6.0).compare_with_design(&design, 0.0).unwrap();
    assert!(
        same.is_within_tolerance(),
        "{:?}",
        (same.excess_volume, same.gouge_volume)
    );
}
