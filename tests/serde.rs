//! Serialization round trips (feature `serde`).
#![cfg(feature = "serde")]
use cutrail::{Cutter, EngineOptions, MotionSegment, Pose, StockShape, Tool};

#[test]
fn geometry_and_options_round_trip_through_json() {
    let seg = MotionSegment::arc(
        Pose::vertical([8.0, 5.0, 6.0]),
        Pose::tilted_about_y([5.0, 8.0, 6.0], 10.0),
        [5.0, 5.0, 6.0],
        [0.0, 0.0, 1.0],
    );
    let json = serde_json::to_string(&seg).unwrap();
    assert_eq!(serde_json::from_str::<MotionSegment>(&json).unwrap(), seg);

    let tool = Tool::bull_nose_end_mill(6.0, 1.0, 12.0).with_shank(cutrail::Shank {
        diameter: 6.0,
        length: 20.0,
    });
    let back: Tool = serde_json::from_str(&serde_json::to_string(&tool).unwrap()).unwrap();
    assert_eq!(back, tool);
    assert!(matches!(back.cutter, Cutter::BullNoseEndMill { .. }));

    let stock = StockShape::Cylinder {
        center: [1.0, 2.0],
        radius: 5.0,
        z0: 0.0,
        z1: 10.0,
    };
    assert_eq!(
        serde_json::from_str::<StockShape>(&serde_json::to_string(&stock).unwrap()).unwrap(),
        stock
    );

    // Options deserialize with defaults for the fields a config file leaves out
    let options: EngineOptions = serde_json::from_str(r#"{"tile_size": 20.0, "collision_check": true}"#).unwrap();
    assert_eq!(options.tile_size, Some(20.0));
    assert!(options.collision_check);
    assert_eq!(options.sweep_tolerance, EngineOptions::default().sweep_tolerance);
}
