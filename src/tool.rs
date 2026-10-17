//! Tool definitions. The cutting part (cutter) is distinguished from the shank and holder.
//! Tools are bodies of revolution described by height above the tip (lowest point) and distance from the axis.

use crate::error::{Error, Result};
use std::f64::consts::FRAC_PI_2;

/// A tool.
#[derive(Clone, Debug, PartialEq)]
pub struct Tool {
    /// The cutting part
    pub cutter: Cutter,
    /// Shank above the flutes (does not cut)
    pub shank: Option<Shank>,
    /// Holder above the shank (does not cut)
    pub holder: Option<Holder>,
}

/// Shape of the cutting part. In every variant `flute_length` is the height from the tip (lowest point) to the top of the cutting part.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum Cutter {
    /// Flat end mill: flat tip, cylindrical side
    FlatEndMill {
        /// Diameter (mm)
        diameter: f64,
        /// Flute length (mm)
        flute_length: f64,
    },
    /// Ball end mill: hemispherical tip with radius = diameter / 2
    BallEndMill {
        /// Diameter (mm)
        diameter: f64,
        /// Flute length (mm); at least the radius
        flute_length: f64,
    },
    /// Bull-nose (corner radius) end mill: the tip corner is a torus of radius `corner_radius`
    BullNoseEndMill {
        /// Diameter (mm)
        diameter: f64,
        /// Corner radius (mm); positive and at most the tool radius
        corner_radius: f64,
        /// Flute length (mm); at least the corner radius
        flute_length: f64,
    },
}

/// Shank (the cylinder above the flutes).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Shank {
    /// Diameter (mm)
    pub diameter: f64,
    /// Length (mm)
    pub length: f64,
}

/// Holder (the cylinder above the shank).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Holder {
    /// Diameter (mm)
    pub diameter: f64,
    /// Length (mm)
    pub length: f64,
}

fn positive(name: &str, v: f64) -> Result<()> {
    if v > 0.0 && v.is_finite() {
        Ok(())
    } else {
        Err(Error::InvalidInput(format!("{name} must be positive: {v}")))
    }
}

impl Tool {
    /// Creates a flat end mill.
    pub fn flat_end_mill(diameter: f64, flute_length: f64) -> Tool {
        Tool { cutter: Cutter::FlatEndMill { diameter, flute_length }, shank: None, holder: None }
    }

    /// Creates a ball end mill.
    pub fn ball_end_mill(diameter: f64, flute_length: f64) -> Tool {
        Tool { cutter: Cutter::BallEndMill { diameter, flute_length }, shank: None, holder: None }
    }

    /// Creates a bull-nose end mill.
    pub fn bull_nose_end_mill(diameter: f64, corner_radius: f64, flute_length: f64) -> Tool {
        Tool { cutter: Cutter::BullNoseEndMill { diameter, corner_radius, flute_length }, shank: None, holder: None }
    }

    /// Attaches a shank.
    pub fn with_shank(mut self, shank: Shank) -> Tool {
        self.shank = Some(shank);
        self
    }

    /// Attaches a holder.
    pub fn with_holder(mut self, holder: Holder) -> Tool {
        self.holder = Some(holder);
        self
    }

    /// Radius of the cutting part.
    pub fn radius(&self) -> f64 {
        match self.cutter {
            Cutter::FlatEndMill { diameter, .. }
            | Cutter::BallEndMill { diameter, .. }
            | Cutter::BullNoseEndMill { diameter, .. } => diameter * 0.5,
        }
    }

    /// Length of the cutting part (height above the tip).
    pub fn flute_length(&self) -> f64 {
        match self.cutter {
            Cutter::FlatEndMill { flute_length, .. }
            | Cutter::BallEndMill { flute_length, .. }
            | Cutter::BullNoseEndMill { flute_length, .. } => flute_length,
        }
    }

    /// Corner radius of the tip (the tool radius for a ball end mill, 0 for a flat end mill).
    pub fn corner_radius(&self) -> f64 {
        match self.cutter {
            Cutter::FlatEndMill { .. } => 0.0,
            Cutter::BallEndMill { diameter, .. } => diameter * 0.5,
            Cutter::BullNoseEndMill { corner_radius, .. } => corner_radius,
        }
    }

    /// Distance from the tip to the farthest point of the cutting part.
    pub fn reach(&self) -> f64 {
        (self.flute_length().powi(2) + self.radius().powi(2)).sqrt()
    }

    /// Non-cutting cylinders (shank, holder) as `(radius, bottom height, top height)`, bottom first.
    pub fn non_cutting_cylinders(&self) -> Vec<(f64, f64, f64)> {
        let mut out = Vec::new();
        let mut h = self.flute_length();
        if let Some(s) = self.shank {
            out.push((s.diameter * 0.5, h, h + s.length));
            h += s.length;
        }
        if let Some(hd) = self.holder {
            out.push((hd.diameter * 0.5, h, h + hd.length));
        }
        out
    }

    /// Revolution profile of the cutting part as `(radius, height above the tip)`, starting at the tip.
    /// Curved parts are subdivided so that the chord error stays within `chord_tolerance`.
    pub fn cutting_profile(&self, chord_tolerance: f64) -> Vec<(f64, f64)> {
        let radius = self.radius();
        let length = self.flute_length();
        let rc = self.corner_radius();
        if rc <= 0.0 {
            return vec![(radius, 0.0), (radius, length)];
        }
        let rings = corner_rings(rc, chord_tolerance);
        let flat = radius - rc;
        let mut profile = Vec::with_capacity(rings + 2);
        for k in 0..=rings {
            let phi = FRAC_PI_2 * k as f64 / rings as f64;
            profile.push((flat + rc * phi.sin(), rc * (1.0 - phi.cos())));
        }
        if length > rc {
            profile.push((radius, length));
        }
        profile
    }

    /// Whether the point at height `height` above the tip and distance `radial` from the axis lies inside the cutting part (boundary included).
    pub fn contains(&self, radial: f64, height: f64) -> bool {
        let radius = self.radius();
        if height < 0.0 || height > self.flute_length() || radial > radius {
            return false;
        }
        let rc = self.corner_radius();
        if rc <= 0.0 || height >= rc {
            return true;
        }
        let flat = radius - rc;
        let dz = rc - height;
        radial <= flat + (rc * rc - dz * dz).max(0.0).sqrt()
    }

    /// Validates the dimensions.
    pub fn validate(&self) -> Result<()> {
        match self.cutter {
            Cutter::FlatEndMill { diameter, flute_length } => {
                positive("tool diameter", diameter)?;
                positive("tool flute length", flute_length)?;
            }
            Cutter::BallEndMill { diameter, flute_length } => {
                positive("tool diameter", diameter)?;
                positive("tool flute length", flute_length)?;
                if flute_length < diameter * 0.5 - 1e-9 {
                    return Err(Error::InvalidInput("ball end mill flute length must be at least the radius".into()));
                }
            }
            Cutter::BullNoseEndMill { diameter, corner_radius, flute_length } => {
                positive("tool diameter", diameter)?;
                positive("corner radius", corner_radius)?;
                positive("tool flute length", flute_length)?;
                if corner_radius > diameter * 0.5 + 1e-9 {
                    return Err(Error::InvalidInput("corner radius must not exceed the tool radius".into()));
                }
                if flute_length < corner_radius - 1e-9 {
                    return Err(Error::InvalidInput("flute length must be at least the corner radius".into()));
                }
            }
        }
        if let Some(s) = self.shank {
            positive("shank diameter", s.diameter)?;
            positive("shank length", s.length)?;
        }
        if let Some(h) = self.holder {
            positive("holder diameter", h.diameter)?;
            positive("holder length", h.length)?;
        }
        Ok(())
    }
}

/// Number of rings (2 to 64) that subdivide the corner arc (a quarter turn) within the chord tolerance.
fn corner_rings(corner_radius: f64, chord_tolerance: f64) -> usize {
    let tol = chord_tolerance.max(1e-9).min(corner_radius * 0.5);
    let step = 2.0 * (1.0 - tol / corner_radius).acos();
    let n = (FRAC_PI_2 / step).ceil();
    if n.is_finite() {
        (n as usize).clamp(2, 64)
    } else {
        64
    }
}
