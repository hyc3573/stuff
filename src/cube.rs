use crate::config::*;
use crate::body::*;
use three_d::*;

pub struct Cube {
    pos_prev: Vecn,
    pos_pred: Vecn,
    pos: Vecn,
    pos_new: Vecn,

    apos_prev: Quat,
    apos_pred: Quat,
    apos: Quat,
    apos_new: Quat,

    vel: Vecn,
    acc: Vecn,

    avel: Vecn,
    aacc: Vecn,

    invmass: Real,
    invinertia: Mat3,
    inertia: Mat3,
}

impl Body for Cube {
    body_common!();
    rigidbody_common!();
}

impl Cube {
    pub fn new(pos: Vecn, apos: Quat, invmass: Real, sidelen: Real) -> Self {
        let inertia: Mat3 = invmass*sidelen*sidelen*Mat3::from_cols(
            vec3(2.0/3.0, -0.25, -0.25),
            vec3(-0.25, 2.0/3.0, -0.25),
            vec3(-0.25, -0.25, 2.0/3.0)
        );
        Self {
            pos_prev: pos,
            pos_pred: pos,
            pos,
            pos_new: pos,

            apos_prev: apos,
            apos_pred: apos,
            apos,
            apos_new: apos,

            vel: Vecn::zero(),
            acc: Vecn::zero(),

            avel: Vecn::zero(),
            aacc: Vecn::zero(),
            
            invmass,
            invinertia: inertia.invert().unwrap(),
            inertia
        }
    }
}
