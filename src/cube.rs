use crate::config::*;
use crate::body::*;
use three_d::*;
use crate::Collider;

#[derive(Clone)]
pub struct RigidBody {
    pos_prev: Vec3,
    pos_pred: Vec3,
    pos: Vec3,
    pos_new: Vec3,

    apos_prev: Quat,
    apos_pred: Quat,
    apos: Quat,
    apos_new: Quat,

    vel: Vec3,
    acc: Vec3,

    avel: Vec3,
    aacc: Vec3,

    invmass: f32,
    invinertia: Mat3,
    inertia: Mat3,
}

impl Body for RigidBody {
    body_common!();
    rigidbody_common!();
}

impl RigidBody {
    pub fn new(pos: Vec3, apos: Quat, invmass: f32, inertia_mass: Mat3) -> Self {
        let inertia = if !invmass.is_zero() {
            inertia_mass*invmass
        } else {Mat3::zero()};

        Self {
            pos_prev: pos,
            pos_pred: pos,
            pos,
            pos_new: pos,

            apos_prev: apos,
            apos_pred: apos,
            apos,
            apos_new: apos,

            vel: Vec3::zero(),
            acc: Vec3::zero(),

            avel: Vec3::zero(),
            aacc: Vec3::zero(),
            
            invmass,
            invinertia: inertia.invert().unwrap(),
            inertia: inertia,
        }
    }
}
