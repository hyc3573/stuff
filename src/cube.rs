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

    fn predict(&mut self, dt: Real) {
        self.pos_prev = self.pos;
        self.vel += self.acc*dt;
        self.pos += self.vel*dt;
        self.pos_new = self.pos;
        self.pos_pred = self.pos;

        self.apos_prev = self.apos;
        self.avel += self.aacc*dt - dt*self.invinertia*self.avel.cross(self.inertia*self.avel);
        self.apos = self.apos + Quat::new(0.0, self.avel.x, self.avel.y, self.avel.y)*self.apos*0.5*dt;
        self.apos = self.apos.normalize();
        self.apos_new = self.apos;
        self.apos_pred = self.apos;
    }
    fn update(&mut self, dt: Real) {
        self.acc = Vecn::zero();
        self.vel = (2.0*self.pos - self.pos_prev - self.pos_pred)/dt;

        self.aacc = Vecn::zero();
        let dq = self.apos*self.apos_prev.invert();
        self.avel = 2.0*vec3(dq.v.x, dq.v.y, dq.v.z)/dt;

        if dq.s < 0.0 {
            self.avel *= -1.0;
        }
    }
    fn iterate(&mut self) {
        self.pos = self.pos_new;
        self.apos = self.apos_new;
    }
}

impl RigidBody for Cube {
    rigidbody_common!();

    fn update_apos(&mut self, dq: Quat) {
        self.apos_new = dq * self.apos_new;
    }

    fn add_force_at(&mut self, f: Vecn, at: Vecn) {
        self.aacc += self.invinertia*at.cross(f);
    }

    fn add_torque(&mut self, t: Vecn) {
        self.aacc += self.invinertia*t;
    }
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
