use crate::config::*;
use crate::body::*;
use three_d::*;

pub struct Particle {
    pos_prev: Vecn,
    pos_pred: Vecn,
    pos: Vecn,
    pos_new: Vecn,
    vel: Vecn,
    acc: Vecn,
    invmass: Real,
}

impl Body for Particle {
    body_common!();

    fn predict(&mut self, dt: Real) {
        self.pos_prev = self.pos;
        self.vel += self.acc*dt;
        self.pos += self.vel*dt;
        self.pos_new = self.pos;
        self.pos_pred = self.pos;
    }
    fn update(&mut self, dt: Real) {
        self.acc = Vecn::zero();
        self.vel = (2.0*self.pos - self.pos_prev - self.pos_pred)/dt;
        // self.vel = (self.pos - self.pos_prev)/dt;
    }
    fn iterate(&mut self) {
        self.pos = self.pos_new;
    }
}

impl Particle {
    pub fn new(pos: Vecn, invmass: Real) -> Self {
        Self {
            pos_prev: pos,
            pos_pred: pos,
            pos,
            pos_new: pos,
            vel: Vecn::zero(),
            acc: Vecn::zero(),
            invmass
        }
    }
}
