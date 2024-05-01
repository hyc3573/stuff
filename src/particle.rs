use crate::config::*;
use crate::body::*;

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
    fn invmass(&self) -> Real {self.invmass}
    fn pos(&self) -> Vecn {self.pos}
    fn vel(&self) -> Vecn {self.vel}
    fn acc(&self) -> Vecn {self.acc}

    fn update_pos(&mut self, dx: Vecn) {
        self.pos_new += dx;
    }
    fn add_force(&mut self, f: Vecn) {
        self.acc += self.invmass*f;
    }

    fn predict(&mut self, dt: Real) {
        self.pos = self.pos_prev + 
            self.vel*dt + Real::from(0.5)*self.acc*dt*dt;
        self.pos_new = self.pos;
        self.pos_pred = self.pos;
    }
    fn update(&mut self, dt: Real) {
        self.acc = Vecn::ZERO;
        self.vel = (2.0*self.pos - self.pos_prev - self.pos_new)/dt;
        self.pos_prev = self.pos;
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
            vel: Vecn::ZERO,
            acc: Vecn::ZERO,
            invmass
        }
    }
}
