use crate::config::*;
use crate::body::*;
use three_d::*;

#[derive(Clone, Copy)]
pub struct Particle {
    pos_prev: Vec3,
    pos_pred: Vec3,
    pos: Vec3,
    pos_new: Vec3,
    vel: Vec3,
    acc: Vec3,
    invmass: f32,
}

impl Body for Particle {
    body_common!();

    fn predict(&mut self, dt: f32) {
        self.pos_prev = self.pos;
        self.vel += self.acc*dt;
        self.pos += self.vel*dt;
        self.pos_new = self.pos;
        self.pos_pred = self.pos;
    }
    fn update(&mut self, dt: f32) {
        self.acc = Vec3::zero();
        self.vel = (2.0*self.pos - self.pos_prev - self.pos_pred)/dt;
        // self.vel = (self.pos - self.pos_prev)/dt;
    }
    fn iterate(&mut self) {
        self.pos = self.pos_new;
    }
}

impl Particle {
    pub fn new(pos: Vec3, invmass: f32) -> Self {
        Self {
            pos_prev: pos,
            pos_pred: pos,
            pos,
            pos_new: pos,
            vel: Vec3::zero(),
            acc: Vec3::zero(),
            invmass
        }
    }
}
