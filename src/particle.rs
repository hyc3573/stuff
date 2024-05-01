use crate::config::*;
use crate::body::*;

pub struct Particle {
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

    fn predict(&mut self, dt: Real) {
        self.pos_new = self.pos + 
            self.vel*dt + Real::from(0.5)*self.acc*dt*dt;
    }
    fn update(&mut self, dt: Real) {
        self.acc = Vecn::ZERO;
        self.vel = (self.pos_new - self.pos)/dt;
        self.pos = self.pos_new;
    }
}
