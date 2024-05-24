use crate::config::*;
use crate::constraint;
use crate::constraint::*;
use crate::particle::Particle;
use crate::body::Body;
use std::cell::RefCell;
use std::rc::Rc;
use three_d::*;

pub struct ParticleDist {
    bodies: [Rc<RefCell<dyn Body>>; 2],
    lambda: f32,
    compliance: f32,
    dist: f32
}

impl Constraint for ParticleDist {
    fn C(&self) -> f32 {
        self.bodies()[0].borrow().pos().distance(self.bodies()[1].borrow().pos()) - self.dist
    }

    fn dC(&self) -> Vec<Vec3> {
        let mut n = (self.bodies()[0].borrow().pos() - self.bodies()[1].borrow().pos());
        if !n.is_zero() {
            n = n.normalize();
        }

        vec!(n, -n)
    }

    constraint_getset!(2);
}

impl ParticleDist {
    pub fn new(bodies: [Rc<RefCell<dyn Body>>; 2], dist: f32, compliance: f32) -> Self {
        Self {
            bodies,
            lambda: 0.0,
            dist,
            compliance
        }
    }
}

pub struct ParticleFix {
    bodies: [Rc<RefCell<dyn Body>>; 1],
    lambda: f32,
    compliance: f32,
    origin: Vec3
}

impl Constraint for ParticleFix {
    fn C(&self) -> f32 {
        self.bodies()[0].borrow().pos().distance(self.origin)
    }

    fn dC(&self) -> Vec<Vec3> {
        let mut n = self.bodies()[0].borrow().pos() - self.origin;
        if !n.is_zero() {
            n = n.normalize();
        }
        vec!(n)
    }

    constraint_getset!(1);
}

impl ParticleFix {
    pub fn new(parts: [Rc<RefCell<dyn Body>>; 1], origin: Vec3, compliance: f32) -> Self {
        Self {
            bodies: parts,
            lambda: 0.0,
            compliance,
            origin
        } 
    }
}

pub struct ParticleSimpleXWall {
    bodies: [Rc<RefCell<dyn Body>>; 1],
    lambda: f32,
    compliance: f32,
}

impl Constraint for ParticleSimpleXWall {
    fn C(&self) -> f32 {
        f32::max(-self.bodies()[0].borrow().pos().x, 0.0)
    }
    fn dC(&self) -> Vec<Vec3> {
        let mut g = Vec3::zero();
        if self.bodies()[0].borrow().pos().x < 0.0 {
            g.x = -1.0;
        }
        vec!(g)
    }

    constraint_getset!(1);
}

impl ParticleSimpleXWall {
    pub fn new(bodies: [Rc<RefCell<dyn Body>>; 1], compliance: f32) -> Self {
        Self {
            bodies,
            lambda: 0.0,
            compliance
        }
    }
}
