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
    lambda: Real,
    compliance: Real,
    dist: Real
}

impl Constraint<2> for ParticleDist {
    fn C(&self) -> Real {
        self.bodies()[0].borrow().pos().distance(self.bodies()[1].borrow().pos()) - self.dist
    }

    fn dC(&self) -> [Vecn; 2] {
        let mut n = (self.bodies()[0].borrow().pos() - self.bodies()[1].borrow().pos());
        if !n.is_zero() {
            n = n.normalize();
        }

        [n, -n]
    }

    constraint_getset!(2);
}

impl ParticleDist {
    pub fn new(bodies: [Rc<RefCell<dyn Body>>; 2], dist: Real, compliance: Real) -> Self {
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
    lambda: Real,
    compliance: Real,
    origin: Vecn
}

impl Constraint<1> for ParticleFix {
    fn C(&self) -> Real {
        self.bodies()[0].borrow().pos().distance(self.origin)
    }

    fn dC(&self) -> [Vecn; 1] {
        let mut n = self.bodies()[0].borrow().pos() - self.origin;
        if !n.is_zero() {
            n = n.normalize();
        }
        [n]
    }

    constraint_getset!(1);
}

impl ParticleFix {
    pub fn new(parts: [Rc<RefCell<dyn Body>>; 1], origin: Vecn, compliance: Real) -> Self {
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
    lambda: Real,
    compliance: Real,
}

impl Constraint<1> for ParticleSimpleXWall {
    fn C(&self) -> Real {
        Real::max(-self.bodies()[0].borrow().pos().x, 0.0)
    }
    fn dC(&self) -> [Vecn; 1] {
        let mut g = Vecn::zero();
        if self.bodies()[0].borrow().pos().x < 0.0 {
            g.x = -1.0;
        }
        [g]
    }

    constraint_getset!(1);
}

impl ParticleSimpleXWall {
    pub fn new(bodies: [Rc<RefCell<dyn Body>>; 1], compliance: Real) -> Self {
        Self {
            bodies,
            lambda: 0.0,
            compliance
        }
    }
}
