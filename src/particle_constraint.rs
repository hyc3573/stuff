use nannou::text::font::DEFAULT_DIRECTORY_NAME;

use crate::config::*;
use crate::constraint::*;
use crate::particle::Particle;
use crate::body::Body;
use std::cell::RefCell;
use std::rc::Rc;

pub struct ParticleDist {
    parts: [Rc<RefCell<Particle>>; 2],
    lambda: Real,
    compliance: Real,
    dist: Real
}

impl Constraint<2, Particle> for ParticleDist {
    fn C(&self) -> Real {
        (self.parts()[0].borrow().pos() - self.parts()[1].borrow().pos()).length() - self.dist
    }

    fn dC(&self) -> [Vecn; 2] {
        let n = (self.parts()[0].borrow().pos() - self.parts()[1].borrow().pos()).normalize_or_zero();

        [n, -n]
    }

    fn parts(&self) -> [Rc<RefCell<Particle>>; 2] {
        self.parts.clone()
    }
    fn compliance(&self) -> Real {
        self.compliance
    }
    fn lambda(&self) -> Real {
        self.lambda
    }
    fn update_lambda(&mut self, dlambda: Real) {
        self.lambda += dlambda;
    }
    
    fn iterate(&mut self) {
        let dlambda = self.dlambda();
        self.lambda += dlambda;
        let dx = self.dx(dlambda);

        self.parts()[0].as_ref().borrow_mut().update_pos(dx[0]);
        self.parts()[1].as_ref().borrow_mut().update_pos(dx[1]);
    }
}

impl ParticleDist {
    pub fn new(parts: [Rc<RefCell<Particle>>; 2], dist: Real, compliance: Real) -> Self {
        Self {
            parts,
            lambda: 0.0,
            dist,
            compliance
        }
    }
}

pub struct ParticleFix {
    parts: [Rc<RefCell<Particle>>; 1],
    lambda: Real,
    compliance: Real,
    origin: Vecn
}

impl Constraint<1, Particle> for ParticleFix {
    fn C(&self) -> Real {
        (self.parts()[0].borrow().pos() - self.origin).length()
    }

    fn dC(&self) -> [Vecn; 1] {
        [(self.parts()[0].borrow().pos() - self.origin).normalize_or_zero()]
    }

    fn parts(&self) -> [Rc<RefCell<Particle>>; 1] {
        self.parts.clone()
    }
    fn compliance(&self) -> Real {
        self.compliance
    }
    fn lambda(&self) -> Real {
        self.lambda
    }
    fn update_lambda(&mut self, dlambda: Real) {
        self.lambda += dlambda;
    }
}

impl ParticleFix {
    pub fn new(parts: [Rc<RefCell<Particle>>; 1], origin: Vecn, compliance: Real) -> Self {
        Self {
            parts,
            lambda: 0.0,
            compliance,
            origin
        } 
    }
}
