use nannou::text::font::DEFAULT_DIRECTORY_NAME;

use crate::config::*;
use crate::constraint::*;
use crate::particle::Particle;
use crate::body::Body;
use std::borrow::BorrowMut;
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
        (self.parts[0].borrow().pos() - self.parts[1].borrow().pos()).length() - self.dist
    }

    fn dC(&self) -> [Vecn; 2] {
        let n = (self.parts[0].borrow().pos() - self.parts[1].borrow().pos()).normalize();

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
    
    fn iterate(&mut self) {
        let dlambda = self.dlambda();
        self.lambda += dlambda;
        let dx = self.dx(dlambda);

        self.parts[0].as_ref().borrow_mut().update_pos(dx[0]);
        self.parts[1].as_ref().borrow_mut().update_pos(dx[1]);
    }
}
