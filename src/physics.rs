use crate::constraint::Constraint;
use crate::config::*;
use crate::particle::*;
use crate::particle_constraint::*;
use crate::Body;
use std::borrow::Borrow;
use std::cell::RefCell;
use std::rc::Rc;
use std::vec::Vec;
use crate::particle_constraint::*;
use three_d::*;

type BodyRc = Rc<RefCell<dyn Body>>;

pub struct Physics {
    particles: Vec<Rc<RefCell<dyn Body>>>,
    constraint: Vec<Box<dyn Constraint>>,
    temp_constraint: Vec<Box<dyn Constraint>>,

    gravity: Vecn,

    substeps: usize,
    iterations: usize
}

impl Physics {
    pub fn new() -> Self {
        Self {
            particles: Vec::new(),
            constraint: Vec::new(),
            temp_constraint: Vec::new(),

            gravity: Vecn::new(0.0, -100.0, 0.0),
            
            substeps: SUBS,
            iterations: ITER
        }
    }

    pub fn add_body<T: Body + 'static>(&mut self, body: T) -> Rc<RefCell<T>>{
        let particle = Rc::new(RefCell::new(body));

        self.particles.push(
            particle.clone()
        );

        particle.clone()
    }
    pub fn add_constraint<T: Constraint + 'static>(&mut self, constraint: T) {
        self.constraint.push(Box::new(constraint));
    }
    pub fn add_temp_constraint<T: Constraint + 'static>(&mut self, constraint: T) {
        self.temp_constraint.push(Box::new(constraint));
    }

    pub fn update(&mut self, dt: Real) {
        let dt = dt/(self.substeps as Real);

        for substep in 0..SUBS {
            for body in &self.particles {
                body.as_ref().borrow_mut().add_force(self.gravity);
                body.as_ref().borrow_mut().predict(dt);
            }

            for single in &mut self.constraint {
                single.reset_lambda();
            }
            for single in &mut self.temp_constraint {
                single.reset_lambda();
            } 

            for iteration in 0..ITER {
                for single in &mut self.constraint {
                    single.iterate(dt);
                } 
                for single in &mut self.temp_constraint {
                    single.iterate(dt);
                } 

                for body in &self.particles {
                    body.as_ref().borrow_mut().iterate();
                }
            }

            for body in &self.particles {
                body.as_ref().borrow_mut().update(dt);
            }
         }

        self.temp_constraint.clear();
    }

    pub fn particles(&self) -> Vec<Rc<RefCell<dyn Body>>> {
        self.particles.clone()
    }
}
