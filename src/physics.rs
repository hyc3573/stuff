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
    bodies: Vec<Rc<RefCell<dyn Body>>>,
    constraint: Vec<Box<dyn Constraint>>,
    temp_constraint: Vec<Box<dyn Constraint>>,

    gravity: Vecn,

    substeps: usize,
    iterations: usize
}

impl Physics {
    pub fn new(gravity: Vecn, substeps: usize, iterations: usize) -> Self {
        Self {
            bodies: Vec::new(),
            constraint: Vec::new(),
            temp_constraint: Vec::new(),

            gravity,
            
            substeps: 60,
            iterations: 1 
        }
    }

    pub fn add_body<T: Body + 'static>(&mut self, body: T) -> Rc<RefCell<T>>{
        let particle = Rc::new(RefCell::new(body));

        self.bodies.push(
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

        for constraint in &mut self.constraint {
            constraint.reset_lambda();
        }
        for constraint in &mut self.temp_constraint {
            constraint.reset_lambda();
        }

        for substep in 0..self.substeps {
            for body in &self.bodies {
                body.as_ref().borrow_mut().add_force(self.gravity);
                body.as_ref().borrow_mut().predict(dt);
            }

            for iteration in 0..self.iterations {
                for constraint in &mut self.constraint {
                    constraint.iterate(dt);
                } 
                for constraint in &mut self.temp_constraint {
                    constraint.iterate(dt);
                } 

                for body in &self.bodies {
                    body.as_ref().borrow_mut().iterate();
                }
            }

            for body in &self.bodies {
                body.as_ref().borrow_mut().update(dt);
            }

            let pos = self.bodies[2].as_ref().borrow().pos();
            // println!("{} {} {}", pos.x, pos.y, pos.z);
        }

        self.temp_constraint.clear();

        
    }

    pub fn bodies(&self) -> Vec<Rc<RefCell<dyn Body>>> {
        self.bodies.clone()
    }
}
