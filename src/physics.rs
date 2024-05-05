use crate::constraint::Constraint;
use crate::config::*;
use crate::particle::*;
use crate::particle_constraint::*;
use std::cell::RefCell;
use std::rc::Rc;
use std::vec::Vec;
use crate::particle_constraint::*;
use crate::body::Body;
use three_d::*;

type BodyRc = Rc<RefCell<dyn Body>>;

pub struct Physics {
    bodies: Vec<Rc<RefCell<dyn Body>>>,
    single_const: Vec<Box<dyn Constraint<1>>>,
    double_const: Vec<Box<dyn Constraint<2>>>,
    temp_single_const: Vec<Box<dyn Constraint<1>>>,
    temp_double_const: Vec<Box<dyn Constraint<2>>>,

    gravity: Vecn,

    substeps: usize,
    iterations: usize
}

impl Physics {
    pub fn new() -> Self {
        Self {
            bodies: Vec::new(),
            single_const: Vec::new(),
            double_const: Vec::new(),
            temp_single_const: Vec::new(),
            temp_double_const: Vec::new(),

            gravity: Vecn::new(0.0, -100.0, 0.0),
            
            substeps: SUBS,
            iterations: ITER
        }
    }

    pub fn add_body<T: Body + 'static>(&mut self, body: T) {
        self.bodies.push(
            Rc::new(RefCell::new(body))
        );
    }
    pub fn add_single_constraint<T: Constraint<1> + 'static>(&mut self, constraint: T) {
        self.single_const.push(Box::new(constraint));
    }
    pub fn add_double_constraint<T: Constraint<2> + 'static>(&mut self, constraint: T) {
        self.double_const.push(Box::new(constraint));
    }
    pub fn add_temp_single_constraint<T: Constraint<1> + 'static>(&mut self, constraint: T) {
        self.temp_single_const.push(Box::new(constraint));
    }
    pub fn add_temp_double_constraint<T: Constraint<2> + 'static>(&mut self, constraint: T) {
        self.temp_double_const.push(Box::new(constraint));
    }

    pub fn update(&mut self, dt: Real) {
        let dt = dt/(self.substeps as Real);

        for substep in 0..SUBS {
            for body in &self.bodies {
                body.as_ref().borrow_mut().add_force(self.gravity);
                body.as_ref().borrow_mut().predict(dt);
            }

            for iteration in 0..ITER {
                for single in &mut self.single_const {
                    single.iterate(dt);
                } 
                for double in &mut self.double_const {
                    double.iterate(dt);
                }

                for single in &mut self.temp_single_const {
                    single.iterate(dt);
                } 
                for double in &mut self.temp_double_const {
                    double.iterate(dt);
                }

                for body in &self.bodies {
                    body.as_ref().borrow_mut().iterate();
                }
            }

            for body in &self.bodies {
                body.as_ref().borrow_mut().update(dt);
            }
        }

        self.temp_single_const.clear();
        self.temp_double_const.clear(); 
    }

    pub fn bodies(&self) -> Vec<Rc<RefCell<dyn Body>>> {
        self.bodies.clone()
    }
}
