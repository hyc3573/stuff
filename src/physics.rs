use crate::constraint::Constraint;
use crate::config::*;
use crate::particle::*;
use crate::particle_constraint::*;
use crate::body::*;
use crate::rigidbody_constraint::RDist;
use crate::Collider;
use std::borrow::Borrow;
use std::cell::RefCell;
use std::rc::Rc;
use std::vec::Vec;
use crate::particle_constraint::*;
use crate::collision::gjk::*;
use three_d::*;
use itertools::Itertools;

type BodyRc = Rc<RefCell<dyn Body>>;

pub struct Physics {
    bodies: Vec<Rc<RefCell<dyn Body>>>,
    colliders: Vec<Box<dyn Collider>>,
    constraint: Vec<Box<dyn Constraint>>,
    temp_constraint: Vec<Box<dyn Constraint>>,

    gravity: Vec3,

    substeps: usize,
    iterations: usize
}

impl Physics {
    pub fn new(gravity: Vec3, substeps: usize, iterations: usize) -> Self {
        Self {
            bodies: Vec::new(),
            colliders: Vec::new(),
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
    pub fn add_collider<T: Collider + 'static>(&mut self, collider: T) {
        self.colliders.push(Box::new(collider) as Box<dyn Collider>);
    }
    pub fn add_constraint<T: Constraint + 'static>(&mut self, constraint: T) {
        self.constraint.push(Box::new(constraint));
    }
    pub fn add_temp_constraint<T: Constraint + 'static>(&mut self, constraint: T) {
        self.temp_constraint.push(Box::new(constraint));
    }

    pub fn update(&mut self, dt: f32) {
        let dt = dt/(self.substeps as f32);

        for substep in 0..self.substeps {
            for body in &self.bodies {
                body.as_ref().borrow_mut().add_force(self.gravity);
                body.as_ref().borrow_mut().predict(dt);
            }

            for i in self.colliders.iter().combinations(2) {
                let a = i[0]; let b = i[1];

                let result = gjk(a, b);
                if let Some(simpl) = result {
                    let (normal, penetration, va, vb) = epa(a, b, simpl);

                    if penetration <= 0.0 {
                        continue;
                    }

                    self.temp_constraint.push(
                        Box::new(
                            RDist::new(
                                [a.get_body(), b.get_body()],
                                [va, vb],
                                0.0,
                                0.1
                            )
                        )
                    )
                }
            }

            for constraint in &mut self.constraint {
                constraint.reset_lambda();
            }
            for constraint in &mut self.temp_constraint {
                constraint.reset_lambda();
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
            self.temp_constraint.clear();
        }
    }

    pub fn bodies(&self) -> Vec<Rc<RefCell<dyn Body>>> {
        self.bodies.clone()
    }
}
