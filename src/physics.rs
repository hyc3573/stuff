use crate::collision;
use crate::constraint::Constraint;
use crate::config::*;
use crate::particle::*;
use crate::particle_constraint::*;
use crate::body::*;
use crate::rigidbody_constraint::RDist;
use crate::rigidbody_constraint::RColl;
use crate::collision::collider::*;
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

        let mut potential_collisions = Vec::<(usize, usize)>::new();
        for pair in self.colliders.iter().enumerate().combinations(2) {
            let (i, a) = pair[0];
            let (j, b) = pair[1];

            if true {
                potential_collisions.push((i, j));
            }
        }
        let collision_pairs = potential_collisions;

        for _ in 0..self.substeps {
            for body in &self.bodies {
                body.as_ref().borrow_mut().add_force(self.gravity);
                body.as_ref().borrow_mut().predict(dt);
            }

            for (i, j, ..) in collision_pairs.iter() {
                let a = &self.colliders[*i]; let b = &self.colliders[*j];

                let result = gjk(a, b, true);
                if let Some(simpl) = result {
                    let (normal, depth, va, vb) = epa(a, b, simpl);

                    if normal.magnitude2().is_nan() {
                        continue;
                    }

                    if depth <= 0.0 {
                        continue;
                    }

                    self.temp_constraint.push(
                        Box::new(
                            RColl::new(
                                [a.get_body(), b.get_body()],
                                [a.get_body().as_ref().borrow().apos().rotate_vector(va),
                                 b.get_body().as_ref().borrow().apos().rotate_vector(vb)],
                                // [va, vb],
                                normal,
                                depth,
                                0.000
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

            for _ in 0..self.iterations {
                for constraint in &mut self.constraint {
                    constraint.iterate(dt/(self.iterations as f32));
                } 
                for constraint in &mut self.temp_constraint {
                    constraint.iterate(dt/(self.iterations as f32));
                } 

                for body in &self.bodies {
                    body.as_ref().borrow_mut().iterate();
                }
            }

            for body in &self.bodies {
                body.as_ref().borrow_mut().update(dt);
            }

            // Velocity update
            for constraint in &mut self.constraint {
                constraint.velocity_update(dt);
            }
            for constraint in &mut self.temp_constraint {
                constraint.velocity_update(dt);
            }


            // let pos = self.bodies[3].as_ref().borrow().pos();
            // println!("{} {} {}", pos.x, pos.y, pos.z);
            self.temp_constraint.clear();
        }

        for body in self.bodies.iter() {
            body.as_ref().borrow_mut().update_velocity(dt);
        }
    }

    pub fn bodies(&self) -> Vec<Rc<RefCell<dyn Body>>> {
        self.bodies.clone()
    }
}
