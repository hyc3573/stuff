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
use std::borrow::BorrowMut;
use std::cell::RefCell;
use std::rc::Rc;
use std::vec::Vec;
use crate::particle_constraint::*;
use crate::collision::gjk::*;
use three_d::*;
use itertools::Itertools;
use crate::timestep_schedule::*;
use std::collections::VecDeque;

type BodyRc = Rc<RefCell<dyn Body>>;

pub struct Physics {
    bodies: Vec<Rc<RefCell<dyn Body>>>,
    colliders: Vec<Box<dyn Collider>>,
    constraint: Vec<Box<dyn Constraint>>,
    temp_constraint: Vec<Box<dyn Constraint>>,

    gravity: Vec3,

    substeps: usize,
    iterations: usize,

    scheduler: Box<dyn TimestepScheduler>
}

impl Physics {
    pub fn new(gravity: Vec3, substeps: usize, iterations: usize) -> Self {
        Self {
            bodies: Vec::new(),
            colliders: Vec::new(),
            constraint: Vec::new(),
            temp_constraint: Vec::new(),

            gravity,
            
            substeps,
            iterations,

            scheduler: Box::new(UniformSchedule::new(substeps)),
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
        let mut potential_collisions = Vec::<(usize, usize, RefCell<Vec<(Vec3, Vec3)>>)>::new();
        for pair in self.colliders.iter().enumerate().combinations(2) {
            let (i, a) = pair[0];
            let (j, b) = pair[1];

            if true {
                potential_collisions.push((i, j, RefCell::new(Vec::new())));
            }
        }
        let collision_pairs = potential_collisions;
        
        for substep in 0..self.substeps {
            let dt = self.scheduler.get(substep, dt);
            // self.bodies[3].as_ref().borrow_mut().add_force(-self.gravity);
            for body in &self.bodies {
                let invmass = body.as_ref().borrow().invmass();
                if invmass > 0.0 {
                    body.as_ref().borrow_mut().add_force(self.gravity/invmass);
                }
                body.as_ref().borrow_mut().predict(dt);
            }

            for (pair_index, (i, j, points)) in collision_pairs.iter().enumerate() {
                let a = &self.colliders[*i]; let b = &self.colliders[*j];

                let result = gjk(a, b, true);
                if let Some(simpl) = result {
                    let (normal, depth, va, vb) = epa(a, b, simpl);
                    // println!("{:?}", normal);
                    // let depth = depth.abs();

                    if normal.magnitude2().is_nan() || depth < 0.0 {
                        continue;
                    }

                    if points.borrow().len() < 4 || true {
                        points.borrow_mut().push((va, vb));
                    }

                    // if collision_normals[pair_index].is_none() {
                    //     collision_normals[pair_index] = Some(normal);
                    // }
                    // let normal = collision_normals[pair_index].unwrap();

                    // let posa = a.get_body().as_ref().borrow().pos();
                    // let posb = b.get_body().as_ref().borrow().pos();
                    // let dx = posa - posb;
                    // let dist = dx.magnitude();
                    // let normal = dx/dist;
                    // let depth = f32::max(0.0, (posa - posb).dot(normal));

                    // println!("------");
                    // println!("{:?} {:?}", a.get_body().as_ref().borrow().pos(), a.get_body().as_ref().borrow().apos());
                    // println!("{}", (a.get_body().as_ref().borrow().pos_at(va) - b.get_body().as_ref().borrow().pos_at(vb)).magnitude());

                    for (va, vb) in points.borrow()[isize::max(0, points.borrow().len() as isize - 4) as usize..].iter() {
                        self.temp_constraint.push(
                            Box::new(
                                RColl::new(
                                    [a.get_body(), b.get_body()],
                                    [a.get_body().as_ref().borrow().apos().rotate_vector(*va), b.get_body().as_ref().borrow().apos().rotate_vector(*va)],
                                    // [a.get_body().as_ref().borrow().pos_at(va), b.get_body().as_ref().borrow().pos_at(vb)],
                                    // [*va, *vb],
                                    // [Vec3::zero(), Vec3::zero()],
                                    // collision_normals[pair_index].unwrap(),
                                    normal,
                                    depth/1.0,
                                    // actual_normal,
                                    // actual_depth,
                                    0.000
                                )
                            )
                        )    
                    }
                }
            }

            for constraint in &mut self.constraint {
                constraint.reset_lambda();
            }
            for constraint in &mut self.temp_constraint {
                constraint.reset_lambda();
            }

            for _ in 0..self.iterations {
                for constraint in &mut self.temp_constraint {
                    constraint.iterate(dt/(self.iterations as f32));
                }
                for constraint in &mut self.constraint {
                    constraint.iterate(dt/(self.iterations as f32));
                } 

                for body in &self.bodies {
                    body.as_ref().borrow_mut().iterate();
                }
            }

            for body in &self.bodies {
                body.as_ref().borrow_mut().update(dt);
            }
            // println!("{:?}", self.bodies[3].as_ref().borrow().vel());

            for body in self.bodies.iter() {
                body.as_ref().borrow_mut().update_velocity(dt);
            }

            // Velocity update
            for constraint in &mut self.temp_constraint {
                constraint.velocity_update(dt);
            }
            for constraint in &mut self.constraint {
                constraint.velocity_update(dt);
            }

            // let pos = self.bodies[3].as_ref().borrow().pos();
            // println!("{} {} {}", pos.x, pos.y, pos.z);
            self.temp_constraint.clear();
        }
    }

    pub fn bodies(&self) -> Vec<Rc<RefCell<dyn Body>>> {
        self.bodies.clone()
    }
}
