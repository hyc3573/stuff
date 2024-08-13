use crate::body::*;
use crate::collision;
use crate::collision::clipping;
use crate::collision::clipping::*;
use crate::collision::collider::*;
use crate::collision::gjk::*;
use crate::config::*;
use crate::constraint::Constraint;
use crate::particle::*;
use crate::particle_constraint::*;
use crate::particle_constraint::*;
use crate::rigidbody_constraint::RColl;
use crate::rigidbody_constraint::RDist;
use crate::timestep_schedule::*;
use ::core::f32;
use itertools::Itertools;
use std::borrow::Borrow;
use std::borrow::BorrowMut;
use std::cell::RefCell;
use std::collections::VecDeque;
use std::rc::Rc;
use std::vec::Vec;
use three_d::*;

type BodyRc = Rc<RefCell<dyn Body>>;

pub struct Physics {
    bodies: Vec<Rc<RefCell<dyn Body>>>,
    colliders: Vec<Box<dyn Collider>>,
    constraint: Vec<Box<dyn Constraint>>,
    temp_constraint: Vec<Box<dyn Constraint>>,

    gravity: Vec3,

    substeps: usize,
    iterations: usize,

    scheduler: Box<dyn TimestepScheduler>,
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

    pub fn add_body<T: Body + 'static>(&mut self, body: T) -> Rc<RefCell<T>> {
        let particle = Rc::new(RefCell::new(body));

        self.bodies.push(particle.clone());

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
        let mut potential_collisions = Vec::<(usize, usize)>::new();
        for pair in self.colliders.iter().enumerate().combinations(2) {
            let (i, a) = pair[0];
            let (j, b) = pair[1];

            if true {
                potential_collisions.push((i, j));
            }
        }
        let collision_pairs = potential_collisions;

        for substep in 0..self.substeps {
            let dt = self.scheduler.get(substep, dt);
            // self.bodies[3].as_ref().borrow_mut().add_force(-self.gravity);
            for body in &self.bodies {
                let invmass = body.as_ref().borrow().invmass();
                if invmass > 0.0 {
                    body.as_ref().borrow_mut().add_force(self.gravity / invmass);
                }
                body.as_ref().borrow_mut().predict(dt);
            }

            for (pair_index, (i, j)) in collision_pairs.iter().enumerate() {
                let a = &self.colliders[*i];
                let b = &self.colliders[*j];

                let result = gjk(a, b, true);
                if let Some(simpl) = result {
                    let (normal, depth, va, vb) = epa(a, b, simpl);
                    // println!("{:?}", normal);
                    // let depth = depth.abs();

                    if normal.magnitude2().is_nan() || depth < 0.0 {
                        continue;
                    }

                    let a_vert = a.support(normal);
                    let a_vertices = a.get_vertices().unwrap();
                    let a_faces = a.get_faces().unwrap();
                    let a_ind = a_vertices
                        .iter()
                        .position(|v| v.distance2(a_vert.to_vec()) < f32::EPSILON * f32::EPSILON)
                        .unwrap();

                    let b_vert = b.support(-normal);
                    let b_vertices = b.get_vertices().unwrap();
                    let b_faces = b.get_faces().unwrap();
                    let b_ind = b_vertices
                        .iter()
                        .position(|v| v.distance2(b_vert.to_vec()) < f32::EPSILON * f32::EPSILON)
                        .unwrap();

                    let mut max_dot = -f32::INFINITY;
                    let mut a_ref: &Vec<usize> = &a_faces[0];
                    for face in a_faces {
                        if face.contains(&a_ind) {
                            let dot = ccw_normal(&face, &a_vertices).dot(-normal);
                            if max_dot < dot {
                                a_ref = face;
                                max_dot = dot;
                            }
                        }
                    }

                    max_dot = -f32::INFINITY;
                    let mut b_ref: &Vec<usize> = &b_faces[0];
                    for face in b_faces {
                        if face.contains(&b_ind) {
                            let dot = ccw_normal(&face, &b_vertices).dot(normal);
                            if max_dot < dot {
                                b_ref = face;
                                max_dot = dot;
                            }
                        }
                    }

                    let a_ref = a_ref;
                    let b_ref = b_ref;

                    let a_face: Vec<Vec3> = a_ref.iter().map(|i| a_vertices[*i]).collect();
                    let b_face: Vec<Vec3> = b_ref.iter().map(|i| b_vertices[*i]).collect();

                    let a_normal = (a_face[1] - a_face[0]).cross(a_face[2] - a_face[0]);
                    let b_normal = (b_face[1] - b_face[0]).cross(b_face[2] - b_face[0]);

                    let inc_face;
                    let ref_face;
                    let mut adj_face_normals = Vec::new();

                    let mut a_is_ref;
                    if a_normal.normalize().dot(normal) < b_normal.normalize().dot(-normal) {
                        inc_face = b_face;
                        ref_face = a_face;
                        a_is_ref = true;

                        for (edge_a, edge_b) in
                            a_ref.iter().chain([a_ref[0]].iter()).tuple_windows()
                        {
                            for face in a_faces {
                                if face == a_ref {
                                    continue;
                                }
                                if face.contains(edge_a) && face.contains(edge_b) {
                                    adj_face_normals
                                        .push((a_vertices[*edge_a], ccw_normal(face, &a_vertices)))
                                }
                            }
                        }
                    } else {
                        inc_face = a_face;
                        ref_face = b_face;
                        a_is_ref = false;

                        for (edge_a, edge_b) in
                            b_ref.iter().chain([b_ref[0]].iter()).tuple_windows()
                        {
                            for face in b_faces {
                                if face == b_ref {
                                    continue;
                                }
                                if face.contains(edge_a) && face.contains(edge_b) {
                                    adj_face_normals
                                        .push((b_vertices[*edge_a], ccw_normal(face, &b_vertices)))
                                }
                            }
                        }
                    }
                    let adj_face_normals = adj_face_normals;
                    
                    let mut result = inc_face.clone();
                    // println!("-----");
                    // println!("{:?}", (ref_face[1] - ref_face[0]).cross(ref_face[2] - ref_face[0]));
                    // println!("{}", result.len());
                    for (origin, clip_normal) in adj_face_normals {
                        // println!("{:?} {:?}", origin, clip_normal);
                        result = clip(&result, &origin, &(clip_normal), true);
                        // println!("{}", result.len());
                    }
                    result = clip(
                        &result,
                        &ref_face[0],
                        &(-(ref_face[0] - ref_face[1]).cross(ref_face[2] - ref_face[0])),
                        false,
                    );

                    for contact_inc in result {
                        // Project contact to incident face

                        let closest_point = ref_face.iter().fold(
                            (-f32::INFINITY, Vec3::zero()),
                            |(max, point_max), v| {
                                let value = v.distance2(contact_inc);
                                if max > value {
                                    (max, point_max)
                                } else {
                                    (value, *v)
                                }
                            }
                        ).1;
                        let point_diff = contact_inc - closest_point;
                        let contact_peneration;

                        let contacts;
                        if !a_is_ref {
                            contact_peneration = point_diff.dot(normal);
                            contacts = [
                                contact_inc - contact_peneration*normal,
                                contact_inc
                            ];
                        } else {
                            contact_peneration = point_diff.dot(-normal);
                            contacts = [
                                contact_inc,
                                contact_inc + contact_peneration*normal
                            ];
                        }
                        println!("{:?}", contacts);
                        println!("{contact_peneration}");

                        self.temp_constraint.push(Box::new(RColl::new(
                            [a.get_body(), b.get_body()],
                            // [a.get_body().as_ref().borrow().apos().rotate_vector(*va), b.get_body().as_ref().borrow().apos().rotate_vector(*va)],
                            // [a.get_body().as_ref().borrow().pos_at(va), b.get_body().as_ref().borrow().pos_at(vb)],
                            contacts,
                            // [Vec3::zero(), Vec3::zero()],
                            // collision_normals[pair_index].unwrap(),
                            normal,
                            contact_peneration,
                            // actual_normal,
                            // actual_depth,
                            0.000001,
                        )))
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
                    constraint.iterate(dt / (self.iterations as f32));
                    // println!("{:?}", self.bodies[2].as_ref().borrow().apos());
                }
                for constraint in &mut self.constraint {
                    constraint.iterate(dt / (self.iterations as f32));
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
