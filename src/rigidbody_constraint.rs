use crate::body::*;
use crate::config::*;
use crate::constraint::*;
use std::cell::RefCell;
use std::iter::zip;
use std::rc::Rc;
use three_d::*;

pub struct RDist {
    bodies: [Rc<RefCell<dyn Body>>; 2],
    offsets: [Vec3; 2],
    lambda: f32,
    compliance: f32,
    dist: f32,
}

impl RDist {
    fn points(&self) -> [Vec3; 2] {
        [0, 1].map(|i| -> Vec3 {
            self.bodies()[i]
                .as_ref()
                .borrow()
                .to_global(self.offsets[i])
        })
    }

    fn true_offsets(&self) -> [Vec3; 2] {
        [0, 1].map(|i| -> Vec3 {
            self.bodies()[i]
                .as_ref()
                .borrow()
                .apos()
                .rotate_vector(self.offsets[i])
        })
    }

    fn local_normals(&self) -> [Vec3; 2] {
        [ 
            self.bodies()[0].borrow().apos().invert().rotate_vector(self.dC()[0]),
            self.bodies()[1].borrow().apos().invert().rotate_vector(-self.dC()[1]),
        ]
    }

    pub fn new(
        bodies: [Rc<RefCell<dyn Body>>; 2],
        offsets: [Vec3; 2],
        dist: f32,
        compliance: f32,
    ) -> Self {
        Self {
            bodies,
            offsets,
            lambda: 0.0,
            compliance,
            dist,
        }
    }
}

impl Constraint for RDist {
    constraint_getset!(2);

    fn C(&self) -> f32 {
        let pos = self.points();

        let result = (pos[0] - pos[1]).magnitude() - self.dist;
        result
    }

    fn dC(&self) -> Vec<Vec3> {
        let pos = self.points();
        let mut n = pos[0] - pos[1];

        if n.magnitude2() > f32::EPSILON * f32::EPSILON {
            n = n.normalize()
        }

        vec![n, -n]
    }

    fn dq(&self, dlambda: f32) -> Vec<Quat> {
        let mut result = Vec::<Quat>::new();
        for i in 0..self.len() {
            result.push(
                0.5 * Quat::from_sv(
                    0.0,
                    self.bodies()[i].as_ref().borrow().invinertia()
                        * self.offsets[i].cross(self.local_normals()[i])
                        * dlambda,
                ) * self.bodies()[i].as_ref().borrow().apos(),
            )
        }

        // result[1] *= -1.0;

        result
        // vec![Quat::zero(), Quat::zero()]
    }

    fn invmass_sum(&self) -> f32 {
        let pos = self.points();
        let mut sum: f32 = 0.0;

        for i in 0..self.len() {
            sum += self.bodies()[i].as_ref().borrow().invmass()
                + self.offsets[i].cross(self.local_normals()[i]).dot(
                    self.bodies()[i].as_ref().borrow().invinertia()
                        * self.offsets[i].cross(self.local_normals()[i]),
                );
        }

        sum
    }
}

pub struct RColl {
    bodies: [Rc<RefCell<dyn Body>>; 2],
    original_velocity: Vec<Vec3>,
    contacts: Vec<(Vec3, Vec3)>,
    normal: Vec3,
    depth: Vec<f32>,
    lambda: Vec<f32>,
    compliance: f32,
}

impl RColl {
    pub fn new(
        bodies: [Rc<RefCell<dyn Body>>; 2],
        contacts: Vec<(Vec3, Vec3)>,
        normal: Vec3,
        depth: Vec<f32>,
        compliance: f32,
    ) -> Self {
        let len = contacts.len();
        let mut result = Self {
            bodies,
            original_velocity: vec![Vec3::zero(); len],
            contacts, // local coordinate
            normal,
            depth,
            lambda: vec![0.0; len],
            compliance,
        };

        let r = result.r();

        for i in 0..result.contacts.len() {
            result.original_velocity[i] = result.bodies[0].as_ref().borrow().vel()
                + result.bodies[0].as_ref().borrow().avel().cross(r[i].0)
                - (result.bodies[1].as_ref().borrow().vel()
                   + result.bodies[1].as_ref().borrow().avel().cross(r[i].1));
        }

        result
    }

    fn r(&self) -> Vec<(Vec3, Vec3)> {
        (0..self.contacts.len())
            .map(|i| {
                (
                    // self.bodies[0].as_ref().borrow().apos().rotate_vector(self.contacts[i].0),
                    // self.bodies[1].as_ref().borrow().apos().rotate_vector(self.contacts[i].1),
                    self.contacts[i].0,
                    self.contacts[i].1,
                )
            })
            .collect()
    }

    fn dq_vec(&self, dlambda: f32, i: usize) -> (Quat, Quat) {
        // println!("{:?}", r.0.cross(self.dC()[0] * dlambda));
        let r = self.r();
        (
            0.5 * Quat::from_sv(
                0.0,
                self.bodies()[0].as_ref().borrow().invinertia()
                    * r[i].0.cross(
                        self.local_normals()[0] * dlambda
                    ),
            ) * self.bodies()[0].as_ref().borrow().apos(),
            0.5 * Quat::from_sv(
                0.0,
                self.bodies()[1].as_ref().borrow().invinertia()
                    * r[i].1.cross(
                        self.local_normals()[1] * dlambda
                    ),
            ) * self.bodies()[1].as_ref().borrow().apos(),
        )

        // result[0] = result[0].invert();
        // result[1] = result[1].invert();
        // println!("{} {} {} {} / {} {} {} {}", result[0].v.x, result[0].v.y, result[0].v.z, result[0].s, result[1].v.x, result[1].v.y, result[1].v.z, result[1].s);
        // vec![Quat::zero(), Quat::zero()]
    }

    fn invmass_sum_vec(&self) -> Vec<f32> {
        let r = self.r();

        (0..self.contacts.len())
            .map(|i| {
                let mut sum: f32 = 0.0;
                sum += self.bodies()[0].as_ref().borrow().invmass();
                sum += self.bodies()[1].as_ref().borrow().invmass();

                sum += r[i].0.cross(self.local_normals()[0]).dot(
                    self.bodies()[0].as_ref().borrow().invinertia() * r[i].0.cross(self.local_normals()[0]),
                );

                sum += r[i].1.cross(self.local_normals()[1]).dot(
                    self.bodies()[1].as_ref().borrow().invinertia() * r[i].1.cross(self.local_normals()[1]),
                );

                sum
            })
            .collect()
    }

    fn local_normals(&self) -> [Vec3; 2] {
        [ 
            self.bodies()[0].borrow().apos().invert().rotate_vector(self.dC()[0]),
            self.bodies()[1].borrow().apos().invert().rotate_vector(-self.dC()[1]),
        ]
    }

    fn C_vec(&self) -> &Vec<f32> {
        &self.depth
    }

    fn dlambda_vec(&self, dt: f32) -> Vec<f32> {
        (0..self.contacts.len())
            .map(|i| {
                let alpha = self.compliance() / (dt * dt);
                let C = self.C_vec()[i];

                let dot = self.invmass_sum_vec()[i];
                let denom = dot + alpha;

                if denom.abs() < f32::EPSILON {
                    return 0.0;
                }

                (-C - alpha * self.lambda()) / (denom)
            })
            .collect()
    }
}

impl Constraint for RColl {
    fn bodies(&self) -> Vec<Rc<RefCell<dyn Body>>> {
        self.bodies.clone().to_vec()
    }
    fn compliance(&self) -> f32 {
        self.compliance
    }
    fn lambda(&self) -> f32 {
        0.0
        // self.lambda
    }
    fn reset_lambda(&mut self) {
        for i in 0..self.contacts.len() {
            self.lambda[i] = 0.0;
        }
    }
    fn update_lambda(&mut self, dlambda: f32) {
        // self.lambda += dlambda;
    }
    fn len(&self) -> usize {
        2
    }

    fn C(&self) -> f32 {
        0.0
    }

    fn dC(&self) -> Vec<Vec3> {
        vec![self.normal, -self.normal]
    }

    fn dx(&self, dlambda: f32) -> Vec<Vec3> {
        let dC = self.dC();

        let mut result = vec![Vec3::zero(); self.len()];
        for j in 0..self.contacts.len() {
            for i in 0..self.len() {
                result[i] += self.bodies()[i].borrow().invmass() * dC[i] * dlambda;
            }
        }
        result
    }

    fn iterate(&mut self, dt: f32) {
        // let mut dx_vec = vec![vec![Vec3::zero(), Vec3::zero()]; self.contacts.len()];
        let mut dq_vec = vec![(Quat::zero(), Quat::zero()); self.contacts.len()];
        let mut dlambda_vec = vec![0.0; self.contacts.len()];

        for i in 0..self.contacts.len() {
            let dlambda = self.dlambda_vec(dt)[i];
            // dx_vec[i] = self.dx(dlambda);
            dq_vec[i] = self.dq_vec(dlambda, i);
            dlambda_vec[i] = dlambda;
        }
        // self.update_lambda(dlambda);

        for i in 0..self.contacts.len() {
            self.lambda[i] += dlambda_vec[i];

            let w0 = self.bodies()[0].as_ref().borrow().invmass();
            let w1 = self.bodies()[1].as_ref().borrow().invmass();

            self.bodies()[0]
                .as_ref()
                .borrow_mut()
                .update_pos(dlambda_vec[i]*self.dC()[0]*w0/(self.contacts.len() as f32));
            self.bodies()[1]
                .as_ref()
                .borrow_mut()
                .update_pos(dlambda_vec[i]*self.dC()[1]*w1/(self.contacts.len() as f32));
            /* if dq_vec[i].0.magnitude2() < f32::EPSILON &&
                dq_vec[i].1.magnitude2() < f32::EPSILON {
                    continue;
                } */
            // println!("{i} 0 {:#?}", dq_vec[i].0);
            // println!("{i} 1 {:#?}", dq_vec[i].1);
            
            self.bodies()[0].as_ref().borrow_mut().add_apos(dq_vec[i].0);
            self.bodies()[1].as_ref().borrow_mut().add_apos(dq_vec[i].1);
        }
    }

    fn velocity_update(&mut self, dt: f32) {
        // println!("------");

        let normal = self.normal;
        let mut dv_vec = vec![Vec3::zero(); self.contacts.len()];

        let r = self.r();

        let iter = 1;
        for n in 0..iter {
            for i in 0..self.contacts.len() {
                let v = (self.bodies[0].as_ref().borrow().vel()
                         + self.bodies[0].as_ref().borrow().avel().cross(r[i].0))
                    - (self.bodies[1].as_ref().borrow().vel()
                       + self.bodies[1].as_ref().borrow().avel().cross(r[i].1));
                // println!("{:?}", v);

                let v_normal = normal.dot(v);
                println!("b{v_normal}");
                let v_tangential = v - normal * v_normal;

                let v_normal_original = normal.dot(self.original_velocity[i]);
                let e = 0.0;
                dv_vec[i] += normal * (-v_normal + f32::min(-e * v_normal_original, 0.0));

                let v_tangential_abs = v_tangential.magnitude();
                let u = 0.0;
                if v_tangential_abs > f32::EPSILON {
                    dv_vec[i] -= v_tangential / v_tangential_abs
                        * f32::min(u * self.lambda[i].abs() / dt / dt, v_tangential_abs);
                } else {
                }

                // let p = dv_vec[j] / self.invmass_sum_vec()[j] / (self.contacts.len() as f32);
                // let p = dv_vec[i] / self.invmass_sum_vec().iter().sum();
                let p = dv_vec[i] / self.invmass_sum_vec()[i];
                // println!("{j}");
                // println!("{}", self.bodies[0].as_ref().borrow().invmass());
                // println!("{:?}", p);

                // println!("{j} {}", self.invmass_sum_vec()[j]);

                for (j, body) in self.bodies.iter().enumerate() {
                    let new_vel = body.as_ref().borrow().vel()
                        + p * body.as_ref().borrow().invmass() * (if j == 0 { 1.0 } else { -1.0 }); // (self.contacts.len() as f32);
                    body.as_ref().borrow_mut().set_vel(new_vel);
                    // body.as_ref().borrow_mut().set_vel(Vec3::zero());

                    let new_avel = body.as_ref().borrow().avel()
                        + body.as_ref().borrow().invinertia()
                        * ((if j == 0 { r[i].0 } else { r[i].1 }).cross(p))
                        * (if j == 0 { 1.0 } else { 1.0 });
                    body.as_ref().borrow_mut().set_avel(new_avel);
                    // body.as_ref().borrow_mut().set_avel(Vec3::zero());
                }

                let v = (self.bodies[0].as_ref().borrow().vel()
                         + self.bodies[0].as_ref().borrow().avel().cross(r[i].0))
                    - (self.bodies[1].as_ref().borrow().vel()
                       + self.bodies[1].as_ref().borrow().avel().cross(r[i].1));
                // println!("{:?}", v);

                let v_normal = normal.dot(v);
                println!("a{v_normal}");
            }
        }

        // let dv: Vec3 = dv_vec.iter().sum::<Vector3<f32>>()/(self.contacts.len() as f32);

        for i in 0..self.contacts.len() {
            
        }

        // let vel = self.bodies[0].as_ref().borrow().vel();
        // self.bodies[0].as_ref().borrow_mut().set_vel(vel - normal*vel.dot(normal));

        // let v_old = v;
        // println!("{}", normal.dot(v));
        // println!("{}", normal.dot(self.bodies[0].as_ref().borrow().vel()));
        // println!("{}", normal.dot(self.bodies[1].as_ref().borrow().vel()));
    }
}
