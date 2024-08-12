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
                        * self.true_offsets()[i].cross(self.dC()[i])
                        * dlambda,
                ) * self.bodies()[i].as_ref().borrow().apos(),
            )
        }

        result[0] *= -1.0;

        result
        // vec![Quat::zero(), Quat::zero()]
    }

    fn invmass_sum(&self) -> f32 {
        let dC = self.dC();
        let pos = self.points();
        let mut sum: f32 = 0.0;

        for i in 0..self.len() {
            sum += self.bodies()[i].as_ref().borrow().invmass()
                + self.true_offsets()[i].cross(dC[0]).dot(
                    self.bodies()[i].as_ref().borrow().invinertia()
                        * self.true_offsets()[i].cross(dC[0]),
                );
        }

        sum
    }
}

pub struct RColl {
    bodies: [Rc<RefCell<dyn Body>>; 2],
    original_velocity: Vec<Vec3>,
    contacts: Vec<Vec3>,
    normal: Vec3,
    depth: f32,
    lambda: f32,
    compliance: f32,
}

impl RColl {
    pub fn new(
        bodies: [Rc<RefCell<dyn Body>>; 2],
        contacts: Vec<Vec3>,
        normal: Vec3,
        depth: f32,
        compliance: f32,
    ) -> Self {
        let mut result = Self {
            bodies,
            original_velocity: Vec::new(),
            contacts, // global coordinate
            normal,
            depth,
            lambda: 0.0,
            compliance,
        };

        let original_velocity = result
            .r()
            .iter()
            .map(|r| {
                (result.bodies[0].as_ref().borrow().vel()
                    + result.bodies[0].as_ref().borrow().avel().cross(r.0))
                    - (result.bodies[1].as_ref().borrow().vel()
                        + result.bodies[1].as_ref().borrow().avel().cross(r.1))
            })
            .collect();

        result.original_velocity = original_velocity;

        result
    }

    fn r(&self) -> Vec<(Vec3, Vec3)> {
        self.contacts
            .iter()
            .enumerate()
            .map(|(i, c)| {
                // self.bodies[i].as_ref().borrow().apos().rotate_vector(self.bodies[i].as_ref().borrow().to_local(*c))
                (
                    self.bodies[0].as_ref().borrow().apos().rotate_vector(self.bodies[0].as_ref().borrow().to_local(*c)),
                    self.bodies[0].as_ref().borrow().apos().rotate_vector(self.bodies[1].as_ref().borrow().to_local(*c)),
                )
            })
            .collect::<Vec<(Vec3, Vec3)>>()
    }
}

impl Constraint for RColl {
    constraint_getset!(2);

    fn C(&self) -> f32 {
        self.depth * 1.0
    }

    fn dC(&self) -> Vec<Vec3> {
        vec![self.normal, -self.normal]
    }

    fn dq(&self, dlambda: f32) -> Vec<Quat> {
        let mut result = vec![Quat::zero(), Quat::zero()];
        for r in self.r() {
            // println!("{:?}", r.0.cross(self.dC()[0] * dlambda));
            result[0] +=
                (0.5 * Quat::from_sv(
                    0.0,
                    self.bodies()[0].as_ref().borrow().invinertia()
                        * r.0.cross(self.dC()[0] * dlambda),
                )) * self.bodies()[0].as_ref().borrow().apos();
            result[1] +=
                (0.5 * Quat::from_sv(
                    0.0,
                    self.bodies()[1].as_ref().borrow().invinertia()
                        * r.1.cross(self.dC()[1] * dlambda),
                )) * self.bodies()[1].as_ref().borrow().apos();
        }

        // result[0] = result[0].invert();
        // result[1] = result[1].invert();
        // println!("{} {} {} {} / {} {} {} {}", result[0].v.x, result[0].v.y, result[0].v.z, result[0].s, result[1].v.x, result[1].v.y, result[1].v.z, result[1].s);

        result
        // vec![Quat::zero(), Quat::zero()]
    }

    fn invmass_sum(&self) -> f32 {
        let mut sum: f32 = 0.0;

        for i in 0..self.len() {
            for r in self.r() {
                sum += self.bodies()[i].as_ref().borrow().invmass()
                    + r.0.cross(self.normal).dot(
                        self.bodies()[i].as_ref().borrow().invinertia() * r.0.cross(self.normal),
                    );

                sum += self.bodies()[i].as_ref().borrow().invmass()
                    + r.1.cross(self.normal).dot(
                        self.bodies()[i].as_ref().borrow().invinertia() * r.1.cross(self.normal),
                    );
            }
        }

        sum
    }

    fn velocity_update(&mut self, dt: f32) {
        // println!("------");

        let normal = self.normal;
        let mut dv = Vec3::zero();

        for (i, r) in self.r().iter().enumerate() {
            let v = (self.bodies[0].as_ref().borrow().vel()
                + self.bodies[0].as_ref().borrow().avel().cross(r.0))
                - (self.bodies[1].as_ref().borrow().vel()
                    + self.bodies[1].as_ref().borrow().avel().cross(r.1));
            // println!("{:?}", v);

            let v_normal = normal.dot(v);
            // println!("{}", v_normal);
            let v_tangential = v - normal * v_normal;


            if v_normal.abs() > 0.0 {
                let v_normal_original = normal.dot(self.original_velocity[i]);
                // println!("{}", v_normal_original);
                let e = 0.9;
                dv += normal * (-v_normal + f32::min(-e * v_normal_original, 0.0));
            } else {
                dv += -normal * v_normal;
            }

            let v_tangential_abs = v_tangential.magnitude();
            let u = 0.0;
            if v_tangential_abs > f32::EPSILON {
                dv -= v_tangential / v_tangential_abs
                    * f32::min(u * self.lambda.abs() / dt, v_tangential_abs);
            }
            // println!("{:?}", dv);
        }

        let p = dv / self.invmass_sum();

        for (i, r) in self.r().iter().enumerate() {
            for (i, body) in self.bodies.iter().enumerate() {
                let new_vel = body.as_ref().borrow().vel()
                    + p * body.as_ref().borrow().invmass() * (if i == 0 { 1.0 } else { -1.0 });
                body.as_ref().borrow_mut().set_vel(new_vel);

                let new_avel = body.as_ref().borrow().avel()
                    + body.as_ref().borrow().invinertia()
                    * ((if i == 0 { r.0 } else { r.1 }).cross(p))
                    * (if i == 0 { 1.0 } else { -1.0 });
                body.as_ref().borrow_mut().set_avel(new_avel);
            }    
        }
        
        // let vel = self.bodies[0].as_ref().borrow().vel();
        // self.bodies[0].as_ref().borrow_mut().set_vel(vel - normal*vel.dot(normal));

        // let v_old = v;
        // println!("{}", normal.dot(v));
        // println!("{}", normal.dot(self.bodies[0].as_ref().borrow().vel()));
        // println!("{}", normal.dot(self.bodies[1].as_ref().borrow().vel()));
    }
}
