use crate::config::*;
use three_d::Mat3;
use three_d::Quat;

macro_rules! body_common {
    {} => {
        fn invmass(&self) -> Real {self.invmass}
        fn pos(&self) -> Vecn {self.pos}
        fn vel(&self) -> Vecn {self.vel}
        fn acc(&self) -> Vecn {self.acc}

        fn update_pos(&mut self, dx: Vecn) {
            self.pos_new += dx;
        }
        fn add_force(&mut self, f: Vecn) {
            self.acc += self.invmass*f;
        }
    }
}

macro_rules! rigidbody_common {
    {} => {
        fn inertia(&self) -> Mat3 {
            self.inertia
        }

        fn invinertia(&self) -> Mat3 {
            self.invinertia
        }

        fn apos(&self) -> Quat {
            self.apos
        }

        fn avel(&self) -> Vecn {
            self.avel
        }

        fn aacc(&self) -> Vecn {
            self.aacc
        }
    }
}

pub trait Body {
    fn invmass(&self) -> Real; 

    fn pos(&self) -> Vecn;
    fn vel(&self) -> Vecn;
    fn acc(&self) -> Vecn;

    fn update_pos(&mut self, dx: Vecn);
    fn add_force(&mut self, f: Vecn);

    fn predict(&mut self, dt: Real);
    fn update(&mut self, dt: Real);
    fn iterate(&mut self);
}

pub trait RigidBody: Body {
    fn inertia(&self) -> Mat3;
    fn invinertia(&self) -> Mat3;

    fn apos(&self) -> Quat;
    fn avel(&self) -> Vecn;
    fn aacc(&self) -> Vecn;

    fn update_apos(&mut self, dq: Quat);
    fn add_force_at(&mut self, f: Vecn, at: Vecn);
    fn add_torque(&mut self, t: Vecn);
}
