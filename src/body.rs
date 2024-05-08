use crate::config::*;
use three_d::Mat3;
use three_d::Quat;
use three_d::Rotation;
use three_d::*;

macro_rules! body_common {
    {} => {
        fn invmass(&self) -> Real {self.invmass}
        fn pos(&self) -> Vecn {self.pos_new}
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
            self.apos_new
        }

        fn avel(&self) -> Vecn {
            self.avel
        }

        fn aacc(&self) -> Vecn {
            self.aacc
        }

        fn predict(&mut self, dt: Real) {
            self.pos_prev = self.pos;
            self.vel += self.acc*dt;
            self.pos += self.vel*dt;
            self.pos_new = self.pos;
            self.pos_pred = self.pos;

            self.apos_prev = self.apos;
            self.avel += self.aacc*dt - dt*self.invinertia*self.avel.cross(self.inertia*self.avel);
            self.apos = self.apos + Quat::new(0.0, self.avel.x, self.avel.y, self.avel.z)*self.apos*0.5*dt;
            self.apos = self.apos.normalize();
            self.apos_new = self.apos;
            self.apos_pred = self.apos;
        }
        fn update(&mut self, dt: Real) {
            self.acc = Vecn::zero();
            self.vel = (2.0*self.pos - self.pos_prev - self.pos_pred)/dt;
            // self.vel = (self.pos - self.pos_prev)/dt;

            self.aacc = Vecn::zero();
            let dq = self.apos*self.apos_prev.invert();
            self.avel = 2.0*vec3(dq.v.x, dq.v.y, dq.v.z)/dt;

            if dq.s < 0.0 {
                self.avel *= -1.0;
            }
        }
        fn iterate(&mut self) {
            self.pos = self.pos_new;
            self.apos = self.apos_new;
        }

        fn update_apos(&mut self, dq: Quat) {
            self.apos_new = dq * self.apos_new;
        }

        fn add_apos(&mut self, dq: Quat) {
            self.apos_new += dq;
            self.apos_new = self.apos_new.normalize();
        }

        fn add_force_at(&mut self, f: Vecn, at: Vecn) {
            self.aacc += self.invinertia*at.cross(f);
        }

        fn add_torque(&mut self, t: Vecn) {
            self.aacc += self.invinertia*t;
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

    fn inertia(&self) -> Mat3 {
        Mat3::zero()
    }
    fn invinertia(&self) -> Mat3 {
        Mat3::zero()
    }

    fn apos(&self) -> Quat {
        Quat::one()
    }
    fn avel(&self) -> Vecn {
        Vecn::zero()
    }
    fn aacc(&self) -> Vecn {
        Vecn::zero()
    }

    fn pos_at(&self, at: Vecn) -> Vecn {
        self.pos() + self.apos().rotate_vector(at)
    }

    fn update_apos(&mut self, dq: Quat) {}
    fn add_apos(&mut self, dq: Quat) {}
    fn add_force_at(&mut self, f: Vecn, at: Vecn) {
        self.add_force(f);
    }
    fn add_torque(&mut self, t: Vecn) {}
}
