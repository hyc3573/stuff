use crate::config::*;
use three_d::Mat3;
use three_d::Quat;
use three_d::Rotation;
use three_d::*;
use crate::collision::collider::Collider;

macro_rules! body_common {
    {} => {
        fn invmass(&self) -> f32 {self.invmass}
        fn pos(&self) -> Vec3 {self.pos}
        fn pos_prev(&self) -> Vec3 {self.pos_prev}
        fn vel(&self) -> Vec3 {self.vel}
        fn set_vel(&mut self, new_vel: Vec3) {
            self.vel = new_vel;
        }
        fn acc(&self) -> Vec3 {self.acc}

        fn update_pos(&mut self, dx: Vec3) {
            // self.pos_new += dx;
            self.pos += dx;
        }
        fn add_force(&mut self, f: Vec3) {
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

        fn apos_prev(&self) -> Quat {
            self.apos_prev
        }

        fn avel(&self) -> Vec3 {
            self.avel
        }

        fn set_avel(&mut self, new_avel: Vec3) {
            self.avel = new_avel;
        }

        fn aacc(&self) -> Vec3 {
            self.aacc
        }

        fn predict(&mut self, dt: f32) {
            self.pos_prev = self.pos;
            self.vel += self.acc*dt;
            self.pos += self.vel*dt;

            self.apos_prev = self.apos;
            self.avel += self.aacc*dt - dt*self.invinertia*self.avel.cross(self.inertia*self.avel);
            self.apos = self.apos + (dt*0.5*Quat::new(0.0, self.avel.x, self.avel.y, self.avel.z))*self.apos;
            self.apos = self.apos.normalize();

            // self.pos_new = self.pos;
            // self.apos_new = self.apos;
        }

        fn update(&mut self, dt: f32) {
            self.acc = Vec3::zero();
            // self.vel = (2.0*self.pos - self.pos_prev - self.pos_pred)/dt;
            self.vel = (self.pos - self.pos_prev)/dt;

            self.aacc = Vec3::zero();
            let dq = self.apos*self.apos_prev.invert();
            self.avel = 2.0*vec3(dq.v.x, dq.v.y, dq.v.z)/dt;

            if dq.s < 0.0 {
                self.avel *= -1.0;
            }
        }
        fn iterate(&mut self) {
            // self.pos = self.pos_new;
            // self.apos_new = self.apos_new.normalize();
            self.apos = self.apos.normalize()
            // self.apos = self.apos_new;
        }

        fn update_apos(&mut self, dq: Quat) {
            // self.apos_new = dq * self.apos_new;
            self.apos = dq * self.apos;
        }

        fn add_apos(&mut self, dq: Quat) {
            // self.apos_new += dq;
            self.apos += dq;
            // self.apos = self.apos.normalize();
        }

        fn add_force_at(&mut self, f: Vec3, at: Vec3) {
            self.aacc += self.invinertia*at.cross(f);
        }

        fn add_torque(&mut self, t: Vec3) {
            self.aacc += self.invinertia*t;
        }

        fn update_velocity(&mut self, dt: f32) {
            self.vel *= DAMP.powf(dt);
            self.avel *= ROT_DAMP.powf(dt);
        }
    }
}

pub trait Body {
    fn invmass(&self) -> f32; 

    fn pos(&self) -> Vec3;
    fn pos_prev(&self) -> Vec3;
    fn vel(&self) -> Vec3;
    fn set_vel(&mut self, new_vel: Vec3);
    fn acc(&self) -> Vec3;

    fn update_pos(&mut self, dx: Vec3);
    fn add_force(&mut self, f: Vec3);

    fn predict(&mut self, dt: f32);
    fn update(&mut self, dt: f32);
    fn update_velocity(&mut self, dt: f32) {
        self.set_vel(self.vel()*DAMP.powf(dt));
    }
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
    fn apos_prev(&self) -> Quat {
        Quat::one()
    }
    fn avel(&self) -> Vec3 {
        Vec3::zero()
    }
    fn set_avel(&mut self, new_avel: Vec3) {}
    fn aacc(&self) -> Vec3 {
        Vec3::zero()
    }

    fn to_global(&self, at: Vec3) -> Vec3 {
        self.pos() + self.apos().rotate_vector(at)
    }
    fn to_local(&self, at: Vec3) -> Vec3 {
        self.apos().invert().rotate_vector(at - self.pos())
    }

    fn update_apos(&mut self, dq: Quat) {}
    fn add_apos(&mut self, dq: Quat) {}
    fn add_force_at(&mut self, f: Vec3, at: Vec3) {
        self.add_force(f);
    }
    fn add_torque(&mut self, t: Vec3) {}

    fn get_matrix(&self) -> Mat4 {
        Mat4::from_translation(self.pos())*Mat4::from(self.apos())
    }
}
