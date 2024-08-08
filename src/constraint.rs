use crate::config::*;
use crate::body::*;
use std::cell::RefCell;
use std::rc::Rc;
use three_d::*;

macro_rules! constraint_getset {
    {$n:expr} => {
        fn bodies(&self) -> Vec<Rc<RefCell<dyn Body>>> {
            self.bodies.clone().to_vec()
        }
        fn compliance(&self) -> f32 {
            self.compliance
        }
        fn lambda(&self) -> f32 {
            self.lambda
        }
        fn reset_lambda(&mut self) {
            self.lambda = 0.0;
        }
        fn update_lambda(&mut self, dlambda: f32) {
            self.lambda += dlambda;
        }
        fn len(&self) -> usize {
            $n
        }
    }
}

pub trait Constraint {
    fn C(&self) -> f32;
    fn dC(&self) -> Vec<Vec3>;

    fn bodies(&self) -> Vec<Rc<RefCell<dyn Body>>>;
    fn compliance(&self) -> f32;
    fn lambda(&self) -> f32;
    fn update_lambda(&mut self, dlambda: f32);
    fn len(&self) -> usize;
    fn reset_lambda(&mut self);
    fn invmass_sum(&self) -> f32 {
        let dC = self.dC();
        let mut dot: f32 = 0.0;
        for i in 0..self.len() {
           dot += dC[i].dot(dC[i])*self.bodies()[i].borrow().invmass()
        }

        dot
    }

    fn dlambda(&self, dt: f32) -> f32 {
        let alpha = self.compliance()/(dt*dt);
        let C = self.C();
        
        let dot = self.invmass_sum();
        let denom = dot + alpha; 

        if denom.abs() < f32::EPSILON {
            return 0.0
        }

        (-C-alpha*self.lambda())/(denom)
    }

    fn dx(&self, dlambda: f32) -> Vec<Vec3> {
        let dC = self.dC();

        let mut result = vec![Vec3::zero(); self.len()];
        for i in 0..self.len() {
            result[i] = self.bodies()[i].borrow().invmass()*dC[i]*dlambda;
        }

        result
    }
    fn dq(&self, dlambda: f32) -> Vec<Quat> {
        vec![Quat::zero(); self.len()]
    }

    fn iterate(&mut self, dt: f32) {
        let dlambda = self.dlambda(dt);
        let dx = self.dx(dlambda);
        let dq = self.dq(dlambda);
        self.update_lambda(dlambda);

        for i in 0..self.len() {
            self.bodies()[i].as_ref().borrow_mut().update_pos(dx[i]/2.0);
            self.bodies()[i].as_ref().borrow_mut().add_apos(dq[i]);
        }
    }

    fn velocity_update(&mut self, dt: f32) {}
}
