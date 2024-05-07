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
        fn compliance(&self) -> Real {
            self.compliance
        }
        fn lambda(&self) -> Real {
            self.lambda
        }
        fn reset_lambda(&mut self) {
            self.lambda = 0.0;
        }
        fn update_lambda(&mut self, dlambda: Real) {
            self.lambda += dlambda;
        }
        fn len(&self) -> usize {
            $n
        }
    }
}

pub trait Constraint {
    fn C(&self) -> Real;
    fn dC(&self) -> Vec<Vecn>;

    fn bodies(&self) -> Vec<Rc<RefCell<dyn Body>>>;
    fn compliance(&self) -> Real;
    fn lambda(&self) -> Real;
    fn update_lambda(&mut self, dlambda: Real);
    fn len(&self) -> usize;
    fn reset_lambda(&mut self);
    fn invmass_sum(&self) -> Real {
        let dC = self.dC();
        let mut dot: Real = 0.0;
        for i in 0..self.len() {
           dot += dC[i].dot(dC[i])*self.bodies()[i].borrow().invmass()
        }

        dot
    }

    fn dlambda(&self, dt: Real) -> Real {
        let alpha = self.compliance()/(dt*dt);
        let C = self.C();
        
        let dot = self.invmass_sum();
        let denom = dot + alpha; 

        if denom == 0.0 {
            return 0.0
        }

        (-C-alpha*self.lambda())/(denom)
    }

    fn dx(&self, dlambda: Real) -> Vec<Vecn> {
        let dC = self.dC();

        let mut result = vec![Vecn::zero(); self.len()];
        for i in 0..self.len() {
            result[i] = self.bodies()[i].borrow().invmass()*dC[i]*dlambda;
        }

        result
    }
    fn dq(&self, dlambda: Real) -> Vec<Quat> {
        vec![Quat::zero(); self.len()]
    }

    fn iterate(&mut self, dt: Real) {
        let dlambda = self.dlambda(dt);
        let dx = self.dx(dlambda);
        let dq = self.dq(dlambda);
        self.update_lambda(dlambda);

        for i in 0..self.len() {
            self.bodies()[i].as_ref().borrow_mut().update_pos(dx[i]/2.0);
            self.bodies()[i].as_ref().borrow_mut().add_apos(dq[i]);
        }
    }
}
