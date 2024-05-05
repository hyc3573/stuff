use crate::config::*;
use crate::body::*;
use std::cell::RefCell;
use std::rc::Rc;
use three_d::*;

macro_rules! constraint_getset {
    {$n:expr} => {
        fn bodies(&self) -> [Rc<RefCell<dyn Body>>; $n] {
            self.bodies.clone()
        }
        fn compliance(&self) -> Real {
            self.compliance
        }
        fn lambda(&self) -> Real {
            self.lambda
        }
        fn update_lambda(&mut self, dlambda: Real) {
            self.lambda += dlambda;
        }
    }
}

pub trait Constraint<const N: usize> {
    fn C(&self) -> Real;
    fn dC(&self) -> [Vecn; N];

    fn bodies(&self) -> [Rc<RefCell<dyn Body>>; N];
    fn compliance(&self) -> Real;
    fn lambda(&self) -> Real;
    fn update_lambda(&mut self, dlambda: Real);
    
    fn dlambda(&self, dt: Real) -> Real {
        let alpha = self.compliance()/(dt*dt);
        let C = self.C();
        let dC = self.dC();
        let mut dot: Real = 0.0;
        for i in 0..N {
           dot += dC[i].dot(dC[i])*self.bodies()[i].borrow().invmass()
        }
        let dot = dot;
        let denom = dot + alpha; 

        if denom == 0.0 {
            return 0.0
        }

        (-C-alpha*self.lambda())/(denom)
    }

    fn dx(&self, dlambda: Real) -> [Vecn; N] {
        let dC = self.dC();

        let mut result = [Vecn::zero(); N];
        for i in 0..N {
            result[i] = self.bodies()[i].borrow().invmass()*dC[i]*dlambda;
        }

        result
    }

    fn iterate(&mut self, dt: Real) {
        let dlambda = self.dlambda(dt);
        let dx = self.dx(dlambda);
        self.update_lambda(dlambda);

        for i in 0..N {
            self.bodies()[i].as_ref().borrow_mut().update_pos(dx[i]/2.0);
        }
    }
}
