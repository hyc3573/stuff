use crate::config::*;
use crate::body::*;
use std::cell::RefCell;
use std::rc::Rc;

pub trait Constraint<const N: usize, T: Body> {
    fn C(&self) -> Real;
    fn dC(&self) -> [Vecn; N];

    fn parts(&self) -> [Rc<RefCell<T>>; N];
    fn compliance(&self) -> Real;
    fn lambda(&self) -> Real;
    fn update_lambda(&mut self, dlambda: Real);
    
    fn dlambda(&self) -> Real {
        let C = self.C();
        let dC = self.dC();
        let mut dot: Real = 0.0;
        for i in 0..N {
           dot += dC[i].length_squared()*self.parts()[i].borrow().invmass()
        }
        let dot = dot;
        let denom = dot + self.compliance();

        if denom == 0.0 {
            return 0.0
        }

        (-C - self.compliance()*self.lambda())/(denom)
    }

    fn dx(&self, dlambda: Real) -> [Vecn; N] {
        let dC = self.dC();

        let mut result = [Vecn::ZERO; N];
        for i in 0..N {
            result[i] = self.parts()[i].borrow().invmass()*dC[i]*dlambda;
        }

        result
    }

    fn iterate(&mut self) {
        let dlambda = self.dlambda();
        let dx = self.dx(dlambda);
        let d = dx[0];
        let pos = self.parts()[0].as_ref().borrow().pos();

        self.update_lambda(dlambda);
        for i in 0..N {
            self.parts()[i].as_ref().borrow_mut().update_pos(dx[i]);
        }
    }
}
