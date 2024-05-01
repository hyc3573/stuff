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
    
    fn dlambda(&self) -> Real {
        let C = self.C();
        let dC = self.dC();
        let mut dot: Real = 0.0;
        for i in 0..2 {
           dot += dC[i].length_squared()*self.parts()[i].borrow().invmass()
        }
        let dot = dot;
        
        (-C - self.compliance()*self.lambda())/(dot + self.compliance())
    }

    fn dx(&self, dlambda: Real) -> [Vecn; N] {
        let dC = self.dC();

        (0..2).map(
            |i| {self.parts()[i].borrow().invmass()*dC[i]*dlambda}
        ).collect::<Vec<Vecn>>().try_into().unwrap()
    }

    fn iterate(&mut self);
}
