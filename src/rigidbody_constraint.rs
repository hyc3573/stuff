use crate::config::*;
use crate::body::*;
use crate::constraint::*;
use std::cell::RefCell;
use std::rc::Rc;
use std::iter::zip;
use three_d::*;

pub struct RDist {
    bodies: [Rc<RefCell<dyn Body>>; 2],
    offsets: [Vecn; 2],
    lambda: Real,
    compliance: Real,
    dist: Real
}

impl RDist {
    fn points(&self) -> [Vecn; 2] {
        let pos: Vec<Vecn> = zip(self.bodies.clone(), self.offsets).map(
            |x| -> Vecn {x.0.as_ref().borrow().pos_at(x.1)}
        ).collect();

        [pos[0], pos[1]]
    }

    fn new(bodies: [Rc<RefCell<dyn Body>>; 2], offsets: [Vecn; 2], dist: Real, compliance: Real) -> Self {
        Self {
            bodies,
            offsets,
            lambda: 0.0,
            compliance,
            dist
        }
    }
}

impl Constraint for RDist {
    constraint_getset!(2);

    fn C(&self) -> Real {
        let pos = self.points();

        (pos[0] - pos[1]).magnitude() - self.dist
    }

    fn dC(&self) -> Vec<Vecn> {
        let pos = self.points();
        let n = (pos[0] - pos[1]).normalize();

        vec!(n, -n)
    }

    fn dq(&self, dlambda: Real) -> Vec<Quat> {
        let mut result = Vec::<Quat>::new();
        for i in 0..self.len() {
            result.push(
                0.5*Quat::from_sv(0.0, self.bodies()[i].as_ref().borrow().invinertia()*self.offsets[i].cross(self.dC()[i]))*self.bodies()[i].as_ref().borrow().apos()
            )
        }

        result
    } 

    fn invmass_sum(&self) -> Real {
        let dC = self.dC();
        let mut sum: Real = 0.0;

        for i in 0..self.len() {
            sum += self.bodies()[i].as_ref().borrow().invmass() + self.offsets[i].cross(dC[i]).dot(self.bodies()[i].as_ref().borrow().invinertia()*self.offsets[i].cross(dC[i]));
        }

        sum
    }
}
