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
        [0, 1].map(
            |i| -> Vecn {
                self.bodies()[i].as_ref().borrow().pos_at(self.offsets[i])
            }
        )
    }

    fn true_offsets(&self) -> [Vecn; 2] {
        [0, 1].map(
            |i| -> Vecn {
                self.bodies()[i].as_ref().borrow().apos().rotate_vector(self.offsets[i])
            }
        )
    }

    pub fn new(bodies: [Rc<RefCell<dyn Body>>; 2], offsets: [Vecn; 2], dist: Real, compliance: Real) -> Self {
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

        let result = (pos[0] - pos[1]).magnitude() - self.dist;
        result
    }

    fn dC(&self) -> Vec<Vecn> {
        let pos = self.points();
        let mut n = pos[0] - pos[1];

        if !n.is_zero() {
            n = n.normalize()
        }

        vec!(n, -n)
    }

    fn dq(&self, dlambda: Real) -> Vec<Quat> {
        let mut result = Vec::<Quat>::new();
        for i in 0..self.len() {
            result.push(
                0.5*Quat::from_sv(0.0, self.bodies()[i].as_ref().borrow().invinertia()*self.true_offsets()[i].cross(self.dC()[i])*dlambda)*self.bodies()[i].as_ref().borrow().apos()
            )
        }

        // result[1] *= -1.0;

        result
        // vec![Quat::zero(), Quat::zero()]
    } 

    fn invmass_sum(&self) -> Real {
        let dC = self.dC();
        let pos = self.points();
        let mut sum: Real = 0.0;

        for i in 0..self.len() {
            sum += self.bodies()[i].as_ref().borrow().invmass() +
                self.true_offsets()[i].cross(dC[0]).dot(self.bodies()[i].as_ref().borrow().invinertia()*self.true_offsets()[i].cross(dC[0]));
        }
        
        sum
    }
}
