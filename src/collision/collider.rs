use three_d::*;
use crate::body::*;
use std::rc::Weak;
use std::cell::RefCell;
use std::vec::Vec;
use std::rc::Rc;

pub trait Collider {
    fn support(&self, dir: Vec3) -> Point3<f32>; 
    fn to_local(&self, pos: Point3<f32>) -> Vec3;
    fn get_body(&self) -> Rc<RefCell<dyn Body>>;
}

#[derive(Clone)]
pub struct SphereCollider {
    parent: Weak<RefCell<dyn Body>>,
    radius: f32
}

impl SphereCollider {
    pub fn new(parent: &Rc<RefCell<dyn Body>>, radius: f32) -> Self {
        Self {
            radius,
            parent: Rc::downgrade(parent)
        }
    }
} 

impl Collider for SphereCollider {
    fn support(&self, dir: Vec3) -> Point3<f32> {
        let pos: Point3<f32> = Point3::origin() + self.parent.upgrade().unwrap().as_ref().borrow().pos();

        pos + dir.normalize() * self.radius
    }
    fn to_local(&self, x: Point3<f32>) -> Vec3 {
        let parent = self.parent.upgrade().unwrap();
        let pos = parent.as_ref().borrow().pos();
        let apos = parent.as_ref().borrow().apos();
        apos.invert().rotate_vector(x.to_vec() - pos)
    }
    fn get_body(&self) -> Rc<RefCell<dyn Body>> {
        self.parent.upgrade().unwrap()
    }
}

#[derive(Clone)]
pub struct CHullCollider {
    parent: Weak<RefCell<dyn Body>>,
    points: Vec<Vec3>
}

impl CHullCollider {
    pub fn new(parent: &Rc<RefCell<dyn Body>>, points: Vec<Vec3>) -> Self {
        Self {
            parent: Rc::downgrade(parent),
            points
        }
    }
}

impl Collider for CHullCollider {
    fn support(&self, dir: Vec3) -> Point3<f32> {
        let parent = self.parent.upgrade().unwrap().clone();
        let pos: Point3<f32> = Point3::origin() + parent.as_ref().borrow().pos();
        let apos = parent.as_ref().borrow().apos(); 

        let mut maximum = f32::MIN;
        let mut maxdir = Vec3::zero();
        for point in &self.points {
            let p = apos.rotate_vector(*point);
            let dot = p.dot(dir);
            if dot > maximum {
                maximum = dot;
                maxdir = p;
            }
        }

        pos + maxdir
    }
    fn to_local(&self, x: Point3<f32>) -> Vec3 {
        let parent = self.parent.upgrade().unwrap();
        let pos = parent.as_ref().borrow().pos();
        let apos = parent.as_ref().borrow().apos();
        apos.invert().rotate_vector(x.to_vec() - pos)
    }
    fn get_body(&self) -> Rc<RefCell<dyn Body>> {
        self.parent.upgrade().unwrap()
    }
}

pub fn support(a: &Box<dyn Collider>, b: &Box<dyn Collider>, dir: Vec3) -> Point3<f32> {
    Point3::origin() + (a.support(dir) - b.support(-dir))
}
