use three_d::*;
use crate::body::*;
use std::rc::Weak;
use std::cell::RefCell;
use std::vec::Vec;
use std::rc::Rc;

pub fn ccw_normal(face: &Vec<usize>, vertices: &Vec<Vec3>) -> Vec3 {
    (vertices[face[1]] - vertices[face[0]]).cross(vertices[face[2]] - vertices[face[0]])
}

pub trait Collider {
    fn support(&self, dir: Vec3) -> Point3<f32>; 
    fn to_local(&self, pos: Point3<f32>) -> Vec3;
    fn get_body(&self) -> Rc<RefCell<dyn Body>>;

    fn get_vertices(&self) -> Option<Vec<Vec3>> {
        None
    }
    fn get_faces(&self) -> Option<&Vec<Vec<usize>>> {
        None
    }
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
pub struct PolyhedraCollider {
    parent: Weak<RefCell<dyn Body>>,
    vertices: Vec<Vec3>,
    faces: Vec<Vec<usize>>,
}

impl PolyhedraCollider {
    pub fn new(parent: &Rc<RefCell<dyn Body>>, polyhedra: (Vec<Vec3>, Vec<Vec<usize>>)) -> Self {
        Self {
            parent: Rc::downgrade(parent),
            vertices: polyhedra.0,
            faces: polyhedra.1
        }
    }
}

impl Collider for PolyhedraCollider {
    fn support(&self, dir: Vec3) -> Point3<f32> {
        let parent = self.parent.upgrade().unwrap().clone();
        let pos: Point3<f32> = Point3::origin() + parent.as_ref().borrow().pos();
        let apos = parent.as_ref().borrow().apos(); 

        let mut maximum = f32::MIN;
        let mut maxdir = Vec3::zero();
        for point in &self.vertices {
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

    fn get_vertices(&self) -> Option<Vec<Vec3>> {
        let parent = self.parent.upgrade().unwrap().clone();
        let pos = parent.as_ref().borrow().pos();
        let apos = parent.as_ref().borrow().apos();
        
        Some(self.vertices.iter().map(
            |v| {
                pos + apos.rotate_vector(*v)
            }
        ).collect())
    }

    fn get_faces(&self) -> Option<&Vec<Vec<usize>>> {
        Some(&self.faces)
    }
}

pub fn support(a: &Box<dyn Collider>, b: &Box<dyn Collider>, dir: Vec3) -> Point3<f32> {
    Point3::origin() + (a.support(dir) - b.support(-dir))
}
