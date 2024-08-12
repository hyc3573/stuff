use super::collider::*;
use super::simplex::*;
use std::collections::HashSet;
use std::vec::Vec;
use three_d::*;
use std::fmt;
use std::mem::swap;

pub fn get_ccw_normal(a: &Point3<f32>, b: &Point3<f32>, c: &Point3<f32>) -> Vec3 {
    (b - a).cross(c - a)
}

fn should_swap_winding(a: Point3<f32>, b: Point3<f32>, c: Point3<f32>, align: Vec3) -> bool {
    let normal = get_ccw_normal(&a, &b, &c);
    if normal.dot(align) < 0.0 {
        return true;
    }
    false
}

pub struct Polytope {
    vertices: Vec<Point3<f32>>,
    dirs: Vec<Vec3>,
    faces: Vec<[usize; 3]>,
}

impl Polytope {
    pub fn from_simplex(a: Simplex) -> Option<Self> {
        if let Simplex::Tetrahedron(A, B, C, D, ad, bd, cd, dd) = a {
            let mut result = Self {
                vertices: vec![A, B, C, D],
                dirs: vec![ad, bd, cd, dd],
                faces: vec![[0, 1, 2], [1, 0, 3], [0, 2, 3], [2, 1, 3]],
            };

            let center =
                Point3::origin() + (A.to_vec() + B.to_vec() + C.to_vec() + D.to_vec()) / 4.0;

            // fix winding order to CCW
            let mut faces: Vec<[usize; 3]> = vec![];
            for [a, b, c] in result.faces.into_iter() {
                faces.push([a, b, c]);
            }
            result.faces = faces;

            let h = (A - B).dot((A - C).cross(A - D));
            if h < 0.0 {
                result.vertices.swap(0, 1);
                result.dirs.swap(0, 1);
            }

            Some(result)
        } else {
            None
        }
    }

    pub fn expand(&mut self, p: Point3<f32>, dir: Vec3) {
        self.vertices.push(p);
        self.dirs.push(dir);
        let mut faces_after: Vec<[usize; 3]> = Vec::new();
        let mut new_edges: HashSet<(usize, usize)> = HashSet::new();
        for face in &self.faces {
            let normal = get_ccw_normal(
                &self.vertices[face[0]],
                &self.vertices[face[1]],
                &self.vertices[face[2]],
            );

            if normal.dot(p - self.vertices[face[0]]) > 0.0 {
                // same direction
                let ab: (usize, usize) = (face[0], face[1]);
                let ba: (usize, usize) = (face[1], face[0]);
                
                let bc: (usize, usize) = (face[1], face[2]);
                let cb: (usize, usize) = (face[2], face[1]);
                
                let ca: (usize, usize) = (face[2], face[0]);
                let ac: (usize, usize) = (face[0], face[2]);

                if new_edges.contains(&ba) {
                    new_edges.remove(&ba);
                } else {
                    new_edges.insert(ab);
                }
                if new_edges.contains(&cb) {
                    new_edges.remove(&cb);
                } else {
                    new_edges.insert(bc);
                }
                if new_edges.contains(&ac) {
                    new_edges.remove(&ac);
                } else {
                    new_edges.insert(ca);
                }
            } else {
                faces_after.push(face.clone());
            }
        }
        for edge in new_edges.iter() {
            let a = edge.0;
            let b = edge.1;
            let c = self.vertices.len() - 1;

            if should_swap_winding(self.vertices[a], self.vertices[b], self.vertices[c], dir) {
                faces_after.push([b, a, c]);
            } else {
                faces_after.push([a, b, c])
            }
        }

        self.faces = faces_after;
    }

    pub fn faces(&self) -> &Vec<[usize; 3]> {
        &self.faces
    }

    pub fn vertices(&self) -> &Vec<Point3<f32>> {
        &self.vertices
    }

    pub fn dirs(&self) -> &Vec<Vec3> {
        &self.dirs
    }
}

impl fmt::Display for Polytope {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for vertex in &self.vertices {
            writeln!(f, "v {} {} {} 1.0", vertex.x, vertex.y, vertex.z)?
        }

        for face in &self.faces {
            writeln!(f, "f {} {} {}", face[0]+1, face[1]+1, face[2]+1)?
            
        }

        Ok(())
    }
}
