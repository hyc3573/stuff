use super::collider::*;
use super::simplex::*;
use std::collections::HashSet;
use std::vec::Vec;
use three_d::*;
use std::fmt;

mod unordered_pair {
    #[derive(PartialEq, Eq, Hash, Clone, Copy)]
    pub struct UnorderedPair<T> {
        a: T,
        b: T,
    }

    impl<T: Ord + Copy> UnorderedPair<T> {
        fn new(a: T, b: T) -> Self {
            if a < b {
                return UnorderedPair { b, a };
            }
            UnorderedPair { a, b }
        }

        pub fn a(&self) -> T {
            self.a
        }
        pub fn b(&self) -> T {
            self.b
        }
    }

    impl<T: Ord + Copy> From<(T, T)> for UnorderedPair<T> {
        fn from(x: (T, T)) -> Self {
            Self::new(x.0, x.1)
        }
    }
}
use self::unordered_pair::UnorderedPair;

fn get_ccw_normal(a: Point3<f32>, b: Point3<f32>, c: Point3<f32>) -> Vec3 {
    // Get outward facing normal for ccw triangle
    return (c - a).cross(b - a);
}

fn should_swap_winding(a: Point3<f32>, b: Point3<f32>, c: Point3<f32>, align: Vec3) -> bool {
    let normal = get_ccw_normal(a, b, c);
    if normal.dot(align) < 0.0 {
        return true;
    }
    false
}

pub struct Polytope {
    vertices: Vec<Point3<f32>>,
    faces: Vec<[usize; 3]>,
}

impl Polytope {
    pub fn from_simplex(a: Simplex) -> Option<Self> {
        if let Simplex::Tetrahedron(A, B, C, D) = a {
            let mut new = Self {
                vertices: vec![A, B, C, D],
                faces: vec![[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]],
            };

            let center =
                Point3::origin() + (A.to_vec() + B.to_vec() + C.to_vec() + D.to_vec()) / 4.0;

            // fix winding order to CCW
            let mut new_indices: Vec<[usize; 3]> = vec![];
            for [a, b, c] in new.faces.into_iter() {
                if should_swap_winding(
                    new.vertices[a],
                    new.vertices[b],
                    new.vertices[c],
                    new.vertices[a] - center,
                ) {
                    new_indices.push([b, a, c]);
                } else {
                    new_indices.push([a, b, c]);
                }
            }
            new.faces = new_indices;

            Some(new)
        } else {
            None
        }
    }

    pub fn expand(&mut self, p: Point3<f32>, dir: Vec3) {
        self.vertices.push(p);
        let mut faces_after: Vec<[usize; 3]> = Vec::new();
        let mut new_edges: HashSet<UnorderedPair<usize>> = HashSet::new();
        for face in &self.faces {
            let normal = get_ccw_normal(
                self.vertices[face[0]],
                self.vertices[face[1]],
                self.vertices[face[2]],
            );

            if normal.dot(p - self.vertices[face[0]]) > 0.0 {
                // same direction
                let ab: UnorderedPair<usize> = (face[0], face[1]).into();
                let bc: UnorderedPair<usize> = (face[1], face[2]).into();
                let ac: UnorderedPair<usize> = (face[0], face[2]).into();

                if !new_edges.contains(&ab) {
                    new_edges.remove(&ab);
                } else {
                    new_edges.insert(ab);
                }
                if !new_edges.contains(&bc) {
                    new_edges.remove(&bc);
                } else {
                    new_edges.insert(ab);
                }
                if !new_edges.contains(&ac) {
                    new_edges.remove(&ac);
                } else {
                    new_edges.insert(ac);
                }
            } else {
                faces_after.push(face.clone());
            }
        }
        for edge in new_edges.iter() {
            let a = edge.a();
            let b = edge.b();
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
