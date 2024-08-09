use std::collections::hash_set::SymmetricDifference;
use std::f32::consts::FRAC_PI_3;

use super::polytope::Polytope;
use super::simplex::*;
use super::{collider::*, polytope};
use three_d::*;

fn triangle_point_proj(t: [&Point3<f32>; 3], point: &Point3<f32>) -> Point3<f32> {
    let ab = t[1] - t[0];
    let ac = t[2] - t[0];
    let bc = t[2] - t[1];

    let n = ab.cross(ac);
    let n1 = -n.cross(ab);
    let n2 = -n.cross(bc);
    let n3 = n.cross(ac);

    let mut p = point.clone();
    if n.dot(t[0] - p) != 0.0 {
        // Outside of the plain
        p = p + 0.5 * n.normalize().dot(t[0] - p) * n;
    }
    let p = p;

    // Project p to the plain of t
    let pa = t[0] - p;
    let pb = t[1] - p;
    let pc = t[2] - p;

    if pa.dot(ac) < 0.0 && pc.dot(ac) > 0.0 && n3.dot(pa) < 0.0 {
        // #2
        return t[0] - ac.dot(-pc) * ac / ac.magnitude2();
    }

    if pc.dot(bc) > 0.0 && pb.dot(bc) < 0.0 && n2.dot(pc) < 0.0 {
        // #3
        return t[1] + bc.dot(-pb) * bc / bc.magnitude2();
    }

    if pa.dot(ab) < 0.0 && pb.dot(ab) > 0.0 && n1.dot(pb) < 0.0 {
        // #5
        return t[0] + ab.dot(-pa) * ab / ab.magnitude2();
    }

    if pa.dot(ab) >= 0.0 && pa.dot(ac) >= 0.0 {
        // #4
        return t[0].to_owned();
    }

    if pb.dot(bc) >= 0.0 && pb.dot(ab) <= 0.0 {
        // #6
        return t[1].to_owned();
    }

    if pc.dot(bc) <= 0.0 && pc.dot(ac) <= 0.0 {
        // # 1
        return t[2].to_owned();
    }

    p
}

fn tri_pnt_dist(t: [&Point3<f32>; 3], p: &Point3<f32>) -> f32 {
    return triangle_point_proj(t, p).distance(*p);
}

pub fn gjk(
    a: &Box<dyn Collider>,
    b: &Box<dyn Collider>,
    return_tetrahedron: bool,
) -> Option<Simplex> {
    let mut simplex = Simplex::Point(support(&a, &b, Vec3::unit_x())); // last inserted item must be at the end

    loop {
        match simplex {
            Simplex::Point(A) => {
                // check if it is origin -> return true
                if A.distance(Point3::origin()) == 0.0 && !return_tetrahedron {
                    return Some(simplex);
                }
                // get direction to the origin
                let dir = Point3::origin() - A;
                // get support point there
                let D = support(&a, &b, dir);
                // check whether it crossed the origin. if not, return false
                if dir.dot(D - Point3::origin()) < 0.0 {
                    return None;
                }
                // expand the simplex to line
                simplex = Simplex::Line(A, D)
            }
            Simplex::Line(A, B) => {
                // check if it contains the origin -> return true
                let AB = B - A;
                let AO = Point3::origin() - A;
                let BO = Point3::origin() - B;
                if AB.cross(AO).magnitude2() == 0.0
                    && AB.cross(BO).magnitude() == 0.0
                    && AB.dot(AO) > 0.0
                    && AB.dot(BO) < 0.0
                    && !return_tetrahedron
                {
                    return Some(simplex);
                }
                // get normal direction towards the origin: triple product
                let dir = AB.cross(BO).cross(AB);
                // get support point there
                let D = support(&a, &b, dir);
                // check wherther it crossed the origin. if not, return false
                if dir.dot(D - Point3::origin()) < 0.0 {
                    return None;
                }
                // expand the simplex to triangle
                simplex = Simplex::Triangle(A, B, D);
            }
            Simplex::Triangle(A, B, C) => {
                // case 1: if its plain contains origin
                let AB = B - A;
                let AC = C - A;
                let AO = Point3::origin() - A;
                let BC = B - C;
                let CO = Point3::origin() - C;
                if AB.cross(AO).dot(AC.cross(AO)) == 0.0 {
                    //         there is three possibilities
                    //         case 1: if the origin is in A-C area:
                    //                 get a normal vector of A-C which points to the origin
                    //                 get a support point
                    //                 check whether it crossed the origin. if not, return false
                    //                 discard B and add that point
                    //         case 2: if the origin is in C-B area:
                    //                 get a normal vector of A-C which points to the origin
                    //                 get a support point
                    //                 check whether it crossed the origin. if not, return false
                    //                 discard A and add that point
                    //         case 3: else -> return true
                    if AC.cross(CO).cross(AC).dot(AC.cross(-AB).cross(AC)) > 0.0 {
                        let dir = AC.cross(CO).cross(AC);
                        assert!(dir.dot(CO) > 0.0);
                        let D = support(&a, &b, dir);
                        if dir.dot(D - Point3::origin()) < 0.0 {
                            return None;
                        }
                        simplex = Simplex::Triangle(A, C, D);
                    } else if BC.cross(CO).cross(BC).dot(BC.cross(AB).cross(BC)) > 0.0 {
                        let dir = BC.cross(CO).cross(BC);
                        assert!(dir.dot(CO) > 0.0);
                        let D = support(&a, &b, dir);
                        if dir.dot(D - Point3::origin()) > 0.0 {
                            return None;
                        }
                        simplex = Simplex::Triangle(A, B, D);
                    } else if !return_tetrahedron {
                        return Some(simplex);
                    }
                }
                // case 2: else
                //         get a normal towards the origin
                //         get a support point
                //         check whether it crossed the origin. if not, return false
                //         expand the simplex to tetrahedron
                else {
                    let mut dir = AB.cross(BC);
                    if dir.dot(Point3::origin() - B) < 0.0 {
                        dir *= -1.0;
                    }
                    let dir = dir;
                    let D = support(&a, &b, dir);
                    if dir.dot(D - Point3::origin()) < 0.0 {
                        return None;
                    }
                    simplex = Simplex::Tetrahedron(A, B, C, D);
                }
            }
            Simplex::Tetrahedron(A, B, C, D) => {
                // there are four possibilities
                // case 1: if the origin is in ABD area:
                //         get a normal vector of ABD which points to the origin
                //         get a support point
                //         check whether it crossed the origin. if not, return false
                //         discard C and add that point
                // case 2: if the origin is in BCD area:
                //         get a normal vector of BCD which points to the origin
                //         get a support point
                //         check whether it crossed the origin. if not, return false
                //         discard A and add that point
                // case 3: if the origin is in ACD area:
                //         get a normal vector of ACD which points to the origin
                //         get a support point
                //         check whether it crossed the origin. if not, return false
                //         discard B and add that point
                // case 4: else -> return true
                let DO = Point3::origin() - D;
                let DA = A - D;
                let DB = B - D;
                let DC = C - D;
                let mut ABDn = (A - B).cross(B - D);
                if ABDn.dot(DC) > 0.0 {
                    ABDn *= -1.0;
                }
                let ABDn = ABDn;
                let mut BCDn = (B - C).cross(C - D);
                if BCDn.dot(DA) > 0.0 {
                    BCDn *= -1.0;
                }
                let BCDn = BCDn;
                let mut ACDn = (A - C).cross(C - D);
                if ACDn.dot(DB) > 0.0 {
                    ACDn *= -1.0;
                }
                let ACDn = ACDn;

                let overABD = ABDn.dot(DO) > 0.0;
                let overBCD = BCDn.dot(DO) > 0.0;
                let overACD = ACDn.dot(DO) > 0.0;

                if overABD && !overBCD && !overACD {
                    let E = support(&a, &b, ABDn);
                    if ABDn.dot(E - Point3::origin()) < 0.0 {
                        return None;
                    }
                    simplex = Simplex::Tetrahedron(A, B, D, E);
                } else if !overABD && overBCD && !overACD {
                    let E = support(&a, &b, BCDn);
                    if BCDn.dot(E - Point3::origin()) < 0.0 {
                        return None;
                    }
                    simplex = Simplex::Tetrahedron(B, C, D, E);
                } else if !overABD && !overBCD && overACD {
                    let E = support(&a, &b, ACDn);
                    if ACDn.dot(E - Point3::origin()) < 0.0 {
                        return None;
                    }
                    simplex = Simplex::Tetrahedron(A, C, D, E);
                } else if !overABD && !overBCD && !overACD {
                    return Some(simplex);
                } else if overABD && overBCD && overACD {
                    return None;
                } else if overABD && overBCD && !overACD {
                    let DBn = -DB.cross(DO).cross(DB);
                    let E = support(&a, &b, DBn);
                    if DBn.dot(E - Point3::origin()) < 0.0 {
                        return None;
                    }
                    simplex = Simplex::Tetrahedron(B, C, D, E);
                } else if !overABD && overBCD && overACD {
                    let DCn = -DC.cross(DO).cross(DC);
                    let E = support(&a, &b, DCn);
                    if DCn.dot(E - Point3::origin()) < 0.0 {
                        return None;
                    }
                    simplex = Simplex::Tetrahedron(A, C, D, E);
                } else if overABD && !overBCD && overACD {
                    let DAn = -DA.cross(DO).cross(DA);
                    let E = support(&a, &b, DAn);
                    if DAn.dot(E - Point3::origin()) < 0.0 {
                        return None;
                    }
                    simplex = Simplex::Tetrahedron(A, B, D, E);
                }
            }
        }
        if simplex.is_dup() {
            return None;
        }
    }
}

pub fn epa(a: &Box<dyn Collider>, b: &Box<dyn Collider>, s: Simplex) -> (Vec3, f32, Vec3, Vec3) {
    let mut simplex = s;
    if let Simplex::Point(A) = simplex {
        let mut s = Simplex::Point(A);
        for dir in [
            Vec3::unit_x(),
            Vec3::unit_y(),
            Vec3::unit_z(),
            -Vec3::unit_x(),
            -Vec3::unit_y(),
            -Vec3::unit_z(),
        ] {
            s = Simplex::Line(A, support(&a, &b, dir));
            if !s.is_dup() {
                break;
            }
        }

        simplex = s;
    }
    if let Simplex::Line(A, B) = simplex {
        // to which direction?
        let axis = [Vec3::unit_x(), Vec3::unit_y(), Vec3::unit_z()];
        let dir = B - A;
        let comps = [dir.x, dir.y, dir.z];

        let mut max = 0.0;
        let mut maxaxis = 0;
        for i in [0, 1, 2] {
            if comps[i].abs() > max {
                max = comps[i].abs();
                maxaxis = i;
            }
        }

        let mut d = dir.cross(axis[maxaxis]);
        let rot = Mat3::from_axis_angle(dir, degrees(60.0));

        let mut s = Simplex::Triangle(A, A, B);
        for i in 0..5 {
            let D = support(&a, &b, d);
            s = Simplex::Triangle(A, B, D);

            if !s.is_dup() {
                break;
            }
        }
    }
    if let Simplex::Triangle(A, B, C) = simplex {
        let dir = (A - B).cross(B - C);
        let D = support(&a, &b, dir);

        let mut s = Simplex::Tetrahedron(A, B, C, D);
        if s.is_dup() {
            let D = support(&a, &b, -dir);

            s = Simplex::Tetrahedron(A, B, C, D);
        }

        simplex = s;
    }

    // println!("---------------");

    let mut polytope = Polytope::from_simplex(s).unwrap();
    // println!("{}", polytope);

    let mut mindist_global = f32::MAX;
    let mut closest_global: usize = 0;
    loop {
        // find closest edge
        let mut mindist = f32::MAX;
        let mut closest_triangle: usize = 0;
        for (i, face) in polytope.faces().iter().enumerate() {
            let triangle = [
                &polytope.vertices()[face[0]],
                &polytope.vertices()[face[1]],
                &polytope.vertices()[face[2]],
            ];
            if (triangle[1] - triangle[0])
                .cross(triangle[2] - triangle[0])
                .magnitude2()
                < f32::EPSILON
            {
                continue;
            }

            let dist = tri_pnt_dist(
                [
                    &polytope.vertices()[face[0]],
                    &polytope.vertices()[face[1]],
                    &polytope.vertices()[face[2]],
                ],
                &Point3::origin(),
            );

            if dist < mindist {
                mindist = dist;
                closest_triangle = i;
            }
        }

        if mindist >= mindist_global {
            let face = polytope.faces()[closest_triangle];
            let triangle = [
                &polytope.vertices()[face[0]],
                &polytope.vertices()[face[1]],
                &polytope.vertices()[face[2]],
            ];

            let n = (triangle[1] - triangle[0]).cross(triangle[2] - triangle[0]);

            let proj = triangle_point_proj(triangle, &Point3::origin());

            let pa = triangle[0] - proj;
            let pb = triangle[1] - proj;
            let pc = triangle[2] - proj;

            let na = (triangle[2] - triangle[1]).cross(proj - triangle[1]);
            let nb = (triangle[0] - triangle[2]).cross(proj - triangle[2]);
            let nc = (triangle[1] - triangle[0]).cross(proj - triangle[0]);

            let bary = (
                n.dot(na) / n.magnitude2(),
                n.dot(nb) / n.magnitude2(),
                n.dot(nc) / n.magnitude2(),
                // alpha, beta, gamma
            );

            // assert!(
            //     bary.0 + bary.1 + bary.2 - 1.0 < f32::EPSILON
            // );
            // let res = triangle[0].to_vec()*bary.0+triangle[1].to_vec()*bary.1+triangle[2].to_vec()*bary.2;
            // println!("{} {} {} == {} {} {}", res.x, res.y, res.z, proj.x, proj.y, proj.z);

            let oa = triangle[0] - Point3::origin();
            let ob = triangle[1] - Point3::origin();
            let oc = triangle[2] - Point3::origin();

            let aa = a.support(oa);
            let ab = a.support(ob);
            let ac = a.support(oc);

            let ba = b.support(-oa);
            let bb = b.support(-ob);
            let bc = b.support(-oc);

            let pa = aa * bary.0 + ab.to_vec() * bary.1 + ac.to_vec() * bary.2;
            let pb = ba * bary.0 + bb.to_vec() * bary.1 + bc.to_vec() * bary.2;

            // assert!(((pa-pb).magnitude() - mindist_global).abs() < f32::EPSILON);
            return (n.normalize(), mindist, a.to_local(pa), b.to_local(pb));
        } else {
            mindist_global = mindist;
            closest_global = closest_triangle;
        }

        // expand
        let face = polytope.faces()[closest_triangle];
        let triangle = [
            &polytope.vertices()[face[0]],
            &polytope.vertices()[face[1]],
            &polytope.vertices()[face[2]],
        ];

        let n = (triangle[1] - triangle[0]).cross(triangle[2] - triangle[0]);

        polytope.expand(support(&a, &b, n), n);
        // println!("{}", polytope);
    }
}

#[cfg(test)]
mod gjk_test {
    use super::*;
    use crate::body::*;
    use crate::collision::chull::*;
    use crate::collision::collider::*;
    use crate::particle::*;
    use std::cell::RefCell;
    use std::rc::Rc;
    use three_d::*;

    #[test]
    fn spherevssphere() {
        let b1 =
            Rc::new(RefCell::new(Particle::new(vec3(0.0, 0.0, 0.0), 1.0))) as Rc<RefCell<dyn Body>>;
        let b2 =
            Rc::new(RefCell::new(Particle::new(vec3(1.0, 0.0, 0.0), 1.0))) as Rc<RefCell<dyn Body>>;

        let c1: Box<dyn Collider> = Box::new(SphereCollider::new(&b1, 0.25));
        let c2: Box<dyn Collider> = Box::new(SphereCollider::new(&b2, 0.25));

        assert!(gjk(&c1, &c2).is_none());

        b1.as_ref().borrow_mut().update_pos(vec3(0.5, 0.0, 0.0));

        assert!(gjk(&c1, &c2).is_some());
    }

    #[test]
    fn triangle_proj() {
        let triangle = [
            &Point3::new(1.0, 0.0, 0.0),
            &Point3::new(0.0, 1.0, 0.0),
            &Point3::new(-1.0, 0.0, 0.0),
        ];

        for (p, r) in [
            (Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)),
            (Point3::new(0.0, 0.5, 1.0), Point3::new(0.0, 0.5, 0.0)),
            (Point3::new(10.0, 0.5, 10.0), Point3::new(1.0, 0.0, 0.0)),
            (Point3::new(1.0, 1.0, 10.0), Point3::new(0.5, 0.5, 0.0)),
            (Point3::new(0.0, -1.0, 10.0), Point3::new(0.0, 0.0, 0.0)),
            (Point3::new(1.0, -1.0, 10.0), Point3::new(1.0, 0.0, 0.0)),
            (Point3::new(-1.0, 1.0, 10.0), Point3::new(-0.5, 0.5, 0.0)),
            (Point3::new(0.5, 0.5, 3.0), Point3::new(0.5, 0.5, 0.0)),
        ] {
            let t = triangle_point_proj(triangle, &p);
            println!("input        {} {} {}", p.x, p.y, p.z);
            println!("ground truth {} {} {}", r.x, r.y, r.z);
            println!("output       {} {} {}", t.x, t.y, t.z);
            assert!(t.distance(r) < f32::EPSILON);
        }
    }
}
