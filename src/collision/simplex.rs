use three_d::*;

#[derive(Debug, Clone, Copy)]
pub enum Simplex {
    Point(Point3<f32>, Vec3),
    Line(Point3<f32>, Point3<f32>, Vec3, Vec3),
    Triangle(Point3<f32>, Point3<f32>, Point3<f32>, Vec3, Vec3, Vec3),
    Tetrahedron(Point3<f32>, Point3<f32>, Point3<f32>, Point3<f32>, Vec3, Vec3, Vec3, Vec3)
}

impl Simplex {
    pub fn is_dup(&self) -> bool {
        match self {
            Self::Point(..) => {false},
            Self::Line(a, b, ..) => {a == b},
            Self::Triangle(a, b, c, ..) => {
                b == c || c == a
            }
            Self::Tetrahedron(a, b, c, d, ..) => {
                a == d ||
                    b == d ||
                    c == d
            }
        }
    }
}
