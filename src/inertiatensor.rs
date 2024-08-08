use three_d::*;

pub fn cubeinertia_mass(sidelen: f32) -> (Mat3, Mat3) {
    let result = sidelen*sidelen*Mat3::from_cols(
        vec3(2.0/3.0, -0.25, -0.25),
        vec3(-0.25, 2.0/3.0, -0.25),
        vec3(-0.25, -0.25, 2.0/3.0)
    );

    (result, result.invert().unwrap())
}

pub fn sphereinertia_mass(radius: f32) -> (Mat3, Mat3) {
    (5.0/2.0*radius*radius*Mat3::one(), 2.0/5.0/radius/radius*Mat3::one())
}

pub fn zeroinertia_mass() -> (Mat3, Mat3) {
    (Mat3::zero(), Mat3::zero())
}
