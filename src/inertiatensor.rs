use three_d::*;

pub fn cubeinertia_mass(sidelen: f32) -> Mat3 {
    sidelen*sidelen*Mat3::from_cols(
        vec3(2.0/3.0, -0.25, -0.25),
        vec3(-0.25, 2.0/3.0, -0.25),
        vec3(-0.25, -0.25, 2.0/3.0)
    )
}

pub fn sphereinertia_mass(radius: f32) -> Mat3 {
    5.0/2.0*radius*radius*Mat3::one()
}
