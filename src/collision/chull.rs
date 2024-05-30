use three_d::*;
use std::vec::Vec;

pub fn cube_chull(sidelen: f32) -> Vec<Vec3> {
    [
        vec3( 0.5,  0.5,  0.5),
        vec3( 0.5,  0.5, -0.5),
        vec3( 0.5, -0.5,  0.5),
        vec3( 0.5, -0.5, -0.5),
        vec3(-0.5,  0.5,  0.5),
        vec3(-0.5,  0.5, -0.5),
        vec3(-0.5, -0.5,  0.5),
        vec3(-0.5, -0.5, -0.5),
    ].iter().map(|x| {
        x * sidelen
    }).collect()
}
