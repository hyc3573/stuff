use std::vec::Vec;
use three_d::*;

pub fn cube_polyhedra(sidelen: f32) -> (Vec<Vec3>, Vec<Vec<usize>>) {
    (
        [
            vec3(0.5, 0.5, 0.5),
            vec3(0.5, 0.5, -0.5),
            vec3(0.5, -0.5, 0.5),
            vec3(0.5, -0.5, -0.5),
            vec3(-0.5, 0.5, 0.5),
            vec3(-0.5, 0.5, -0.5),
            vec3(-0.5, -0.5, 0.5),
            vec3(-0.5, -0.5, -0.5),
        ]
        .iter()
        .map(|x| x * sidelen)
        .collect(),
        (
            vec![
                vec![0, 4, 5, 1],
                vec![0, 1, 3, 2],
                vec![1, 5, 7, 3],
                vec![0, 2, 6, 4],
                vec![4, 6, 7, 5],
                vec![2, 3, 7, 6]
            ]
        ),
    )
}
