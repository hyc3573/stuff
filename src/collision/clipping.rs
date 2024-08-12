use std::vec::Vec;
use three_d::*;

pub fn clip(
    polygon: &Vec<Vec3>,
    plane_origin: &Vec3,
    plane_normal: &Vec3,
    leave_clipped: bool,
) -> Vec<Vec3> {
    if polygon.is_empty() {
        return Vec::new();
    }

    let mut result: Vec<Vec3> = Vec::new();
    let mut current_index = polygon.len();
    let mut dots: Vec<f32> = Vec::new();
    for (i, point) in polygon.iter().enumerate() {
        let dot = (point - plane_origin).dot(*plane_normal);
        dots.push(dot);
        if dot > 0.0 {
            current_index = i;
        }
    }

    if current_index == polygon.len() {
        return Vec::new();
    }

    if polygon.len() == 1 {
        return polygon.clone();
    }

    let len = polygon.len();
    for i in current_index..(current_index + len) {
        let dot1 = dots[i % len].abs();
        let dot2 = dots[(i + 1) % len].abs();

        if dots[i % len] >= 0.0 && dots[(i + 1) % len] < 0.0 {
            result.push(polygon[i % len]);
            
            if leave_clipped {
                let new_point = (polygon[i % len] * dot2
                    + polygon[(i + 1) % len] * dot1)
                    / (dot1 + dot2);
                result.push(new_point);
            }
        } else if dots[i % len] < 0.0 && dots[(i + 1) % len] >= 0.0 {
            if leave_clipped {
                let new_point = (polygon[i % len] * dot2
                    + polygon[(i + 1) % len] * dot1)
                    / (dot1 + dot2);
                result.push(new_point);
            }
        } else if dots[i % len] >= 0.0 && dots[(i + 1) % len] >= 0.0 {
            result.push(polygon[i % len]);
        }
    }

    result
}
