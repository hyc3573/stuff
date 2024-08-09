extern crate stuff_lib;
use stuff_lib::collision::chull::cube_chull;
use three_d::*;
use stuff_lib::collision::gjk::*;
use stuff_lib::collision::collider::*;
use stuff_lib::body::*;
use stuff_lib::cube::*;
use stuff_lib::inertiatensor::*;
use std::ops::Deref;
use std::rc::Rc;
use std::cell::RefCell;

const VEL: f32 = 0.01;

fn main() {
    let window = Window::new(WindowSettings {
        title: "F".to_string(),
        max_size: Some((1920, 1080)),
        ..Default::default()
    }).unwrap();

    let context = window.gl();

    let mut camera = Camera::new_perspective(
        window.viewport(),
        vec3(5.0, 2.0, 2.5),
        vec3(0.0, 0.0, -0.5),
        vec3(0.0, 1.0, 0.0),
        degrees(45.0),
        0.1,
        1000.0,
    );
    let mut control = OrbitControl::new(*camera.target(), 1.0, 100.0);

    let mut cube1 = Gm::new(
        Mesh::new(&context, &CpuMesh::cube()),
        PhysicalMaterial::new_transparent(
            &context,
            &CpuMaterial {
                albedo: Srgba {
                    r: 0,
                    g: 0,
                    b: 255,
                    a: 70,
                },
                ..Default::default()
            },
        ),
    );
    cube1.set_transformation(Mat4::from_translation(vec3(0.0, 0.0, 0.0)) * Mat4::from_scale(0.5));

    let mut cube2 = Gm::new(
        Mesh::new(&context, &CpuMesh::cube()),
        PhysicalMaterial::new_transparent(
            &context,
            &CpuMaterial {
                albedo: Srgba {
                    r: 0,
                    g: 0,
                    b: 255,
                    a: 70,
                },
                ..Default::default()
            },
        ),
    );
    cube2.set_transformation(Mat4::from_translation(vec3(0.0, 0.0, 1.3)) * Mat4::from_scale(0.5));

    let mut point1 = Gm::new(
        Mesh::new(&context, &CpuMesh::sphere(16)),
        PhysicalMaterial::new_transparent(
            &context,
            &CpuMaterial {
                albedo: Srgba {
                    r: 255,
                    g: 0,
                    b: 0,
                    a: 0
                },
                ..Default::default()
            }
        )
    );
    let mut point1 = Gm::new(
        Mesh::new(&context, &CpuMesh::sphere(16)),
        PhysicalMaterial::new_transparent(
            &context,
            &CpuMaterial {
                albedo: Srgba {
                    r: 255,
                    g: 0,
                    b: 0,
                    a: 200
                },
                ..Default::default()
            }
        )
    );
    let mut point2 = Gm::new(
        Mesh::new(&context, &CpuMesh::sphere(16)),
        PhysicalMaterial::new_transparent(
            &context,
            &CpuMaterial {
                albedo: Srgba {
                    r: 0,
                    g: 255,
                    b: 0,
                    a: 200
                },
                ..Default::default()
            }
        )
    );

    let light0 = DirectionalLight::new(&context, 1.0, Srgba::WHITE, &vec3(0.0, -0.5, -0.5));
    let light1 = DirectionalLight::new(&context, 1.0, Srgba::WHITE, &vec3(0.0, 0.5, 0.5));

    let c1b: Rc<RefCell<dyn Body>> = Rc::new(RefCell::new(RigidBody::new(
        vec3(1.2486387, -0.065917, 0.12459592),
        Quat::from_sv(
            0.43294528,
            vec3(0.60419583, 0.17948799, -0.6444299)
        ),
        1.0,
        cubeinertia_mass(1.0)
    )));
    let c2b: Rc<RefCell<dyn Body>> = Rc::new(RefCell::new(RigidBody::new(
        vec3(0., -0.0, 0.),
        Quat::one(),
        1.0,
        cubeinertia_mass(1.0)
    )));

    let c1c: Box<dyn Collider> = Box::new(CHullCollider::new(
        &c1b,
        cube_chull(1.0)
    ));
    let c2c: Box<dyn Collider> = Box::new(CHullCollider::new(
        &c2b,
        cube_chull(1.0)
    ));

    window.render_loop(move |mut frame_input| {
        camera.set_viewport(frame_input.viewport);
        control.handle_events(&mut camera, &mut frame_input.events);

        for event in &frame_input.events {
            match event {
                Event::KeyPress { kind, .. } => {
                    match kind {
                        Key::W => {
                            c1b.as_ref().borrow_mut().update_pos(vec3(0.0, VEL, 0.0));
                        }
                        Key::A => {
                            c1b.as_ref().borrow_mut().update_pos(vec3(-VEL, 0.0, 0.0));
                        }
                        Key::S => {
                            c1b.as_ref().borrow_mut().update_pos(vec3(0.0, -VEL, 0.0));
                        }
                        Key::D => {
                            c1b.as_ref().borrow_mut().update_pos(vec3(VEL, 0.0, 0.0));
                        }
                        Key::Q => {
                            c1b.as_ref().borrow_mut().update_pos(vec3(0.0, 0.0, VEL));
                        }
                        Key::E => {
                            c1b.as_ref().borrow_mut().update_pos(vec3(0.0, 0.0, -VEL));
                        }
                        _ => {}
                    }
                }
                _ => {}
            }
        }

        c1b.as_ref().borrow_mut().iterate();

        cube1.set_transformation(Mat4::from_translation(
            c1b.as_ref().borrow().pos()
        ) * Mat4::from_scale(0.5) * Mat4::from(c1b.as_ref().borrow().apos()));
        cube2.set_transformation(Mat4::from_translation(
            c2b.as_ref().borrow().pos()
        ) * Mat4::from_scale(0.5) * Mat4::from(c2b.as_ref().borrow().apos()));

        let p1 = c1b.as_ref().borrow().pos();
        let p2 = c2b.as_ref().borrow().pos();
        // println!("{} {} {} / {} {} {}", p1.x, p1.y, p1.z, p2.x, p2.y, p2.z);

        let gjkres = gjk(&c1c, &c2c, true);
        let mut draw_points = false;
        if let Some(simplex) = gjkres {
            draw_points = true;
            
            let (normal, depth, va, vb) = epa(&c1c, &c2c, simplex);

            let pa = c1b.as_ref().borrow().pos_at(va);
            let pb = c2b.as_ref().borrow().pos_at(vb);

            point1.set_transformation(
                Mat4::from_translation(
                    pa
                )*Mat4::from_scale(0.01)
            );
            point2.set_transformation(
                Mat4::from_translation(
                    pb
                )*Mat4::from_scale(0.01)
            )
        }
        
        if !draw_points {
            frame_input
                .screen()
                .clear(ClearState::color_and_depth(0.8, 0.8, 0.8, 1.0, 1.0))
                .render(
                    &camera,
                    [&cube1, &cube2].into_iter(),
                    &[&light0, &light1],
                );
        } else {
            frame_input
                .screen()
                .clear(ClearState::color_and_depth(0.8, 0.8, 0.8, 1.0, 1.0))
                .render(
                    &camera,
                    [&cube1, &cube2, &point1, &point2].into_iter(),
                    &[&light0, &light1],
                );
        }
        FrameOutput::default()
    });

}
