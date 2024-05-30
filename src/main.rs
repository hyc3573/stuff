#[macro_use]
mod config;
#[macro_use]
mod body;
#[macro_use]
mod constraint;
mod rigidbody_constraint;
mod cube;
mod particle;
mod particle_constraint;
mod inertiatensor;
mod physics;
mod collision;

use crate::body::Body;
use crate::config::*;
use crate::particle::*;
use crate::particle_constraint::*;
use crate::particle_constraint::*;
use collision::chull::cube_chull;
use collision::collider::*;
use collision::gjk::*;
use constraint::Constraint;
use physics::*;
use cube::*;
use body::*;
use inertiatensor::*;
use rigidbody_constraint::RDist;
use std::borrow::BorrowMut;
use std::cell::RefCell;
use std::rc::Rc;
use std::vec::Vec;
use std::time::Instant;
use three_d::*;

fn main() {
    let window = Window::new(WindowSettings {
        title: "XPBD RIGID BODY".to_string(),
        ..Default::default()
    })
    .unwrap();
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
    // let mut control = OrbitControl::new(*camera.target(), 1.0, 100.0);
    let mut control = FirstPersonControl::new(0.01);
    let mut control = OrbitControl::new(
        vec3(0.0, 0.0, 0.0),
        0.2,
        200.0
    );

    let mut sphere1 = Gm::new(
        Mesh::new(&context, &CpuMesh::sphere(16)),
        PhysicalMaterial::new_transparent(
            &context,
            &CpuMaterial {
                albedo: Srgba {
                    r: 255,
                    g: 0,
                    b: 0,
                    a: 200,
                },
                ..Default::default()
            },
        ),
    );

    let mut sphere2 = Gm::new(
        Mesh::new(&context, &CpuMesh::sphere(16)),
        PhysicalMaterial::new_transparent(
            &context,
            &CpuMaterial {
                albedo: Srgba {
                    r: 255,
                    g: 0,
                    b: 0,
                    a: 200,
                },
                ..Default::default()
            },
        ),
    );

    let mut cube1 = Gm::new(
        Mesh::new(&context, &CpuMesh::cube()),
        PhysicalMaterial::new_opaque(
            &context,
            &CpuMaterial {
                name: "asdf".to_string(),
                albedo: Srgba::BLUE,
                ..Default::default()
            }
        )
    );
    let mut cube2 = Gm::new(
        Mesh::new(&context, &CpuMesh::cube()),
        PhysicalMaterial::new_opaque(
            &context,
            &CpuMaterial {
                name: "salfdsalkj".to_string(),
                albedo: Srgba::BLACK,
                ..Default::default()
            }
        )
    );

    let mut line1 = Gm::new(
        Mesh::new(&context, &CpuMesh::cylinder(10)),
        PhysicalMaterial::new_opaque(
            &context,
            &CpuMaterial {
                name: "adjfks".to_string(),
                albedo: Srgba::GREEN,
                ..Default::default()
            }
        )
    );
    let mut line2 = Gm::new(
        Mesh::new(&context, &CpuMesh::cylinder(10)),
        PhysicalMaterial::new_opaque(
            &context,
            &CpuMaterial {
                name: "adjfks".to_string(),
                albedo: Srgba::GREEN,
                ..Default::default()
            }
        )
    );

    let mut xaxis = Gm::new(
        Mesh::new(&context, &CpuMesh::cylinder(10)),
        PhysicalMaterial::new_opaque(
            &context,
            &CpuMaterial {
                name: "adjfks".to_string(),
                albedo: Srgba::RED,
                ..Default::default()
            }
        )
    );
    xaxis.set_transformation(Mat4::from_nonuniform_scale(1.0, 0.05, 0.05));
    let mut yaxis = Gm::new(
        Mesh::new(&context, &CpuMesh::cylinder(10)),
        PhysicalMaterial::new_opaque(
            &context,
            &CpuMaterial {
                name: "adjfks".to_string(),
                albedo: Srgba::GREEN,
                ..Default::default()
            }
        )
    );
    yaxis.set_transformation(
        Mat4::from_axis_angle(Vec3::unit_z(), Rad::turn_div_4()) *
            Mat4::from_nonuniform_scale(1.0, 0.05, 0.05)
    );
    let mut zaxis = Gm::new(
        Mesh::new(&context, &CpuMesh::cylinder(10)),
        PhysicalMaterial::new_opaque(
            &context,
            &CpuMaterial {
                name: "adjfks".to_string(),
                albedo: Srgba::BLUE,
                ..Default::default()
            }
        )
    );
    zaxis.set_transformation(
        Mat4::from_axis_angle(Vec3::unit_y(), Rad::turn_div_4()) *
            Mat4::from_nonuniform_scale(1.0, 0.05, 0.05)
    );

    let light0 = DirectionalLight::new(&context, 1.0, Srgba::WHITE, &vec3(0.0, -0.5, -0.5));
    let light1 = DirectionalLight::new(&context, 1.0, Srgba::WHITE, &vec3(0.0, 0.5, 0.5));

    let mut physics = Physics::new(vec3(0.0, -100.0, 0.0), 1, 1);
    let part1 = physics.add_body(
        Particle::new(vec3(0.0, 0.0, 0.0), 1.0)
    );
    let part2 = physics.add_body(
        Particle::new(vec3(1.5, 0.0, 0.0), 1.0)
    );
    let rigid1 = physics.add_body(
        RigidBody::new(
            vec3(3.0, 0.0, 0.0),
            Quat::one(),
            1.0,
            cubeinertia_mass(
                1.0
            ),
        ),
    );
    let rigid2 = physics.add_body(
        RigidBody::new(
            vec3(0.0, -3.0, 0.0),
            Quat::one(),
            1.0,
            cubeinertia_mass(
                1.0
            )
        )
    );
    physics.add_constraint(
        ParticleDist::new(
            [
                part1.clone(),
                part2.clone()
            ],
            1.5, 0.0
        )
    );
    physics.add_constraint(
        ParticleFix::new(
            [part1.clone()],
            vec3(0.0, 0.0, 0.0), 0.0
        )
    );
    // physics.add_constraint(
    //     ParticleFix::new(
    //         [part2.clone()],
    //         vec3(1.5, 0.0, 0.0), 0.0
    //     )
    // );
    let attach = vec3(-0.5, -0.5, -0.5);
    let dist = (rigid1.as_ref().borrow().pos_at(attach) - part2.as_ref().borrow().pos()).magnitude();
    physics.add_constraint(
        RDist::new(
            [part2.clone(), rigid1.clone()],
            [vec3(0.0, 0.0, 0.0), attach],
            dist, 0.0000
        )
    );
    physics.add_constraint(
        ParticleFix::new(
            [rigid2.clone()],
            rigid2.as_ref().borrow().pos(),
            0.0
        )
    );

    let c1 = CHullCollider::new(
        &(rigid1.clone() as Rc<RefCell<dyn Body>>), cube_chull(1.0)
    );
    physics.add_collider(c1);
    let c2 = CHullCollider::new(
        &(rigid2.clone() as Rc<RefCell<dyn Body>>), cube_chull(1.0)
    );
    physics.add_collider(c2);

    let mut dtclock = Instant::now();

    window.render_loop(move |mut frame_input| {
        let avel = rigid1.as_ref().borrow().avel();
        
        let dt: f32 = dtclock.elapsed().as_secs_f64() as f32;
        dtclock = Instant::now();
        
        camera.set_viewport(frame_input.viewport);
        control.handle_events(&mut camera, &mut frame_input.events);

        physics.update(dt*0.1);

        let pos = physics.bodies()[1].as_ref().borrow().pos();

        sphere1.set_transformation(Mat4::from_translation(
            part1.as_ref().borrow().pos()
        ) * Mat4::from_scale(0.2));
        sphere2.set_transformation(Mat4::from_translation(
            part2.as_ref().borrow().pos()
            // vec3(0.3, 0.4, 0.0)
        ) * Mat4::from_scale(0.2));
        // println!("{} {} {} / {} {} {} / {} {} {}", mat.x.x, mat.x.y, mat.x.z, mat.y.x, mat.y.y, mat.y.z, mat.z.x, mat.z.y, mat.z.z);
        cube1.set_transformation(Mat4::from_translation(
            rigid1.as_ref().borrow().pos()
        ) * Mat4::from_scale(0.5) * Mat4::from(rigid1.as_ref().borrow().apos()));
        cube2.set_transformation(Mat4::from_translation(
            rigid2.as_ref().borrow().pos()
        ) * Mat4::from_scale(0.5) * Mat4::from(rigid2.as_ref().borrow().apos()));

        let pos1 = Point3::origin() + part1.as_ref().borrow().pos();
        let pos2 = Point3::origin() + part2.as_ref().borrow().pos();
        let pos3 = Point3::origin() + rigid1.as_ref().borrow().pos_at(attach);

        let v1 = pos2 - pos1;
        let v2 = pos3 - pos2;

        let v1m = v1.magnitude();
        let v1n = v1.normalize();
        let v1d = Quat::between_vectors(Vec3::unit_x(), v1n);
        let v2m = v2.magnitude();
        let v2n = v2.normalize();
        let v2d = Quat::between_vectors(Vec3::unit_x(), v2n);

        line1.set_transformation(Mat4::from_translation(pos1.to_vec()) * Mat4::from(v1d) * Mat4::from_nonuniform_scale(v1m, 0.1, 0.1));
        line2.set_transformation(Mat4::from_translation(pos2.to_vec()) * Mat4::from(v2d) * Mat4::from_nonuniform_scale(v2m, 0.1, 0.1));

        frame_input
            .screen()
            .clear(ClearState::color_and_depth(0.8, 0.8, 0.8, 1.0, 1.0))
            .render(
                &camera,
                [&sphere1, &sphere2, &cube1, &line1, &line2, &xaxis, &yaxis, &zaxis, &cube2].into_iter(),
                &[&light0, &light1]
            );

        FrameOutput::default()
    });
}

// fn main() {
//     nannou::app(model)
//         .update(update)
//         .event(event)
//         .simple_window(view)
//         .run();
// }

// struct Model {
//     physics: Physics
// }

// fn model(app: &App) -> Model {
//     let particles = vec![
//         Rc::new(RefCell::new(Particle::new(Vecn::new(0.0, 0.0, 0.0), 1.0))),
//         Rc::new(RefCell::new(Particle::new(Vecn::new(30.0, 40.0, 0.0), 1.0))),
//         // Rc::new(RefCell::new(Particle::new(Vecn::new(60.0, 80.0), 1.0))),
//     ];
//     let mut model = Model {
//         physics: Physics::new()
//     };

//     model.physics.add_body(
//         Particle::new(Vecn::new(0.0, 0.0, 0.0), 1.0)
//     );
//     model.physics.add_single_constraint(
//         ParticleFix::new(
//             [model.physics.bodies()[0].clone()], Vecn::new(0.0, 0.0, 0.0), 0.0
//         )
//     );

//     model
// }

// fn event(app: &App, model: &mut Model, event: Event) {
//     match event {
//         Event::WindowEvent { simple: Some(wev) , .. } => {
//             match wev {
//                 KeyPressed(k) => match k {
//                     Key::A => {
//                         println!("asdf");
//                     }
//                     _ => {}
//                 }
//                 MousePressed(MouseButton::Left) => {
//                     // model.temp_single_part_const.push(
//                     //     Box::new(ParticleFix::new(
//                     //         [model.particles[2].clone()], Vecn::new(app.mouse.x, app.mouse.y), 10.0
//                     //     ))
//                     // )
//                     let pos1 = Vecn::new(app.mouse.x as Real, app.mouse.y as Real, 0.0);
//                     model.physics.add_body(
//                         Particle::new(
//                             pos1,
//                             1.0
//                         )
//                     );
//                     let len = model.physics.bodies().len();
//                     let pos2 = model.physics.bodies()[len-2].as_ref().borrow().pos();
//                     model.physics.add_double_constraint(
//                         ParticleDist::new(
//                             [
//                                 model.physics.bodies()[len-1].clone(),
//                                 model.physics.bodies()[len-2].clone()
//                             ],
//                             (pos1 - pos2).length(),
//                             0.0
//                         )
//                     )
//                 }
//                 _ => {}
//             }

//         }
//         _ => {}
//     }
// }

// fn update(app: &App, model: &mut Model, update: Update) {
//     let dt = update.since_last.as_secs_f64() as Real;

//     if app.mouse.buttons.right().is_down() {
//         let pos = Vec3::new(app.mouse.x as Real, app.mouse.y as Real, 0.0);
//         let len = model.physics.bodies().len();

//         model.physics.add_temp_single_constraint(
//             ParticleFix::new(
//                 [model.physics.bodies()[len-1].clone()],
//                 pos,
//                 0.001
//             )
//         )
//     }

//     model.physics.update(dt);
// }

// fn view(app: &App, model: &Model, frame: Frame){
//     let draw = app.draw();

//     draw.background().color(BLACK);

//     for particle in &model.physics.bodies() {
//         draw.ellipse().color(RED).radius(5.0).xy(
//             particle.as_ref().borrow_mut().pos().xy()
//         );
//     }

//     draw.to_frame(app, &frame).unwrap();
// }
