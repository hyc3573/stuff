extern crate stuff_lib;
use three_d::*;

use stuff_lib::body::Body;
use stuff_lib::config::*;
use stuff_lib::particle::*;
use stuff_lib::particle_constraint::*;
use stuff_lib::collision::chull::cube_polyhedra;
use stuff_lib::collision::collider::*;
use stuff_lib::constraint::Constraint;
use stuff_lib::body::*;
use stuff_lib::cube::*;
use stuff_lib::physics::*;
use stuff_lib::rigidbody_constraint::*;
use std::cell::RefCell;
use std::rc::Rc;
use std::time::Instant;
use stuff_lib::inertiatensor::*;

const CUBES: usize = 5;

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

    let light0 = DirectionalLight::new(&context, 1.0, Srgba::WHITE, &vec3(0.0, -0.5, -0.5));
    let light1 = DirectionalLight::new(&context, 1.0, Srgba::WHITE, &vec3(0.0, 0.5, 0.5));

    let mut base_cube = Gm::new(
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

    let mut cubes = Vec::new();
    for i in 0..CUBES {
        cubes.push(
            Gm::new(
                Mesh::new(&context, &CpuMesh::cube()),
                PhysicalMaterial::new_opaque(
                    &context,
                    &CpuMaterial {
                        name: "asdf".to_string(),
                        albedo: Srgba::BLUE,
                        ..Default::default()
                    }
                )
            )   
        )
    }

    let mut physics = Physics::new(vec3(0.0, -5.0, 0.0), SUBS, ITER);

    let base_body = physics.add_body(
        RigidBody::new(
            vec3(0.0, -5.0, 0.0),
            Quat::one(),
            0.0/100.0,
            zeroinertia_mass()
        )
    );
    physics.add_collider(
        PolyhedraCollider::new(
            &(base_body.clone() as Rc<RefCell<dyn Body>>),
            cube_polyhedra(9.0)
        )
    );

    let mut bodies = Vec::new();
    for i in 0..CUBES {
        bodies.push(physics.add_body(
            RigidBody::new(
                vec3(0.0, 0.0 + ((i+1) as f32)*2.0, 0.01*i as f32 + 0.00),
                Quat::one(),
                1.0/100.0,
                cubeinertia_mass(1.0)
            )
        ));
        physics.add_collider(
            PolyhedraCollider::new(
                &(bodies[bodies.len() - 1].clone() as Rc<RefCell<dyn Body>>),
                cube_polyhedra(1.0)
            )
        );
    }
    let bodies = bodies;

    let mut dtclock = Instant::now();
    window.render_loop(move |mut frame_input| {
        let dt: f32 = dtclock.elapsed().as_secs_f32();
        dtclock = Instant::now();

        camera.set_viewport(frame_input.viewport);
        control.handle_events(&mut camera, &mut frame_input.events);

        physics.update(dt*1.0);

        base_cube.set_transformation(
            base_body.as_ref().borrow().get_matrix()*Mat4::from_scale(4.5)
        );

        for (i, cube) in cubes.iter_mut().enumerate() {
            cube.set_transformation(
                bodies[i].as_ref().borrow().get_matrix()*Mat4::from_scale(0.5)
            )
        }

        let mut drawlists = vec![&base_cube as &dyn Object];
        for cube in &cubes {
            drawlists.push(cube as &dyn Object)
        }

        frame_input
            .screen()
            .clear(ClearState::color_and_depth(0.8, 0.8, 0.8, 1.0, 1.0))
            .render(
                &camera,
                drawlists.iter(),
                &[&light0, &light1]
            );

        FrameOutput::default()
    })
}
