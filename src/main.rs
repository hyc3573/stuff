#[macro_use]
mod config;
#[macro_use]
mod body;
#[macro_use]
mod constraint;
mod particle_constraint;
mod cube;
mod particle;
mod physics;

extern crate nannou;
use constraint::Constraint;
use nannou::glam::Vec3;
use nannou::glam::Vec3Swizzles;
use nannou::prelude::*;
use crate::config::*;
use crate::particle::*;
use crate::particle_constraint::*;
use std::borrow::BorrowMut;
use std::cell::RefCell;
use std::rc::Rc;
use std::vec::Vec;
use crate::particle_constraint::*;
use crate::body::Body;
use physics::*;

fn main() {
    nannou::app(model)
        .update(update)
        .event(event)
        .simple_window(view)
        .run();
}

struct Model {
    physics: Physics
}

fn model(app: &App) -> Model {
    let particles = vec![
        Rc::new(RefCell::new(Particle::new(Vecn::new(0.0, 0.0, 0.0), 1.0))),
        Rc::new(RefCell::new(Particle::new(Vecn::new(30.0, 40.0, 0.0), 1.0))),
        // Rc::new(RefCell::new(Particle::new(Vecn::new(60.0, 80.0), 1.0))),
    ];
    let mut model = Model {
        physics: Physics::new()
    };

    model.physics.add_body(
        Particle::new(Vecn::new(0.0, 0.0, 0.0), 1.0)
    );
    model.physics.add_single_constraint(
        ParticleFix::new(
            [model.physics.bodies()[0].clone()], Vecn::new(0.0, 0.0, 0.0), 0.0
        )
    );

    model
}


fn event(app: &App, model: &mut Model, event: Event) {
    match event {
        Event::WindowEvent { simple: Some(wev) , .. } => {
            match wev {
                KeyPressed(k) => match k {
                    Key::A => {
                        println!("asdf");
                    }
                    _ => {}
                }
                MousePressed(MouseButton::Left) => {
                    // model.temp_single_part_const.push(
                    //     Box::new(ParticleFix::new(
                    //         [model.particles[2].clone()], Vecn::new(app.mouse.x, app.mouse.y), 10.0
                    //     ))
                    // )
                    let pos1 = Vecn::new(app.mouse.x as Real, app.mouse.y as Real, 0.0);
                    model.physics.add_body(
                        Particle::new(
                            pos1,
                            1.0
                        )
                    );
                    let len = model.physics.bodies().len();
                    let pos2 = model.physics.bodies()[len-2].as_ref().borrow().pos();
                    model.physics.add_double_constraint(
                        ParticleDist::new(
                            [
                                model.physics.bodies()[len-1].clone(),
                                model.physics.bodies()[len-2].clone()
                            ],
                            (pos1 - pos2).length(),
                            0.0
                        )
                    )
                }
                _ => {}
            }
            
        }
        _ => {}
    }
}


fn update(app: &App, model: &mut Model, update: Update) {
    let dt = update.since_last.as_secs_f64() as Real;

    if app.mouse.buttons.right().is_down() {
        let pos = Vec3::new(app.mouse.x as Real, app.mouse.y as Real, 0.0);
        let len = model.physics.bodies().len();

        model.physics.add_temp_single_constraint(
            ParticleFix::new(
                [model.physics.bodies()[len-1].clone()],
                pos,
                0.001
            )
        )
    }

    model.physics.update(dt);
}

fn view(app: &App, model: &Model, frame: Frame){
    let draw = app.draw();

    draw.background().color(BLACK);

    for particle in &model.physics.bodies() {
        draw.ellipse().color(RED).radius(5.0).xy(
            particle.as_ref().borrow_mut().pos().xy()
        );
    }

    draw.to_frame(app, &frame).unwrap();
}
