extern crate nannou;
use constraint::Constraint;
use nannou::prelude::*;
mod particle;
mod config;
mod body;
mod constraint;
mod particle_constraint;
use crate::config::*;
use crate::particle::*;
use crate::particle_constraint::*;
use std::borrow::BorrowMut;
use std::cell::RefCell;
use std::rc::Rc;
use std::vec::Vec;
use crate::particle_constraint::*;
use crate::body::Body;

fn main() {
    nannou::app(model)
        .update(update)
        .event(event)
        .simple_window(view)
        .run();
}

struct Model {
    particles: Vec<Rc<RefCell<Particle>>>,
    single_part_const: Vec<Box<dyn Constraint<1, Particle>>>,
    double_part_const: Vec<Box<dyn Constraint<2, Particle>>>,
}

fn model(_app: &App) -> Model {
    let particles = vec![
        Rc::new(RefCell::new(Particle::new(Vecn::new(0.0, 0.0), 1.0))),
        Rc::new(RefCell::new(Particle::new(Vecn::new(30.0, 40.0), 1.0))),
        Rc::new(RefCell::new(Particle::new(Vecn::new(60.0, 80.0), 1.0))),
    ];
    Model {
        particles: particles.clone(),
        single_part_const: vec![
            Box::new(ParticleFix::new([particles[0].clone()], Vecn::new(0.0, 0.0), 0.0))
        ],
        double_part_const: vec![
            Box::new(ParticleDist::new([particles[0].clone(), particles[1].clone()], 50.0, 0.0)),
            Box::new(ParticleDist::new([particles[1].clone(), particles[2].clone()], 50.0, 0.0))
        ]
    }
}


fn event(_app: &App, _model: &mut Model, event: Event) {
    match event {
        Event::WindowEvent { simple: Some(wev) , .. } => {
            match wev {
                KeyPressed(k) => match k {
                    Key::A => {
                        println!("asdf");
                    }
                    _ => {}
                }
                _ => {}
            }
            
        }
        _ => {}
    }
}


fn update(_app: &App, model: &mut Model, update: Update) {
    let dt = update.since_last.as_secs_f32();

    for particle in &model.particles {
        particle.as_ref().borrow_mut().add_force(Vecn::new(0.0, -100.0));
        particle.as_ref().borrow_mut().predict(dt);
    }

    for iteration in 0..30 {
        for single in &mut model.single_part_const {
            single.iterate();
        } 
        for double in &mut model.double_part_const {
            double.iterate();
        }

        for particle in &model.particles {
            particle.as_ref().borrow_mut().iterate();
        }
    }

    for particle in &model.particles {
        particle.as_ref().borrow_mut().update(dt);
    }

    let dist = model.double_part_const[0].C();
}

fn view(app: &App, model: &Model, frame: Frame){
    let draw = app.draw();

    draw.background().color(BLACK);

    for particle in &model.particles {
        draw.ellipse().color(RED).radius(5.0).xy(
            particle.as_ref().borrow_mut().pos()
        );
    }

    draw.to_frame(app, &frame).unwrap();
}
