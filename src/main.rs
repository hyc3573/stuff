extern crate nannou;
use nannou::prelude::*;
mod particle;
mod config;
mod body;
mod constraint;
mod particle_constraint;
use crate::config::*;

fn main() {
    nannou::app(model)
        .update(update)
        .event(event)
        .simple_window(view)
        .run();
}

struct Model {}

fn model(_app: &App) -> Model {
    Model {}
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


fn update(_app: &App, _model: &mut Model, _update: Update) {
}

fn view(_app: &App, _model: &Model, frame: Frame){
    frame.clear(PURPLE);
}
