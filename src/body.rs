use crate::config::*;

pub trait Body {
    fn invmass(&self) -> Real; 
    fn pos(&self) -> Vecn;
    fn vel(&self) -> Vecn;
    fn acc(&self) -> Vecn;

    fn update_pos(&mut self, dx: Vecn);

    fn predict(&mut self, dt: Real);
    fn update(&mut self, dt: Real);
}
