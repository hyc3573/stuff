pub trait TimestepScheduler {
    fn get(&self, i: usize, dt: f32) -> f32;
}

pub struct ExpSchedule {
    multiplier: f32,
    substeps: usize
}

impl ExpSchedule {
    pub fn new(substeps: usize) -> Self {
        Self {
            multiplier: 1.0/((1 << substeps) - 1) as f32,
            substeps
        }
    }
}

impl TimestepScheduler for ExpSchedule {
    fn get(&self, i: usize, dt: f32) -> f32 {
        dt * self.multiplier * ((1 << (self.substeps - i - 1)) as f32)
    }
}

pub struct UniformSchedule {
    substeps: usize
}

impl UniformSchedule {
    pub fn new(substeps: usize) -> Self {
        Self {
            substeps
        }
    }
}

impl TimestepScheduler for UniformSchedule {
    fn get(&self, i: usize, dt: f32) -> f32 {
        dt / self.substeps as f32
    }
}
