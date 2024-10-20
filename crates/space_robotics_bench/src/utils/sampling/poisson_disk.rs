use pyo3::prelude::*;
use rand::seq::SliceRandom;
use rand_xoshiro::rand_core::SeedableRng;
use rayon::prelude::*;

const RADIUS_DECAY: f32 = 0.98;

macro_rules! sample_poisson_disk_impl {
    ($dim:expr) => {
        paste::paste! {
            #[pyfunction]
            #[must_use] pub fn [< sample_poisson_disk_ $dim d >](
                num_samples: usize,
                bounds: [[f32; $dim]; 2],
                radius: f32,
            ) -> Vec<[f32; $dim]> {
                let dimensions = {
                    let mut dims = bounds[1];
                    for (i, d) in dims.iter_mut().enumerate() {
                        *d -= bounds[0][i];
                    }
                    dims
                };

                let mut distribution = fast_poisson::Poisson::<$dim>::new().with_dimensions(dimensions, radius);

                let mut decay_factor = RADIUS_DECAY;
                loop {
                    let points = distribution.generate();
                    match points.len() {
                        n if n < num_samples => {
                            distribution.set_dimensions(dimensions, decay_factor * radius);
                            decay_factor *= RADIUS_DECAY;
                            continue;
                        }
                        n if n == num_samples => {
                            break points
                                .into_iter()
                                .map(|mut point| {
                                    for (i, p) in point.iter_mut().enumerate() {
                                        *p += bounds[0][i];
                                    }
                                    point
                                })
                                .collect()
                        }
                        _ => {
                            break points
                                .choose_multiple(
                                    &mut rand_xoshiro::Xoshiro128StarStar::from_entropy(),
                                    num_samples,
                                )
                                .cloned()
                                .map(|mut point| {
                                    for (i, p) in point.iter_mut().enumerate() {
                                        *p += bounds[0][i];
                                    }
                                    point
                                })
                                .collect()
                        }
                    }
                }
            }

            #[pyfunction]
            #[must_use] pub fn [< sample_poisson_disk_ $dim d_looped >](
                num_samples: [usize; 2],
                bounds: [[f32; $dim]; 2],
                radius: f32,
            ) -> Vec<Vec<[f32; $dim]>> {
                (0..num_samples[0])
                    .into_par_iter()
                    .map(|_| [< sample_poisson_disk_ $dim d >](num_samples[1], bounds, radius))
                    .collect()
            }

        }
    };
    [$( $dim:expr ),*] => {
        $( sample_poisson_disk_impl!($dim); )*
    };
}

sample_poisson_disk_impl![2, 3];
