use pyo3::prelude::*;
use space_robotics_bench::utils::sampling::{
    sample_poisson_disk_2d, sample_poisson_disk_2d_looped, sample_poisson_disk_3d,
    sample_poisson_disk_3d_looped,
};

pub(crate) fn register(parent: &Bound<'_, PyModule>) -> PyResult<()> {
    let m = PyModule::new_bound(parent.py(), crate::macros::python_module_name!())?;

    m.add_function(wrap_pyfunction!(sample_poisson_disk_2d, &m)?)?;
    m.add_function(wrap_pyfunction!(sample_poisson_disk_2d_looped, &m)?)?;
    m.add_function(wrap_pyfunction!(sample_poisson_disk_3d, &m)?)?;
    m.add_function(wrap_pyfunction!(sample_poisson_disk_3d_looped, &m)?)?;

    crate::macros::python_add_submodule!(parent, m)
}
