use pyo3::prelude::*;
mod sampling;

pub(crate) fn register(parent: &Bound<'_, PyModule>) -> PyResult<()> {
    let m = PyModule::new_bound(parent.py(), crate::macros::python_module_name!())?;

    sampling::register(&m)?;

    crate::macros::python_add_submodule!(parent, m)
}
