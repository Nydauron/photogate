//! Marker types and traits relating to initialization
//!
//! This module holds marker types and a marker trait for signaling the type system when a type has
//! been or yet to be initialized. This allows for drivers to implement type guards forcing the end
//! users to call initalization functions.

/// A marker type signaling an initalized state
pub struct Initialized;
/// A marker type signaling an uninitalized state
pub struct Uninitalized;

/// A sealed marker trait used for stating which types have been initialized
///
/// It should be noted that since this trait is sealed, no further implementations can be made
/// beyond this crate.
pub trait InitializationState: sealed::Sealed {}

mod sealed {
    use super::{Initialized, Uninitalized};

    pub trait Sealed {}
    impl Sealed for Initialized {}
    impl Sealed for Uninitalized {}
}

impl InitializationState for Initialized {}
impl InitializationState for Uninitalized {}
