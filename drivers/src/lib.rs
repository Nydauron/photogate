#![cfg_attr(not(test), no_std)]
#![feature(lint_reasons)]
#![allow(
    incomplete_features,
    reason = "Drivers use constant generics to calculate the needed buffer sizes"
)]
#![feature(generic_const_exprs)]

#[cfg(test)]
extern crate std;

extern crate alloc;

pub mod display;
