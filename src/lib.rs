//! # OXISLAM
//!
//! A lightweight, `no_std`-compatible SLAM library for planetary surface navigation.
//!
//! ## Feature Flags
//! - `std` (default): Enables standard library support for development
//! - `embedded`: Enables `heapless` + `defmt` for bare-metal targets

// When building without std (embedded), tell the compiler
#![cfg_attr(not(feature = "std"), no_std)]

// Deny unsafe code — safety-critical project
#![deny(unsafe_code)]

// Warn on missing docs for public items
#![warn(missing_docs)]

// ---- Sub-modules ----
pub mod backend;
pub mod sensor;
pub mod utils;
// pub mod frontend;  // Phase 2
// pub mod map;       // Phase 5


// ---- Error type ----

/// All errors that OXISLAM can produce
#[derive(Debug, Clone, PartialEq)]
pub enum OxiError {
    /// A matrix that should be invertible is singular
    SingularMatrix,
    /// Not enough features were detected or tracked
    NotEnoughFeatures,
    /// IMU timestamp is out of order or invalid
    InvalidTimestamp,
    /// A numerical computation produced NaN or infinity
    NumericalInstability,
    /// A fixed-size buffer is full (embedded builds)
    BufferFull,
}

// Implement Display for OxiError so it can be printed
impl core::fmt::Display for OxiError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            OxiError::SingularMatrix => write!(f, "singular matrix encountered"),
            OxiError::NotEnoughFeatures => write!(f, "not enough features"),
            OxiError::InvalidTimestamp => write!(f, "invalid timestamp"),
            OxiError::NumericalInstability => write!(f, "numerical instability (NaN/Inf)"),
            OxiError::BufferFull => write!(f, "fixed-size buffer full"),
        }
    }
}

/// Convenience type alias for Results throughout the library
pub type OxiResult<T> = Result<T, OxiError>;

