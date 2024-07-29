// SPDX-License-Identifier: GPL-2.0

//! Common clock framework.
//!
//! C header: [`include/linux/clk.h`](../../../../include/linux/clk.h)

use crate::{
    bindings,
    error::{to_result, Result},
    str::CStr,
    types::Opaque,
};
use core::mem::ManuallyDrop;

/// Represents `struct clk *`.
///
/// # Invariants
///
/// The pointer is valid.
pub struct Clk(Opaque<bindings::clk>);

impl Clk {
    /// Create Clk from raw ptr
    pub fn from_raw<'a>(ptr: *mut bindings::clk) -> &'a mut Self {
        let ptr = ptr.cast::<Self>();
        unsafe { &mut *ptr }
    }

    /// Returns a raw pointer to the inner C struct.
    #[inline]
    pub fn as_ptr(&self) -> *mut bindings::clk {
        self.0.get()
    }

    /// Get clk rate
    pub fn get_rate(&self) -> u64 {
        // SAFETY: call ffi and ptr is valid
        unsafe { bindings::clk_get_rate(self.0.get()) }
    }

    /// clk enable
    pub fn prepare_enable(&self) -> Result {
        // SAFETY: call ffi and ptr is valid
        unsafe {
            to_result(bindings::clk_prepare(self.0.get()))?;
            let ret = to_result(bindings::clk_enable(self.0.get()));
            if ret.is_err() {
                bindings::clk_unprepare(self.0.get());
                return ret;
            }
        }
        Ok(())
    }

    pub fn name(&self) -> &str {
        // SAFETY: if clk is valid, name is valid. name must be a UTF-8 string.
        unsafe {
            CStr::from_char_ptr(bindings::__clk_get_name(self.0.get()))
                .to_str()
                .unwrap()
        }
    }
}

impl Drop for Clk {
    fn drop(&mut self) {
        // SAFETY: The pointer is valid by the type invariant.
        unsafe { bindings::clk_put(self.0.get()) };
    }
}

// SAFETY: `Clk` is not restricted to a single thread so it is safe
// to move it between threads.
unsafe impl Send for Clk {}

/// A clock variant that is prepared and enabled.
pub struct EnabledClk(Clk);

impl EnabledClk {
    /// Returns value of the rate field of `struct clk`.
    pub fn get_rate(&self) -> u64 {
        self.0.get_rate()
    }

    /// Disables and later unprepares the underlying hardware clock prematurely.
    ///
    /// This function should not be called in atomic context.
    pub fn disable_unprepare(self) -> *mut bindings::clk {
        let clk = ManuallyDrop::new(self);
        // SAFETY: The pointer is valid by the type invariant.
        unsafe { bindings::clk_disable_unprepare(clk.0.as_ptr()) };
        core::mem::replace(&mut clk.0.as_ptr(), core::ptr::null_mut())
    }
}

impl Drop for EnabledClk {
    fn drop(&mut self) {
        // SAFETY: The pointer is valid by the type invariant.a
        unsafe { bindings::clk_disable_unprepare(self.0.as_ptr()) };
    }
}
