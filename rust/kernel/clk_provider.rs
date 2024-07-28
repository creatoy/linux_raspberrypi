// SPDX-License-Identifier: GPL-2.0

//! Common clock framework.
//!
//! C header: [`include/linux/clk.h`](../../../../include/linux/clk-provider.h)

use crate::{
    bindings,
    error::{from_result, Result},
    str::CStr,
    types::{ForeignOwnable, Opaque},
};
use macros::vtable;
/// Represents `struct clk_core`
///
/// # Invariants
pub struct ClkCore(Opaque<bindings::clk_core>);

/// Represents `struct clk_rate_request`
pub struct ClkRateRequest(bindings::clk_rate_request);

/// Represents `struct clk_duty`
pub struct ClkDuty(bindings::clk_duty);

/// Represents `struct clk_hw`
///
/// # Invariants
///
/// The pointer is valid.
pub struct ClkHw(Opaque<bindings::clk_hw>);

impl ClkHw {
    /// Create ClkHw from raw ptr
    pub fn from_raw<'a>(ptr: *mut bindings::clk_hw) -> &'a mut Self {
        let ptr = ptr.cast::<Self>();
        unsafe { &mut *ptr }
    }

    /// Returns a raw pointer to the inner C struct.
    #[inline]
    pub fn as_ptr(&self) -> *mut bindings::clk_hw {
        self.0.get()
    }

    // Unsafe: Argumets Wrapped in CStr.
    unsafe fn register_clkdev(&mut self, con_id: &'static CStr, dev_id: &'static CStr) -> i32 {
        unsafe {
            bindings::clk_hw_register_clkdev(
                self.0.get(),
                con_id.as_char_ptr(),
                dev_id.as_char_ptr(),
            )
        }
    }

    // How to implement clk_hw api?
    /*
    pub fn prepare_enable(&mut self) -> Result {
        // SAFETY: call ffi and ptr is valid
        unsafe {
            to_result(bindings::clk_ops::prepare(self.as_ptr()))?;
            let ret = to_result(bindings::clk_ops::enable(self.as_ptr()));
            if ret.is_err() {
                bindings::clk_ops::unprepare(self.as_ptr());
                return ret;
            }
        }
        Ok(())
    }
    */
}

/*
impl Drop for ClkHw {
    fn drop(&mut self) {
        // SAFETY: Type Invariant ptr is valid.
        unsafe {
            bindings::clk_ops::terminate(self.as_ptr());
        }
    }
}
*/

// TODO: Implement clk_hw_register_clkdev
// unsafe fn clk_hw_register_clkdev() -> Result {}
// TODO: Implement devm_clk_hw_register
// unsafe fn devm_clk_hw_register() -> Result {}

/// Represents `struct clk_ops`
pub struct ClkOps(Opaque<bindings::clk_ops>);

// TODO: Create (new) from functions ptr
impl ClkOps {
    // TODO!
    pub fn from_raw(ptr: *const bindings::clk_ops) -> Self {
        // let ptr = ptr.cast::<Self>();
        // unsafe { *ptr }
        unimplemented!()
    }

    #[inline]
    pub fn as_ptr(&self) -> *mut bindings::clk_ops {
        self.0.get()
    }
}

// TODO: Implement All ClkOps methods
#[vtable]
pub trait ClkOpsBase {
    /// User data that will be accessible to all operations
    type Data: ForeignOwnable + Send + Sync = ();

    fn set_rate(_hw: &mut ClkHw, _rate: u32, _parent_rate: u32) -> Result<i32>;

    fn round_rate(_hw: &mut ClkHw, _rate: u32, _round_rate: *mut u32) -> Result<i32>;

    fn recalc_rate(_hw: &mut ClkHw, parent_rate: u32) -> u32;
}

unsafe extern "C" fn set_rate_callback<T: ClkOpsBase>(
    hw: *mut bindings::clk_hw,
    rate: core::ffi::c_ulong,
    parent_rate: core::ffi::c_ulong,
) -> core::ffi::c_int {
    from_result(|| {
        unsafe {
            let hw = ClkHw::from_raw(hw);
        }
        T::set_rate(hw, rate, parent_rate)
    })
}

unsafe extern "C" fn round_rate_callback<T: ClkOpsBase>(
    hw: *mut bindings::clk_hw,
    rate: core::ffi::c_ulong,
    round_rate: *mut core::ffi::c_ulong,
) -> core::ffi::c_int {
    from_result(|| {
        unsafe {
            let hw = ClkHw::from_raw(hw);
        }
        T::round_rate(hw, rate, round_rate)
    })
}

unsafe extern "C" fn recalc_rate_callback<T: ClkOpsBase>(
    hw: *mut bindings::clk_hw,
    parent_rate: core::ffi::c_ulong,
) -> core::ffi::c_ulong {
    unsafe {
        let hw = ClkHw::from_raw(hw);
    }
    T::recalc_rate(hw, parent_rate)
}
