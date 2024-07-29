// SPDX-License-Identifier: GPL-2.0

//! Common clock framework.
//!
//! C header: [`include/linux/clk.h`](../../../../include/linux/clk-provider.h)

use crate::{
    bindings,
    error::{from_result, to_result, Result},
    str::CStr,
    types::{ForeignOwnable, Opaque},
};
use alloc::vec::Vec;
use core::{marker::PhantomData, mem::MaybeUninit};
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
    pub fn from_raw<'a>(ptr: *mut bindings::clk_hw) -> &'a Self {
        let ptr = ptr.cast::<Self>();
        unsafe { &*ptr }
    }

    /// Returns a raw pointer to the inner C struct.
    #[inline]
    pub fn as_ptr(&self) -> *mut bindings::clk_hw {
        self.0.get()
    }

    pub fn name(&self) -> &CStr {
        // SAFETY: if clk_hw is valid, name is valid. name must be UTF-8 string.
        unsafe { CStr::from_char_ptr(bindings::clk_hw_get_name(self.0.get())) }
    }
    // Register one clock lookup for a struct clk_hw
    pub fn register_clkdev(&mut self, con_id: &'static CStr, dev_id: &'static CStr) -> Result {
        let ret = unsafe {
            bindings::clk_hw_register_clkdev(
                self.0.get(),
                con_id.as_char_ptr(),
                dev_id.as_char_ptr(),
            )
        };
        to_result(ret)
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

pub struct ClkInitData(bindings::clk_init_data);

impl ClkInitData {
    /// Create a new ClkInitData
    pub fn new() -> Self {
        let up = unsafe { MaybeUninit::<bindings::clk_init_data>::zeroed().assume_init() };
        Self(up)
    }

    /// Set the name config of the clk_init_data
    ///
    /// It will automatically set the num_parents to the length of parent_names.
    pub fn name_config(mut self, name: &CStr, parent_names: Vec<&'static CStr>) -> Self {
        self.0.name = name.as_char_ptr();
        self.0.num_parents = parent_names.len() as u8;
        self.0.parent_names = {
            let mut vec: Vec<*const i8> = Vec::with_capacity(parent_names.len());
            for s in parent_names {
                vec.try_push(s.as_char_ptr());
            }
            vec
        };
        self
    }

    pub fn ops<T>(mut self, ops: T) -> Self
    where
        T: ClkOps,
    {
        let ops = Adapter::<T>::build();
        self.0.ops = ops as *const bindings::clk_ops;
        self
    }

    pub fn flags(mut self, flags: u64) -> Self {
        self.0.flags = flags;
        self
    }
    /// Create ClkInitData from raw ptr
    ///
    /// # Safety
    ///
    /// The pointer must be valid.
    pub unsafe fn from_raw(ptr: *const bindings::clk_init_data) -> Self {
        let ptr = ptr.cast::<Self>();
        *ptr
    }
}

/// Trait about a clock device's operations
#[vtable]
pub trait ClkOps {
    /// User data that will be accessible to all operations
    type Data: ForeignOwnable + Send + Sync = ();

    /// Prepare the clock for enabling.
    fn prepare(hw: &ClkHw) -> i32;
    /// Release the clock from its prepared state.
    fn unprepare(hw: &ClkHw);
    /// Queries the hardware to determine if the clock is prepared.
    fn is_prepared(hw: &ClkHw) -> i32;
    /// Unprepare the clock atomically
    fn unprepare_unused(hw: &ClkHw);
    /// Enable the clock atomically
    fn enable(hw: &ClkHw) -> i32;
    /// Disable the clock atomically
    fn disable(hw: &ClkHw);
    /// Queries the hardware to determine if the clock is enabled
    fn is_enabled(hw: &ClkHw) -> i32;
    /// Disable the clock atomically
    fn disable_unused(hw: &ClkHw);
    /// Save the context of the clock in prepration for poweroff
    fn save_context(hw: &ClkHw) -> i32;
    /// Restore the context of the clock after a restoration of power
    fn restore_context(hw: &ClkHw);
    /// Recalculate the rate of this clock, by querying hardware
    fn recalc_rate(hw: &ClkHw, parent_rate: u64) -> u64;
    /// Given a target rate as input, returns the closest rate actually supported by the clock
    fn round_rate(hw: &ClkHw, rate: u64, parent_rate: &mut u64) -> i64;
    /// Given a target rate as input, returns the closest rate actually supported by the clock, and optionally the
    /// parent clock that should be used to provide the clock rate.
    fn determine_rate(hw: &ClkHw, req: *mut bindings::clk_rate_request) -> i32;
    /// Change the input source of this clock
    /// Returns 0 on success, -EERROR otherwise.
    fn set_parent(hw: &ClkHw, index: u8) -> Result;
    /// Queries the hardware to determine the parent of a clock
    fn get_parent(hw: &ClkHw) -> u8;
    /// Change the rate of this clock
    /// Returns 0 on success, -EERROR otherwise.
    fn set_rate(hw: &ClkHw, rate: u64, parent_rate: u64) -> Result;
    /// Change the rate and the parent of this clock
    /// Returns 0 on success, -EERROR otherwise.
    fn set_rate_and_parent(hw: &ClkHw, rate: u64, parent_rate: u64, index: u8) -> Result;
    /// Recalculate the accuracy of this clock
    fn recalc_accuracy(hw: &ClkHw, parent_accuracy: u64) -> u64;
    /// Queries the hardware to get the current phase of a clock
    /// error codes on failure.
    fn get_phase(hw: &ClkHw) -> i32;
    /// Shift the phase this clock signal in degrees specified by the second argument
    /// Returns 0 on success, -EERROR otherwise.
    fn set_phase(hw: &ClkHw, degrees: i32) -> Result;
    /// Queries the hardware to get the current duty cycle ratio of a clock
    fn get_duty_cycle(hw: &ClkHw, duty: *mut bindings::clk_duty) -> i32;
    /// Apply the duty cycle ratio to this clock signal specified by the numerator (2nd argurment) and denominator (3rd  argument)
    /// Returns 0 on success, -EERROR otherwise.
    fn set_duty_cycle(hw: &ClkHw, duty: *mut bindings::clk_duty) -> Result;
    /// Perform platform-specific initialization magic
    /// Returns 0 on success, -EERROR otherwise.
    fn init(hw: &ClkHw) -> Result;
    /// Free any resource allocated by init
    fn terminate(hw: &ClkHw);
    /// Set up type-specific debugfs entries for this clock
    /// Returns 0 on success, -EERROR otherwise.
    fn debug_init(hw: &ClkHw, dentry: *mut bindings::dentry);
}

pub(crate) struct Adapter<T: ClkOps>(PhantomData<T>);

impl<T: ClkOps> Adapter<T> {
    unsafe extern "C" fn prepare_callback(clk_hw: *mut bindings::clk_hw) -> core::ffi::c_int {
        let hw = unsafe { ClkHw::from_raw(clk_hw) };
        T::prepare(&hw)
    }

    unsafe extern "C" fn unprepare_callback(clk_hw: *mut bindings::clk_hw) {
        let hw = unsafe { ClkHw::from_raw(clk_hw) };
        T::unprepare(&hw);
    }

    unsafe extern "C" fn is_prepared_callback(clk_hw: *mut bindings::clk_hw) -> core::ffi::c_int {
        let hw = unsafe { ClkHw::from_raw(clk_hw) };
        T::is_prepared(&hw)
    }

    unsafe extern "C" fn unprepare_unused_callback(clk_hw: *mut bindings::clk_hw) {
        let hw = unsafe { ClkHw::from_raw(clk_hw) };
        T::unprepare_unused(&hw);
    }

    unsafe extern "C" fn enable_callback(clk_hw: *mut bindings::clk_hw) -> core::ffi::c_int {
        let hw = unsafe { ClkHw::from_raw(clk_hw) };
        T::enable(&hw)
    }

    unsafe extern "C" fn disable_callback(clk_hw: *mut bindings::clk_hw) {
        let hw = unsafe { ClkHw::from_raw(clk_hw) };
        T::disable(&hw);
    }

    unsafe extern "C" fn is_enabled_callback(clk_hw: *mut bindings::clk_hw) -> core::ffi::c_int {
        let hw = unsafe { ClkHw::from_raw(clk_hw) };
        T::is_enabled(&hw)
    }

    unsafe extern "C" fn disable_unused_callback(clk_hw: *mut bindings::clk_hw) {
        let hw = unsafe { ClkHw::from_raw(clk_hw) };
        T::disable_unused(&hw);
    }

    unsafe extern "C" fn save_context_callback(clk_hw: *mut bindings::clk_hw) -> core::ffi::c_int {
        let hw = unsafe { ClkHw::from_raw(clk_hw) };
        T::save_context(&hw)
    }

    unsafe extern "C" fn restore_context_callback(clk_hw: *mut bindings::clk_hw) {
        let hw = unsafe { ClkHw::from_raw(clk_hw) };
        T::restore_context(&hw);
    }

    unsafe extern "C" fn recalc_rate_callback(
        clk_hw: *mut bindings::clk_hw,
        parent_rate: u64,
    ) -> core::ffi::c_ulong {
        let hw = unsafe { ClkHw::from_raw(clk_hw) };
        T::recalc_rate(&hw, parent_rate)
    }

    unsafe extern "C" fn round_rate_callback(
        clk_hw: *mut bindings::clk_hw,
        rate: u64,
        parent_rate: *mut u64,
    ) -> core::ffi::c_long {
        let hw = unsafe { ClkHw::from_raw(clk_hw) };
        T::round_rate(&hw, rate, unsafe { &mut *parent_rate })
    }

    unsafe extern "C" fn determine_rate_callback(
        clk_hw: *mut bindings::clk_hw,
        req: *mut bindings::clk_rate_request,
    ) -> core::ffi::c_int {
        let hw = unsafe { ClkHw::from_raw(clk_hw) };
        T::determine_rate(&hw, req)
    }

    unsafe extern "C" fn set_parent_callback(
        clk_hw: *mut bindings::clk_hw,
        index: u8,
    ) -> core::ffi::c_int {
        from_result(|| {
            let hw = unsafe { ClkHw::from_raw(clk_hw) };
            T::set_parent(&hw, index).and(Ok(0))
        })
    }

    unsafe extern "C" fn get_parent_callback(clk_hw: *mut bindings::clk_hw) -> bindings::u8_ {
        let hw = unsafe { ClkHw::from_raw(clk_hw) };
        T::get_parent(&hw)
    }

    unsafe extern "C" fn set_rate_callback(
        clk_hw: *mut bindings::clk_hw,
        rate: u64,
        parent_rate: u64,
    ) -> core::ffi::c_int {
        from_result(|| {
            let hw = unsafe { ClkHw::from_raw(clk_hw) };
            T::set_rate(&hw, rate, parent_rate).and(Ok(0))
        })
    }

    unsafe extern "C" fn set_rate_and_parent_callback(
        clk_hw: *mut bindings::clk_hw,
        rate: u64,
        parent_rate: u64,
        index: u8,
    ) -> core::ffi::c_int {
        from_result(|| {
            let hw = unsafe { ClkHw::from_raw(clk_hw) };
            T::set_rate_and_parent(&hw, rate, parent_rate, index).and(Ok(0))
        })
    }

    unsafe extern "C" fn recalc_accuracy_callback(
        clk_hw: *mut bindings::clk_hw,
        parent_accuracy: u64,
    ) -> core::ffi::c_ulong {
        let hw = unsafe { ClkHw::from_raw(clk_hw) };
        T::recalc_accuracy(&hw, parent_accuracy)
    }

    unsafe extern "C" fn get_phase_callback(clk_hw: *mut bindings::clk_hw) -> core::ffi::c_int {
        let hw = unsafe { ClkHw::from_raw(clk_hw) };
        T::get_phase(&hw)
    }

    unsafe extern "C" fn set_phase_callback(
        clk_hw: *mut bindings::clk_hw,
        degrees: i32,
    ) -> core::ffi::c_int {
        from_result(|| {
            let hw = unsafe { ClkHw::from_raw(clk_hw) };
            T::set_phase(&hw, degrees).and(Ok(0))
        })
    }

    unsafe extern "C" fn get_duty_cycle_callback(
        clk_hw: *mut bindings::clk_hw,
        duty: *mut bindings::clk_duty,
    ) -> core::ffi::c_int {
        let hw = unsafe { ClkHw::from_raw(clk_hw) };
        T::get_duty_cycle(&hw, duty)
    }

    unsafe extern "C" fn set_duty_cycle_callback(
        clk_hw: *mut bindings::clk_hw,
        duty: *mut bindings::clk_duty,
    ) -> core::ffi::c_int {
        from_result(|| {
            let hw = unsafe { ClkHw::from_raw(clk_hw) };
            T::set_duty_cycle(&hw, duty).and(Ok(0))
        })
    }

    unsafe extern "C" fn init_callback(clk_hw: *mut bindings::clk_hw) -> core::ffi::c_int {
        from_result(|| {
            let hw = unsafe { ClkHw::from_raw(clk_hw) };
            T::init(&hw).and(Ok(0))
        })
    }

    unsafe extern "C" fn terminate_callback(clk_hw: *mut bindings::clk_hw) {
        let hw = unsafe { ClkHw::from_raw(clk_hw) };
        T::terminate(&hw);
    }

    unsafe extern "C" fn debug_init_callback(
        clk_hw: *mut bindings::clk_hw,
        dentry: *mut bindings::dentry,
    ) {
        let hw = unsafe { ClkHw::from_raw(clk_hw) };
        T::debug_init(&hw, dentry);
    }

    const VTABLE: bindings::clk_ops = bindings::clk_ops {
        prepare: Some(Adapter::<T>::prepare_callback),
        unprepare: Some(Adapter::<T>::unprepare_callback),
        is_prepared: Some(Adapter::<T>::is_prepared_callback),
        unprepare_unused: Some(Adapter::<T>::unprepare_unused_callback),
        enable: Some(Adapter::<T>::enable_callback),
        disable: Some(Adapter::<T>::disable_callback),
        is_enabled: Some(Adapter::<T>::is_enabled_callback),
        disable_unused: Some(Adapter::<T>::disable_unused_callback),
        save_context: Some(Adapter::<T>::save_context_callback),
        restore_context: Some(Adapter::<T>::restore_context_callback),
        recalc_rate: Some(Adapter::<T>::recalc_rate_callback),
        round_rate: Some(Adapter::<T>::round_rate_callback),
        determine_rate: Some(Adapter::<T>::determine_rate_callback),
        set_parent: Some(Adapter::<T>::set_parent_callback),
        get_parent: Some(Adapter::<T>::get_parent_callback),
        set_rate: Some(Adapter::<T>::set_rate_callback),
        set_rate_and_parent: Some(Adapter::<T>::set_rate_and_parent_callback),
        recalc_accuracy: Some(Adapter::<T>::recalc_accuracy_callback),
        get_phase: Some(Adapter::<T>::get_phase_callback),
        set_phase: Some(Adapter::<T>::set_phase_callback),
        get_duty_cycle: Some(Adapter::<T>::get_duty_cycle_callback),
        set_duty_cycle: Some(Adapter::<T>::set_duty_cycle_callback),
        init: Some(Adapter::<T>::init_callback),
        terminate: Some(Adapter::<T>::terminate_callback),
        debug_init: Some(Adapter::<T>::debug_init_callback),
    };

    const fn build() -> &'static bindings::clk_ops {
        &Self::VTABLE
    }
}
