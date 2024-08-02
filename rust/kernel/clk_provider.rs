// SPDX-License-Identifier: GPL-2.0

//! Common clock framework.
//!
//! C header: [`include/linux/clk.h`](../../../../include/linux/clk-provider.h)

use crate::{
    bindings,
    clk::Clk,
    device::{Device, RawDevice},
    error::{from_result, to_result, Result},
    prelude::*,
    str::CStr,
    types::{ForeignOwnable, Opaque},
};
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
    pub unsafe fn from_raw<'a>(ptr: *mut bindings::clk_hw) -> &'a Self {
        let ptr = ptr.cast::<Self>();
        unsafe { &*ptr }
    }

    /// Returns a raw pointer to the inner C struct.
    #[inline]
    pub fn as_mut_ptr(&self) -> *mut bindings::clk_hw {
        self.0.get()
    }

    pub fn name(&self) -> &CStr {
        // SAFETY: if clk_hw is valid, name is valid. name must be UTF-8 string.
        unsafe { CStr::from_char_ptr(bindings::clk_hw_get_name(self.0.get())) }
    }

    pub fn set_init_data(&mut self, init_data: &ClkInitData) {
        // SAFETY: call ffi and ptr is valid
        let hw = unsafe { &mut *self.0.get() };
        hw.init = init_data.as_ptr();
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
    pub fn name_config(mut self, name: &CStr, parent_names: &[*const i8]) -> Self {
        self.0.name = name.as_char_ptr();
        self.0.num_parents = parent_names.len() as u8;
        self.0.parent_names = parent_names.as_ptr();
        self
    }

    pub fn ops<T>(mut self) -> Self
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
    pub unsafe fn from_raw<'a>(ptr: *const bindings::clk_init_data) -> &'a Self {
        let ptr = ptr.cast::<Self>();
        unsafe { &*ptr }
    }

    /// Returns a raw pointer to the inner C struct.
    #[inline]
    pub fn as_ptr(&self) -> *const bindings::clk_init_data {
        &self.0
    }
}

/// Trait about a clock device's operations
#[vtable]
pub trait ClkOps {
    /// User data that will be accessible to all operations
    type Data: ForeignOwnable + Send + Sync = ();

    /// Prepare the clock for enabling.
    fn prepare(hw: &ClkHw) -> i32 {
        0
    }
    /// Release the clock from its prepared state.
    fn unprepare(hw: &ClkHw) {}
    /// Queries the hardware to determine if the clock is prepared.
    fn is_prepared(hw: &ClkHw) -> i32 {
        0
    }
    /// Unprepare the clock atomically
    fn unprepare_unused(hw: &ClkHw) {}
    /// Enable the clock atomically
    fn enable(hw: &ClkHw) -> i32 {
        0
    }
    /// Disable the clock atomically
    fn disable(hw: &ClkHw) {}
    /// Queries the hardware to determine if the clock is enabled
    fn is_enabled(hw: &ClkHw) -> i32 {
        0
    }
    /// Disable the clock atomically
    fn disable_unused(hw: &ClkHw) {}
    /// Save the context of the clock in prepration for poweroff
    fn save_context(hw: &ClkHw) -> i32 {
        0
    }
    /// Restore the context of the clock after a restoration of power
    fn restore_context(hw: &ClkHw) {}
    /// Recalculate the rate of this clock, by querying hardware
    fn recalc_rate(hw: &ClkHw, parent_rate: u64) -> u64 {
        0
    }
    /// Given a target rate as input, returns the closest rate actually supported by the clock
    fn round_rate(hw: &ClkHw, rate: u64, parent_rate: &mut u64) -> i64 {
        0
    }
    /// Given a target rate as input, returns the closest rate actually supported by the clock, and optionally the
    /// parent clock that should be used to provide the clock rate.
    fn determine_rate(hw: &ClkHw, req: *mut bindings::clk_rate_request) -> i32 {
        0
    }
    /// Change the input source of this clock
    /// Returns 0 on success, -EERROR otherwise.
    fn set_parent(hw: &ClkHw, index: u8) -> Result {
        Err(ENOTSUPP)
    }
    /// Queries the hardware to determine the parent of a clock
    fn get_parent(hw: &ClkHw) -> u8 {
        0
    }
    /// Change the rate of this clock
    /// Returns 0 on success, -EERROR otherwise.
    fn set_rate(hw: &ClkHw, rate: u64, parent_rate: u64) -> Result {
        Err(ENOTSUPP)
    }
    /// Change the rate and the parent of this clock
    /// Returns 0 on success, -EERROR otherwise.
    fn set_rate_and_parent(hw: &ClkHw, rate: u64, parent_rate: u64, index: u8) -> Result {
        Err(ENOTSUPP)
    }
    /// Recalculate the accuracy of this clock
    fn recalc_accuracy(hw: &ClkHw, parent_accuracy: u64) -> u64 {
        0
    }
    /// Queries the hardware to get the current phase of a clock
    /// error codes on failure.
    fn get_phase(hw: &ClkHw) -> Result<i32> {
        Err(ENOTSUPP)
    }
    /// Shift the phase this clock signal in degrees specified by the second argument
    /// Returns 0 on success, -EERROR otherwise.
    fn set_phase(hw: &ClkHw, degrees: i32) -> Result {
        Err(ENOTSUPP)
    }
    /// Queries the hardware to get the current duty cycle ratio of a clock
    fn get_duty_cycle(hw: &ClkHw, duty: *mut bindings::clk_duty) -> i32 {
        0
    }
    /// Apply the duty cycle ratio to this clock signal specified by the numerator (2nd argurment) and denominator (3rd  argument)
    /// Returns 0 on success, -EERROR otherwise.
    fn set_duty_cycle(hw: &ClkHw, duty: *mut bindings::clk_duty) -> Result {
        Err(ENOTSUPP)
    }
    /// Perform platform-specific initialization magic
    /// Returns 0 on success, -EERROR otherwise.
    fn init(hw: &ClkHw) -> Result {
        Err(ENOTSUPP)
    }
    /// Free any resource allocated by init
    fn terminate(hw: &ClkHw) {}
    /// Set up type-specific debugfs entries for this clock
    /// Returns 0 on success, -EERROR otherwise.
    fn debug_init(hw: &ClkHw, dentry: *mut bindings::dentry) -> Result {
        Err(ENOTSUPP)
    }
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
        from_result(|| T::get_phase(&hw))
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
        prepare: if T::HAS_PREPARE {
            Some(Adapter::<T>::prepare_callback)
        } else {
            None
        },
        unprepare: if T::HAS_UNPREPARE {
            Some(Adapter::<T>::unprepare_callback)
        } else {
            None
        },
        is_prepared: if T::HAS_IS_PREPARED {
            Some(Adapter::<T>::is_prepared_callback)
        } else {
            None
        },
        unprepare_unused: if T::HAS_UNPREPARE_UNUSED {
            Some(Adapter::<T>::unprepare_unused_callback)
        } else {
            None
        },
        enable: if T::HAS_ENABLE {
            Some(Adapter::<T>::enable_callback)
        } else {
            None
        },
        disable: if T::HAS_DISABLE {
            Some(Adapter::<T>::disable_callback)
        } else {
            None
        },
        is_enabled: if T::HAS_IS_ENABLED {
            Some(Adapter::<T>::is_enabled_callback)
        } else {
            None
        },
        disable_unused: if T::HAS_DISABLE_UNUSED {
            Some(Adapter::<T>::disable_unused_callback)
        } else {
            None
        },
        save_context: if T::HAS_SAVE_CONTEXT {
            Some(Adapter::<T>::save_context_callback)
        } else {
            None
        },
        restore_context: if T::HAS_RESTORE_CONTEXT {
            Some(Adapter::<T>::restore_context_callback)
        } else {
            None
        },
        recalc_rate: if T::HAS_RECALC_RATE {
            Some(Adapter::<T>::recalc_rate_callback)
        } else {
            None
        },
        round_rate: if T::HAS_ROUND_RATE {
            Some(Adapter::<T>::round_rate_callback)
        } else {
            None
        },
        determine_rate: if T::HAS_DETERMINE_RATE {
            Some(Adapter::<T>::determine_rate_callback)
        } else {
            None
        },
        set_parent: if T::HAS_SET_PARENT {
            Some(Adapter::<T>::set_parent_callback)
        } else {
            None
        },
        get_parent: if T::HAS_GET_PARENT {
            Some(Adapter::<T>::get_parent_callback)
        } else {
            None
        },
        set_rate: if T::HAS_SET_RATE {
            Some(Adapter::<T>::set_rate_callback)
        } else {
            None
        },
        set_rate_and_parent: if T::HAS_SET_RATE_AND_PARENT {
            Some(Adapter::<T>::set_rate_and_parent_callback)
        } else {
            None
        },
        recalc_accuracy: if T::HAS_RECALC_ACCURACY {
            Some(Adapter::<T>::recalc_accuracy_callback)
        } else {
            None
        },
        get_phase: if T::HAS_GET_PHASE {
            Some(Adapter::<T>::get_phase_callback)
        } else {
            None
        },
        set_phase: if T::HAS_SET_PHASE {
            Some(Adapter::<T>::set_phase_callback)
        } else {
            None
        },
        get_duty_cycle: if T::HAS_GET_DUTY_CYCLE {
            Some(Adapter::<T>::get_duty_cycle_callback)
        } else {
            None
        },
        set_duty_cycle: if T::HAS_SET_DUTY_CYCLE {
            Some(Adapter::<T>::set_duty_cycle_callback)
        } else {
            None
        },
        init: if T::HAS_INIT {
            Some(Adapter::<T>::init_callback)
        } else {
            None
        },
        terminate: if T::HAS_TERMINATE {
            Some(Adapter::<T>::terminate_callback)
        } else {
            None
        },
        debug_init: if T::HAS_DEBUG_INIT {
            Some(Adapter::<T>::debug_init_callback)
        } else {
            None
        },
    };

    const fn build() -> &'static bindings::clk_ops {
        &Self::VTABLE
    }
}
