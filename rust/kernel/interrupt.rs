// SPDX-License-Identifier: GPL-2.0

//! Generic devices that are part of the kernel's driver model.
//!
//! C header: [`include/linux/device.h`](../../../../include/linux/.h)

use crate::{
    device::{Device, RawDevice},
    error::{to_result, Result},
    irq,
    str::CStr,
};
use core::marker::PhantomData;

pub const IRQF_TRIGGER_NONE: u32 = bindings::IRQF_TRIGGER_NONE;
pub const IRQF_TRIGGER_RISING: u32 = bindings::IRQF_TRIGGER_RISING;
pub const IRQF_TRIGGER_FALLING: u32 = bindings::IRQF_TRIGGER_FALLING;
pub const IRQF_TRIGGER_HIGH: u32 = bindings::IRQF_TRIGGER_HIGH;
pub const IRQF_TRIGGER_LOW: u32 = bindings::IRQF_TRIGGER_LOW;
pub const IRQF_TRIGGER_MASK: u32 = bindings::IRQF_TRIGGER_MASK;
pub const IRQF_TRIGGER_PROBE: u32 = bindings::IRQF_TRIGGER_PROBE;
pub const IRQF_SHARED: u32 = bindings::IRQF_SHARED;
pub const IRQF_PROBE_SHARED: u32 = bindings::IRQF_PROBE_SHARED;
pub const __IRQF_TIMER: u32 = bindings::__IRQF_TIMER;
pub const IRQF_PERCPU: u32 = bindings::IRQF_PERCPU;
pub const IRQF_NOBALANCING: u32 = bindings::IRQF_NOBALANCING;
pub const IRQF_IRQPOLL: u32 = bindings::IRQF_IRQPOLL;
pub const IRQF_ONESHOT: u32 = bindings::IRQF_ONESHOT;
pub const IRQF_NO_SUSPEND: u32 = bindings::IRQF_NO_SUSPEND;
pub const IRQF_FORCE_RESUME: u32 = bindings::IRQF_FORCE_RESUME;
pub const IRQF_NO_THREAD: u32 = bindings::IRQF_NO_THREAD;
pub const IRQF_EARLY_RESUME: u32 = bindings::IRQF_EARLY_RESUME;
pub const IRQF_COND_SUSPEND: u32 = bindings::IRQF_COND_SUSPEND;
pub const IRQF_NO_AUTOEN: u32 = bindings::IRQF_NO_AUTOEN;
pub const IRQF_NO_DEBUG: u32 = bindings::IRQF_NO_DEBUG;
pub const IRQF_TIMER: u32 = bindings::IRQF_TIMER;
pub const IRQ_NOTCONNECTED: u32 = bindings::IRQ_NOTCONNECTED;

pub trait IrqHandler {
    /// User data that will be accessible to all operations
    type Context;

    fn handler(irq: i32, ctx: &mut Self::Context) -> irq::Return;
}

pub(crate) struct Adapter<T, H>(PhantomData<T>, PhantomData<H>)
where
    H: IrqHandler<Context = T>;

impl<T, H> Adapter<T, H>
where
    H: IrqHandler<Context = T>,
{
    unsafe extern "C" fn handler_callback(
        arg1: i32,
        arg2: *mut core::ffi::c_void,
    ) -> bindings::irqreturn_t {
        let dev = unsafe { &mut *(arg2 as *const _ as *mut _) };
        H::handler(arg1, dev) as bindings::irqreturn_t
    }

    const VTABLE: bindings::irq_handler_t = Some(Self::handler_callback);

    const fn build() -> &'static bindings::irq_handler_t {
        &Self::VTABLE
    }
}

pub fn request_irq<T, H>(irq: u32, handler: H, flags: u64, name: &'static CStr, dev: &T) -> Result
where
    H: IrqHandler<Context = T>,
{
    let ret = unsafe {
        bindings::request_threaded_irq(
            irq,
            *Adapter::<T, H>::build(),
            None,
            flags,
            name.as_char_ptr(),
            dev as *const _ as *mut core::ffi::c_void,
        )
    };
    to_result(ret)
}

pub fn request_percpu_irq<T, H>(
    irq: u32,
    handler: H,
    dev_name: &'static CStr,
    percpu_dev_id: &T,
) -> Result
where
    H: IrqHandler<Context = T>,
{
    let ret = unsafe {
        bindings::__request_percpu_irq(
            irq,
            *Adapter::<T, H>::build(),
            0,
            dev_name.as_char_ptr(),
            percpu_dev_id as *const _ as *mut core::ffi::c_void,
        )
    };
    to_result(ret)
}

pub fn devm_request_irq<T, H>(
    dev: Device,
    irq: u32,
    handler: H,
    flags: u64,
    dev_name: &'static CStr,
    dev_id: &T,
) -> Result
where
    H: IrqHandler<Context = T>,
{
    let ret = unsafe {
        bindings::devm_request_threaded_irq(
            dev.raw_device(),
            irq,
            *Adapter::<T, H>::build(),
            None,
            flags,
            dev_name.as_char_ptr(),
            dev_id as *const _ as *mut core::ffi::c_void,
        )
    };
    to_result(ret)
}
