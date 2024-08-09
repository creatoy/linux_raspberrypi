use crate::{error::Result, irq};

pub const IRQF_TRIGGER_NONE: u32 = bindings::IRQF_TRIGGER_NONE;
pub const IRQF_TRIGGER_RISING: u32 = bindings::IRQF_TRIGGER_RISING;
pub const IRQF_TRIGGER_FALLING: u32 = bindings::IRQF_TRIGGER_FALLING;
pub const IRQF_TRIGGER_HIGH: u32 = bindings::IRQF_TRIGGER_HIGH;
pub const IRQF_TRIGGER_LOW: u32 = bindings::LINUX_IRQF_TRIGGER_LOW;
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

pub type IrqHandler = Option<fn(i32, *mut core::ffi::c_void) -> irq::Return>;



pub fn request_irq<T>(irq:u32, handler:IrqHandler, flags:u32, name:&'static CStr,dev:&T) -> Result {
	let ret = unsafe {
		bindings::
	}
}