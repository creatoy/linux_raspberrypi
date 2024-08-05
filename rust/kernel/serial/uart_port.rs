// SPDX-License-Identifier: GPL-2.0

//! Support for uart drivers.
//!
//! C header: [`include/linux/serial_core.h`]

use super::uart_driver::UartDriver;

use crate::{
    bindings, 
    dev_err, 
    error::{code::*, Result}, 
    device, 
    pr_err, pr_warn, 
    types::ForeignOwnable
};

use core::{ 
    ffi::c_void,
    marker::{PhantomData, PhantomPinned},
    pin::Pin,
    mem::MaybeUninit,
};

use macros::vtable;

/// Trait about a uart core port device's operations
#[vtable]
pub trait UartPortOps {
    /// User data that will be accessible to all operations
    type Data: ForeignOwnable + Send + Sync = () ;

    /// * @tx_empty:      check if the UART TX FIFO is empty
    fn tx_empty(_port: &UartPort) -> u32;

    /// * @set_mctrl:    set the modem control register
    fn set_mctrl(_port: &UartPort, mctrl: u32);

    /// * @get_mctrl:    get the modem control register
    fn get_mctrl(_port: &UartPort) -> u32;

    /// * @stop_tx:      stop transmitting
    fn stop_tx(_port: &UartPort);

    /// * @start_tx:    start transmitting
    fn start_tx(_port: &UartPort);

    /// * @throttle:     stop receiving
    fn throttle(_port: &UartPort);

    /// * @unthrottle:   start receiving
    fn unthrottle(_port: &UartPort);

    /// * @send_xchar:  send a break character
    fn send_xchar(_port: &UartPort, ch: i8);

    /// * @stop_rx:      stop receiving
    fn stop_rx(_port: &UartPort);

    /// * @start_rx:    start receiving
    fn start_rx(_port: &UartPort);

    /// * @enable_ms:    enable modem status interrupts
    fn enable_ms(_port: &UartPort);

    /// * @break_ctl:   set the break control
    fn break_ctl(_port: &UartPort, ctl: i32);

    /// * @startup:      start the UART
    fn startup(_port: &UartPort) -> i32;

    /// * @shutdown:     shutdown the UART
    fn shutdown(_port: &UartPort);

    /// * @flush_buffer: flush the UART buffer
    fn flush_buffer(_port: &UartPort);

    /// * @set_termios: set the termios structure
    fn set_termios(
        _port: &UartPort,
        _new: *mut bindings::ktermios,
        _old: *const bindings::ktermios,
    );

    /// * @set_ldisc:    set the line discipline
    fn set_ldisc(_port: &UartPort, _arg2: *mut bindings::ktermios);

    /// * @pm:            power management
    fn pm(_port: &UartPort, _state: u32, _oldstate: u32);

    /// * @type:          get the type of the UART
    fn port_type(_port: &UartPort) -> *const i8;

    /// * @release_port: release the UART port
    fn release_port(uart_port: &UartPort);

    /// * @request_port: request the UART port
    fn request_port(uart_port: & UartPort) -> i32;

    /// * @config_port:  configure the UART port
    fn config_port(uart_port: &UartPort, flags: i32);

    /// * @verify_port:  verify the UART port
    fn verify_port(uart_port: &UartPort, ser: *mut bindings::serial_struct) -> i32;

    /// * @ioctl:        ioctl handler
    fn ioctl(uart_port: &UartPort, arg2: u32, arg3: u64) -> i32;

    /// #[cfg(CONFIG_CONSOLE_POLL)]
    fn poll_init(uart_port: &UartPort) -> i32;
    /// #[cfg(CONFIG_CONSOLE_POLL)]
    fn poll_put_char(uart_port: &UartPort, arg2: u8);
    /// #[cfg(CONFIG_CONSOLE_POLL)]
    fn poll_get_char(uart_port: &UartPort) -> i32;
}

/// Wraps the kernel's `struct uart_port`.
///
/// # Invariants
///
/// The pointer is non-null and valid, and has a non-zero reference count..
#[derive(Default, Copy, Clone)]
#[repr(transparent)]
pub struct UartPort(bindings::uart_port);

impl UartPort {
    /// Creates a new instance of the uart port object.
    pub const fn new() -> Self {
        let up = unsafe { MaybeUninit::<bindings::uart_port>::zeroed().assume_init() };
        Self(up)
    }

    /// Creates a reference to a [`UartPort`] from a valid pointer.
    ///
    /// # Safety
    ///
    /// Callers must ensure that `ptr` is valid, non-null, and has a non-zero reference count for
    /// the entire duration when the returned reference exists.
    pub unsafe fn from_raw<'a>(ptr: *mut bindings::uart_port) -> &'a Self {
        // SAFETY: Guaranteed by the safety requirements of the function.
        unsafe { &*ptr.cast() }
    }

    /// Returns a raw pointer to the inner C struct.
    #[inline]
    pub fn as_ptr(&self) -> *mut bindings::uart_port {
        &self.0 as *const _ as *mut _ 
    }

    /// Setup Uart Port
    pub fn setup(
        mut self,
        membase: *mut u8,
        mapbase: bindings::resource_size_t,
        irq: u32,
        iotype:u8,
        flags:u64,
        has_sysrq: u8,
        fifosize: u32,
        index: u32,
    ) -> Self {
        self.0.membase = membase;
        self.0.mapbase = mapbase;
        self.0.irq = irq;
        self.0.iotype = iotype;
        self.0.flags = flags;
        self.0.line = index;
        self.0.has_sysrq = has_sysrq;
        self.0.fifosize = fifosize;
        self
    }


    
}

/// A registration of a reset controller.
pub struct PortRegistration<T: UartPortOps> {
    uart_port: UartPort,
    dev: Option<device::Device>,
    is_registered: bool,
    _p: PhantomData<T>,
    _pin:PhantomPinned,
}

impl<T: UartPortOps> PortRegistration<T> {
    /// Creates a new [`ResetRegistration`] but does not register it yet.
    ///
    /// It is allowed to move.
    pub fn new() -> Self {
        Self {
            uart_port: UartPort::new(),
            dev: None,
            is_registered: false,
            _p: PhantomData,
            _pin:PhantomPinned,
        }
    }

    /// Registers a port in uart driver of the kernel.
    /// 
    /// use `uart_add_one_port` to register this device.
    pub fn register(
        self: Pin<&mut Self>,
        dev:  &dyn device::RawDevice,
        uart: &'static UartDriver,
        index:usize,
        data: T::Data,
    ) -> Result {
        // SAFETY: We never move out of `this`.
        let this = unsafe { self.get_unchecked_mut() };
        if this.is_registered {
            pr_warn!("this uart port driver is already registered\n");
            return Err(EINVAL);
        }
        let port = &mut this.uart_port;
        port.0.dev = dev.raw_device();
            // port.irq = irq;
            // port.membase = membase;
            // port.mapbase = mapbase;
            // port.flags = flags;
            // port.line = index;
        port.0.ops = Adapter::<T>::build();
        port.0.private_data = <T::Data as ForeignOwnable>::into_foreign(data) as *mut c_void;

        let ret = unsafe {bindings::uart_add_one_port(uart.as_ptr(), port.as_ptr())};
        if ret < 0 {
            // SAFETY: `data_pointer` was returned by `into_foreign` above.
            dev_err!(dev, "Failed to add AMBA-PL011 port driver\n");
        }

        this.dev = Some(device::Device::from_dev(dev));
        this.is_registered = true;
        Ok(())
    }
}

impl <T: UartPortOps> Drop  for PortRegistration<T> {
    fn drop(&mut self) {
        // Free data as well.
        // SAFETY: `data_pointer` was returned by `into_foreign` during registration.
        pr_err!("uart port dropped.\n")
    }
}

// SAFETY: `Registration` doesn't offer any methods or access to fields when shared between threads
// or CPUs, so it is safe to share it.
unsafe impl<T: UartPortOps> Sync for PortRegistration<T> {}

// SAFETY: Registration with and unregistration from the gpio subsystem can happen from any thread.
// Additionally, `T::Data` (which is dropped during unregistration) is `Send`, so it is ok to move
// `Registration` to different threads.
unsafe impl<T: UartPortOps> Send for PortRegistration<T> {}

pub(crate) struct Adapter<T:UartPortOps>(PhantomData<T>);

impl<T: UartPortOps> Adapter<T> {
    unsafe extern "C" fn tx_empty_callback(uart_port: *mut bindings::uart_port) -> core::ffi::c_uint {
        // let port = ManuallyDrop::new(UartPort(uart_port));
        let port = unsafe{ UartPort::from_raw(uart_port) };
        T::tx_empty(port)
    }

    unsafe extern "C" fn set_mctrl_callback(uart_port: *mut bindings::uart_port, mctrl: core::ffi::c_uint) {
        // let port = ManuallyDrop::new(UartPort(uart_port));
        let port = unsafe{ UartPort::from_raw(uart_port) };
        T::set_mctrl(port, mctrl)
    }

    unsafe extern "C" fn get_mctrl_callback(uart_port: *mut bindings::uart_port) -> core::ffi::c_uint {
        // let port = ManuallyDrop::new(UartPort(uart_port));
        let port = unsafe{ UartPort::from_raw(uart_port) };
        T::get_mctrl(port)
    }

    unsafe extern "C" fn stop_tx_callback(uart_port: *mut bindings::uart_port) {
        // let port = ManuallyDrop::new(UartPort(uart_port));
        let port = unsafe{ UartPort::from_raw(uart_port) };
        T::stop_tx(port)
    }

    unsafe extern "C" fn start_tx_callback(uart_port: *mut bindings::uart_port) {
        // let port = ManuallyDrop::new(UartPort(uart_port));
        let port = unsafe{ UartPort::from_raw(uart_port) };
        T::start_tx(port)
    }

    unsafe extern "C" fn throttle_callback(uart_port: *mut bindings::uart_port) {
        // let port = ManuallyDrop::new(UartPort(uart_port));
        let port = unsafe{ UartPort::from_raw(uart_port) };
        T::throttle(port)
    }

    unsafe extern "C" fn unthrottle_callback(uart_port: *mut bindings::uart_port) {
        // let port = ManuallyDrop::new(UartPort(uart_port));
        let port = unsafe{ UartPort::from_raw(uart_port) };
        T::unthrottle(port)
    }

    unsafe extern "C" fn send_xchar_callback(uart_port: *mut bindings::uart_port, ch: core::ffi::c_char) {
        // let port = ManuallyDrop::new(UartPort(uart_port));
        let port = unsafe{ UartPort::from_raw(uart_port) };
        T::send_xchar(port, ch)
    }

    unsafe extern "C" fn stop_rx_callback(uart_port: *mut bindings::uart_port) {
        // let port = ManuallyDrop::new(UartPort(uart_port));
        let port = unsafe{ UartPort::from_raw(uart_port) };
        T::stop_rx(port)
    }

    unsafe extern "C" fn start_rx_callback(uart_port: *mut bindings::uart_port) {
        // let port = ManuallyDrop::new(UartPort(uart_port));
        let port = unsafe{ UartPort::from_raw(uart_port) };
        T::start_rx(port)
    }

    unsafe extern "C" fn enable_ms_callback(uart_port: *mut bindings::uart_port) {
        // let port = ManuallyDrop::new(UartPort(uart_port));
        let port = unsafe{ UartPort::from_raw(uart_port) };
        T::enable_ms(port)
    }

    unsafe extern "C" fn break_ctl_callback(uart_port: *mut bindings::uart_port, ctl: core::ffi::c_int) {
        // let port = ManuallyDrop::new(UartPort(uart_port));
        let port = unsafe{ UartPort::from_raw(uart_port) };
        T::break_ctl(port, ctl)
    }

    unsafe extern "C" fn startup_callback(uart_port: *mut bindings::uart_port) -> core::ffi::c_int {
        // let port = ManuallyDrop::new(UartPort(uart_port));
        let port = unsafe{ UartPort::from_raw(uart_port) };
        T::startup(port)
    }

    unsafe extern "C" fn shutdown_callback(uart_port: *mut bindings::uart_port) {
        // let port = ManuallyDrop::new(UartPort(uart_port));
        let port = unsafe{ UartPort::from_raw(uart_port) };
        T::shutdown(port)
    }

    unsafe extern "C" fn flush_buffer_callback(uart_port: *mut bindings::uart_port) {
        // let port = ManuallyDrop::new(UartPort(uart_port));
        let port = unsafe{ UartPort::from_raw(uart_port) };
        T::flush_buffer(port)
    }

    unsafe extern "C" fn set_termios_callback(
        uart_port: *mut bindings::uart_port,
        new: *mut bindings::ktermios,
        old: *const bindings::ktermios,
    ) {
        // let port = ManuallyDrop::new(UartPort(uart_port));
        let port = unsafe{ UartPort::from_raw(uart_port) };
        T::set_termios(port, new, old)
    }

    unsafe extern "C" fn set_ldisc_callback(uart_port: *mut bindings::uart_port, arg2: *mut bindings::ktermios) {
        // let port = ManuallyDrop::new(UartPort(uart_port));
        let port = unsafe{ UartPort::from_raw(uart_port) };
        T::set_ldisc(port, arg2)
    }

    unsafe extern "C" fn pm_callback(
        uart_port: *mut bindings::uart_port,
        state: core::ffi::c_uint,
        oldstate: core::ffi::c_uint,
    ) {
        // let port = ManuallyDrop::new(UartPort(uart_port));
        let port = unsafe{ UartPort::from_raw(uart_port) };
        T::pm(port, state, oldstate)
    }

    unsafe extern "C" fn port_type_callback(uart_port: *mut bindings::uart_port) -> *const core::ffi::c_char {
        // let port = ManuallyDrop::new(UartPort(uart_port));
        let port = unsafe{ UartPort::from_raw(uart_port) };
        T::port_type(port)
    }

    unsafe extern "C" fn release_port_callback(uart_port: *mut bindings::uart_port) {
        // let port = ManuallyDrop::new(UartPort(uart_port));
        let port = unsafe{ UartPort::from_raw(uart_port) };
        T::release_port(port)
    }

    unsafe extern "C" fn request_port_callback(uart_port: *mut bindings::uart_port) -> core::ffi::c_int {
        // let port = ManuallyDrop::new(UartPort(uart_port));
        let port = unsafe{ UartPort::from_raw(uart_port) };
        T::request_port(port)
    }

    unsafe extern "C" fn config_port_callback(uart_port: *mut bindings::uart_port, arg2: core::ffi::c_int) {
        // let port = ManuallyDrop::new(UartPort(uart_port));
        let port = unsafe{ UartPort::from_raw(uart_port) };
        T::config_port(port, arg2)
    }

    unsafe extern "C" fn verify_port_callback(
        uart_port: *mut bindings::uart_port,
        ser: *mut bindings::serial_struct,
    ) -> core::ffi::c_int {
        // let port = ManuallyDrop::new(UartPort(uart_port));
        let port = unsafe{ UartPort::from_raw(uart_port) };
        T::verify_port(port, ser)
    }

    unsafe extern "C" fn ioctl_callback(
        uart_port: *mut bindings::uart_port,
        arg2: core::ffi::c_uint,
        arg3: core::ffi::c_ulong,
    ) -> core::ffi::c_int {
        // let port = ManuallyDrop::new(UartPort(uart_port));
        let port = unsafe{ UartPort::from_raw(uart_port) };
        T::ioctl(port, arg2, arg3)
    }

    unsafe extern "C" fn poll_init_callback(uart_port: *mut bindings::uart_port) -> core::ffi::c_int {
        // let port = ManuallyDrop::new(UartPort(uart_port));
        let port = unsafe{ UartPort::from_raw(uart_port) };
        T::poll_init(port)
    }

    unsafe extern "C" fn poll_put_char_callback(uart_port: *mut bindings::uart_port, ch: core::ffi::c_uchar) {
        // let port = ManuallyDrop::new(UartPort(uart_port));
        let port = unsafe{ UartPort::from_raw(uart_port) };
        T::poll_put_char(port, ch)
    }

    unsafe extern "C" fn poll_get_char_callback(uart_port: *mut bindings::uart_port) -> core::ffi::c_int {
        // let port = ManuallyDrop::new(UartPort(uart_port));
        let port = unsafe{ UartPort::from_raw(uart_port) };
        T::poll_get_char(port)
    }

    const VTABLE: bindings::uart_ops = bindings::uart_ops {
        tx_empty:   Some(Adapter::<T>::tx_empty_callback),
        set_mctrl:  Some(Adapter::<T>::set_mctrl_callback),
        get_mctrl:  Some(Adapter::<T>::get_mctrl_callback),
        stop_tx:    Some(Adapter::<T>::stop_tx_callback),
        start_tx:   Some(Adapter::<T>::start_tx_callback),
        throttle:   Some(Adapter::<T>::throttle_callback),
        unthrottle: Some(Adapter::<T>::unthrottle_callback),
        send_xchar: Some(Adapter::<T>::send_xchar_callback),
        stop_rx:    Some(Adapter::<T>::stop_rx_callback),
        start_rx:   Some(Adapter::<T>::start_rx_callback),
        enable_ms:  Some(Adapter::<T>::enable_ms_callback),
        break_ctl:  Some(Adapter::<T>::break_ctl_callback),
        startup:    Some(Adapter::<T>::startup_callback),
        shutdown:   Some(Adapter::<T>::shutdown_callback),
        flush_buffer: Some(Adapter::<T>::flush_buffer_callback),
        set_termios:  Some(Adapter::<T>::set_termios_callback),
        set_ldisc:    Some(Adapter::<T>::set_ldisc_callback),
        pm:         Some(Adapter::<T>::pm_callback),
        type_:      Some(Adapter::<T>::port_type_callback),
        release_port: Some(Adapter::<T>::release_port_callback),
        request_port: Some(Adapter::<T>::request_port_callback),
        config_port:  Some(Adapter::<T>::config_port_callback),
        verify_port:  Some(Adapter::<T>::verify_port_callback),
        ioctl:        Some(Adapter::<T>::ioctl_callback),

        #[cfg(CONFIG_CONSOLE_POLL)]
        poll_init:    Some(Adapter::<T>::poll_init_callback),
        #[cfg(CONFIG_CONSOLE_POLL)]
        poll_put_char: Some(Adapter::<T>::poll_put_char_callback),
        #[cfg(CONFIG_CONSOLE_POLL)]
        poll_get_char: Some(Adapter::<T>::poll_get_char_callback),
    };

    /// Returns Static Reference to the C ops struct.
    const fn build() -> &'static bindings::uart_ops {
        &Self::VTABLE
    }
}