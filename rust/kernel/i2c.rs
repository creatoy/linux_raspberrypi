use crate::{
    device::{Device, RawDevice},
    error::{from_result, to_result, Result},
    prelude::*,
    str::CStr,
    types::{ForeignOwnable, Opaque},
};
use alloc::vec::Vec;
use core::mem::MaybeUninit;
use core::{ffi::c_void, marker::PhantomData};
use macros::vtable;

pub const I2C_M_RD: u32 = bindings::I2C_M_RD;
pub const I2C_M_TEN: u32 = bindings::I2C_M_TEN;
pub const I2C_M_DMA_SAFE: u32 = bindings::I2C_M_DMA_SAFE;
pub const I2C_M_RECV_LEN: u32 = bindings::I2C_M_RECV_LEN;
pub const I2C_M_NO_RD_ACK: u32 = bindings::I2C_M_NO_RD_ACK;
pub const I2C_M_IGNORE_NAK: u32 = bindings::I2C_M_IGNORE_NAK;
pub const I2C_M_REV_DIR_ADDR: u32 = bindings::I2C_M_REV_DIR_ADDR;
pub const I2C_M_NOSTART: u32 = bindings::I2C_M_NOSTART;
pub const I2C_M_STOP: u32 = bindings::I2C_M_STOP;
pub const I2C_FUNC_I2C: u32 = bindings::I2C_FUNC_I2C;
pub const I2C_FUNC_10BIT_ADDR: u32 = bindings::I2C_FUNC_10BIT_ADDR;
pub const I2C_FUNC_PROTOCOL_MANGLING: u32 = bindings::I2C_FUNC_PROTOCOL_MANGLING;
pub const I2C_FUNC_SMBUS_PEC: u32 = bindings::I2C_FUNC_SMBUS_PEC;
pub const I2C_FUNC_NOSTART: u32 = bindings::I2C_FUNC_NOSTART;
pub const I2C_FUNC_SLAVE: u32 = bindings::I2C_FUNC_SLAVE;
pub const I2C_FUNC_SMBUS_BLOCK_PROC_CALL: u32 = bindings::I2C_FUNC_SMBUS_BLOCK_PROC_CALL;
pub const I2C_FUNC_SMBUS_QUICK: u32 = bindings::I2C_FUNC_SMBUS_QUICK;
pub const I2C_FUNC_SMBUS_READ_BYTE: u32 = bindings::I2C_FUNC_SMBUS_READ_BYTE;
pub const I2C_FUNC_SMBUS_WRITE_BYTE: u32 = bindings::I2C_FUNC_SMBUS_WRITE_BYTE;
pub const I2C_FUNC_SMBUS_READ_BYTE_DATA: u32 = bindings::I2C_FUNC_SMBUS_READ_BYTE_DATA;
pub const I2C_FUNC_SMBUS_WRITE_BYTE_DATA: u32 = bindings::I2C_FUNC_SMBUS_WRITE_BYTE_DATA;
pub const I2C_FUNC_SMBUS_READ_WORD_DATA: u32 = bindings::I2C_FUNC_SMBUS_READ_WORD_DATA;
pub const I2C_FUNC_SMBUS_WRITE_WORD_DATA: u32 = bindings::I2C_FUNC_SMBUS_WRITE_WORD_DATA;
pub const I2C_FUNC_SMBUS_PROC_CALL: u32 = bindings::I2C_FUNC_SMBUS_PROC_CALL;
pub const I2C_FUNC_SMBUS_READ_BLOCK_DATA: u32 = bindings::I2C_FUNC_SMBUS_READ_BLOCK_DATA;
pub const I2C_FUNC_SMBUS_WRITE_BLOCK_DATA: u32 = bindings::I2C_FUNC_SMBUS_WRITE_BLOCK_DATA;
pub const I2C_FUNC_SMBUS_READ_I2C_BLOCK: u32 = bindings::I2C_FUNC_SMBUS_READ_I2C_BLOCK;
pub const I2C_FUNC_SMBUS_WRITE_I2C_BLOCK: u32 = bindings::I2C_FUNC_SMBUS_WRITE_I2C_BLOCK;
pub const I2C_FUNC_SMBUS_HOST_NOTIFY: u32 = bindings::I2C_FUNC_SMBUS_HOST_NOTIFY;
pub const I2C_FUNC_SMBUS_BYTE: u32 = bindings::I2C_FUNC_SMBUS_BYTE;
pub const I2C_FUNC_SMBUS_BYTE_DATA: u32 = bindings::I2C_FUNC_SMBUS_BYTE_DATA;
pub const I2C_FUNC_SMBUS_WORD_DATA: u32 = bindings::I2C_FUNC_SMBUS_WORD_DATA;
pub const I2C_FUNC_SMBUS_BLOCK_DATA: u32 = bindings::I2C_FUNC_SMBUS_BLOCK_DATA;
pub const I2C_FUNC_SMBUS_I2C_BLOCK: u32 = bindings::I2C_FUNC_SMBUS_I2C_BLOCK;
pub const I2C_FUNC_SMBUS_EMUL: u32 = bindings::I2C_FUNC_SMBUS_EMUL;
pub const I2C_FUNC_SMBUS_EMUL_ALL: u32 = bindings::I2C_FUNC_SMBUS_EMUL_ALL;

// No BIT macros.
pub const I2C_AQ_NO_CLK_STRETCH: u32 = 1 << 4;
/// Represents i2c_adapter_quirks
///
pub struct I2cAdapterQuirks(bindings::i2c_adapter_quirks);

impl I2cAdapterQuirks {
    pub fn new() -> Self {
        let up = unsafe { MaybeUninit::<bindings::i2c_adapter_quirks>::zeroed().assume_init() };
        Self(up)
    }

    pub fn as_ptr(&self) -> *mut bindings::i2c_adapter_quirks {
        &self.0 as *const _ as *mut _
    }

    pub fn set_flags(mut self, flags: u64) -> Self {
        self.0.flags = flags;
        self
    }
}

/// Represents i2c_msg
///
/// Note: buf is a raw pointer
/// Note: all primitive fields are __u16 type in C, represented as u16 in Rust.
#[derive(Clone)]
pub struct I2cMsg(bindings::i2c_msg);

impl I2cMsg {
    pub unsafe fn from_raw<'a>(ptr: *mut bindings::i2c_msg) -> &'a mut Self {
        let ptr = ptr.cast::<Self>();
        unsafe { &mut *ptr }
    }

    /// return flags of i2c_msg
    pub fn flags(&self) -> u16 {
        self.0.flags as u16
    }

    /// return len of i2c_msg
    pub fn len(&self) -> u16 {
        self.0.len as u16
    }

    /// return addr of i2c_msg
    pub fn addr(&self) -> u16 {
        self.0.addr as u16
    }

    pub fn buf_ptr(&self) -> *mut u8 {
        self.0.buf
    }

    pub fn write_to_buf(&self, data: &[u8]) -> Option<usize> {
        let data_ptr = data.as_ptr();
        if data_ptr.is_null() || self.0.buf.is_null() {
            return None;
        }
        let len = self.len() as usize;
        let len = len.min(data.len());
        // Safety: buf is valid for len bytes, no contiguity.
        unsafe {
            core::ptr::copy_nonoverlapping(data_ptr, self.0.buf, len);
        }

        Some(len)
    }

    /// return buf of i2c_msg and transfer ownership of the buf
    pub fn buf_to_vec(&self) -> Option<Vec<u8>> {
        let len = self.len() as usize;
        let buf = self.0.buf as *const _ as *mut u8;
        if buf.is_null() {
            return None;
        }
        // Safety: buf is valid for len bytes, no contiguity.
        let mut v = Vec::try_with_capacity(len).expect("Vec::try_with_capacity failed");
        for i in 0..len {
            v.try_push(unsafe { *buf.add(i) });
        }
        // let vec: Vec<u8> = unsafe { Vec::from_raw_parts(buf, len, len) };
        Some(v)
    }
}

/*
impl Default for I2cMsg {
    fn default() -> Self {
        Self(bindings::i2c_msg::default())
    }
}*/

/// Represents i2c_adapter
///
pub struct I2cAdapter(pub bindings::i2c_adapter);
impl I2cAdapter {
    pub unsafe fn from_raw<'a>(ptr: *mut bindings::i2c_adapter) -> &'a mut Self {
        let ptr = ptr.cast::<Self>();
        unsafe { &mut *ptr }
    }

    pub fn as_ptr(&self) -> *mut bindings::i2c_adapter {
        &self.0 as *const _ as *mut _
    }

    pub unsafe fn i2c_get_adapdata<T>(&self) -> *mut T {
        unsafe { bindings::dev_get_drvdata(&self.0.dev as *const _ as *mut _) as *mut T }
    }

    pub unsafe fn i2c_set_adapdata<T>(&mut self, data: *mut T) {
        unsafe { bindings::dev_set_drvdata(&self.0.dev as *const _ as *mut _, data as *mut c_void) }
    }

    pub fn i2c_add_adapter(&self) -> Result {
        let ret = unsafe { bindings::i2c_add_adapter(self.as_ptr()) };
        to_result(ret)
    }

    pub fn set_name(&mut self, name: &CStr) {
        let len = name.len().min(self.0.name.len() - 1);
        let s = name.as_bytes();
        for b in &s[0..len] {
            self.0.name[0] = *b as i8;
        }
        self.0.name[len] = 0;
    }

    pub unsafe fn set_owner(&mut self, owner: *mut bindings::module) {
        self.0.owner = owner
    }

    pub fn set_class(&mut self, class: u32) {
        self.0.class = class
    }

    pub unsafe fn set_algorithm<T: I2cAlgorithm>(&mut self) {
        self.0.algo = Adapter::<T>::build()
    }

    pub unsafe fn setup_device(&mut self, device: &Device) {
        let dev_ptr = device.raw_device();
        self.0.dev.parent = dev_ptr;
        unsafe {
            self.0.dev.of_node = (*dev_ptr).of_node;
        }
    }

    pub unsafe fn set_quirks(&mut self, quirks: &I2cAdapterQuirks) {
        self.0.quirks = &quirks.0 as *const _;
    }

    pub fn timeout(&self) -> usize {
        unsafe { self.0.timeout as usize }
    }

    //pub fn set_up(self)
}

impl Drop for I2cAdapter {
    fn drop(&mut self) {
        unsafe { bindings::i2c_del_adapter(self.as_ptr()) }
    }
}
/// Represents i2c_smbus_data
///
pub struct I2cSmbusData(Opaque<bindings::i2c_smbus_data>);

impl I2cSmbusData {
    pub unsafe fn from_raw<'a>(ptr: *mut bindings::i2c_smbus_data) -> &'a mut Self {
        let ptr = ptr.cast::<Self>();
        unsafe { &mut *ptr }
    }
}

/// Represents i2c_algorithm
///
#[vtable]
pub trait I2cAlgorithm {
    /// User data that will be accessible to all operations
    type Data: ForeignOwnable + Send + Sync = ();

    // Caution: May <Result>!

    fn master_xfer(adap: &mut I2cAdapter, msgs: Vec<I2cMsg>, num: i32) -> Result<i32> {
        Err(ENOTSUPP)
    }

    fn master_xfer_atomic(adap: &mut I2cAdapter, msgs: Vec<I2cMsg>, num: i32) -> Result<i32> {
        Err(ENOTSUPP)
    }

    // Caution: read_write is c_char, flags is c_ushort!
    fn smbus_xfer(
        adap: &mut I2cAdapter,
        addr: u16,
        flags: u16,
        read_write: i8,
        command: u8,
        size: i32,
        data: &mut I2cSmbusData,
    ) -> Result<i32> {
        Err(ENOTSUPP)
    }

    fn smbus_xfer_atomic(
        adap: &mut I2cAdapter,
        addr: u16,
        flags: u16,
        read_write: i8,
        command: u8,
        size: i32,
        data: &mut I2cSmbusData,
    ) -> Result<i32> {
        Err(ENOTSUPP)
    }

    fn functionality(adap: &mut I2cAdapter) -> u32 {
        0
    }
}

pub(crate) struct Adapter<T: I2cAlgorithm>(PhantomData<T>);

impl<T: I2cAlgorithm> Adapter<T> {
    unsafe extern "C" fn master_xfer_callback(
        adap: *mut bindings::i2c_adapter,
        msgs: *mut bindings::i2c_msg,
        num: i32,
    ) -> core::ffi::c_int {
        let adapter = unsafe { I2cAdapter::from_raw(adap) };

        let mut messages = Vec::try_with_capacity(num as usize).expect("Failed to allocate memory");
        for i in 0..num as usize {
            messages.try_push(I2cMsg(unsafe { *msgs.add(i) }));
        }

        from_result(|| T::master_xfer(adapter, messages, num))
    }

    unsafe extern "C" fn master_xfer_atomic_callback(
        adap: *mut bindings::i2c_adapter,
        msgs: *mut bindings::i2c_msg,
        num: i32,
    ) -> core::ffi::c_int {
        let adapter = unsafe { I2cAdapter::from_raw(adap) };

        let mut messages = Vec::try_with_capacity(num as usize).expect("Failed to allocate memory");
        for i in 0..num as usize {
            messages[i] = unsafe { I2cMsg(*msgs.add(i)) };
        }

        from_result(|| T::master_xfer_atomic(adapter, messages, num))
    }

    unsafe extern "C" fn smbus_xfer_callback(
        adap: *mut bindings::i2c_adapter,
        addr: u16,
        flags: u16,
        read_write: i8,
        command: u8,
        size: i32,
        data: *mut bindings::i2c_smbus_data,
    ) -> core::ffi::c_int {
        let adapter = unsafe { I2cAdapter::from_raw(adap) };
        let smbus_data = unsafe { I2cSmbusData::from_raw(data) };
        from_result(|| T::smbus_xfer(adapter, addr, flags, read_write, command, size, smbus_data))
    }

    unsafe extern "C" fn smbus_xfer_atomic_callback(
        adap: *mut bindings::i2c_adapter,
        addr: u16,
        flags: u16,
        read_write: i8,
        command: u8,
        size: i32,
        data: *mut bindings::i2c_smbus_data,
    ) -> core::ffi::c_int {
        let adapter = unsafe { I2cAdapter::from_raw(adap) };
        let smbus_data = unsafe { I2cSmbusData::from_raw(data) };
        from_result(|| {
            T::smbus_xfer_atomic(adapter, addr, flags, read_write, command, size, smbus_data)
        })
    }

    unsafe extern "C" fn functionality_callback(
        adap: *mut bindings::i2c_adapter,
    ) -> core::ffi::c_uint {
        let adapter = unsafe { I2cAdapter::from_raw(adap) };
        T::functionality(adapter)
    }

    const VTABLE: bindings::i2c_algorithm = bindings::i2c_algorithm {
        master_xfer: if T::HAS_MASTER_XFER {
            Some(Adapter::<T>::master_xfer_callback)
        } else {
            None
        },
        master_xfer_atomic: if T::HAS_MASTER_XFER_ATOMIC {
            Some(Adapter::<T>::master_xfer_atomic_callback)
        } else {
            None
        },
        smbus_xfer: if T::HAS_SMBUS_XFER {
            Some(Adapter::<T>::smbus_xfer_callback)
        } else {
            None
        },
        smbus_xfer_atomic: if T::HAS_SMBUS_XFER_ATOMIC {
            Some(Adapter::<T>::smbus_xfer_atomic_callback)
        } else {
            None
        },
        functionality: if T::HAS_FUNCTIONALITY {
            Some(Adapter::<T>::functionality_callback)
        } else {
            None
        },
    };

    const fn build() -> &'static bindings::i2c_algorithm {
        &Self::VTABLE
    }
}
