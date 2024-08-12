use crate::{
    error::{from_result, to_result, Result},
    types::{ForeignOwnable, Opaque},
};
use alloc::vec::{self, Vec};
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

    pub fn set_flags(mut self, flags: u64) -> Self {
        self.0.flags = flags;
        self
    }
}

/// Represents i2c_msg
///
/// Note: buf is a raw pointer
/// Note: all primitive fields are __u16 type in C, represented as u16 in Rust.
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

    /// return buf of i2c_msg and transfer ownership of the buf
    pub fn buf_to_vec(&self) -> Option<Vec<u8>> {
        let len = self.len() as usize;
        let buf = self.0.buf as *const _ as *mut u8;
        if buf.is_null() {
            return None;
        }
        // Safety: buf is valid for len bytes, no contiguity.
        let vec: Vec<u8> = unsafe { Vec::from_raw_parts(buf, len, len) };
        Some(vec)
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
pub struct I2cAdapter(bindings::i2c_adapter);
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

    pub fn timeout(&self) -> usize {
        unsafe { self.0.timeout as usize }
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

    fn master_xfer(adap: &mut I2cAdapter, msgs: &mut I2cMsg, num: i32) -> Result<i32>;

    fn master_xfer_atomic(adap: &mut I2cAdapter, msgs: &mut I2cMsg, num: i32) -> Result<i32>;

    // Caution: read_write is c_char, flags is c_ushort!
    fn smbus_xfer(
        adap: &mut I2cAdapter,
        addr: u16,
        flags: u16,
        read_write: i8,
        command: u8,
        size: i32,
        data: &mut I2cSmbusData,
    ) -> Result<i32>;

    fn smbus_xfer_atomic(
        adap: &mut I2cAdapter,
        addr: u16,
        flags: u16,
        read_write: i8,
        command: u8,
        size: i32,
        data: &mut I2cSmbusData,
    ) -> Result<i32>;

    fn functionality(adap: &mut I2cAdapter) -> u32;
}

pub(crate) struct Adapter<T: I2cAlgorithm>(PhantomData<T>);

impl<T: I2cAlgorithm> Adapter<T> {
    unsafe extern "C" fn master_xfer_callback(
        adap: *mut bindings::i2c_adapter,
        msgs: *mut bindings::i2c_msg,
        num: i32,
    ) -> core::ffi::c_int {
        let adapter = unsafe { I2cAdapter::from_raw(adap) };
        let messages = unsafe { I2cMsg::from_raw(msgs) };
        from_result(|| T::master_xfer(adapter, messages, num))
    }

    unsafe extern "C" fn master_xfer_atomic_callback(
        adap: *mut bindings::i2c_adapter,
        msgs: *mut bindings::i2c_msg,
        num: i32,
    ) -> core::ffi::c_int {
        let adapter = unsafe { I2cAdapter::from_raw(adap) };
        let messages = unsafe { I2cMsg::from_raw(msgs) };
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
