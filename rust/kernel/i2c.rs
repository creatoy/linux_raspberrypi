use core::marker::PhantomData;

use crate::{str, types::Opaque};
use alloc::vec::Vec;
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

/// Represents i2c_msg
///
pub struct I2cMsg(bindings::i2c_msg);

impl I2cMsg {
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

    pub fn buf_to_vec(&self) -> Option<Vec<u8>> {
        let len = self.len() as usize;
        let buf = self.0.buf as *const u8;
        if buf.is_null() {
            return None;
        }
        // Safety: buf is valid for len bytes, no contiguity.
        let slice = unsafe { core::slice::from_raw_parts(buf, len) };
        Some(slice.to_vec())
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
pub struct I2cAdapter(Opaque<bindings::i2c_adapter>);

/// Represents i2c_smbus_data
///
pub struct I2cSmbusData(Opaque<bindings::i2c_smbus_data>);

/// Represents i2c_algorithm
///
#[vtable]
pub trait I2cAlgorithm {
    /// User data that will be accessible to all operations
    type Data: ForeignOwnable + Send + Sync = ();

    // Caution: May <Result>!

    fn master_xfer(adap: &mut I2cAdapter, msgs: &mut I2cMsg, num: i32) -> i32;

    fn master_xfer_atomic(adap: &mut I2cAdapter, msgs: &mut I2cMsg, num: i32) -> i32;

    // Caution: read_write is c_char, flags is c_ushort!
    fn smbus_xfer(
        adap: &mut I2cAdapter,
        addr: u16,
        flags: u16,
        read_write: u8,
        command: u8,
        size: i32,
        data: &mut I2cSmbusData,
    ) -> i32;

    fn smbus_xfer_atomic(
        adap: &mut I2cAdapter,
        addr: u16,
        flags: u16,
        read_write: u8,
        command: u8,
        size: i32,
        data: &mut I2cSmbusData,
    ) -> i32;
}

pub(crate) struct Adapter<T: I2cAlgorithm>(PhantomData<T>);

impl<T: I2cAlgorithm> Adapter<T> {
    // TODO!
}
