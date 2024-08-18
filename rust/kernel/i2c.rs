use crate::{
    device::{Device, RawDevice},
    error::{from_result, to_result, Result},
    prelude::ENOTSUPP,
    str::CStr,
    types::{ForeignOwnable, Opaque},
    ThisModule,
};
use alloc::vec::Vec;
use core::{ffi::c_void, marker::PhantomData, mem};
use core::{mem::MaybeUninit, slice};
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
pub const I2C_SMBUS_BLOCK_MAX: u32 = 32;
pub const I2C_SMBUS_READ: u32 = 1;
pub const I2C_SMBUS_WRITE: u32 = 0;
pub const I2C_SMBUS_QUICK: u32 = 0;
pub const I2C_SMBUS_BYTE: u32 = 1;
pub const I2C_SMBUS_BYTE_DATA: u32 = 2;
pub const I2C_SMBUS_WORD_DATA: u32 = 3;
pub const I2C_SMBUS_PROC_CALL: u32 = 4;
pub const I2C_SMBUS_BLOCK_DATA: u32 = 5;
pub const I2C_SMBUS_I2C_BLOCK_BROKEN: u32 = 6;
pub const I2C_SMBUS_BLOCK_PROC_CALL: u32 = 7;
pub const I2C_SMBUS_I2C_BLOCK_DATA: u32 = 8;
pub const I2C_MAX_STANDARD_MODE_FREQ: u32 = 100000;
pub const I2C_MAX_FAST_MODE_FREQ: u32 = 400000;
pub const I2C_MAX_FAST_MODE_PLUS_FREQ: u32 = 1000000;
pub const I2C_MAX_TURBO_MODE_FREQ: u32 = 1400000;
pub const I2C_MAX_HIGH_SPEED_MODE_FREQ: u32 = 3400000;
pub const I2C_MAX_ULTRA_FAST_MODE_FREQ: u32 = 5000000;
pub const I2C_DEVICE_ID_NXP_SEMICONDUCTORS: u32 = 0;
pub const I2C_DEVICE_ID_NXP_SEMICONDUCTORS_1: u32 = 1;
pub const I2C_DEVICE_ID_NXP_SEMICONDUCTORS_2: u32 = 2;
pub const I2C_DEVICE_ID_NXP_SEMICONDUCTORS_3: u32 = 3;
pub const I2C_DEVICE_ID_RAMTRON_INTERNATIONAL: u32 = 4;
pub const I2C_DEVICE_ID_ANALOG_DEVICES: u32 = 5;
pub const I2C_DEVICE_ID_STMICROELECTRONICS: u32 = 6;
pub const I2C_DEVICE_ID_ON_SEMICONDUCTOR: u32 = 7;
pub const I2C_DEVICE_ID_SPRINTEK_CORPORATION: u32 = 8;
pub const I2C_DEVICE_ID_ESPROS_PHOTONICS_AG: u32 = 9;
pub const I2C_DEVICE_ID_FUJITSU_SEMICONDUCTOR: u32 = 10;
pub const I2C_DEVICE_ID_FLIR: u32 = 11;
pub const I2C_DEVICE_ID_O2MICRO: u32 = 12;
pub const I2C_DEVICE_ID_ATMEL: u32 = 13;
pub const I2C_DEVICE_ID_NONE: u32 = 65535;
pub const I2C_CLIENT_PEC: u32 = 4;
pub const I2C_CLIENT_TEN: u32 = 16;
pub const I2C_CLIENT_SLAVE: u32 = 32;
pub const I2C_CLIENT_HOST_NOTIFY: u32 = 64;
pub const I2C_CLIENT_WAKE: u32 = 128;
pub const I2C_CLIENT_SCCB: u32 = 36864;
pub const I2C_ALF_IS_SUSPENDED: u32 = 0;
pub const I2C_ALF_SUSPEND_REPORTED: u32 = 1;
pub const I2C_CLASS_HWMON: u32 = 1;
pub const I2C_CLASS_DDC: u32 = 8;
pub const I2C_CLASS_SPD: u32 = 128;
pub const I2C_CLASS_DEPRECATED: u32 = 256;
pub const I2C_CLIENT_END: u32 = 65534;
// No BIT macros.
pub const I2C_AQ_NO_CLK_STRETCH: u32 = 1 << 4;

/// Represents i2c_adapter_quirks
///
#[derive(Clone)]
#[repr(transparent)]
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
/// Note: all primitive fields are __u16 type in C, represented as u16 in Rust.
/// Note: buf is a raw pointer
/// Note: struct i2c_msg *msg is a i2c_msg ptr buf.
pub struct I2cMsg(bindings::i2c_msg);

impl I2cMsg {
    // Create I2CMsg buf from raw pointer
    pub fn from_raw<'a>(ptr: *mut bindings::i2c_msg, len: usize) -> Vec<Self> {
        let buf_ptr = unsafe { Vec::from_raw_parts(ptr.cast::<I2cMsg>(), len, len) };
        unsafe { buf_ptr }
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

/// Represents i2c_adapter
///
pub struct I2cAdapter(bindings::i2c_adapter);
impl I2cAdapter {
    /// Create a new instance of the I2cAdapter
    pub const fn new() -> Self {
        let up = unsafe { MaybeUninit::<bindings::i2c_adapter>::zeroed().assume_init() };
        Self(up)
    }

    /// Create I2CMsg from raw pointer
    pub fn from_raw<'a>(ptr: *mut bindings::i2c_adapter) -> &'a Self {
        let ptr = ptr.cast::<Self>();
        unsafe { &*ptr }
    }

    #[inline]
    pub fn as_ptr(&self) -> *mut bindings::i2c_adapter {
        &self.0 as *const _ as *mut _
    }

    pub unsafe fn i2c_get_adapdata<T>(&self) -> *mut T {
        unsafe { bindings::dev_get_drvdata(&self.0.dev as *const _ as *mut _) as *mut T }
    }

    pub unsafe fn i2c_set_adapdata<T>(&mut self, data: *mut T) {
        unsafe { bindings::dev_set_drvdata(&self.0.dev as *const _ as *mut _, data as *mut c_void) }
    }

    pub fn i2c_add_adapter(&mut self) -> Result {
        let ret = unsafe { bindings::i2c_add_adapter(self.as_ptr()) };
        to_result(ret)
    }

    pub fn timeout(&self) -> usize {
        unsafe { self.0.timeout as usize }
    }

    pub fn set_up<T: I2cAlgorithm>(
        mut self,
        name: &CStr,
        owner: &'static ThisModule,
        class: u32,
        device: &Device,
        quirks: I2cAdapterQuirks,
    ) -> Self {
        unsafe {
            // set_name
            bindings::snprintf(
                self.0.name.as_mut_ptr(),
                mem::size_of_val(&self.0.name) as u64,
                name.as_char_ptr(),
            );
            self.0.owner = owner.as_ptr();
            self.0.class = class;
            self.0.algo = Adapter::<T>::build();

            let dev_ptr = device.raw_device();
            self.0.dev.parent = dev_ptr;
            self.0.dev.of_node = (*dev_ptr).of_node;

            self.0.quirks = &quirks.0 as *const _;
        };
        self
    }
}

impl Drop for I2cAdapter {
    fn drop(&mut self) {
        unsafe { bindings::i2c_del_adapter(self.as_ptr()) }
    }
}
/// Represents i2c_smbus_data
///
pub struct I2cSmbusData(bindings::i2c_smbus_data);

impl I2cSmbusData {
    pub fn from_raw<'a>(ptr: *mut bindings::i2c_smbus_data) -> &'a Self {
        let ptr = ptr.cast::<Self>();
        unsafe { &*ptr }
    }
}

/// Represents i2c_client
///
pub struct I2cClient(bindings::i2c_client);

impl I2cClient {
    pub fn from_raw<'a>(ptr: *mut bindings::i2c_client) -> &'a Self {
        let ptr = ptr.cast::<Self>();
        unsafe { &*ptr }
    }
}
/// Represents i2c_algorithm
///
#[vtable]
pub trait I2cAlgorithm {
    /// User data that will be accessible to all operations
    type Data: ForeignOwnable + Send + Sync = ();

    // Caution: May <Result>!

    fn master_xfer(adap: &I2cAdapter, msgs: Vec<I2cMsg>, num: i32) -> Result<i32> {
        Err(ENOTSUPP)
    }

    fn master_xfer_atomic(adap: &I2cAdapter, msgs: Vec<I2cMsg>, num: i32) -> Result<i32> {
        Err(ENOTSUPP)
    }

    // Caution: read_write is c_char, flags is c_ushort!
    fn smbus_xfer(
        adap: &I2cAdapter,
        addr: u16,
        flags: u16,
        read_write: i8,
        command: u8,
        size: i32,
        data: &I2cSmbusData,
    ) -> Result<i32> {
        Err(ENOTSUPP)
    }

    fn smbus_xfer_atomic(
        adap: &I2cAdapter,
        addr: u16,
        flags: u16,
        read_write: i8,
        command: u8,
        size: i32,
        data: &I2cSmbusData,
    ) -> Result<i32> {
        Err(ENOTSUPP)
    }

    fn functionality(adap: &I2cAdapter) -> u32 {
        0
    }

    // IS_ENABLED(CONFIG_I2C_SLAVE)
    fn reg_slave(i2c_client: &I2cClient) -> Result<()> {
        Err(ENOTSUPP)
    }

    fn unreg_slave(i2c_client: &I2cClient) -> Result<()> {
        Err(ENOTSUPP)
    }
}

pub(crate) struct Adapter<T: I2cAlgorithm>(PhantomData<T>);

impl<T: I2cAlgorithm> Adapter<T> {
    unsafe extern "C" fn master_xfer_callback(
        adap: *mut bindings::i2c_adapter,
        msgs: *mut bindings::i2c_msg,
        num: core::ffi::c_int,
    ) -> core::ffi::c_int {
        from_result(|| {
            let adap = unsafe { I2cAdapter::from_raw(adap) };
            let msgs = unsafe { I2cMsg::from_raw(msgs, num as usize) };
            T::master_xfer(adap, msgs, num)
        })
    }
    unsafe extern "C" fn master_xfer_atomic_callback(
        adap: *mut bindings::i2c_adapter,
        msgs: *mut bindings::i2c_msg,
        num: core::ffi::c_int,
    ) -> core::ffi::c_int {
        from_result(|| {
            let adap = unsafe { I2cAdapter::from_raw(adap) };
            let msgs = unsafe { I2cMsg::from_raw(msgs, num as usize) };
            T::master_xfer_atomic(adap, msgs, num)
        })
    }

    unsafe extern "C" fn smbus_xfer_callback(
        adap: *mut bindings::i2c_adapter,
        addr: u16,
        flags: core::ffi::c_ushort,
        read_write: core::ffi::c_char,
        command: u8,
        size: core::ffi::c_int,
        data: *mut bindings::i2c_smbus_data,
    ) -> core::ffi::c_int {
        from_result(|| {
            let adap = unsafe { I2cAdapter::from_raw(adap) };
            let data = unsafe { I2cSmbusData::from_raw(data) };
            T::smbus_xfer(adap, addr, flags, read_write, command, size, data)
        })
    }

    unsafe extern "C" fn smbus_xfer_atomic_callback(
        adap: *mut bindings::i2c_adapter,
        addr: u16,
        flags: core::ffi::c_ushort,
        read_write: core::ffi::c_char,
        command: u8,
        size: core::ffi::c_int,
        data: *mut bindings::i2c_smbus_data,
    ) -> core::ffi::c_int {
        from_result(|| {
            let adap = unsafe { I2cAdapter::from_raw(adap) };
            let data = unsafe { I2cSmbusData::from_raw(data) };
            T::smbus_xfer_atomic(adap, addr, flags, read_write, command, size, data)
        })
    }

    unsafe extern "C" fn functionality_callback(adap: *mut bindings::i2c_adapter) -> u32 {
        let adap = unsafe { I2cAdapter::from_raw(adap) };
        T::functionality(adap)
    }

    unsafe extern "C" fn reg_slave_callback(client: *mut bindings::i2c_client) -> core::ffi::c_int {
        0
    }

    unsafe extern "C" fn unreg_slave_callback(
        client: *mut bindings::i2c_client,
    ) -> core::ffi::c_int {
        0
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
        #[cfg(CONFIG_I2C_SLAVE)]
        reg_slave: if T::HAS_REG_SLAVE {
            Some(Adapter::<T>::reg_slave_callback)
        } else {
            None
        },
        #[cfg(CONFIG_I2C_SLAVE)]
        unreg_slave: if T::HAS_UNREG_SLAVE {
            Some(Adapter::<T>::unreg_slave_callback)
        } else {
            None
        },
    };

    const fn build() -> &'static bindings::i2c_algorithm {
        &Adapter::<T>::VTABLE
    }
}
