// SPDX-License-Identifier: GPL-2.0

//! BCM2835 master mode driver

use kernel::driver::DeviceRemoval;
use kernel::prelude::*;

use kernel::bindings;
use kernel::clk::Clk;
use kernel::clk_provider::ClkHw;
use kernel::clk_provider::ClkInitData;
use kernel::clk_provider::ClkOps;
use kernel::completion::Completion;
use kernel::device::Device;
use kernel::device::RawDevice;
use kernel::driver;
use kernel::io_mem::IoMem;
use kernel::of::DeviceId;
use kernel::platform;
use kernel::str::CString;
use kernel::sync::Arc;

use kernel::c_str;
use kernel::container_of;
use kernel::define_of_id_table;
use kernel::module_platform_driver;

/// I2C 地址预留空间
const I2C_SIZE: usize = 0x100;

/// I2C 控制寄存器地址偏移
pub const BCM2835_I2C_C: usize = 0x0;
/// I2C 状态寄存器地址偏移
pub const BCM2835_I2C_S: usize = 0x4;
/// I2C 数据长度寄存器地址偏移
pub const BCM2835_I2C_DLEN: usize = 0x8;
/// I2C 从机地址寄存器地址偏移
pub const BCM2835_I2C_A: usize = 0xc;
/// I2C 数据 FIFO 寄存器地址偏移
pub const BCM2835_I2C_FIFO: usize = 0x10;
/// I2C 时钟分频寄存器地址偏移
pub const BCM2835_I2C_DIV: usize = 0x14;
/// I2C 数据延迟寄存器地址偏移
pub const BCM2835_I2C_DEL: usize = 0x18;
/// 16-bit field for the number of SCL cycles to wait after rising SCL
/// before deciding the slave is not responding. 0 disables the
/// timeout detection.
/// I2C 时钟超时寄存器地址偏移
pub const BCM2835_I2C_CLKT: usize = 0x1c;

// I2C control register bit fields
pub const BCM2835_I2C_C_READ: u32 = 1 << 0;
pub const BCM2835_I2C_C_CLEAR: u32 = 1 << 4; /* bits 4 and 5 both clear */
pub const BCM2835_I2C_C_ST: u32 = 1 << 7;
pub const BCM2835_I2C_C_INTD: u32 = 1 << 8;
pub const BCM2835_I2C_C_INTT: u32 = 1 << 9;
pub const BCM2835_I2C_C_INTR: u32 = 1 << 10;
pub const BCM2835_I2C_C_I2CEN: u32 = 1 << 15;

// I2C status register bit fields
pub const BCM2835_I2C_S_TA: u32 = 1 << 0;
pub const BCM2835_I2C_S_DONE: u32 = 1 << 1;
pub const BCM2835_I2C_S_TXW: u32 = 1 << 2;
pub const BCM2835_I2C_S_RXR: u32 = 1 << 3;
pub const BCM2835_I2C_S_TXD: u32 = 1 << 4;
pub const BCM2835_I2C_S_RXD: u32 = 1 << 5;
pub const BCM2835_I2C_S_TXE: u32 = 1 << 6;
pub const BCM2835_I2C_S_RXF: u32 = 1 << 7;
pub const BCM2835_I2C_S_ERR: u32 = 1 << 8;
pub const BCM2835_I2C_S_CLKT: u32 = 1 << 9;
pub const BCM2835_I2C_S_LEN: u32 = 1 << 10; /* Fake bit for SW error reporting */

pub const BCM2835_I2C_FEDL_SHIFT: u32 = 16;
pub const BCM2835_I2C_REDL_SHIFT: u32 = 0;

pub const BCM2835_I2C_CDIV_MIN: u32 = 0x0002;
pub const BCM2835_I2C_CDIV_MAX: u32 = 0xFFFE;

/// SMBUs-recommended 35ms
pub const CLK_TOUT_MS: u32 = 35;

/// Maximum number of debug messages
pub const BCM2835_DEBUG_MAX: usize = 512;

/// TODO: Wait for implemented Debug feature
struct Bcm2835Debug {
    // TODO: Implement i2c_msg
    // msg:
    msg_idx: i32,
    remain: usize,
    status: u32,
}

/// TODO: Wait for implemented device struct
struct Bcm2835I2cDev {
    dev: Device,
    regs: IoMem<I2C_SIZE>,
    irq: i32,
    // TODO: Implement adapter
    // adapter:
    completion: Completion,
    // TODO: Implement i2c_msg
    // curr_msg:
    bus_clk: Clk,
    num_msgs: i32,
    // omit debug fields for now.
    msg_err: u32,
    // May Wrong! Vec<u8> for *mut u8
    // C use u8* as a pointer msg_buf
    // Use Vec<u8> as a stack buf
    msg_buf: Vec<u8>,
    msg_buf_remaining: usize,
    debug: [Bcm2835Debug; BCM2835_DEBUG_MAX],
    debug_num: u32,
    debug_num_msgs: u32,
}

struct Bcm2835I2cDevice {
    drv_reg: Pin<Box<platform::Registration<Bcm2835I2cDriver>>>,
}

module! {
    type: Bcm2835I2cDevice,
    name:"i2c_bcm2835_rust",
    author:"<NAME> <<EMAIL>>",
    description:"BCM2835 I2C driver (written in rust)",
    license:"GPL",
    alias: ["platform:i2c_bcm2835"],
    params: {
        debug: u32 {
            default: 0,
            permissions: 0o644,
            description: "1=err, 2=isr, 3=xfer",
        },
        clk_out_ms: u32 {
            default: 35,
            permissions: 0o644,
            description: "clock-stretch timeout (mS)",
        },
    },
}

fn bcm2835_i2c_writel(i2c_dev: &mut Bcm2835I2cDev, reg: usize, val: u32) {
    let i2c_reg = i2c_dev.regs.get();
    let addr = i2c_reg.wrapping_add(reg);
    unsafe { bindings::writel(val as _, addr as _) }
}

fn bcm2835_i2c_readl(i2c_dev: &mut Bcm2835I2cDev, reg: usize) -> u32 {
    let i2c_reg = i2c_dev.regs.get();
    let addr = i2c_reg.wrapping_add(reg);
    unsafe { bindings::readl(addr as _) }
}

fn to_clk_bcm2835_i2c(hw_ptr: &ClkHw) -> &mut ClkBcm2835I2c {
    unsafe { &mut *(container_of!(hw_ptr, ClkBcm2835I2c, hw) as *mut ClkBcm2835I2c) }
}

struct ClkBcm2835I2c {
    hw: ClkHw,
    i2c_dev: &'static mut Bcm2835I2cDev,
}

impl ClkBcm2835I2c {
    fn from_raw<'a>(ptr: *mut Self) -> &'a mut Self {
        let prt = ptr.cast::<Self>();
        unsafe { &mut *prt }
    }
}

fn clk_bcm2835_i2c_calc_divider(rate: u64, parent_rate: u64) -> Result<u64> {
    let mut divider = parent_rate.div_ceil(rate) as u32;

    /*
     * Per the datasheet, the register is always interpreted as an even
     * number, by rounding down. In other words, the LSB is ignored. So,
     * if the LSB is set, increment the divider to avoid any issue.
     */
    if (divider & 0x1) != 0 {
        divider += 1;
    }
    if (divider < BCM2835_I2C_CDIV_MIN) || (divider > BCM2835_I2C_CDIV_MAX) {
        return Err(EINVAL);
    }

    return Ok(divider as u64);
}

fn clk_bcm2835_i2c_set_rate(hw: &ClkHw, rate: u64, parent_rate: u64) -> Result<()> {
    let div = to_clk_bcm2835_i2c(hw);
    let divider = clk_bcm2835_i2c_calc_divider(rate, parent_rate)?;

    bcm2835_i2c_writel(&mut div.i2c_dev, BCM2835_I2C_DIV, divider as u32);

    /*
     * Number of core clocks to wait after falling edge before
     * outputting the next data bit.  Note that both FEDL and REDL
     * can't be greater than CDIV/2.
     */
    let fedl = 1.max(divider / 16);

    /*
     * Number of core clocks to wait after rising edge before
     * sampling the next incoming data bit.
     */
    let redl = 1.max(divider / 4);

    bcm2835_i2c_writel(
        &mut div.i2c_dev,
        BCM2835_I2C_DEL,
        ((fedl << BCM2835_I2C_FEDL_SHIFT) | (redl << BCM2835_I2C_REDL_SHIFT)) as u32,
    );

    /*
     * Set the clock stretch timeout.
     */
    let clk_tout: u32;
    let rate = rate as u32;
    if rate > 0xffff * 1000 / CLK_TOUT_MS {
        clk_tout = 0xffff;
    } else {
        clk_tout = CLK_TOUT_MS * rate / 1000;
    }

    bcm2835_i2c_writel(&mut div.i2c_dev, BCM2835_I2C_CLKT, clk_tout);

    Ok(())
}

fn clk_bcm2835_i2c_round_rate(hw: &ClkHw, rate: u64, parent_rate: &mut u64) -> i64 {
    let Ok(divider) = clk_bcm2835_i2c_calc_divider(rate, *parent_rate) else {
        return 0;
    };

    parent_rate.div_ceil(divider) as i64
}

fn clk_bcm2835_i2c_recalc_rate(hw: &ClkHw, parent_rate: u64) -> u64 {
    let div = to_clk_bcm2835_i2c(hw);
    let divider = bcm2835_i2c_readl(&mut div.i2c_dev, BCM2835_I2C_DIV) as u64;

    parent_rate.div_ceil(divider)
}

struct ClkBcm2835I2cOps;

#[vtable]
impl ClkOps for ClkBcm2835I2cOps {
    fn set_rate(hw: &ClkHw, rate: u64, parent_rate: u64) -> Result<()> {
        clk_bcm2835_i2c_set_rate(hw, rate, parent_rate)
    }

    fn round_rate(hw: &ClkHw, rate: u64, parent_rate: &mut u64) -> i64 {
        clk_bcm2835_i2c_round_rate(hw, rate, parent_rate)
    }

    fn recalc_rate(hw: &ClkHw, parent_rate: u64) -> u64 {
        clk_bcm2835_i2c_recalc_rate(hw, parent_rate)
    }
}

impl Bcm2835I2cDev {
    pub(crate) fn bcm2835_i2c_register_div(
        dev: &'static mut Device,
        mclk: &Clk,
        i2c_dev: &'static mut Bcm2835I2cDev,
    ) -> Result<&'static mut Clk> {
        let name = CString::try_from_fmt(fmt!("{}_div", dev.name()))?;
        let mclk_name = mclk.name();
        let parent_names = [mclk_name.as_char_ptr()];
        // Here: impl device.rs Device struct
        // devm_alloc::<ClkBcm2835I2c>
        let clk_i2c = unsafe {
            let raw_ptr = dev.kzalloc::<ClkBcm2835I2c>()?;
            let clk_i2c = ClkBcm2835I2c::from_raw(raw_ptr);
            let init_data = ClkInitData::new()
                .name_config(&name, &parent_names)
                .ops::<ClkBcm2835I2cOps>()
                .flags(0);
            clk_i2c.hw.set_init_data(&init_data);
            clk_i2c.i2c_dev = i2c_dev;

            clk_i2c.hw.register_clkdev(c_str!("div"), dev.name())?;

            clk_i2c
        };

        // Ensure these objects live long enough
        // TODO: Try to achieve this in a more elegant way
        // let _ = (name, parent_names, init_data);

        dev.clk_register(&mut clk_i2c.hw)
    }

    pub(crate) fn bcm2835_fill_txfifo(i2c_dev: &'static mut Bcm2835I2cDev) {
        while i2c_dev.msg_buf_remaining > 0 {
            let val: u32 = bcm2835_i2c_readl(i2c_dev, BCM2835_I2C_S);
            if !(val & BCM2835_I2C_S_TXD) != 0 {
                break;
            }
            // May Wrong!
            // Safety: Copy and retain the element.
            let idx = i2c_dev.msg_buf.len() - 1;
            let cur = i2c_dev.msg_buf[idx] as u32;
            bcm2835_i2c_writel(i2c_dev, BCM2835_I2C_FIFO, cur);
            i2c_dev.msg_buf_remaining -= 1;
        }
    }

    pub(crate) fn bcm2835_fill_rxfifo(i2c_dev: &'static mut Bcm2835I2cDev) {
        while i2c_dev.msg_buf_remaining > 0 {
            let val: u32 = bcm2835_i2c_readl(i2c_dev, BCM2835_I2C_S);
            if !(val & BCM2835_I2C_S_RXD) != 0 {
                break;
            }
            // May Wrong!
            let cur = bcm2835_i2c_readl(i2c_dev, BCM2835_I2C_FIFO) as u8;
            // Safety: msg_buf_remaining > 0
            let _ = i2c_dev.msg_buf.try_push(cur);
            i2c_dev.msg_buf_remaining -= 1;
        }
    }
}

struct Bcm2835I2cData {}

impl DeviceRemoval for Bcm2835I2cData {
    fn device_remove(&self) {
        // TODO: remove i2c device data
    }
}

kernel::module_of_id_table!(BCM2835_I2C_MOD_TABLE, BCM2835_I2C_ID_TABLE);

kernel::define_of_id_table! {BCM2835_I2C_ID_TABLE, (), [
    (DeviceId::Compatible(b"brcm,bcm2711-i2c"), None),
    (DeviceId::Compatible(b"brcm,bcm2835-i2c"), None),
]}

struct Bcm2835I2cDriver;
impl platform::Driver for Bcm2835I2cDriver {
    kernel::driver_of_id_table!(BCM2835_I2C_ID_TABLE);
    // type Data = Arc<Bcm2835I2cData>;
    type Data = ();

    fn probe(
        dev: &mut platform::Device,
        id_info: core::prelude::v1::Option<&Self::IdInfo>,
    ) -> Result<Self::Data> {
        // let pdev = dev.
        // TODO: initialize and probe i2c driver
        Ok(())
    }

    fn remove(_data: &Self::Data) -> Result {
        // TODO: remove i2c driver
        Ok(())
    }

    // TODO: complete the table
    // define_of_id_table! {(), [
    //     (of::DeviceId::Compatible(b"brcm,bcm2711-i2c"), None),
    //     (of::DeviceId::Compatible(b"brcm,bcm2835-i2c"), None),
    // ]}
}

impl kernel::Module for Bcm2835I2cDevice {
    fn init(module: &'static ThisModule) -> Result<Self> {
        pr_info!("BCM2835 i2c bus device driver (init)\n");

        let drv_reg =
            platform::Registration::<Bcm2835I2cDriver>::new_pinned(c_str!("i2c-bcm2835"), module)?;

        Ok(Bcm2835I2cDevice { drv_reg })
    }
}

impl Drop for Bcm2835I2cDevice {
    fn drop(&mut self) {
        pr_info!("BCM2835 i2c bus device driver (exit)\n");
    }
}
