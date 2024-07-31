// SPDX-License-Identifier: GPL-2.0

//! BCM2835 master mode driver

use kernel::clk_provider::{ClkInitData, ClkOps};
use kernel::prelude::*;

use kernel::bindings;
use kernel::clk::Clk;
use kernel::clk_provider::ClkHw;
use kernel::completion::Completion;
use kernel::container_of;
use kernel::device::Device;
use kernel::device::RawDevice;
use kernel::io_mem::IoMem;

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

/// TODO: Wait for implemented Debug feature
struct Bcm2835Debug {
    // msg: I2cMsgFlags,
    msg_idx: i32,
    remian: usize,
    status: u32,
}

/// TODO: Wait for implemented device struct
struct Bcm2835I2cDev {
    dev: Device,
    irq: i32,
    regs: IoMem<I2C_SIZE>,
    // adapter: Adapter<Bcm2835I2cDev>,
    completion: Completion,
    // curr_msg: I2cMsg,
    bus_clk: Clk,
    num_msgs: i32,
    // omit debug fields for now.
    // msg_err: u32,
    // msg_buf: Vec<u8>,
    // msg_buf_remaining: usize,
    // debug: [Option<BCM2835_Debug>; 4]
    // debug_num: u32,
    // debug_num_msgs: u32,
}

struct Bcm2835I2cData {}

struct Bcm2835I2cDevice;

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

impl Bcm2835I2cDevice {
    fn register_div(&mut self, dev: &mut Device, mclk: &Clk) -> Result<&mut Clk> {
        let name = dev.name();
        let mclk_name = mclk.name();

        // TODO: clk_ops implementation
        todo!()
    }
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
    i2c_dev: Bcm2835I2cDev,
}

impl Clk for ClkBcm2835I2c {
    fn new(hw: ClkHw, i2c_dev: Bcm2835I2cDev) -> Self {
        ClkBcm2835I2c { hw, i2c_dev }
    }
}

fn clk_bcm2835_i2c_calc_divider(rate: u64, parent_rate: u64) -> Result<u64> {
    let mut divider = parent_rate.div_ceil(rate);

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

    return Ok(divider);
}

fn clk_bcm2835_i2c_set_rate(hw: &ClkHw, rate: u64, parent_rate: u64) -> Result<()> {
    let div = to_clk_bcm2835_i2c(hw);
    let divider = clk_bcm2835_i2c_calc_divider(rate, parent_rate)?;

    bcm2835_i2c_writel(&mut div.i2c_dev, BCM2835_I2C_DIV, divider);

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
        (fedl << BCM2835_I2C_FEDL_SHIFT) | (redl << BCM2835_I2C_REDL_SHIFT),
    );

    /*
     * Set the clock stretch timeout.
     */
    let clk_tout: u32;
    if rate > 0xffff * 1000 / CLK_TOUT_MS {
        clk_tout = 0xffff;
    } else {
        clk_tout = CLK_TOUT_MS * rate / 1000;
    }

    bcm2835_i2c_writel(&mut div.i2c_dev, BCM2835_I2C_CLKT, clk_tout);

    Ok(())
}

fn clk_bcm2835_i2c_round_rate(hw: &ClkHw, rate: u64, parent_rate: &mut u64) -> i64 {
    let divider = clk_bcm2835_i2c_calc_divider(rate, *parent_rate).unwrap();

    parent_rate.div_ceil(divider)
}

fn clk_bcm2835_i2c_recalc_rate(hw: &ClkHw, parent_rate: u64) -> u32 {
    let div = to_clk_bcm2835_i2c(hw);
    let divider = bcm2835_i2c_readl(&mut div.i2c_dev, BCM2835_I2C_DIV);

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

    fn recalc_rate(hw: &ClkHw, parent_rate: u64) -> u32 {
        clk_bcm2835_i2c_recalc_rate(hw, parent_rate)
    }
}

fn bcm2835_i2c_register_div(dev: &mut Device, mclk: &Clk, i2c_dev: &mut Bcm2835I2cDev) -> Result<&Clk> {
    let name:&CStr = format!("i2c_div_{}", dev.name());
    let mclk_name = mclk.name();
    let parent_names = &[mclk_name];

    let init = ClkInitData::new().ops::<ClkBcm2835I2cOps>().name_config(name, parent_names).flags(0);

}

impl kernel::Module for Bcm2835I2cDevice {
    fn init(_module: &'static ThisModule) -> Result<Self> {
        pr_info!("BCM2835 i2c bus device driver (init)\n");

        Ok(Bcm2835I2cDevice {})
    }
}

impl Drop for Bcm2835I2cDevice {
    fn drop(&mut self) {
        pr_info!("BCM2835 i2c bus device driver (exit)\n");
    }
}
