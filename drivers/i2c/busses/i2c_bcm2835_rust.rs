// SPDX-License-Identifier: GPL-2.0

//! BCM2835 master mode driver
use core::iter::IntoIterator;

use kernel::{
    bindings,
    clk::Clk,
    clk_provider::{ClkHw, ClkInitData, ClkOps},
    completion::Completion,
    device::{self, Device, RawDevice},
    driver::DeviceRemoval,
    error::to_result,
    i2c::{
        self, I2cAdapter, I2cAdapterQuirks, I2cAlgorithm, I2cMsg, I2C_M_IGNORE_NAK, I2C_M_NOSTART,
        I2C_M_NO_RD_ACK, I2C_M_RD, I2C_M_RECV_LEN, I2C_M_REV_DIR_ADDR, I2C_M_STOP, I2C_M_TEN,
    },
    io_mem::IoMem,
    irq,
    of::DeviceId,
    platform,
    prelude::*,
    str::CString,
    sync::Arc,
    {c_str, container_of, define_of_id_table, module_platform_driver, new_completion},
};

/// I2C 地址预留空间
const I2C_SIZE: usize = 0x200;

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

pub const DEBUG: i32 = 3;

/// SMBUs-recommended 35ms
pub const CLK_TOUT_MS: u32 = 35;

/// Maximum number of debug messages
pub const BCM2835_DEBUG_MAX: usize = 512;
struct Bcm2835Debug {
    msg: I2cMsg,
    msg_idx: i32,
    remain: usize,
    status: u32,
}

// May Wrong! Vec<T> for *mut T
// C use u8* as a ptr msg_buf,i2c_msg* as a ptr i2c_msg
// C use NULL, Rust use None.
// Use Vec<T> as a ptr buf
struct Bcm2835I2cDev {
    dev: Device,
    reg_base: *mut u8,
    irq: i32,
    adapter: I2cAdapter,
    completion: Completion,
    curr_msg: Option<Vec<I2cMsg>>,
    curr_msg_idx: usize,
    bus_clk: Clk,
    num_msgs: i32,
    msg_err: u32,
    msg_buf: Option<Vec<u8>>,
    msg_buf_remaining: usize,
    debug: [Bcm2835Debug; BCM2835_DEBUG_MAX],
    debug_num: u32,
    debug_num_msgs: u32,
}

unsafe impl Sync for Bcm2835I2cDev {}
unsafe impl Send for Bcm2835I2cDev {}

impl Bcm2835I2cDev {
    unsafe fn from_ptr<'a>(ptr: *mut Self) -> &'a mut Self {
        unsafe { &mut *ptr.cast() }
    }

    unsafe fn as_ptr(&self) -> *mut Self {
        self as *const _ as *mut _
    }
}

fn to_clk_bcm2835_i2c(hw_ptr: &ClkHw) -> &mut ClkBcm2835I2c<'_> {
    unsafe { &mut *(container_of!(hw_ptr, ClkBcm2835I2c<'_>, hw) as *mut ClkBcm2835I2c<'_>) }
}

struct ClkBcm2835I2c<'c> {
    hw: ClkHw,
    i2c_dev: &'c Bcm2835I2cDev,
}

impl<'c> ClkBcm2835I2c<'c> {
    fn from_raw<'a>(ptr: *mut Self) -> &'a mut Self {
        let ptr = ptr.cast::<Self>();
        unsafe { &mut *ptr }
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
    let divider = clk_bcm2835_i2c_calc_divider(rate, parent_rate)? as u32;

    div.i2c_dev.bcm2835_i2c_writel(BCM2835_I2C_DIV, divider);

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

    div.i2c_dev.bcm2835_i2c_writel(
        BCM2835_I2C_DEL,
        ((fedl << BCM2835_I2C_FEDL_SHIFT) | (redl << BCM2835_I2C_REDL_SHIFT)) as u32,
    );

    /*
     * Set the clock stretch timeout.
     */
    let clk_tout: u32 = {
        if unsafe { rate as u32 > 0xffff * 1000 / CLK_TOUT_MS } {
            0xffff
        } else {
            unsafe { CLK_TOUT_MS * rate as u32 / 1000 }
        }
    };

    div.i2c_dev.bcm2835_i2c_writel(BCM2835_I2C_CLKT, clk_tout);

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
    let divider = div.i2c_dev.bcm2835_i2c_readl(BCM2835_I2C_DIV) as u64;

    parent_rate.div_ceil(divider)
}

struct ClkBcm2835I2cOps;

#[vtable]
impl ClkOps for ClkBcm2835I2cOps {
    fn set_rate(hw: &ClkHw, rate: u64, parent_rate: u64) -> Result<()> {
        clk_bcm2835_i2c_set_rate(hw, rate, parent_rate)
    }

    // bad implementation! u64 and u32 in diff env.
    fn round_rate(hw: &ClkHw, rate: u64, parent_rate: &mut u64) -> i64 {
        clk_bcm2835_i2c_round_rate(hw, rate, parent_rate).into()
    }

    fn recalc_rate(hw: &ClkHw, parent_rate: u64) -> u64 {
        clk_bcm2835_i2c_recalc_rate(hw, parent_rate)
    }
}

impl Bcm2835I2cDev {
    pub(crate) fn bcm2835_i2c_writel(&self, reg: usize, val: u32) {
        let i2c_reg = self.reg_base;
        let addr = i2c_reg.wrapping_add(reg);
        unsafe { bindings::writel(val as _, addr as _) }
    }

    pub(crate) fn bcm2835_i2c_readl(&self, reg: usize) -> u32 {
        let i2c_reg = self.reg_base;
        let addr = i2c_reg.wrapping_add(reg);
        unsafe { bindings::readl(addr as _) }
    }

    pub(crate) fn bcm2835_i2c_register_div(&mut self, mclk: &Clk) -> Result<&mut Clk> {
        let name = CString::try_from_fmt(fmt!("{}_div", self.dev.name()))?;
        let mclk_name = mclk.name();
        let parent_names = [mclk_name.as_char_ptr()];
        let clk_i2c = unsafe {
            let raw_ptr = self.dev.kzalloc::<ClkBcm2835I2c<'_>>()?;
            let clk_i2c = ClkBcm2835I2c::from_raw(raw_ptr);
            let init_data = ClkInitData::new()
                .name_config(&name, &parent_names)
                .set_ops::<ClkBcm2835I2cOps>()
                .set_flags(0);
            clk_i2c.hw.set_init_data(&init_data);
            clk_i2c.i2c_dev = self;

            clk_i2c.hw.register_clkdev(c_str!("div"), self.dev.name())?;

            clk_i2c
        };

        // Ensure these objects live long enough
        // TODO: Try to achieve this in a more elegant way
        // let _ = (name, parent_names, init_data);

        self.dev.clk_register(&mut clk_i2c.hw)
    }

    pub(crate) fn bcm2835_fill_txfifo(&mut self) {
        pr_info!("fill txfifo:");
        while self.msg_buf_remaining > 0 {
            let val: u32 = self.bcm2835_i2c_readl(BCM2835_I2C_S);
            if val & BCM2835_I2C_S_TXD == 0 {
                break;
            }

            // May Wrong!
            if let Some(buf) = self.msg_buf.as_ref() {
                // Safety: msg_buf_remaining > 0
                let idx = buf.len() - self.msg_buf_remaining;
                let byte = buf[idx] as u32;
                pr_info!("\t{:x}", byte);
                self.bcm2835_i2c_writel(BCM2835_I2C_FIFO, byte);
                self.msg_buf_remaining -= 1;
            };
        }
        pr_info!("\n");
    }

    pub(crate) fn bcm2835_drain_rxfifo(&mut self) {
        pr_info!("drain rxfifo:");
        while self.msg_buf_remaining > 0 {
            let val: u32 = self.bcm2835_i2c_readl(BCM2835_I2C_S);
            if val & BCM2835_I2C_S_RXD == 0 {
                break;
            }

            // May Wrong!
            // if let Some(buf) = self.msg_buf.as_mut() {}
            if let Some(mut buf) = self.msg_buf.take() {
                let idx = buf.len() - self.msg_buf_remaining;
                buf[idx] = self.bcm2835_i2c_readl(BCM2835_I2C_FIFO) as u8;
                pr_info!("\t{:x}", buf[idx]);
                self.msg_buf = Some(buf);
                self.msg_buf_remaining -= 1;
            }
        }
        pr_info!("\n");
        // let data = self.curr_msg[self.curr_msg_idx].buf_ptr();
        let Some(msgs) = self.curr_msg.as_mut() else {
            return;
        };
        let mut msg = msgs[self.curr_msg_idx].clone();
        msg.write_to_buf(self.msg_buf.as_ref().unwrap());
    }

    pub(crate) fn bcm2835_i2c_start_transfer(&mut self) {
        let mut c: u32 = BCM2835_I2C_C_ST | BCM2835_I2C_C_I2CEN;
        // Safely extract and process the current message
        if let Some(mut curr_msg) = self.curr_msg.take() {
            if self.num_msgs == 0 {
                // self.curr_msg = None;
                // self.curr_msg_idx = 0;
                return;
            }

            pr_info!("bcm2835_i2c_start_transfer: {}", self.curr_msg_idx);
            let msg = &curr_msg[self.curr_msg_idx];
            let last_msg = self.num_msgs == 1;

            self.num_msgs -= 1;
            self.msg_buf = msg.buf_to_vec();
            self.msg_buf_remaining = msg.len() as usize;

            if msg.flags() as u32 & I2C_M_RD != 0 {
                c |= BCM2835_I2C_C_READ | BCM2835_I2C_C_INTR;
            } else {
                c |= BCM2835_I2C_C_INTT;
            }

            if last_msg {
                c |= BCM2835_I2C_C_INTD;
            }

            self.bcm2835_i2c_writel(BCM2835_I2C_A, msg.addr() as u32);
            self.bcm2835_i2c_writel(BCM2835_I2C_DLEN, msg.len() as u32);
            self.bcm2835_i2c_writel(BCM2835_I2C_C, c);
            self.curr_msg = Some(curr_msg);
            debug_add(self, !0);
        }
    }

    pub(crate) fn bcm2835_i2c_finish_transfer(&mut self) {
        self.curr_msg = None;
        self.curr_msg_idx = 0;
        self.num_msgs = 0;

        self.msg_buf = None;
        self.msg_buf_remaining = 0;
    }
}

fn bcm2835_i2c_isr(this_irq: i32, i2c_dev: &mut Bcm2835I2cDev) -> irq::Return {
    // let i2c_dev = unsafe { &mut *data.i2c_dev_ptr.cast::<Bcm2835I2cDev>() };
    dev_info!(
        i2c_dev.dev,
        "Interrupt callback {}, i2c_dev_ptr: {:?}\n",
        this_irq,
        unsafe { i2c_dev.as_ptr() }
    );

    let mut val: u32 = i2c_dev.bcm2835_i2c_readl(BCM2835_I2C_S);
    dev_info!(i2c_dev.dev, "i2c status val: {:x}\n", val);
    if i2c_dev.curr_msg.is_some() {
        debug_add(i2c_dev, val);
    }

    let err: u32 = val & (BCM2835_I2C_S_CLKT | BCM2835_I2C_S_ERR);
    if err != 0 && (val & BCM2835_I2C_S_TA) == 0 {
        i2c_dev.msg_err = err;
        dev_err!(i2c_dev.dev, "i2c device error: {:x}\n", err);
    }

    if val & BCM2835_I2C_S_DONE != 0 {
        match i2c_dev.curr_msg {
            // Note: we represent the ptr buf with vec and the ptr to 0th element is the same as the ptr to the place in C.
            Some(ref msgs) if msgs[i2c_dev.curr_msg_idx].flags() as u32 & I2C_M_RD != 0 => {
                i2c_dev.bcm2835_drain_rxfifo();
                val = i2c_dev.bcm2835_i2c_readl(BCM2835_I2C_S);
                pr_info!("drain readl: 0x{:04X}", val);
            }
            None => {
                dev_err!(i2c_dev.dev, "Got unexpected interrupt (from firmware?)\n");
                // return irq::Return::Handled;
            }
            _ => {}
        }

        if (val & BCM2835_I2C_S_RXD) != 0 || i2c_dev.msg_buf_remaining != 0 {
            i2c_dev.msg_err = BCM2835_I2C_S_LEN;
        }

        return goto_complete(i2c_dev);
    }

    if val & BCM2835_I2C_S_TXW != 0 {
        if i2c_dev.msg_buf_remaining == 0 {
            i2c_dev.msg_err = val | BCM2835_I2C_S_LEN;
            return goto_complete(i2c_dev);
        }

        i2c_dev.bcm2835_fill_txfifo();

        if i2c_dev.num_msgs != 0 && i2c_dev.msg_buf_remaining == 0 {
            // let curr_msg = i2c_dev.curr_msg.as_mut().expect("curr_msg is None");
            // curr_msg.remove(0);
            i2c_dev.curr_msg_idx += 1;
            i2c_dev.bcm2835_i2c_start_transfer();
        }

        return irq::Return::Handled;
    }

    if val & BCM2835_I2C_S_RXR != 0 {
        if i2c_dev.msg_buf_remaining == 0 {
            i2c_dev.msg_err = val | BCM2835_I2C_S_LEN;
            return goto_complete(i2c_dev);
        }

        i2c_dev.bcm2835_drain_rxfifo();
        return irq::Return::Handled;
    }

    irq::Return::None
}

unsafe extern "C" fn bcm2835_i2c_isr_cb(this_irq: i32, data: *mut core::ffi::c_void) -> u32 {
    let i2c_dev = unsafe { Bcm2835I2cDev::from_ptr(data as *mut Bcm2835I2cDev) };
    bcm2835_i2c_isr(this_irq, i2c_dev) as u32
}

fn goto_complete(i2c_dev: &mut Bcm2835I2cDev) -> irq::Return {
    i2c_dev.bcm2835_i2c_writel(BCM2835_I2C_C, BCM2835_I2C_C_CLEAR);
    i2c_dev.bcm2835_i2c_writel(
        BCM2835_I2C_S,
        BCM2835_I2C_CLKT as u32 | BCM2835_I2C_S_ERR | BCM2835_I2C_S_DONE,
    );
    i2c_dev.completion.complete();

    irq::Return::Handled
}

fn debug_print_status(i2c_dev: &Bcm2835I2cDev, d: &Bcm2835Debug) {
    let status = d.status;
    pr_info!(
        "isr: remain={} status=0x{:08X} : {} {} {} {} {} {} {} {} {} {}\n",
        d.remain,
        status,
        if status & BCM2835_I2C_S_TA != 0 {
            "TA "
        } else {
            ""
        },
        if status & BCM2835_I2C_S_DONE != 0 {
            "DONE "
        } else {
            ""
        },
        if status & BCM2835_I2C_S_TXW != 0 {
            "TXW "
        } else {
            ""
        },
        if status & BCM2835_I2C_S_RXR != 0 {
            "RXR "
        } else {
            ""
        },
        if status & BCM2835_I2C_S_TXD != 0 {
            "TXD "
        } else {
            ""
        },
        if status & BCM2835_I2C_S_RXD != 0 {
            "RXD "
        } else {
            ""
        },
        if status & BCM2835_I2C_S_TXE != 0 {
            "TXE "
        } else {
            ""
        },
        if status & BCM2835_I2C_S_RXF != 0 {
            "RXF "
        } else {
            ""
        },
        if status & BCM2835_I2C_S_ERR != 0 {
            "ERR "
        } else {
            ""
        },
        if status & BCM2835_I2C_S_CLKT != 0 {
            "CLKT "
        } else {
            ""
        },
    )
}

fn debug_print_msg(i2c_dev: &Bcm2835I2cDev, msg: &I2cMsg, idx: i32, total: i32, fname: &str) {
    let flags = msg.flags() as u32;
    pr_info!(
        "{fname}: msg({idx}/{total}) {} addr=0x{:02x}, len={} flags={}{}{}{}{}{}{} [i2c{}]\n",
        if flags & I2C_M_RD != 0 {
            "read"
        } else {
            "write"
        },
        msg.addr(),
        msg.len(),
        if flags & I2C_M_TEN != 0 { "TEN" } else { "" },
        if flags & I2C_M_RECV_LEN != 0 {
            "RECV_LEN"
        } else {
            ""
        },
        if flags & I2C_M_NO_RD_ACK != 0 {
            "NO_RD_ACK"
        } else {
            ""
        },
        if flags & I2C_M_IGNORE_NAK != 0 {
            "IGNORE_NAK"
        } else {
            ""
        },
        if flags & I2C_M_REV_DIR_ADDR != 0 {
            "REV_DIR_ADDR"
        } else {
            ""
        },
        if flags & I2C_M_NOSTART != 0 {
            "NOSTART"
        } else {
            ""
        },
        if flags & I2C_M_STOP != 0 { "STOP" } else { "" },
        i2c_dev.adapter.0.nr,
    );
}

fn debug_print(i2c_dev: &Bcm2835I2cDev, fname: &str) {
    if i2c_dev.debug_num >= BCM2835_DEBUG_MAX as u32 {
        pr_info!("BCM2835_DEBUG_MAX reached\n");
        return;
    }
    for d in &i2c_dev.debug[..i2c_dev.debug_num as usize] {
        if d.status == !0 {
            debug_print_msg(
                i2c_dev,
                &d.msg,
                d.msg_idx,
                i2c_dev.debug_num_msgs as i32,
                fname,
            );
        } else {
            debug_print_status(i2c_dev, d);
        }
    }
}

fn debug_add(i2c_dev: &mut Bcm2835I2cDev, s: u32) {
    dev_info!(
        i2c_dev.dev,
        "debug_add[{}]: status=0x{:08X}\n",
        i2c_dev.debug_num,
        s
    );
    if i2c_dev.debug_num_msgs == 0 || i2c_dev.debug_num >= BCM2835_DEBUG_MAX as u32 {
        return;
    }

    let n = i2c_dev.debug_num as usize;

    i2c_dev.debug[n].msg = i2c_dev.curr_msg.as_ref().unwrap()[i2c_dev.curr_msg_idx].clone();
    i2c_dev.debug[n].msg_idx = i2c_dev.debug_num_msgs as i32 - i2c_dev.num_msgs;
    i2c_dev.debug[n].remain = i2c_dev.msg_buf_remaining;
    i2c_dev.debug[n].status = s;
    i2c_dev.debug_num += 1;
}

fn bcm2835_i2c_xfer(adap: &mut I2cAdapter, msgs: Vec<I2cMsg>, num: i32) -> Result<i32> {
    let i2c_dev = unsafe { &mut (*adap.i2c_get_adapdata::<Bcm2835I2cDev>()) };
    let mut ignore_nak = false;

    if DEBUG > 0 {
        i2c_dev.debug_num_msgs = num as u32;
    }

    if DEBUG > 2 {
        let mut idx = 1;
        for msg in &msgs {
            debug_print_msg(i2c_dev, &msg, idx, num, "i2c_xfer");
            idx += 1;
        }
    }

    dev_info!(i2c_dev.dev, "i2c_xfer: num = {}\n", num);

    for msg in &msgs[0..msgs.len() - 1] {
        dev_info!(
            i2c_dev.dev,
            "  msg: addr = 0x{:x}, flags = 0x{:x}, len = {}\n",
            msg.addr(),
            msg.flags(),
            msg.len()
        );

        if msg.flags() as u32 & I2C_M_RD != 0 {
            dev_warn!(
                i2c_dev.dev,
                "only one read message supported, has to be last\n"
            );
            return Err(ENOTSUPP);
        }
        if msg.flags() as u32 & I2C_M_IGNORE_NAK != 0 {
            ignore_nak = true;
        }
    }

    i2c_dev.curr_msg = Some(msgs);
    i2c_dev.curr_msg_idx = 0;
    i2c_dev.num_msgs = num;
    i2c_dev.msg_err = 0;
    i2c_dev.completion.reinit();

    i2c_dev.bcm2835_i2c_start_transfer();

    let time_left = i2c_dev
        .completion
        .wait_for_completion_timeout_sec(adap.timeout());

    i2c_dev.bcm2835_i2c_finish_transfer();

    if ignore_nak {
        i2c_dev.msg_err &= !BCM2835_I2C_S_ERR;
    }

    if DEBUG > 1 || (DEBUG != 0 && (time_left == 0 || i2c_dev.msg_err != 0)) {
        debug_print(i2c_dev, "start_transfer");
    }

    i2c_dev.debug_num_msgs = 0;
    i2c_dev.debug_num = 0;

    if time_left == 0 {
        i2c_dev.bcm2835_i2c_writel(BCM2835_I2C_C, BCM2835_I2C_C_CLEAR);
        dev_err!(i2c_dev.dev, "i2c transfer timed out\n");
        return Err(ETIMEDOUT);
    }

    if i2c_dev.msg_err == 0 {
        return Ok(num);
    }

    if unsafe { DEBUG != 0 } {
        dev_err!(i2c_dev.dev, "i2c transfer failed: {}\n", i2c_dev.msg_err);
    }

    if i2c_dev.msg_err & BCM2835_I2C_S_ERR != 0 {
        return Err(EREMOTEIO);
    }

    Err(EIO)
}

fn bcm2835_i2c_func(adap: &I2cAdapter) -> u32 {
    let f = i2c::I2C_FUNC_I2C | i2c::I2C_FUNC_SMBUS_EMUL | i2c::I2C_FUNC_PROTOCOL_MANGLING;
    pr_info!("i2c func = 0x{:x}\n", f);
    f
}

// I2C_AQ ..
// const BCM2835_I2C_QUIRKS: I2cAdapterQuirks =
//    I2cAdapterQuirks::new().set_flags(i2c::I2C_AQ_NO_CLK_STRETCH as u64);

struct Bcm2835I2cAlgo;

#[vtable]
impl I2cAlgorithm for Bcm2835I2cAlgo {
    fn master_xfer(adap: &mut I2cAdapter, msgs: Vec<I2cMsg>, num: i32) -> Result<i32> {
        bcm2835_i2c_xfer(adap, msgs, num)
    }

    fn functionality(adap: &mut I2cAdapter) -> u32 {
        bcm2835_i2c_func(adap)
    }
}

struct Bcm2835I2cData {
    pub(crate) dev: Device,
    pub(crate) i2c_dev_ptr: *mut Bcm2835I2cDev,
}

unsafe impl Sync for Bcm2835I2cData {}
unsafe impl Send for Bcm2835I2cData {}

struct Bcm2835I2cDriver;

module_platform_driver! {
    type: Bcm2835I2cDriver,
    name:"i2c_bcm2835_rust",
    author:"<NAME> <<EMAIL>>",
    description:"BCM2835 I2C bus driver (written in rust)",
    license:"GPL",
    initcall: "arch",
    alias: ["platform:i2c-bcm2835"],
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

kernel::module_of_id_table!(BCM2835_I2C_MOD_TABLE, BCM2835_I2C_ID_TABLE);

kernel::define_of_id_table! {BCM2835_I2C_ID_TABLE, (), [
    (DeviceId::Compatible(b"brcm,bcm2711-i2c"), None),
    (DeviceId::Compatible(b"brcm,bcm2835-i2c"), None),
]}

type DeviceData = device::Data<(), (), Bcm2835I2cData>;

impl platform::Driver for Bcm2835I2cDriver {
    kernel::driver_of_id_table!(BCM2835_I2C_ID_TABLE);

    type Data = Arc<DeviceData>;

    fn probe(
        pdev: &mut platform::Device,
        id_info: core::prelude::v1::Option<&Self::IdInfo>,
    ) -> Result<Self::Data> {
        dev_info!(
            pdev,
            "BCM2835 i2c bus device ({}) driver probe.\n",
            pdev.name()
        );

        let dev = unsafe { Device::from_dev(pdev) };
        dev_info!(pdev, "dev: {:?}\n", dev.raw_device());
        let i2c_dev_ptr: *mut Bcm2835I2cDev = dev.kzalloc::<Bcm2835I2cDev>()?;
        dev_info!(pdev, "i2c_dev_ptr: {:?}\n", i2c_dev_ptr);

        let i2c_dev = unsafe { Bcm2835I2cDev::from_ptr(i2c_dev_ptr) };
        i2c_dev.dev = dev.clone();
        // i2c_dev.completion = Arc::pin_init(new_completion!())?;
        i2c_dev.completion.init_completion();

        let reg_base = pdev.ioremap_resource(0)?;
        i2c_dev.reg_base = reg_base;
        dev_info!(pdev, "I2c bus device reg_base: {:?}\n", reg_base);

        let mclk = dev.clk_get()?;

        let bus_clk = i2c_dev.bcm2835_i2c_register_div(mclk)?;

        let mut bus_clk_rate = 0;
        if let Err(_) = dev.of_property_read_u32(c_str!("clock-frequency"), &mut bus_clk_rate) {
            bus_clk_rate = bindings::I2C_MAX_STANDARD_MODE_FREQ;
        };
        dev_info!(pdev, "I2c bus device clock-frequency: {}\n", bus_clk_rate);

        let ret =
            unsafe { bindings::clk_set_rate_exclusive(bus_clk.as_ptr(), bus_clk_rate as u64) };
        if ret < 0 {
            dev_err!(
                pdev,
                "Could not set clock frequency: {:?}\n",
                to_result(ret)
            );
            to_result(ret)?;
        }

        bus_clk.prepare_enable()?;
        // i2c_dev.bus_clk.prepare_enable()?;

        let irq = pdev.irq_resource(0)?;

        let ret = unsafe {
            bindings::request_threaded_irq(
                irq as u32,
                Some(bcm2835_i2c_isr_cb),
                None,
                bindings::IRQF_SHARED as u64,
                dev.name().as_char_ptr(),
                i2c_dev_ptr as *mut core::ffi::c_void,
            )
        };
        if ret < 0 {
            dev_err!(pdev, "Could not request IRQ: {}\n", irq);
            to_result(ret)?;
        }
        i2c_dev.irq = irq;
        dev_info!(pdev, "I2c bus device IRQ: {}\n", irq);

        // TODO: setup i2c_adapter
        let quirks = I2cAdapterQuirks::new().set_flags(i2c::I2C_AQ_NO_CLK_STRETCH as u64);
        unsafe {
            i2c_dev.adapter.i2c_set_adapdata(i2c_dev.as_ptr());
            // TODO: set owner
            // i2c_dev.adapter.set_owner((&bindings::__this_module) as *const _ as *mut _);
            i2c_dev.adapter.set_class(bindings::I2C_CLASS_DEPRECATED);
            let full_name = bindings::of_node_full_name((*pdev.raw_device()).of_node);
            let adap_name =
                CString::try_from_fmt(fmt!("bcm2835 ({})", CStr::from_char_ptr(full_name)))?;
            i2c_dev.adapter.set_name(&adap_name);
            i2c_dev.adapter.set_algorithm::<Bcm2835I2cAlgo>();
        }

        /*
         * Disable the hardware clock stretching timeout. SMBUS
         * specifies a limit for how long the device can stretch the
         * clock, but core I2C doesn't.
         */
        i2c_dev.bcm2835_i2c_writel(BCM2835_I2C_CLKT, 0);
        i2c_dev.bcm2835_i2c_writel(BCM2835_I2C_C, 0);

        let ret = unsafe { bindings::i2c_add_adapter(i2c_dev.adapter.as_ptr()) };
        if ret < 0 {
            dev_info!(pdev, "Could not add I2C adapter: {:?}\n", to_result(ret));
            unsafe {
                bindings::free_irq(irq as u32, i2c_dev_ptr as *mut core::ffi::c_void);
            }
        }
        let _ = to_result(ret)?;

        let dev_data = kernel::new_device_data!(
            (),
            (),
            Bcm2835I2cData { dev, i2c_dev_ptr },
            "BCM2835_I2C device data"
        )?;

        Ok(dev_data.into())
    }

    fn remove(data: &Self::Data) -> Result {
        pr_info!("BCM2835 i2c bus device driver remove.\n");
        // TODO: remove i2c driver
        Ok(())
    }
}

impl DeviceRemoval for Bcm2835I2cData {
    fn device_remove(&self) {
        pr_info!("BCM2835 i2c bus device driver remove.\n");
        unsafe {
            bindings::devm_kfree(
                self.dev.raw_device(),
                self.i2c_dev_ptr as *const _ as *const core::ffi::c_void,
            );
        }
    }
}
