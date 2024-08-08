// SPDX-License-Identifier: GPL-2.0

//! BCM2835 master mode driver
use kernel::{
    bindings,
    clk::Clk,
    clk_provider::{ClkHw, ClkInitData, ClkOps},
    completion::Completion,
    device::{self, Device, RawDevice},
    driver::DeviceRemoval,
    error::to_result,
    i2c::{self, I2cAdapter, I2cAdapterQuirks, I2cMsg, I2C_M_NOSTART, I2C_M_RD},
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

pub const DEBUG: i32 = 0;

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
    bus_clk: Clk,
    num_msgs: i32,
    msg_err: u32,
    msg_buf: Option<Vec<u8>>,
    msg_buf_remaining: usize,
    debug: [Bcm2835Debug; BCM2835_DEBUG_MAX],
    debug_num: u32,
    debug_num_msgs: u32,
}

impl Bcm2835I2cDev {
    unsafe fn from_ptr(ptr: *mut Self) -> &'static mut Self {
        unsafe { &mut *ptr }
    }

    unsafe fn as_ptr(&self) -> *mut Self {
        self as *const _ as *mut _
    }
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
    pub(crate) fn bcm2835_i2c_writel(&mut self, reg: usize, val: u32) {
        let i2c_reg = self.reg_base;
        let addr = i2c_reg.wrapping_add(reg);
        unsafe { bindings::writel(val as _, addr as _) }
    }

    pub(crate) fn bcm2835_i2c_readl(&mut self, reg: usize) -> u32 {
        let i2c_reg = self.reg_base;
        let addr = i2c_reg.wrapping_add(reg);
        unsafe { bindings::readl(addr as _) }
    }

    pub(crate) fn bcm2835_i2c_register_div(
        &'static mut self,
        mclk: &Clk,
    ) -> Result<&'static mut Clk> {
        let name = CString::try_from_fmt(fmt!("{}_div", self.dev.name()))?;
        let mclk_name = mclk.name();
        let parent_names = [mclk_name.as_char_ptr()];
        let clk_i2c = unsafe {
            let raw_ptr = self.dev.kzalloc::<ClkBcm2835I2c>()?;
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

        self.dev.clk_register(&mut clk_i2c.hw)
    }

    pub(crate) fn bcm2835_fill_txfifo(&mut self) {
        while self.msg_buf_remaining > 0 {
            let val: u32 = self.bcm2835_i2c_readl(BCM2835_I2C_S);
            if !(val & BCM2835_I2C_S_TXD) != 0 {
                break;
            }
            // May Wrong!
            if let Some(mut buf) = self.msg_buf.take() {
                // Safety: msg_buf_remaining > 0
                let byte = buf[0] as u32;
                self.bcm2835_i2c_writel(BCM2835_I2C_FIFO, byte);
                buf.remove(0);
                self.msg_buf = Some(buf);
                self.msg_buf_remaining -= 1;
            }
        }
    }

    pub(crate) fn bcm2835_drain_rxfifo(&mut self) {
        while self.msg_buf_remaining > 0 {
            let val: u32 = self.bcm2835_i2c_readl(BCM2835_I2C_S);
            if !(val & BCM2835_I2C_S_RXD) != 0 {
                break;
            }
            // May Wrong!
            if let Some(mut buf) = self.msg_buf.take() {
                buf[0] = self.bcm2835_i2c_readl(BCM2835_I2C_FIFO) as u8;
                buf.remove(0);
                self.msg_buf = Some(buf);
                self.msg_buf_remaining -= 1;
            }
        }
    }

    pub(crate) fn bcm2835_i2c_start_transfer(&mut self) {
        let mut c: u32 = BCM2835_I2C_C_ST | BCM2835_I2C_C_I2CEN;
        // Safely extract and process the current message
        if let Some(mut curr_msg) = self.curr_msg.take() {
            if self.num_msgs == 0 {
                return;
            }

            let msg = &curr_msg[0];
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
        }
        //debug add
    }

    pub(crate) fn bcm2835_i2c_finish_transfer(&mut self) {
        self.curr_msg = None;
        self.num_msgs = 0;

        self.msg_buf = None;
        self.msg_buf_remaining = 0;
    }
}

fn bcm2835_i2c_isr(this_irq: i32, data: *mut core::ffi::c_void) -> irq::Return {
    let i2c_dev = unsafe { &mut *(data as *mut Bcm2835I2cDev) };

    let mut val: u32 = i2c_dev.bcm2835_i2c_readl(BCM2835_I2C_S);
    let err: u32 = val & (BCM2835_I2C_S_CLKT | BCM2835_I2C_S_ERR);

    if err != 0 && (val & BCM2835_I2C_S_TA) == 0 {
        i2c_dev.msg_err = err;
    }

    if val & BCM2835_I2C_S_DONE != 0 {
        match i2c_dev.curr_msg {
            // Note: we represent the ptr buf with vec and the ptr to 0th element is the same as the ptr to the place in C.
            Some(ref mut msg) if msg[0].flags() as u32 & I2C_M_RD != 0 => {
                i2c_dev.bcm2835_drain_rxfifo();
                val = i2c_dev.bcm2835_i2c_readl(BCM2835_I2C_S);
            }
            None => {
                dev_err!(i2c_dev.dev, "Got unexpected interrupt (from firmware?)\n");
                return irq::Return::Handled;
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
            if let Some(ref mut curr_msg) = i2c_dev.curr_msg {
                curr_msg.remove(0);
            }
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
    bcm2835_i2c_isr(this_irq, data) as u32
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

fn bcm2835_i2c_xfer(adap: I2cAdapter, msgs: Vec<I2cMsg>, num: i32) -> Result<()> {
    let i2c_dev = unsafe { &mut (*adap.i2c_get_adapdata::<Bcm2835I2cDev>()) };
    let mut ignore_nak = false;

    if unsafe { DEBUG != 0 } {
        i2c_dev.debug_num_msgs = num as u32;
    }

    if unsafe { DEBUG > 2 } {
        // for 0..num {
        //     print msg
        // }
    }

    for msg in &msgs {
        if msg.flags() as u32 & I2C_M_RD != 0 {
            dev_warn!(
                i2c_dev.dev,
                "only one read message supported, has to be last\n"
            );
            return Err(EOPENSTALE);
        }
        if msg.flags() as u32 & I2C_M_NOSTART != 0 {
            ignore_nak = true;
        }
    }

    i2c_dev.curr_msg = Some(msgs);
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

    if unsafe { DEBUG > 1 || (DEBUG != 0 && (time_left == 0 || i2c_dev.msg_err != 0)) } {
        // debug print
    }

    i2c_dev.debug_num_msgs = 0;
    i2c_dev.debug_num = 0;

    if time_left == 0 {
        i2c_dev.bcm2835_i2c_writel(BCM2835_I2C_C, BCM2835_I2C_C_CLEAR);
        dev_err!(i2c_dev.dev, "i2c transfer timed out\n");
        return Err(ETIMEDOUT);
    }

    if i2c_dev.msg_err == 0 {
        return to_result(num);
    }

    if unsafe { DEBUG != 0 } {
        dev_err!(i2c_dev.dev, "i2c transfer failed: {}\n", i2c_dev.msg_err);
    }

    if i2c_dev.msg_err & BCM2835_I2C_S_ERR != 0 {
        return Err(EREMOTEIO);
    }

    Err(EIO)
}

fn bcm2835_i2c_func(adap: I2cAdapter) -> u32 {
    i2c::I2C_FUNC_I2C | i2c::I2C_FUNC_10BIT_ADDR | i2c::I2C_FUNC_PROTOCOL_MANGLING
}

// I2C_AQ ..
// const BCM2835_I2C_QUIRKS: I2cAdapterQuirks =
//    I2cAdapterQuirks::new().set_flags(i2c::I2C_AQ_NO_CLK_STRETCH as u64);

struct Bcm2835I2cAlgo;

struct Bcm2835I2cData {}
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

        let mut dev = unsafe { Device::new(pdev.raw_device()) };
        let i2c_dev_ptr: *mut Bcm2835I2cDev = dev.kzalloc::<Bcm2835I2cDev>()?;

        let i2c_dev = unsafe { Bcm2835I2cDev::from_ptr(i2c_dev_ptr) };
        i2c_dev.dev = dev.clone();
        i2c_dev.completion.reinit();
        i2c_dev.reg_base = pdev.ioremap_resource(0)?;
        dev_info!(pdev, "I2c bus device reg_base: {:?}\n", i2c_dev.reg_base);

        let mclk = i2c_dev.dev.clk_get()?;

        // TODO: fix lifetime issue
        let bus_clk = Bcm2835I2cDev::bcm2835_i2c_register_div(&mut dev, mclk, i2c_dev)?;

        // return a new register clock
        let new_clk = i2c_dev.bcm2835_i2c_register_div(mclk)?;
        i2c_dev.bus_clk = *new_clk;

        let mut bus_clk_rate = 0;
        if let Err(_) = i2c_dev
            .dev
            .of_property_read_u32(c_str!("clock-frequency"), &mut bus_clk_rate)
        {
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

        i2c_dev.bus_clk.prepare_enable()?;

        let irq = pdev.irq_resource(0)?;
        i2c_dev.irq = irq;

        let ret = unsafe {
            bindings::request_threaded_irq(
                irq as u32,
                Some(bcm2835_i2c_isr_cb),
                None,
                bindings::IRQF_SHARED as u64,
                c_str!("i2c_bcm2835_rust").as_char_ptr(),
                i2c_dev_ptr as *mut core::ffi::c_void,
            )
        };
        if ret < 0 {
            dev_err!(pdev, "Could not request IRQ: {}\n", irq);
            to_result(ret)?;
        }

        // TODO: setup i2c_adapter
        let quirks = I2cAdapterQuirks::new().set_flags(i2c::I2C_AQ_NO_CLK_STRETCH as u64);
        let mut adap = i2c_dev.adapter;
        unsafe {
            adap.i2c_set_adapdata(i2c_dev);
            // adap.set_owner();
            adap.set_class(bindings::I2C_CLASS_DEPRECATED);
        }
        i2c_dev.adapter = adap;

        /*
         * Disable the hardware clock stretching timeout. SMBUS
         * specifies a limit for how long the device can stretch the
         * clock, but core I2C doesn't.
         */
        i2c_dev.bcm2835_i2c_writel(BCM2835_I2C_CLKT, 0);
        i2c_dev.bcm2835_i2c_writel(BCM2835_I2C_C, 0);

        let ret = unsafe { bindings::i2c_add_adapter(adap.as_ptr()) };
        if ret < 0 {
            dev_info!(pdev, "Could not add I2C adapter: {:?}\n", to_result(ret));
            unsafe {
                bindings::free_irq(irq as u32, i2c_dev_ptr as *mut core::ffi::c_void);
            }
        }
        let _ = to_result(ret)?;

        let dev_data =
            kernel::new_device_data!((), (), Bcm2835I2cData {}, "BCM2835_I2C device data")?;
        /*
         * Disable the hardware clock stretching timeout. SMBUS
         * specifies a limit for how long the device can stretch the
         * clock, but core I2C doesn't.
         */
        i2c_dev.bcm2835_i2c_writel(BCM2835_I2C_CLKT, 0);
        i2c_dev.bcm2835_i2c_writel(BCM2835_I2C_C, 0);
        Ok(dev_data.into())
    }

    fn remove(_data: &Self::Data) -> Result {
        pr_info!("BCM2835 i2c bus device driver remove.\n");
        // TODO: remove i2c driver
        Ok(())
    }
}
