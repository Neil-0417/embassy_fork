use core::cmp::Reverse;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicU8, Ordering};

use embassy_hal_internal::atomic_ring_buffer::RingBuffer;
use embassy_hal_internal::{Peri, PeripheralType};
use maitake_sync::WaitCell;
use nxp_pac::lpuart::Dozeen;

use crate::clocks::periph_helpers::{Div4, LpuartClockSel, LpuartConfig};
use crate::clocks::{ClockError, Gate, PoweredClock, WakeGuard, enable_and_reset};
use crate::dma::DmaRequest;
use crate::gpio::{AnyPin, SealedPin};
use crate::interrupt;
// Re-exported: these name the field types of the public `Config`/`BbqConfig` structs.
pub use crate::pac::lpuart::{
    Idlecfg as IdleConfig, Ilt as IdleType, M as DataBits, Msbf as MsbFirst, Pt as Parity, Rst, Rxflush,
    Sbns as StopBits, Swap, Tc, Tdre, Txctsc as TxCtsConfig, Txctssrc as TxCtsSource, Txflush,
};

pub(crate) mod bbq;
mod blocking;
mod buffered;
mod dma;

pub use bbq::{
    BbqConfig, BbqError, BbqHalfParts, BbqInterruptHandler, BbqParts, BbqRxMode, LpuartBbq, LpuartBbqRx, LpuartBbqTx,
};
pub use blocking::Blocking;
pub use buffered::{Buffered, BufferedInterruptHandler};
pub use dma::{Dma, RingBufferedLpuartRx};

mod sealed {
    pub trait Sealed {}
}

pub(crate) struct State {
    tx_waker: WaitCell,
    tx_buf: RingBuffer,
    rx_waker: WaitCell,
    rx_buf: RingBuffer,
    tx_rx_refmask: TxRxRefMask,
}

/// Value corresponding to either the Tx or the Rx part of the Uart being active.
#[derive(Clone, Copy)]
#[repr(u8)]
enum TxRxRef {
    Rx = 0b01,
    Tx = 0b10,
}

/// Mask that stores whether a Tx and/or Rx part of the Uart is active.
///
/// Used in constructors and Drop to manage the peripheral lifetime.
struct TxRxRefMask(AtomicU8);

impl TxRxRefMask {
    pub const fn new() -> Self {
        Self(AtomicU8::new(0))
    }

    /// Atomically signal that either the Tx or Rx has been dropped.
    ///
    /// Returns `true` if after this call all parts are inactive.
    pub fn set_inactive_fetch_last(&self, value: TxRxRef) -> bool {
        let value = value as u8;
        self.0.fetch_and(!value, Ordering::AcqRel) & !value == 0
    }

    /// Atomically signal that either the Tx or Rx has been created.
    pub fn set_active(&self, value: TxRxRef) {
        let value = value as u8;
        self.0.fetch_or(value, Ordering::AcqRel);
    }

    /// Atomically determine if either channels have been created, but not dropped. Clears the state.
    ///
    /// Should only be relevant when any of the parts have been leaked using [core::mem::forget].
    pub fn fetch_any_alive_reset(&self) -> bool {
        self.0.swap(0, Ordering::AcqRel) != 0
    }
}

impl Default for State {
    fn default() -> Self {
        Self::new()
    }
}

impl State {
    /// Create a new state instance
    pub const fn new() -> Self {
        Self {
            tx_waker: WaitCell::new(),
            tx_buf: RingBuffer::new(),
            rx_waker: WaitCell::new(),
            rx_buf: RingBuffer::new(),
            tx_rx_refmask: TxRxRefMask::new(),
        }
    }
}

pub(crate) struct Info {
    pub(crate) regs: crate::pac::lpuart::Lpuart,
    pub(crate) int_disable: fn(),
    pub(crate) mrcc_disable: unsafe fn(),
}

unsafe impl Sync for Info {}

impl Info {
    #[inline(always)]
    fn regs(&self) -> crate::pac::lpuart::Lpuart {
        self.regs
    }
}

pub(crate) trait SealedInstance: Gate<MrccPeriphConfig = LpuartConfig> {
    fn info() -> &'static Info;
    fn state() -> &'static State;

    const CLOCK_INSTANCE: crate::clocks::periph_helpers::LpuartInstance;
    const TX_DMA_REQUEST: DmaRequest;
    const RX_DMA_REQUEST: DmaRequest;
    const PERF_INT_INCR: fn();
    const PERF_INT_WAKE_INCR: fn();
}

/// Trait for LPUART peripheral instances
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + 'static + Send {
    type Interrupt: interrupt::typelevel::Interrupt;
}

#[doc(hidden)]
#[macro_export]
macro_rules! impl_lpuart_instance {
    ($n:expr) => {
        paste::paste! {
            impl crate::lpuart::SealedInstance for crate::peripherals::[<LPUART $n>] {
                fn info() -> &'static crate::lpuart::Info {
                    use crate::interrupt::typelevel::Interrupt;

                    static INFO: crate::lpuart::Info = crate::lpuart::Info {
                        regs: crate::pac::[<LPUART $n>],
                        int_disable: crate::interrupt::typelevel::[<LPUART $n>]::disable,
                        mrcc_disable: crate::clocks::disable::<crate::peripherals::[<LPUART $n>]>,
                    };
                    &INFO
                }

                fn state() -> &'static crate::lpuart::State {
                    static STATE: crate::lpuart::State = crate::lpuart::State::new();
                    &STATE
                }

                const CLOCK_INSTANCE: crate::clocks::periph_helpers::LpuartInstance
                    = crate::clocks::periph_helpers::LpuartInstance::[<Lpuart $n>];
                const TX_DMA_REQUEST: DmaRequest = DmaRequest::[<Lpuart $n Tx>];
                const RX_DMA_REQUEST: DmaRequest = DmaRequest::[<Lpuart $n Rx>];
                const PERF_INT_INCR: fn() = crate::perf_counters::[<incr_interrupt_lpuart $n>];
                const PERF_INT_WAKE_INCR: fn() = crate::perf_counters::[<incr_interrupt_lpuart $n _wake>];
            }

            impl crate::lpuart::Instance for crate::peripherals::[<LPUART $n>] {
                type Interrupt = crate::interrupt::typelevel::[<LPUART $n>];
            }

            crate::impl_lpuart_bbq_instance!($n);
        }
    };
}

/// Perform software reset on the LPUART peripheral
fn perform_software_reset(info: &'static Info) {
    // Software reset - set and clear RST bit (Global register)
    info.regs().global().write(|w| w.set_rst(Rst::Reset));
    info.regs().global().write(|w| w.set_rst(Rst::NoEffect));
}

/// Disable both transmitter and receiver
fn disable_transceiver(info: &'static Info) {
    info.regs().ctrl().modify(|w| {
        w.set_te(false);
        w.set_re(false);
    });
}

/// Calculate and configure baudrate settings
fn configure_baudrate(info: &'static Info, baudrate_bps: u32, clock_freq: u32) -> Result<(), Error> {
    let (osr, sbr) = calculate_baudrate(baudrate_bps, clock_freq)?;

    // Configure BAUD register
    info.regs().baud().modify(|w| {
        // Clear and set OSR
        w.set_osr(osr - 1);
        // Clear and set SBR
        w.set_sbr(sbr);
        // Set BOTHEDGE if OSR is between 4 and 7
        w.set_bothedge(osr > 3 && osr < 8);
    });

    Ok(())
}

/// Configure frame format (stop bits, data bits)
fn configure_frame_format(info: &'static Info, config: &Config) {
    // Configure stop bits
    info.regs().baud().modify(|w| w.set_sbns(config.stop_bits_count));

    // Clear M10 for now (10-bit mode)
    info.regs().baud().modify(|w| w.set_m10(false));
}

/// Configure control settings (parity, data bits, idle config, pin swap)
fn configure_control_settings(info: &'static Info, config: &Config) {
    info.regs().ctrl().modify(|w| {
        // Parity configuration
        if let Some(parity) = config.parity_mode {
            w.set_pe(true);
            w.set_pt(parity);
        } else {
            w.set_pe(false);
        };

        // Allow the lpuart to wake from deep sleep if configured to
        // work in deep sleep mode.
        //
        // NOTE: this is the default state, and setting this to `Dozeen::DISABLED`
        // seems to actively *stop* the uart, regardless of whether automatic clock
        // gating is used, or if the device never goes to deep sleep at all (e.g.
        // in WfeUngated configuration). For now, let's not touch this unless we
        // actually need to, e.g. *forcing* the lpuart to sleep!
        w.set_dozeen(Dozeen::Enabled);

        // Data bits configuration
        match config.data_bits_count {
            DataBits::Data8 => {
                if config.parity_mode.is_some() {
                    w.set_m(DataBits::Data9); // 8 data + 1 parity = 9 bits
                } else {
                    w.set_m(DataBits::Data8); // 8 data bits only
                }
            }
            DataBits::Data9 => w.set_m(DataBits::Data9),
        };

        // Idle configuration
        w.set_idlecfg(config.rx_idle_config);
        w.set_ilt(config.rx_idle_type);

        // Swap TXD/RXD if configured
        if config.swap_txd_rxd {
            w.set_swap(Swap::Swap);
        } else {
            w.set_swap(Swap::Standard);
        }
    });
}

/// Configure FIFO settings and watermarks
fn configure_fifo(info: &'static Info, config: &Config) {
    // Configure WATER register for FIFO watermarks
    info.regs().water().write(|w| {
        w.set_rxwater(config.rx_fifo_watermark);
        w.set_txwater(config.tx_fifo_watermark);
    });

    // Enable TX/RX FIFOs
    info.regs().fifo().modify(|w| {
        w.set_txfe(true);
        w.set_rxfe(true);
    });

    // Flush FIFOs
    info.regs().fifo().modify(|w| {
        w.set_txflush(Txflush::TxfifoRst);
        w.set_rxflush(Rxflush::RxfifoRst);
    });
}

/// Clear all status flags
fn clear_all_status_flags(info: &'static Info) {
    info.regs().stat().modify(|_w| {
        // Write back all values, clearing the W1C fields implicitly.
    });
}

/// Configure hardware flow control if enabled
fn configure_flow_control(info: &'static Info, enable_tx_cts: bool, enable_rx_rts: bool, config: &Config) {
    if enable_rx_rts || enable_tx_cts {
        info.regs().modir().modify(|w| {
            w.set_txctsc(config.tx_cts_config);
            w.set_txctssrc(config.tx_cts_source);
            w.set_rxrtse(enable_rx_rts);
            w.set_txctse(enable_tx_cts);
        });
    }
}

/// Configure bit order (MSB first or LSB first)
fn configure_bit_order(info: &'static Info, msb_first: MsbFirst) {
    info.regs().stat().modify(|w| w.set_msbf(msb_first));
}

/// Enable transmitter and/or receiver based on configuration
fn enable_transceiver(info: &'static Info, enable_tx: bool, enable_rx: bool) {
    info.regs().ctrl().modify(|w| {
        if enable_tx {
            w.set_te(true);
        }
        if enable_rx {
            w.set_re(true);
        }
    });
}

// Calculate the best OSR and SBR values for the desired baud rate and
// source clock frequency. The calculation is biased towards lowest
// possible diff and highest possible OSR value. A larger OSR favors
// better noise tolerance, so in case of a tie, we break the tie by
// largest OSR.
//
// Note that we compute and return OSR+1, the
// caller is responsible for subtracting 1 when writing to the
// register, as the hardware expects OSR-1.
fn calculate_baudrate(baudrate: u32, src_clock_hz: u32) -> Result<(u8, u16), Error> {
    (4..=32)
        .flat_map(|osr| {
            // Ideal SBR
            let ideal_sbr = (src_clock_hz / (baudrate * osr)) as i32;

            // Search through a small window around the ideal SBR to
            // find the best match.
            (-2..=2i32).filter_map(move |delta| {
                let sbr = ideal_sbr + delta;

                if (1..=0x1fff).contains(&sbr) {
                    let sbr = sbr as u32;
                    let calculated_baud = src_clock_hz / (osr * sbr);
                    let diff = calculated_baud.abs_diff(baudrate);
                    (diff <= (baudrate / 100) * 3).then_some((diff, osr, sbr))
                } else {
                    None
                }
            })
        })
        .min_by_key(|&(diff, osr, _)| (diff, Reverse(osr)))
        .map(|(_, osr, sbr)| (osr as u8, sbr as u16))
        .ok_or(Error::UnsupportedBaudrate)
}

/// Wait for all transmit operations to complete
fn wait_for_tx_complete(info: &'static Info) {
    // Wait for TX FIFO to empty
    while info.regs().water().read().txcount() != 0 {
        // Wait for TX FIFO to drain
    }

    // Wait for last character to shift out (TC = Transmission Complete)
    while info.regs().stat().read().tc() == Tc::Active {
        // Wait for transmission to complete
    }
}

fn check_and_clear_rx_errors(info: &'static Info) -> Result<(), Error> {
    let stat = info.regs().stat().read();

    // Check for overrun first - other error flags are prevented when OR is set
    let or_set = stat.or();
    let pf_set = stat.pf();
    let fe_set = stat.fe();
    let nf_set = stat.nf();

    // Clear all errors before returning
    info.regs().stat().write(|w| {
        w.set_or(or_set);
        w.set_pf(pf_set);
        w.set_fe(fe_set);
        w.set_nf(nf_set);
    });

    // Return error source
    if or_set {
        Err(Error::Overrun)
    } else if pf_set {
        Err(Error::Parity)
    } else if fe_set {
        Err(Error::Framing)
    } else if nf_set {
        Err(Error::Noise)
    } else {
        Ok(())
    }
}

fn has_rx_data_pending(info: &'static Info) -> bool {
    if info.regs().param().read().rxfifo() > 0 {
        // FIFO is available - check RXCOUNT in WATER register
        info.regs().water().read().rxcount() > 0
    } else {
        // No FIFO - check RDRF flag in STAT register
        info.regs().stat().read().rdrf()
    }
}

/// Read-only snapshot of an LPUART's key registers.
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LpuartSnapshot {
    /// Receiver overrun: at least one byte was dropped by the hardware.
    pub overrun: bool,
    /// Framing error latched.
    pub framing_error: bool,
    /// Noise error latched.
    pub noise_error: bool,
    /// Parity error latched.
    pub parity_error: bool,
    /// Receive data register / FIFO is not empty.
    pub rx_full: bool,
    /// Idle line detected.
    pub idle: bool,
    /// Receiver enabled (`CTRL.RE`).
    pub rx_enabled: bool,
    /// Receiver is mid-frame right now (`STAT.RAF`).
    pub rx_active: bool,
    /// An edge was seen on the RX pin since this flag was last cleared (`STAT.RXEDGIF`).
    pub rx_pin_edge: bool,
    /// RX FIFO underflow latched (`FIFO.RXUF`): a read happened with the FIFO empty.
    pub rx_underflow: bool,
    /// Bytes currently waiting in the RX FIFO.
    pub rx_fifo_count: u8,
    /// Bytes currently waiting in the TX FIFO.
    pub tx_fifo_count: u8,
    /// RX DMA requests enabled (`BAUD.RDMAE`).
    pub rx_dma_enabled: bool,
    /// TX DMA requests enabled (`BAUD.TDMAE`).
    pub tx_dma_enabled: bool,
    /// Raw `STAT`.
    pub stat: u32,
    /// Raw `CTRL`.
    pub ctrl: u32,
    /// Raw `BAUD`.
    pub baud: u32,
    /// Raw `FIFO`.
    pub fifo: u32,
    /// Raw `WATER`.
    pub water: u32,
}

fn lpuart_regs(lpuart: usize) -> crate::pac::lpuart::Lpuart {
    match lpuart {
        0 => crate::pac::LPUART0,
        1 => crate::pac::LPUART1,
        2 => crate::pac::LPUART2,
        3 => crate::pac::LPUART3,
        4 => crate::pac::LPUART4,
        5 => crate::pac::LPUART5,
        _ => panic!("no such LPUART instance"),
    }
}

fn port_regs(port: usize) -> crate::pac::port::Port {
    match port {
        0 => crate::pac::PORT0,
        1 => crate::pac::PORT1,
        2 => crate::pac::PORT2,
        3 => crate::pac::PORT3,
        4 => crate::pac::PORT4,
        #[cfg(feature = "mcxa5xx")]
        5 => crate::pac::PORT5,
        _ => panic!("no such PORT instance"),
    }
}

/// Read-only snapshot of one pin's `PORT` control register.
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PinSnapshot {
    /// PORT instance index.
    pub port: u8,
    /// Pin index within the port.
    pub pin: u8,
    /// Selected alternate function (`PCR.MUX`).
    pub mux: u8,
    /// Input buffer enabled (`PCR.IBE`); a UART RX pin reads nothing without it.
    pub input_buffer: bool,
    /// Pull resistor enabled (`PCR.PE`).
    pub pull_enabled: bool,
    /// Pull direction is up (`PCR.PS`).
    pub pull_up: bool,
    /// Input inversion enabled (`PCR.INV`).
    pub inverted: bool,
    /// Raw `PCR`.
    pub pcr: u32,
}

/// Capture one pin's `PORT` control register.
///
/// Useful for proving that a peripheral pin is still muxed to the peripheral and
/// has its input buffer enabled, rather than having been re-claimed as GPIO by
/// another driver at runtime.
pub fn pin_snapshot(port: usize, pin: usize) -> PinSnapshot {
    let pcr = port_regs(port).pcr(pin).read();

    PinSnapshot {
        port: port as u8,
        pin: pin as u8,
        mux: pcr.mux().to_bits(),
        input_buffer: pcr.ibe().to_bits() != 0,
        pull_enabled: pcr.pe().to_bits() != 0,
        pull_up: pcr.ps().to_bits() != 0,
        inverted: pcr.inv().to_bits() != 0,
        pcr: pcr.0,
    }
}

/// RX overruns seen by the ISR, per LPUART instance.
///
/// The BBQ interrupt handler clears `STAT.OR` on every entry, so a poller can never
/// observe the flag. Losses are only visible through this running count.
static RX_OVERRUNS: [core::sync::atomic::AtomicU32; 6] = [const { core::sync::atomic::AtomicU32::new(0) }; 6];

/// Idle-line events that ended an RX DMA transfer early, per LPUART instance.
static RX_IDLE_FINALIZES: [core::sync::atomic::AtomicU32; 6] = [const { core::sync::atomic::AtomicU32::new(0) }; 6];

pub(crate) fn lpuart_index(regs: crate::pac::lpuart::Lpuart) -> usize {
    (0..6).find(|&i| lpuart_regs(i).as_ptr() == regs.as_ptr()).unwrap_or(0)
}

pub(crate) fn note_rx_overrun(lpuart: usize) {
    if let Some(count) = RX_OVERRUNS.get(lpuart) {
        count.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    }
}

pub(crate) fn note_rx_idle_finalize(lpuart: usize) {
    if let Some(count) = RX_IDLE_FINALIZES.get(lpuart) {
        count.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    }
}

/// Number of RX overruns the ISR has cleared since boot.
///
/// Every increment is at least one byte the hardware dropped before DMA (or the ISR)
/// could take it.
pub fn rx_overrun_count(lpuart: usize) -> u32 {
    RX_OVERRUNS
        .get(lpuart)
        .map(|c| c.load(core::sync::atomic::Ordering::Relaxed))
        .unwrap_or(0)
}

/// Number of times an idle line ended an RX DMA transfer early since boot.
///
/// Each one stops and re-arms the DMA, which is the window where bytes can be lost.
pub fn rx_idle_finalize_count(lpuart: usize) -> u32 {
    RX_IDLE_FINALIZES
        .get(lpuart)
        .map(|c| c.load(core::sync::atomic::Ordering::Relaxed))
        .unwrap_or(0)
}

/// Capture an LPUART's registers without disturbing an in-flight transfer.
///
/// `lpuart` is the instance index, so `LPUART1` is `1`.
///
/// Every access is a read, so error flags are left latched for the driver to
/// clear. `overrun` is the flag to watch when bytes go missing: it means the RX
/// FIFO filled up before DMA (or the ISR) drained it.
pub fn snapshot(lpuart: usize) -> LpuartSnapshot {
    let regs = lpuart_regs(lpuart);
    let stat = regs.stat().read();
    let ctrl = regs.ctrl().read();
    let baud = regs.baud().read();
    let fifo = regs.fifo().read();
    let water = regs.water().read();

    LpuartSnapshot {
        overrun: stat.or(),
        framing_error: stat.fe(),
        noise_error: stat.nf(),
        parity_error: stat.pf(),
        rx_full: stat.rdrf(),
        idle: stat.idle(),
        rx_enabled: ctrl.re(),
        rx_active: stat.raf().to_bits() != 0,
        rx_pin_edge: stat.rxedgif(),
        rx_underflow: fifo.rxuf(),
        rx_fifo_count: water.rxcount(),
        tx_fifo_count: water.txcount(),
        rx_dma_enabled: baud.rdmae(),
        tx_dma_enabled: baud.tdmae(),
        stat: stat.0,
        ctrl: ctrl.0,
        baud: baud.0,
        fifo: fifo.0,
        water: water.0,
    }
}

/// Poll an LPUART and its RX DMA channel forever, logging each sample at `warn`.
///
/// Spawn this alongside the driver to watch for the RX path stalling or dropping
/// bytes. Purely read-only, so it never perturbs an in-flight transfer.
///
/// `lpuart` is the instance index, `dma`/`channel` identify the RX channel, and
/// `rx_pin` is the `(port, pin)` of the RX pad, so `LPUART1` with `p.DMA0_CH1`
/// on `P1_8` is `(1, 0, 1, (1, 8))`.
///
/// Without the `defmt` feature this still polls, but emits nothing.
///
/// ```rust,ignore
/// use embassy_mcxa::lpuart::monitor_rx;
/// use embassy_time::Duration;
///
/// #[embassy_executor::task]
/// async fn rx_monitor() {
///     monitor_rx(1, 0, 1, (1, 8), Duration::from_millis(100)).await
/// }
///
/// spawner.must_spawn(rx_monitor());
/// ```
pub async fn monitor_rx(
    lpuart: usize,
    dma: usize,
    channel: usize,
    rx_pin: (usize, usize),
    interval: embassy_time::Duration,
) -> ! {
    loop {
        let _u = snapshot(lpuart);
        let _d = crate::dma::channel_snapshot(dma, channel);
        let _p = pin_snapshot(rx_pin.0, rx_pin.1);
        let _or = rx_overrun_count(lpuart);
        let _il = rx_idle_finalize_count(lpuart);

        #[cfg(feature = "defmt")]
        defmt::warn!(
            "lpuart{=usize} dma{=usize}ch{=usize}: erq={=bool} active={=bool} done={=bool} int={=bool} err={} mux={=u8} nbytes={=u32} citer={=u16}/{=u16} daddr={=u32:#010x} tcd_csr={=u16:#06x} | uart: re={=bool} raf={=bool} rx_edge={=bool} overrun={=bool} underflow={=bool} framing={=bool} noise={=bool} parity={=bool} rx_full={=bool} idle={=bool} rx_fifo={=u8} rx_dma_en={=bool} | counts: overruns={=u32} idle_finalizes={=u32} | rx pin P{=u8}_{=u8}: mux={=u8} ibe={=bool} pe={=bool} ps={=bool} inv={=bool}",
            lpuart,
            dma,
            channel,
            _d.erq,
            _d.active,
            _d.done,
            _d.int,
            _d.errors,
            _d.mux_src,
            _d.nbytes,
            _d.citer,
            _d.biter,
            _d.daddr,
            _d.tcd_csr,
            _u.rx_enabled,
            _u.rx_active,
            _u.rx_pin_edge,
            _u.overrun,
            _u.rx_underflow,
            _u.framing_error,
            _u.noise_error,
            _u.parity_error,
            _u.rx_full,
            _u.idle,
            _u.rx_fifo_count,
            _u.rx_dma_enabled,
            _or,
            _il,
            _p.port,
            _p.pin,
            _p.mux,
            _p.input_buffer,
            _p.pull_enabled,
            _p.pull_up,
            _p.inverted,
        );

        embassy_time::Timer::after(interval).await;
    }
}

impl<T: SealedPin> sealed::Sealed for T {}

pub trait TxPin<T: Instance>: Into<AnyPin> + sealed::Sealed + PeripheralType {
    const MUX: crate::pac::port::Mux;
    /// convert the pin to appropriate function for Lpuart Tx  usage
    fn as_tx(&self);
}

pub trait RxPin<T: Instance>: Into<AnyPin> + sealed::Sealed + PeripheralType {
    const MUX: crate::pac::port::Mux;
    /// convert the pin to appropriate function for Lpuart Rx  usage
    fn as_rx(&self);
}

pub trait CtsPin<T: Instance>: Into<AnyPin> + sealed::Sealed + PeripheralType {
    const MUX: crate::pac::port::Mux;
    /// convert the pin to appropriate function for Lpuart Cts usage
    fn as_cts(&self);
}

pub trait RtsPin<T: Instance>: Into<AnyPin> + sealed::Sealed + PeripheralType {
    const MUX: crate::pac::port::Mux;
    /// convert the pin to appropriate function for Lpuart Rts usage
    fn as_rts(&self);
}

#[doc(hidden)]
#[macro_export]
macro_rules! impl_lpuart_pin {
    ($inst:ident, $pin:ident, $alt:ident, TXD) => {
        impl crate::lpuart::TxPin<crate::peripherals::$inst> for crate::peripherals::$pin {
            const MUX: crate::pac::port::Mux = crate::pac::port::Mux::$alt;
            fn as_tx(&self) {
                use crate::gpio::SealedPin;
                self.set_pull(crate::gpio::Pull::Disabled);
                self.set_slew_rate(crate::gpio::SlewRate::Fast.into());
                self.set_drive_strength(crate::gpio::DriveStrength::Normal.into());
                self.set_function(<Self as crate::lpuart::TxPin<crate::peripherals::$inst>>::MUX);
                self.set_enable_input_buffer(false);
            }
        }
    };
    ($inst:ident, $pin:ident, $alt:ident, RXD) => {
        impl crate::lpuart::RxPin<crate::peripherals::$inst> for crate::peripherals::$pin {
            const MUX: crate::pac::port::Mux = crate::pac::port::Mux::$alt;
            fn as_rx(&self) {
                use crate::gpio::SealedPin;
                self.set_pull(crate::gpio::Pull::Disabled);
                self.set_function(<Self as crate::lpuart::RxPin<crate::peripherals::$inst>>::MUX);
                self.set_enable_input_buffer(true);
            }
        }
    };
    ($inst:ident, $pin:ident, $alt:ident, CTS_B) => {
        impl crate::lpuart::CtsPin<crate::peripherals::$inst> for crate::peripherals::$pin {
            const MUX: crate::pac::port::Mux = crate::pac::port::Mux::$alt;
            fn as_cts(&self) {
                use crate::gpio::SealedPin;
                self.set_pull(crate::gpio::Pull::Disabled);
                self.set_function(<Self as crate::lpuart::CtsPin<crate::peripherals::$inst>>::MUX);
                self.set_enable_input_buffer(true);
            }
        }
    };
    ($inst:ident, $pin:ident, $alt:ident, RTS_B) => {
        impl crate::lpuart::RtsPin<crate::peripherals::$inst> for crate::peripherals::$pin {
            const MUX: crate::pac::port::Mux = crate::pac::port::Mux::$alt;
            fn as_rts(&self) {
                use crate::gpio::SealedPin;
                self.set_pull(crate::gpio::Pull::Disabled);
                self.set_slew_rate(crate::gpio::SlewRate::Fast.into());
                self.set_drive_strength(crate::gpio::DriveStrength::Normal.into());
                self.set_function(<Self as crate::lpuart::RtsPin<crate::peripherals::$inst>>::MUX);
                self.set_enable_input_buffer(false);
            }
        }
    };
}

/// LPUART error types
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Read error
    Read,
    /// Buffer overflow
    Overrun,
    /// Noise error
    Noise,
    /// Framing error
    Framing,
    /// Parity error
    Parity,
    /// Failure
    Fail,
    /// Invalid argument
    InvalidArgument,
    /// Lpuart baud rate cannot be supported with the given clock
    UnsupportedBaudrate,
    /// RX FIFO Empty
    RxFifoEmpty,
    /// TX FIFO Full
    TxFifoFull,
    /// TX Busy
    TxBusy,
    /// Clock Error
    ClockSetup(ClockError),
    /// Other internal errors or unexpected state.
    Other,
}

impl From<crate::dma::InvalidParameters> for Error {
    fn from(_value: crate::dma::InvalidParameters) -> Self {
        Error::Other
    }
}

impl From<maitake_sync::Closed> for Error {
    fn from(_value: maitake_sync::Closed) -> Self {
        Error::Other
    }
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::Read => write!(f, "Read error"),
            Error::Overrun => write!(f, "Buffer overflow"),
            Error::Noise => write!(f, "Noise error"),
            Error::Framing => write!(f, "Framing error"),
            Error::Parity => write!(f, "Parity error"),
            Error::Fail => write!(f, "Failure"),
            Error::InvalidArgument => write!(f, "Invalid argument"),
            Error::UnsupportedBaudrate => write!(f, "Unsupported baud rate"),
            Error::RxFifoEmpty => write!(f, "RX FIFO empty"),
            Error::TxFifoFull => write!(f, "TX FIFO full"),
            Error::TxBusy => write!(f, "TX busy"),
            Error::ClockSetup(e) => write!(f, "Clock setup error: {:?}", e),
            Error::Other => write!(f, "Other error"),
        }
    }
}

impl core::error::Error for Error {}

/// Lpuart config
#[derive(Debug, Clone, Copy)]
pub struct Config {
    /// Power state required for this peripheral
    pub power: PoweredClock,
    /// Clock source
    pub source: LpuartClockSel,
    /// Clock divisor
    pub div: Div4,
    /// Baud rate in bits per second
    pub baudrate_bps: u32,
    /// Parity configuration
    pub parity_mode: Option<Parity>,
    /// Number of data bits
    pub data_bits_count: DataBits,
    /// MSB First or LSB First configuration
    pub msb_first: MsbFirst,
    /// Number of stop bits
    pub stop_bits_count: StopBits,
    /// TX FIFO watermark
    pub tx_fifo_watermark: u8,
    /// RX FIFO watermark
    pub rx_fifo_watermark: u8,
    /// TX CTS source
    pub tx_cts_source: TxCtsSource,
    /// TX CTS configure
    pub tx_cts_config: TxCtsConfig,
    /// RX IDLE type
    pub rx_idle_type: IdleType,
    /// RX IDLE configuration
    pub rx_idle_config: IdleConfig,
    /// Swap TXD and RXD pins
    pub swap_txd_rxd: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            baudrate_bps: 115_200u32,
            parity_mode: None,
            data_bits_count: DataBits::Data8,
            msb_first: MsbFirst::LsbFirst,
            stop_bits_count: StopBits::One,
            tx_fifo_watermark: 0,
            rx_fifo_watermark: 1,
            tx_cts_source: TxCtsSource::Cts,
            tx_cts_config: TxCtsConfig::Start,
            rx_idle_type: IdleType::FromStart,
            rx_idle_config: IdleConfig::Idle1,
            swap_txd_rxd: false,
            power: PoweredClock::NormalEnabledDeepSleepDisabled,
            source: LpuartClockSel::FroLfDiv,
            div: Div4::no_div(),
        }
    }
}

/// Driver mode.
#[allow(private_bounds)]
pub trait Mode: sealed::Sealed {}

/// Lpuart driver.
pub struct Lpuart<'a, M: Mode> {
    tx: LpuartTx<'a, M>,
    rx: LpuartRx<'a, M>,
}

struct TxPins<'a> {
    tx_pin: Peri<'a, AnyPin>,
    cts_pin: Option<Peri<'a, AnyPin>>,
}

impl<'a> TxPins<'a> {
    fn take(self) -> (Peri<'a, AnyPin>, Option<Peri<'a, AnyPin>>) {
        unsafe {
            let tx_pin = self.tx_pin.clone_unchecked();
            let cts_pin = self.cts_pin.as_ref().map(|p| p.clone_unchecked());
            core::mem::forget(self);
            (tx_pin, cts_pin)
        }
    }
}

struct RxPins<'a> {
    rx_pin: Peri<'a, AnyPin>,
    rts_pin: Option<Peri<'a, AnyPin>>,
}

impl<'a> RxPins<'a> {
    fn take(self) -> (Peri<'a, AnyPin>, Option<Peri<'a, AnyPin>>) {
        unsafe {
            let rx_pin = self.rx_pin.clone_unchecked();
            let rts_pin = self.rts_pin.as_ref().map(|p| p.clone_unchecked());
            core::mem::forget(self);
            (rx_pin, rts_pin)
        }
    }
}

impl Drop for TxPins<'_> {
    fn drop(&mut self) {
        self.tx_pin.set_as_disabled();
        if let Some(cts_pin) = &self.cts_pin {
            cts_pin.set_as_disabled();
        }
    }
}

impl Drop for RxPins<'_> {
    fn drop(&mut self) {
        self.rx_pin.set_as_disabled();
        if let Some(rts_pin) = &self.rts_pin {
            rts_pin.set_as_disabled();
        }
    }
}

/// Lpuart TX driver.
pub struct LpuartTx<'a, M: Mode> {
    info: &'static Info,
    state: &'static State,
    mode: M,
    _tx_pins: TxPins<'a>,
    _wg: Option<WakeGuard>,
    _phantom: PhantomData<&'a mut ()>,
}

/// Lpuart Rx driver.
pub struct LpuartRx<'a, M: Mode> {
    info: &'static Info,
    state: &'static State,
    mode: M,
    _rx_pins: RxPins<'a>,
    _wg: Option<WakeGuard>,
    _phantom: PhantomData<&'a mut ()>,
}

fn disable_peripheral(info: &'static Info) {
    // Clear all status flags
    clear_all_status_flags(info);

    // Disable interrupts at the NVIC level
    (info.int_disable)();

    // Disable the module - clear all CTRL register bits
    info.regs().ctrl().write(|w| w.0 = 0);

    // Disable at the MRCC level
    unsafe {
        (info.mrcc_disable)();
    }
}

impl<M: Mode> Drop for LpuartTx<'_, M> {
    fn drop(&mut self) {
        // Wait for TX operations to complete. We cannot load more items
        // into the fifo as we have exclusive access to the LpuartTx
        wait_for_tx_complete(self.info);

        // Disable transmit interrupts to prevent usage of the buffer space
        cortex_m::interrupt::free(|_| {
            self.info.regs().ctrl().modify(|w| w.set_tie(false));
        });

        // De-init the tx buffer, as once '_ ends we no longer can guarantee
        // our usage of the buffer is sound.
        unsafe {
            self.state.tx_buf.deinit();
        }

        if self.state.tx_rx_refmask.set_inactive_fetch_last(TxRxRef::Tx) {
            disable_peripheral(self.info);
        }
    }
}

impl<M: Mode> Drop for LpuartRx<'_, M> {
    fn drop(&mut self) {
        // Disable receive interrupts to prevent future usage of the buffer space
        cortex_m::interrupt::free(|_| {
            self.info.regs().ctrl().modify(|w| {
                w.set_rie(false); // RX interrupt
                w.set_orie(false); // Overrun interrupt
                w.set_peie(false); // Parity error interrupt
                w.set_feie(false); // Framing error interrupt
                w.set_neie(false); // Noise error interrupt
            });
        });

        // De-init the rx buffer, as once '_ ends we no longer can guarantee
        // our usage of the buffer is sound.
        unsafe {
            self.state.rx_buf.deinit();
        }

        if self.state.tx_rx_refmask.set_inactive_fetch_last(TxRxRef::Rx) {
            disable_peripheral(self.info);
        }
    }
}

impl<'a, M: Mode> Lpuart<'a, M> {
    fn init<T: Instance>(
        enable_tx: bool,
        enable_rx: bool,
        enable_tx_cts: bool,
        enable_rx_rts: bool,
        config: Config,
    ) -> Result<Option<WakeGuard>, Error> {
        // Check if the peripheral was leaked using [core::mem::forget], and clean up the peripheral.
        if T::state().tx_rx_refmask.fetch_any_alive_reset() {
            disable_peripheral(T::info());
        }

        // Enable clocks
        let conf = LpuartConfig {
            power: config.power,
            source: config.source,
            div: config.div,
            instance: T::CLOCK_INSTANCE,
        };
        let parts = unsafe { enable_and_reset::<T>(&conf).map_err(Error::ClockSetup)? };

        // Perform initialization sequence
        perform_software_reset(T::info());
        disable_transceiver(T::info());
        configure_baudrate(T::info(), config.baudrate_bps, parts.freq)?;
        configure_frame_format(T::info(), &config);
        configure_control_settings(T::info(), &config);
        configure_fifo(T::info(), &config);
        clear_all_status_flags(T::info());
        configure_flow_control(T::info(), enable_tx_cts, enable_rx_rts, &config);
        configure_bit_order(T::info(), config.msb_first);
        enable_transceiver(T::info(), enable_tx, enable_rx);

        Ok(parts.wake_guard)
    }

    /// Split the Lpuart into a transmitter and receiver
    pub fn split(self) -> (LpuartTx<'a, M>, LpuartRx<'a, M>) {
        (self.tx, self.rx)
    }

    /// Split the Lpuart into a transmitter and receiver by mutable reference
    pub fn split_ref(&mut self) -> (&mut LpuartTx<'a, M>, &mut LpuartRx<'a, M>) {
        (&mut self.tx, &mut self.rx)
    }
}

impl<'a, M: Mode> LpuartTx<'a, M> {
    fn new_inner<T: Instance>(
        tx_pin: Peri<'a, AnyPin>,
        cts_pin: Option<Peri<'a, AnyPin>>,
        mode: M,
        wg: Option<WakeGuard>,
    ) -> Self {
        T::state().tx_rx_refmask.set_active(TxRxRef::Tx);

        Self {
            info: T::info(),
            state: T::state(),
            mode,
            _tx_pins: TxPins { tx_pin, cts_pin },
            _wg: wg,
            _phantom: PhantomData,
        }
    }
}

impl<'a, M: Mode> LpuartRx<'a, M> {
    fn new_inner<T: Instance>(
        rx_pin: Peri<'a, AnyPin>,
        rts_pin: Option<Peri<'a, AnyPin>>,
        mode: M,
        _wg: Option<WakeGuard>,
    ) -> Self {
        T::state().tx_rx_refmask.set_active(TxRxRef::Rx);

        Self {
            info: T::info(),
            state: T::state(),
            mode,
            _rx_pins: RxPins { rx_pin, rts_pin },
            _wg,
            _phantom: PhantomData,
        }
    }
}

impl embedded_hal_nb::serial::Error for Error {
    fn kind(&self) -> embedded_hal_nb::serial::ErrorKind {
        match *self {
            Self::Framing => embedded_hal_nb::serial::ErrorKind::FrameFormat,
            Self::Overrun => embedded_hal_nb::serial::ErrorKind::Overrun,
            Self::Parity => embedded_hal_nb::serial::ErrorKind::Parity,
            Self::Noise => embedded_hal_nb::serial::ErrorKind::Noise,
            _ => embedded_hal_nb::serial::ErrorKind::Other,
        }
    }
}

impl<M: Mode> embedded_hal_nb::serial::ErrorType for LpuartRx<'_, M> {
    type Error = Error;
}

impl<M: Mode> embedded_hal_nb::serial::ErrorType for LpuartTx<'_, M> {
    type Error = Error;
}

impl<M: Mode> embedded_hal_nb::serial::ErrorType for Lpuart<'_, M> {
    type Error = Error;
}

impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

impl<M: Mode> embedded_io::ErrorType for LpuartRx<'_, M> {
    type Error = Error;
}

impl<M: Mode> embedded_io::ErrorType for LpuartTx<'_, M> {
    type Error = Error;
}

impl<M: Mode> embedded_io::ErrorType for Lpuart<'_, M> {
    type Error = Error;
}
