#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use ch32_hal::{delay::Delay, interrupt, println, rcc::*};
use ch32_metapac as pac;
use embassy_executor::Spawner;
use panic_halt as _;
use embassy_net_driver_channel as ch;
use embassy_time::{Ticker, Duration};
use embassy_futures::select::{select3, Either3};
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{Stack, StackResources};
use core::mem::MaybeUninit;

struct EthDriver {
    eth: pac::eth::Eth,
}

#[derive(Debug)]
enum EthError {
    Busy,
    CRC,
    Nibble,
    RxOverSize,
}

impl EthDriver {
    fn new(eth: pac::eth::Eth) -> Self {
        Self { eth }
    }

    fn disable_interrupts(&self) {
        self.eth.eie().write(|w| {
            w.set_rxie(false);
            w.set_txie(false);
            w.set_intie(false);
            w.set_linkie(false);
            w.set_r_en50(true); // enable 50 Ohms termination resistor on RX
            w.set_rxerie(false);
            w.set_txerie(false);
        })
    }

    fn init(&self) {
        // reset RX and TX paths
        println!("econ1 = {:x}", self.eth.econ1().read().0);
        self.eth.econ1().write(|w| {
            w.set_rx_rst(true);
            w.set_rx_rst(true);
        });
        println!("econ1 = {:x}", self.eth.econ1().read().0);
        self.eth.eir().write(|w| w.0 = 0xff);
        self.disable_interrupts();
        self.eth.econ1().write(|w| {
            w.set_rx_rst(false);
            w.set_rx_rst(false);
        });
        println!("econ1 = {:x}", self.eth.econ1().read().0);
    }

    fn start_send(&self, buf: &[u8], len: usize) -> Result<(), EthError> {
        if self.tx_busy() {
            return Err(EthError::Busy);
        }
        // setup DMA
        let address: *const u8 = buf.as_ptr();
        self.eth.etxst().write(|w| w.set_etxst(address as u16));
        self.eth.etxln().write(|w| w.set_etxln(len as u16));
        // enable short packet padding (0-64B) and CRC insertion (4B)
        self.eth.macon2().write(|w| {
            w.set_padcfg(0b001);
            w.set_txcrcen(true);
            w.set_fuldpx(true); // also set half-duplex
        });
        // start send
        self.eth.econ1().write(|w| {
            w.set_tx_rts(true); // start transmit
        });
        Ok(())
    }

    fn start_receive(&self, buf: &mut [u8], max_len: usize) -> Result<(), EthError> {
        if !self.rx_done() {
            return Err(EthError::Busy);
        }
        // setup DMA
        let address: *const u8 = buf.as_ptr();
        println!("RX address: {}", address as u16);
        self.eth.erxst().write(|w| w.set_erxst(address as u16));
        self.eth.erxln().write(|w| w.set_erxln(max_len as u16));
        // Enable MAC RX
        self.eth.macon1().write(|w| w.set_marxen(true));
        // set MAC max packet receive len
        self.eth.mamxfl().write(|w| w.set_mamxfl(1536));
        // Enable RX
        self.eth.econ1().write(|w| {
            w.set_rx_en(true);
        });
        Ok(())
    }

    fn tx_busy(&self) -> bool {
        self.eth.econ1().read().tx_rst()
    }

    fn rx_done(&self) -> bool {
        !self.eth.estat().read().rxbusy()
    }

    fn set_mac(&self, mac: &[u8; 6]) {
        self.eth.maadr0().write(|w| w.set_maadr(mac[0]));
        self.eth.maadr1().write(|w| w.set_maadr(mac[1]));
        self.eth.maadr2().write(|w| w.set_maadr(mac[2]));
        self.eth.maadr3().write(|w| w.set_maadr(mac[3]));
        self.eth.maadr4().write(|w| w.set_maadr(mac[4]));
        self.eth.maadr5().write(|w| w.set_maadr(mac[5]));
    }

    fn read_mac(&self, mac: &mut [u8; 6]) {
        mac[0] = self.eth.maadr0().read().0;
        mac[1] = self.eth.maadr1().read().0;
        mac[2] = self.eth.maadr2().read().0;
        mac[3] = self.eth.maadr3().read().0;
        mac[4] = self.eth.maadr4().read().0;
        mac[5] = self.eth.maadr5().read().0;
    }

    fn get_mac(&self, mac: &mut [u8; 6]) {
        /// Read the pre-programmed unique MAC
        const ADDRESS: *const u8 = 0x1FFFF7E8 as *const u8;
        unsafe {
            let mac_bytes: &[u8] = core::slice::from_raw_parts(ADDRESS, 6);
            mac.copy_from_slice(mac_bytes); // Copy the bytes into the array
        }
    }

    fn status_raw(&self) -> u8 {
        self.eth.estat().read().0
    }

    fn status(&self) -> Result<(), EthError> {
        let st = self.eth.estat().read();
        if st.rxcrcer() {
            self.eth.estat().write(|w| w.set_rxcrcer(true)); // clear CRC error status?
            return Err(EthError::CRC);
        }
        if st.rxmore() {
            return Err(EthError::RxOverSize);
        }
        if st.rxnibble() {
            return Err(EthError::Nibble);
        }
        Ok(())
    }

    fn smi_wreg(&self, reg: u8, value: u16) {
        self.eth.miwr().write(|w| {
            w.set_write(true);
            w.set_mirdl(reg & 0x1F);
            w.set_wr(value);
            println!("reg[{}] w[{:x}]", reg & 0x1F, w.0);
        });
    }

    fn smi_read_reg(&self, reg: u8) -> u16 {
        self.eth.miregadr().write(|w| {
            w.set_miregadr(reg & 0x1F);
            println!("reg[{}] r[{:x}]", reg & 0x1F, w.0);
        });
        self.eth.mird().read().rd()
    }
}

const MTU: usize = 1536;
const CHAN: usize = 8;

struct Runner<'d> {
    eth: EthDriver,
    delay: Delay,
    ch: ch::Runner<'d, MTU>,
    rx_buf: [u8; MTU],
}

impl<'d> Runner<'d> {
    fn new<'a: 'd>(eth: EthDriver, delay: Delay, state: &'a mut ch::State<MTU, CHAN, CHAN>) -> (Runner<'d>, ch::Device<'a, MTU>) {
        let mut mac = [0u8; 6];
        println!("Init Eth");
        eth.init();
        println!("Init done");
        eth.get_mac(&mut mac); // get MAC from ROM
        eth.set_mac(&mac);
        println!("MAC: {:?}", mac);
        eth.read_mac(&mut mac);
        println!("MAC readback: {:?}", mac);
        println!("Reset PHY");
        eth.smi_wreg(0, 0x8000); // reset PHY
        let mut rx_buf = [0_u8; MTU];
        println!("Start rx: {:?}", eth.start_receive(&mut rx_buf, MTU));

        let (ch, device) = ch::new(state, ch::driver::HardwareAddress::Ethernet(mac));

        (
            Self {
                eth,
                delay,
                ch,
                rx_buf,
            },
            device,
        )
    }

    async fn run(mut self) -> ! {
        loop {
            self.delay.delay_ms(1000);
        let (state_chan, mut rx_chan, mut tx_chan) = self.ch.split();
        let mut tick = Ticker::every(Duration::from_millis(500));
        loop {
            match select3(
                rx_chan.rx_buf(),
                tx_chan.tx_buf(),
                tick.next(),
            )
            .await
            {
                Either3::First(p) => {
                    println!("TODO: rx");
                    /*
                    if let Ok(n) = self.mac.read_frame(p).await {
                        rx_chan.rx_done(n);
                    }
                    */
                }
                Either3::Second(p) => {
                    println!(
                        "Start tx: {:?}",
                        self.eth.start_send(&p, p.len())
                    );
                    println!("Eth: {:?}({:x})", self.eth.status(), self.eth.status_raw());
                    println!("PHY ST: {:x}", self.eth.smi_read_reg(1));
                    tx_chan.tx_done();
                }
                Either3::Third(()) => {
                    println!("tick");
                    /*
                    if self.eth.is_link_up().await {
                        state_chan.set_link_state(LinkState::Up);
                    } else {
                        state_chan.set_link_state(LinkState::Down);
                    }
                    */
                }
            }
        }
        }
    }
}

#[embassy_executor::task]
async fn ethernet_task(runner: Runner<'static>) -> ! {
    println!("Spawning eth");
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, ch::Device<'static, MTU>>) -> ! {
    println!("Spawning net");
    runner.run().await
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    let p = ch32_hal::init(ch32_hal::Config {
        // 60 MHz from internal HSI until I get a proper 16MHz crystal
        rcc: ch32_hal::rcc::Config {
            hse: None,
            sys: Sysclk::PLL,
            pll_src: PllSource::HSI,
            pll: Some(Pll {
                prediv: PllPreDiv::DIV2,
                mul: PllMul::MUL15,
            }),
            pllx: None,
            ahb_pre: AHBPrescaler::DIV1,
            apb1_pre: APBPrescaler::DIV1,
            apb2_pre: APBPrescaler::DIV1,
            ls: LsConfig::default(),
            hspll_src: HsPllSource::HSI,
            hspll: None,
        },
        dma_interrupt_priority: interrupt::Priority::P0,
    });
    ch32_hal::debug::SDIPrint::enable();
    let mut delay = Delay {};
    delay.delay_ms(100); // let some time to the debug interface to start
    let extend = pac::EXTEND;
    extend.ctr().modify(|w| w.set_eth_10m_en(true));

    // 10M Ethernet enabled, clock enabled
    println!("Enable Eth");
    let rcc = pac::RCC;
    // For 120MHz sys clock
    // rcc.cfgr0()
    //     .modify(|w| w.set_ethpre(pac::rcc::vals::Ethpre::DIV2));
    println!("Eth clock enabled: {:?}", extend.ctr().read().eth_10m_en());
    println!("RCC_CFG0.HPRE {:x}", rcc.cfgr0().read().hpre().to_bits());
    println!("RCC_CFG0.SWS {:x}", rcc.cfgr0().read().sws().to_bits());
    println!(
        "RCC_CFG0.ETHPRE {:x}",
        rcc.cfgr0().read().ethpre().to_bits()
    );

    // eth peripheral init
    let eth = EthDriver::new(pac::ETH);
    static mut STATE: MaybeUninit<ch::State<MTU, CHAN, CHAN>> = MaybeUninit::uninit();
    let (eth_runner, device) = unsafe {
        let state = STATE.write(ch::State::new());
        Runner::new(eth, delay, state)
    };

    // Init network stack
    static mut RESOURCES: MaybeUninit<StackResources<2>> = MaybeUninit::uninit();
    let (stack, runner) = unsafe {
        let resources = RESOURCES.write(StackResources::<2>::new());
        embassy_net::new(
            device,
            embassy_net::Config::dhcpv4(Default::default()),
            resources,
            1234,
        )
    };

    println!("Spawning");
    println!("spawner: {:?}", spawner.spawn(ethernet_task(eth_runner)));
    println!("spawner: {:?}", spawner.spawn(net_task(runner)));

loop {
    println!("Starting");
}

/*
    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];
    let mut rx_meta = [PacketMetadata::EMPTY; 16];
    let mut tx_meta = [PacketMetadata::EMPTY; 16];
    let mut buf = [0; 4096];
    loop {
        let mut socket = UdpSocket::new(stack, &mut rx_meta, &mut rx_buffer, &mut tx_meta, &mut tx_buffer);
        socket.bind(1234).unwrap();

        loop {
            let (n, ep) = socket.recv_from(&mut buf).await.unwrap();
            if let Ok(s) = core::str::from_utf8(&buf[..n]) {
                println!("rxd from {}: {}", ep, s);
            }
            socket.send_to(&buf[..n], ep).await.unwrap();
        }
    }
    */
}
