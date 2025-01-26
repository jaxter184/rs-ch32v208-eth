#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use ch32_hal::{delay::Delay, interrupt, println, rcc::*};
use ch32_metapac as pac;
use core::pin::Pin;
use embassy_executor::Spawner;
use panic_halt as _;

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

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
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

    println!("Starting");

    // ARP packet that should be visible with Wireshark on the receiving end
    let arp: [u8; 6 + 6 + 5 * 2 + 6 + 4 + 6 + 4] = [
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, /* destination MAC address */
        0x84, 0xc2, 0xe4, 0x01, 0x02, 0x03, /* source MAC address */
        0x08, 0x06, /* Type:ARP */
        0x00, 0x01, /* Hardware type:Ethernet(1) */
        0x08, 0x00, /* Protocol type:IPv4 */
        0x06, 0x04, /* Hardware size and Protocol size*/
        0x00, 0x01, /* Opcode:request(1) */
        0x84, 0xc2, 0xe4, 0x01, 0x02, 0x03, /* Sender MAC address */
        0xc0, 0xa8, 0x1, 0x0f, /* Sender IP address */
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* Target MAC address */
        0xc0, 0xa8, 0x1, 0x2, /* Target IP address */
    ];
    let mut buf: [u8; 1536] = [0; 1536];
    println!("Start rx: {:?}", eth.start_receive(&mut buf, 1536));
    // let pinned_buf = Pin::new(buf.as_ref());
    // println!("Start rx: {:?}", eth.start_receive(pinned_buf.as_, 1536));

    loop {
        delay.delay_ms(1000);
        println!(
            "Start tx: {:?}",
            eth.start_send(&arp, 6 + 6 + 5 * 2 + 6 + 4 + 6 + 4)
        );
        println!("Eth: {:?}({:x})", eth.status(), eth.status_raw());
        println!("PHY ST: {:x}", eth.smi_read_reg(1));
        // TODO: RX
    }
}
