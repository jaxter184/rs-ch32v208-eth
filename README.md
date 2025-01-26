# CH32V208 Ethernet & Embassy experimentation
This is a test project to test the CH32 ethernet capability and hopefully assist in the development
of a proper driver in [ch32-hal](https://github.com/ch32-rs/ch32-hal).

## Reference
As can be seen in the [Reference Manual](https://www.wch-ic.com/downloads/CH32FV2x_V3xRM_PDF.html),
CH32 have two Ethernet controllers:
- A 10/100/1G MAC + 10M PHY or 10/100M PHY - present in the other lines of uC (RM section 27.1)
- A 10M MAC + PHY, no MII interface - present in the CH32V208, CH579 and a few other uC (RM section 27.2)

### 10/100/1G MAC (CH32V307)
Comparing STM32 RM and CH32 RM indicate that the this MAC is most likely compatible with `eth_v1a`
driver from embassy-stm32:
- present in v1b and v1c, but not in doc:
  - no mention of regs::Dmarswtr
  - no mention of edfe
- present in v1, but not in doc:
  - no mention of MLT
  - no mention of dmasbmr

disclaimer: I didn't check all the registers

**This isn't the target of this repo**

### 10M MAC (CH32V208, CH579)
This is the simplest of the two. It's barely more complex than an SPI controller, so a driver can
implemented from scratch relatively quickly.

There is a reference usage in [openwch/ch32v20x](https://github.com/openwch/ch32v20x) (in C).
**Note**: MournRiverStudio doesn't have HAL for the 32V208 's Ethernet peripheral, and the included
SVD doesn't show the same registers as the doc.

It's restrictive on the system clock, as FSYS has to be 60 or 120MHz. Other than that, it's easy to
start:

- Init Eth peripheral
  - setup interrupts as required
- Enable Ethernet
  - Set sys clock to 60 or 120MHz
  - Set ETH_10M_EN in EXTEND.CTR (this enables the clock and PHY power)
  - Set ETHPRE in RCC.CFGR0 if sys clock is 120MHz
- Send data (tested, working)
  - set RX DMA (buffer address, len)
  - set TXCRCEN in MACON2
- receive data (TBD)
  - set RX DMA (buffer address, len)
  - set MARXEN in MACON1
  - set RXEN in ECON1


## ch32-metapac / ch32-data development branches
The ETH part of the ch32-metapac is still in work-in-progress, so ch32-metapac will have to be
built manually from branch [add-eth10m](https://github.com/chmousset/rs-ch32-data/tree/add-eth10m)
of ch32-data.

## Building / running this project
This assumes you already installed the RISCV toolchain and [wchisp](https://ch32-rs.github.io/wchisp/)
with the WCH-LINKE debug interface.

Any CH32V208 board **should** be supported without modification, like the CH32V208-EVT, without
guarantee.

```bash
git clone https://github.com/chmousset/rs-ch32-data.git ch32-data
git clone https://github.com/chmousset/rs-ch32v208-eth.git ch32v208-eth
cd rs-ch32-data
git checkout add-eth10m
./d gen
cd ../ch32v208-eth
cargo run --release
```
