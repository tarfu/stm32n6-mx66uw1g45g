#![no_std]

use stm32_metapac as pac;
use pac::gpio;
use pac::xspi;
use pac::rcc::vals::{Icsel, Xspisel};
use pac::xspi::vals::{Cssel, CcrAdmode, CcrAdsize, CcrDmode, CcrImode, Fmode, Fthres, Mtyp};
use pac::pwr::vals::{Vddio3sv, Vddio3vrsel};
use pac::syscfg::vals::{Vddio3cccrCs, Vddio3cccrEn};
use flash_algorithm::ErrorCode;

// Flash memory address on STM32N6 (XSPI2 non-secure alias per STM32N6 header)
pub const FLASH_BASE: u32 = 0x70000000;

// Flash page size (256 bytes per MX66UW1G45G datasheet)
pub const FLASH_PAGE_SIZE: usize = 256;

// Flash commands (SPI mode, MX66UW1G45G datasheet)
pub const CMD_WRITE_ENABLE: u32 = 0x06;
pub const CMD_READ_STATUS_REG: u32 = 0x05;
pub const CMD_READ_ID: u32 = 0x9F;
pub const CMD_RESET_ENABLE: u32 = 0x66;
pub const CMD_RESET_MEMORY: u32 = 0x99;
pub const CMD_PAGE_PROGRAM_4B: u32 = 0x12;
pub const CMD_SECTOR_ERASE_4B: u32 = 0x21;
pub const CMD_CHIP_ERASE: u32 = 0x60;
pub const CMD_FAST_READ_4B: u32 = 0x0C;

// Status register bits
pub const SR_WIP: u8 = 0x01;

pub struct Xspi {
    pub regs: xspi::Xspi,
}

impl Xspi {
    pub fn new() -> Self {
        // Use secure alias (0x5802_a000) instead of non-secure (0x4802_a000)
        // because XSPI2 is a secure peripheral and writes via non-secure alias fault
        Self { regs: unsafe { xspi::Xspi::from_ptr(0x5802_a000 as *mut ()) } }
    }

    pub fn wait_not_busy(&self) -> Result<(), ErrorCode> {
        for _ in 0..0xFFFFu32 {
            if !self.regs.sr().read().busy() {
                return Ok(());
            }
            cortex_m::asm::nop();
        }
        Err(ErrorCode::new(0x2001).unwrap())
    }

    pub fn init(&self) -> Result<(), ErrorCode> {
        // 0. Reset XSPI2 peripheral (enable_and_reset pattern from embassy)
        let rcc = pac::RCC;
        rcc.ahb5rstr().modify(|w| w.set_xspi2rst(true));
        rcc.ahb5enr().modify(|w| w.set_xspi2en(true));
        let _ = rcc.ahb5enr().read(); // dummy read for sync
        cortex_m::asm::dsb();
        rcc.ahb5rstr().modify(|w| w.set_xspi2rst(false));

        // 1. Disable XSPI while configuring
        self.regs.cr().write(|w| w.set_en(false));
        self.wait_not_busy()?;

        // 2. DCR1: device size, memory type, chip select high time
        self.regs.dcr1().modify(|w| {
            w.set_devsize(26);                             // 2^27 = 128 MiB
            w.set_mtyp(Mtyp::B_0X1);                      // Macronix
            w.set_csht(xspi::vals::Csht::from_bits(1));   // 2 cycles
        });

        // 3. DCR2: wrap size (separate write before prescaler)
        self.regs.dcr2().modify(|w| {
            w.set_wrapsize(xspi::vals::Wrapsize::from_bits(0)); // None
        });

        // 4. CR: FIFO threshold + CSSEL (select NCS1)
        self.regs.cr().modify(|w| {
            w.set_fthres(Fthres::from_bits(3)); // 4 bytes
            w.set_cssel(Cssel::B_0X0);          // NCS1 (PN1) — B_0X0 = NCS1 active
        });

        self.wait_not_busy()?;

        // 5. DCR2: prescaler (separate write, triggers calibration)
        self.regs.dcr2().modify(|w| {
            w.set_prescaler(7); // prescaler=7 → divide by 8: 64MHz/8 = 8MHz
        });
        self.wait_not_busy()?;

        // 6. TCR: sample shift + delay hold quarter cycle
        self.regs.tcr().modify(|w| {
            w.set_sshift(true);
            w.set_dhqc(true);
        });

        // 7. Enable XSPI
        self.regs.cr().modify(|w| w.set_en(true));

        // 8. Reset flash to known state
        self.reset_flash()?;

        Ok(())
    }

    fn reset_flash(&self) -> Result<(), ErrorCode> {
        self.exec_command(CMD_RESET_ENABLE)?;
        self.exec_command(CMD_RESET_MEMORY)?;
        // Brief delay for reset completion
        for _ in 0..100_000 {
            cortex_m::asm::nop();
        }
        Ok(())
    }

    pub fn exec_command(&self, cmd: u32) -> Result<(), ErrorCode> {
        self.wait_not_busy()?;
        self.regs.fcr().write(|w| w.set_ctcf(true));
        self.regs.ccr().write(|w| {
            w.set_imode(CcrImode::B_0X1);
            w.set_admode(CcrAdmode::B_0X0);
            w.set_dmode(CcrDmode::B_0X0);
        });
        self.regs.ir().write(|w| w.set_instruction(cmd));
        self.wait_tcf()
    }

    pub fn read_reg(&self, cmd: u32, buf: &mut [u8]) -> Result<(), ErrorCode> {
        self.wait_not_busy()?;
        self.regs.fcr().write(|w| {
            w.set_ctcf(true);
            w.set_ctef(true);
        });
        self.regs.dlr().write(|w| w.set_dl(buf.len() as u32 - 1));
        self.regs.ccr().write(|w| {
            w.set_imode(CcrImode::B_0X1);
            w.set_admode(CcrAdmode::B_0X0);
            w.set_dmode(CcrDmode::B_0X1);
        });
        self.regs.cr().modify(|w| w.set_fmode(Fmode::B_0X1));
        self.regs.ir().write(|w| w.set_instruction(cmd));

        for byte in buf.iter_mut() {
            let mut got_data = false;
            for _ in 0..0xFFFFu32 {
                if self.regs.sr().read().flevel() > 0 {
                    got_data = true;
                    break;
                }
                cortex_m::asm::nop();
            }
            if !got_data {
                return Err(ErrorCode::new(0x2002).unwrap());
            }
            *byte = self.regs.dr().read().data() as u8;
        }

        self.wait_tcf()?;
        self.regs.cr().modify(|w| w.set_fmode(Fmode::B_0X0));
        Ok(())
    }

    pub fn write_with_addr(&self, cmd: u32, addr: u32, data: &[u8]) -> Result<(), ErrorCode> {
        self.wait_not_busy()?;
        self.regs.fcr().write(|w| {
            w.set_ctcf(true);
            w.set_ctef(true);
        });
        self.regs.dlr().write(|w| w.set_dl(data.len() as u32 - 1));
        self.regs.ccr().write(|w| {
            w.set_imode(CcrImode::B_0X1);
            w.set_admode(CcrAdmode::B_0X1);
            w.set_adsize(CcrAdsize::B_0X3);
            w.set_dmode(CcrDmode::B_0X1);
        });
        self.regs.cr().modify(|w| w.set_fmode(Fmode::B_0X0));
        self.regs.ir().write(|w| w.set_instruction(cmd));
        self.regs.ar().write(|w| w.set_address(addr));

        // Use volatile byte writes (STRB) to push exactly 1 byte per write.
        // The PAC's dr().write() generates a 32-bit STR which pushes 4 bytes.
        let dr_ptr = unsafe { (self.regs.as_ptr() as *mut u8).add(0x50) };
        for &byte in data {
            let mut got_space = false;
            for _ in 0..0xFFFFu32 {
                if self.regs.sr().read().flevel() < 32 {
                    got_space = true;
                    break;
                }
                cortex_m::asm::nop();
            }
            if !got_space {
                return Err(ErrorCode::new(0x2003).unwrap());
            }
            unsafe { core::ptr::write_volatile(dr_ptr, byte) };
        }

        self.wait_tcf()
    }

    pub fn exec_command_with_addr(&self, cmd: u32, addr: u32) -> Result<(), ErrorCode> {
        self.wait_not_busy()?;
        self.regs.fcr().write(|w| w.set_ctcf(true));
        self.regs.ccr().write(|w| {
            w.set_imode(CcrImode::B_0X1);
            w.set_admode(CcrAdmode::B_0X1);
            w.set_adsize(CcrAdsize::B_0X3);
            w.set_dmode(CcrDmode::B_0X0);
        });
        self.regs.ir().write(|w| w.set_instruction(cmd));
        self.regs.ar().write(|w| w.set_address(addr));
        self.wait_tcf()
    }

    pub fn read_data(&self, addr: u32, buf: &mut [u8]) -> Result<(), ErrorCode> {
        self.wait_not_busy()?;
        self.regs.fcr().write(|w| {
            w.set_ctcf(true);
            w.set_ctef(true);
        });
        self.regs.dlr().write(|w| w.set_dl(buf.len() as u32 - 1));
        // TCR: 8 dummy cycles for FAST_READ_4B
        self.regs.tcr().modify(|w| w.set_dcyc(8));
        self.regs.ccr().write(|w| {
            w.set_imode(CcrImode::B_0X1);
            w.set_admode(CcrAdmode::B_0X1);
            w.set_adsize(CcrAdsize::B_0X3);
            w.set_dmode(CcrDmode::B_0X1);
        });
        self.regs.cr().modify(|w| w.set_fmode(Fmode::B_0X1)); // Indirect read
        self.regs.ir().write(|w| w.set_instruction(CMD_FAST_READ_4B));
        self.regs.ar().write(|w| w.set_address(addr)); // Triggers transfer

        // Read data in 4-byte chunks (32-bit DR read pops up to 4 bytes from FIFO)
        let mut offset = 0usize;
        let mut remaining = buf.len();
        while remaining > 0 {
            let chunk = core::cmp::min(remaining, 4);
            let mut got_data = false;
            for _ in 0..0xFFFFu32 {
                if self.regs.sr().read().flevel() as usize >= chunk {
                    got_data = true;
                    break;
                }
                cortex_m::asm::nop();
            }
            if !got_data {
                self.regs.cr().modify(|w| w.set_fmode(Fmode::B_0X0));
                return Err(ErrorCode::new(0x2006).unwrap());
            }
            let word = self.regs.dr().read().data();
            buf[offset..offset + chunk].copy_from_slice(&word.to_le_bytes()[..chunk]);
            offset += chunk;
            remaining -= chunk;
        }

        self.wait_tcf()?;
        // Reset TCR dummy cycles and FMODE
        self.regs.tcr().modify(|w| w.set_dcyc(0));
        self.regs.cr().modify(|w| w.set_fmode(Fmode::B_0X0));
        Ok(())
    }

    fn wait_tcf(&self) -> Result<(), ErrorCode> {
        for _ in 0..0xFFFFu32 {
            if self.regs.sr().read().tcf() {
                self.regs.fcr().write(|w| w.set_ctcf(true));
                return Ok(());
            }
            cortex_m::asm::nop();
        }
        Err(ErrorCode::new(0x2004).unwrap())
    }
}

// ---- Peripheral init functions ----

pub fn init_clocks() {
    let rcc = pac::RCC;

    // 1. Enable SYSCFG clock (needed for compensation cells)
    rcc.apb4hensr().write(|w| w.set_syscfgens(true));
    let _ = rcc.apb4hensr().read();

    // 2. Enable GPION + PWR clocks
    rcc.ahb4enr().modify(|w| {
        w.set_gpionen(true);
        w.set_pwren(true);
    });

    // 3. Enable XSPI2 + XSPIM clocks
    rcc.ahb5enr().modify(|w| {
        w.set_xspi2en(true);
        w.set_xspimen(true);
    });

    // 3b. Enable RIFSC + RISAF clocks
    rcc.ahb3enr().modify(|w| {
        w.set_rifscen(true);
        w.set_risafen(true);
    });
    let _ = rcc.ahb3enr().read();

    // 4. Configure IC4: source=PLL2 (bypass=HSI 64MHz), divider=1
    rcc.iccfgr(3).write(|w| {
        w.set_icsel(Icsel::PLL2);
        w.set_icint(pac::rcc::vals::Icint::from_bits(0));
    });
    rcc.divensr().modify(|w| w.set_ic4ens(true));

    // 5. Select IC4 as XSPI2 kernel clock source
    rcc.ccipr6().modify(|w| {
        w.set_xspi2sel(Xspisel::IC4);
    });
}

pub fn init_power() {
    let pwr = pac::PWR;
    let _val = pwr.svmcr3().read();
    pwr.svmcr3().modify(|w| {
        w.set_vddio3sv(Vddio3sv::B_0X1);
        w.set_vddio3vmen(true);
        w.set_vddio3vrsel(Vddio3vrsel::B_0X1);
    });
}

pub fn init_compensation_cells() {
    let syscfg = pac::SYSCFG;
    syscfg.vddio3cccr().write(|w| {
        w.set_ransrc(0x7);
        w.set_rapsrc(0x8);
        w.set_en(Vddio3cccrEn::B_0X1);
        w.set_cs(Vddio3cccrCs::B_0X1);
    });
}

pub fn init_gpio() {
    let gpion = pac::GPION;

    let xspi_pins: &[usize] = &[0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 11];

    gpion.moder().modify(|w| {
        for &pin in xspi_pins {
            w.set_moder(pin, gpio::vals::Moder::ALTERNATE);
        }
    });

    gpion.ospeedr().modify(|w| {
        for &pin in xspi_pins {
            w.set_ospeedr(pin, gpio::vals::Ospeedr::VERY_HIGH_SPEED);
        }
    });

    gpion.afr(0).modify(|w| {
        for &pin in &[0u8, 1, 2, 3, 4, 5, 6] {
            w.set_afr(pin as usize, 9);
        }
    });
    gpion.afr(1).modify(|w| {
        for &pin in &[8u8, 9, 10, 11] {
            w.set_afr((pin - 8) as usize, 9);
        }
    });

    gpion.pupdr().modify(|w| {
        w.set_pupdr(1, gpio::vals::Pupdr::PULL_UP);
    });
}

pub fn init_risaf() {
    // Configure RISAF2 (XSPI2 memory region filter) to allow access to 0x70000000.
    let risaf2 = pac::RISAF2;

    // Region 0: non-secure, all CIDs read+write, full address range
    risaf2.reg_cidcfgr(0).write(|w| {
        for i in 0..8 {
            w.set_rdenc(i, true);
            w.set_wrenc(i, true);
        }
    });
    risaf2.reg_endr(0).write(|w| w.set_baddend(0xFFFF_FFFF));
    risaf2.reg_cfgr(0).write(|w| {
        w.set_bren(true);
        w.set_sec(false); // Non-secure so debug probe can access
    });
}

pub fn init_xspim() {
    let xspim = pac::XSPIM;
    xspim.cr().modify(|w| {
        w.set_muxen(false);
        w.set_req2ack_time(1);
        w.set_cssel_ovr_en(true);
        w.set_cssel_ovr_o2(false);    // false=NCS1 for XSPI2 (PN1)
    });
}
