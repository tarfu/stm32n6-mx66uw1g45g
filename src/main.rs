#![no_std]
#![no_main]

use flash_algorithm::*;
use stm32_metapac as pac;
use pac::xspi::vals::{CcrAdmode, CcrAdsize, CcrDmode, CcrImode, Fmode,
    WccrImode, WccrAdmode, WccrAdsize, WccrDmode};

use stm32n6_mx66uw1g45g::*;

algorithm!(Algorithm, {
    device_name: "MX66UW1G45G",
    device_type: DeviceType::ExtSpi,
    flash_address: 0x70000000,
    flash_size: 0x8000000,
    page_size: 0x10000,
    empty_value: 0xFF,
    program_time_out: 100000,
    erase_time_out: 300000,
    sectors: [{
        size: 0x10000,
        address: 0x0,
    }]
});

struct Algorithm {
    xspi: Xspi,
}

impl Algorithm {
    fn read_sr(&self) -> Result<u8, ErrorCode> {
        let mut buf = [0u8; 1];
        self.xspi.read_reg(CMD_READ_STATUS_REG, &mut buf)?;
        Ok(buf[0])
    }

    fn wait_write_finish(&self, max_iters: u32) -> Result<(), ErrorCode> {
        for _ in 0..max_iters {
            let sr = self.read_sr()?;
            if sr & SR_WIP == 0 {
                return Ok(());
            }
        }
        Err(ErrorCode::new(0x2005).unwrap())
    }

    fn write_enable(&self) -> Result<(), ErrorCode> {
        self.xspi.exec_command(CMD_WRITE_ENABLE)
    }

    #[allow(dead_code)]
    fn read_id(&self) -> Result<[u8; 3], ErrorCode> {
        let mut buf = [0u8; 3];
        self.xspi.read_reg(CMD_READ_ID, &mut buf)?;
        Ok(buf)
    }
}

impl FlashAlgorithm for Algorithm {
    fn new(_address: u32, _clock: u32, _function: Function) -> Result<Self, ErrorCode> {
        // Reset XSPI2 early, BEFORE reconfiguring clocks.
        // After UnInit, XSPI2 is in memory-mapped mode. Reconfiguring IC4 while
        // XSPI2 is active can cause a clock glitch that leaves it stuck busy.
        // Resetting it first ensures it's idle (EN=0) before any clock changes.
        let rcc = pac::RCC;
        rcc.ahb5enr().modify(|w| w.set_xspi2en(true));
        rcc.ahb5rstr().modify(|w| w.set_xspi2rst(true));
        cortex_m::asm::dsb();
        rcc.ahb5rstr().modify(|w| w.set_xspi2rst(false));
        cortex_m::asm::dsb();

        init_clocks();
        init_risaf();
        init_power();
        init_compensation_cells();
        init_gpio();
        init_xspim();

        let xspi = Xspi::new();
        xspi.init()?;

        Ok(Self { xspi })
    }

    fn erase_all(&mut self) -> Result<(), ErrorCode> {
        self.write_enable()?;
        self.xspi.exec_command(CMD_CHIP_ERASE)?;
        // Chip erase: ~150s typical for 1Gbit. At ~30μs/poll, 20M iters ≈ 600s.
        self.wait_write_finish(20_000_000)
    }

    fn erase_sector(&mut self, addr: u32) -> Result<(), ErrorCode> {
        let flash_addr = addr - FLASH_BASE;
        self.write_enable()?;
        self.xspi.exec_command_with_addr(CMD_BLOCK_ERASE_64K_4B, flash_addr)?;
        // 64KB block erase: max 2s typical. At ~30μs/poll, 500K iters ≈ 15s.
        self.wait_write_finish(500_000)
    }

    fn program_page(&mut self, addr: u32, data: &[u8]) -> Result<(), ErrorCode> {
        let flash_addr = addr - FLASH_BASE;
        let mut offset = 0usize;
        while offset < data.len() {
            let chunk_len = core::cmp::min(FLASH_PAGE_SIZE, data.len() - offset);
            self.write_enable()?;
            self.xspi.write_with_addr(
                CMD_PAGE_PROGRAM_4B,
                flash_addr + offset as u32,
                &data[offset..offset + chunk_len],
            )?;
            // Page program: max 1.5ms. At ~30μs/poll, 10K iters ≈ 300ms.
            self.wait_write_finish(10_000)?;
            offset += chunk_len;
        }
        Ok(())
    }

    fn verify(&mut self, address: u32, size: u32, data: Option<&[u8]>) -> Result<(), u32> {
        let flash_addr = address - FLASH_BASE;
        let expected = match data {
            Some(d) => d,
            None => return Ok(()),
        };

        let mut buf = [0u8; 256];
        let mut offset = 0u32;
        while offset < size {
            let chunk = core::cmp::min(256, (size - offset) as usize);
            if self.xspi.read_data(flash_addr + offset, &mut buf[..chunk]).is_err() {
                return Err(address + offset);
            }
            for i in 0..chunk {
                if buf[i] != expected[offset as usize + i] {
                    return Err(address + offset + i as u32);
                }
            }
            offset += chunk as u32;
        }

        Ok(())
    }

    fn blank_check(&mut self, address: u32, size: u32, pattern: u8) -> Result<(), ErrorCode> {
        let flash_addr = address - FLASH_BASE;

        let mut buf = [0u8; 256];
        let mut offset = 0u32;
        while offset < size {
            let chunk = core::cmp::min(256, (size - offset) as usize);
            if self.xspi.read_data(flash_addr + offset, &mut buf[..chunk]).is_err() {
                return Err(ErrorCode::new(1).unwrap());
            }
            for i in 0..chunk {
                if buf[i] != pattern {
                    return Err(ErrorCode::new(1).unwrap());
                }
            }
            offset += chunk as u32;
        }

        Ok(())
    }
}

impl Drop for Algorithm {
    fn drop(&mut self) {
        // Wait for any pending flash operation to finish
        let _ = self.wait_write_finish(500_000);
        let _ = self.xspi.wait_not_busy();

        // Abort any ongoing XSPI transaction
        self.xspi.regs.cr().modify(|w| w.set_abort(true));
        while self.xspi.regs.cr().read().abort() {}

        // Clear all XSPI flags
        self.xspi.regs.fcr().write(|w| {
            w.set_ctcf(true);
            w.set_ctef(true);
        });

        // Set FMODE=0x0 (indirect write) before configuring
        self.xspi.regs.cr().modify(|w| w.set_fmode(Fmode::B_0X0));

        // Configure write side (WCCR/WTCR/WIR) matching embassy order
        self.xspi.regs.wccr().write(|w| {
            w.set_imode(WccrImode::B_0X1);
            w.set_admode(WccrAdmode::B_0X1);
            w.set_adsize(WccrAdsize::B_0X3);
            w.set_dmode(WccrDmode::B_0X1);
        });
        self.xspi.regs.wtcr().write(|w| w.set_dcyc(0));
        self.xspi.regs.wir().write(|w| w.set_instruction(CMD_PAGE_PROGRAM_4B));

        // Configure read side using write (not modify) for clean register state
        self.xspi.regs.dlr().write(|w| w.set_dl(0));

        // TCR: 8 dummy cycles for FAST_READ_4B, preserve sshift/dhqc
        self.xspi.regs.tcr().write(|w| {
            w.set_dcyc(8);
            w.set_sshift(true);
            w.set_dhqc(true);
        });

        // CCR: single-line SPI read with 4-byte address (clean write, all other bits zero)
        self.xspi.regs.ccr().write(|w| {
            w.set_imode(CcrImode::B_0X1);
            w.set_admode(CcrAdmode::B_0X1);
            w.set_adsize(CcrAdsize::B_0X3);
            w.set_dmode(CcrDmode::B_0X1);
        });

        // IR: FAST_READ_4B instruction (0x0C)
        self.xspi.regs.ir().write(|w| w.set_instruction(CMD_FAST_READ_4B));

        let _ = self.xspi.wait_not_busy();

        // Switch to memory-mapped mode
        self.xspi.regs.cr().modify(|w| {
            w.set_fmode(Fmode::B_0X3);
            w.set_tcen(false);
        });
    }
}
