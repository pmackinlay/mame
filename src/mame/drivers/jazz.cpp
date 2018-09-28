// license:BSD-3-Clause
// copyright-holders:Patrick Mackinlay

/*
 * An emulation of systems based on the Jazz computer architecture, originally
 * developed by Microsoft. Specific systems which implemented this architecture
 * include the MIPS Magnum/Millenium 4000 and Olivetti M700-10.
 *
 * References:
 *
 *   https://www.linux-mips.org/wiki/Jazz
 *   http://gunkies.org/wiki/MIPS_Magnum
 *   http://www.sensi.org/~alec/mips/mips-history.html
 *
 * TODO
 *   - everything (skeleton only)
 *
 * Unconfirmed parts lists from ARCSystem reference design (which appears to
 * be very similar or identical to the Jazz system) taken from:
 *   https://www.linux-mips.org/archives/riscy/1993-12/msg00013.html
 *
 *   Ref   Part                      Function
 *
 * System board:
 *
 *         Dallas DS1287             RTC and NVRAM
 *         Dallas DS1225Y            8k non-volatile SRAM
 *         WD16C552                  Dual serial and parallel port controller
 *         Intel N82077A             Floppy drive controller
 *         National DP83932BFV       Ethernet controller
 *         Intel 82358               EISA Bus Controller
 *         Intel 82357               EISA Integrated System Peripheral (ISP)
 *         Intel 82352 x 2           EISA Bus Buffer (EBB)
 *         Emulex FAS216             SCSI controller
 *         27C01                     128k EPROM
 *         28F020                    256k flash memory
 *         NEC μPD31432              ARC address path ASIC
 *         NEC μPD31431 x 2          ARC data path ASIC
 *         NEC μPD30400              R4000PC/50 CPU
 *
 * Audio board:
 *
 *         Crystal CS4215            Audio codec
 *         Altera FPGA x 4           Audio DMA
 *
 * Video board:
 *
 *         27C010                    128k EPROM
 *         IMS G364-11S              Video controller
 *         NEC μPD42274V-80 x 16     256kx4 VRAM (2MiB)
 */

#include "emu.h"

#include "includes/jazz.h"

#include "debugger.h"

#define VERBOSE 0
#include "logmacro.h"

void jazz_state::machine_start()
{
}

void jazz_state::machine_reset()
{
}

void jazz_state::init_common()
{
	// map the configured ram and vram
	m_maincpu->space(0).install_ram(0x00000000, 0x00000000 | m_ram->mask(), m_ram->pointer());
	m_maincpu->space(0).install_ram(0x40000000, 0x40000000 | m_vram->mask(), m_vram->pointer());
}

void jazz_state::jazz_common_map(address_map &map)
{
	map(0x1fc00000, 0x1fc3ffff).r(m_flash, FUNC(amd_28f020_device::read));

	map(0x40000000, 0x407fffff).ram().share("vram"); // local video memory
	map(0x60000000, 0x60000007).lr8("video_rom", []() { return 0x10; }).umask64(0xff);
	map(0x60080000, 0x60081fff).m(m_ramdac, FUNC(g364_device::map)); // local video control

	// FIXME: lots of guesswork and assumptions here for now
	map(0x80000000, 0x80000fff).m(m_mct_adr, FUNC(jazz_mct_adr_device::map));
	map(0x80000238, 0x8000023f).r(m_pic[0], FUNC(pic8259_device::acknowledge)).umask64(0xffffffff); // eisa_irq_ack

	map(0x80001000, 0x800010ff).m(m_network, FUNC(dp83932c_device::map)).umask32(0x0000ffff);
	map(0x80002000, 0x8000200f).m(m_scsi, FUNC(ncr53c94_device::map));
	map(0x80003000, 0x8000300f).m(m_fdc, FUNC(n82077aa_device::map));
	map(0x80004000, 0x80004007).lrw8("rtc",
		[this](address_space &space, offs_t offset) { return m_rtc->read(space, 1); },
		[this](address_space &space, offs_t offset, u8 data) { m_rtc->write(space, 1, data); }).umask64(0xff);
	map(0x80005000, 0x80005007).rw(m_kbdc, FUNC(at_keyboard_controller_device::data_r), FUNC(at_keyboard_controller_device::data_w)).umask64(0x00ff);
	map(0x80005000, 0x80005007).rw(m_kbdc, FUNC(at_keyboard_controller_device::status_r), FUNC(at_keyboard_controller_device::command_w)).umask64(0xff00);
	map(0x80006000, 0x80006007).rw(m_ace[0], FUNC(ns16550_device::ins8250_r), FUNC(ns16550_device::ins8250_w));
	map(0x80007000, 0x80007007).rw(m_ace[1], FUNC(ns16550_device::ins8250_r), FUNC(ns16550_device::ins8250_w));
	map(0x80008000, 0x80008007).rw(m_lpt, FUNC(pc_lpt_device::read), FUNC(pc_lpt_device::write)).umask64(0xffffffff);
	map(0x80009000, 0x8000afff).ram().share("nvram"); // 9000-9fff unprotected/a000-afff protected?
	//map(0x8000b000, 0x8000bfff).ram().share("nvram"); // read-only?  also sonic IO access?
	//map(0x8000c000, 0x8000cfff) // sound
	//map(0x8000d000, 0x8000dfff).noprw(); // dummy dma device?

	map(0x8000f000, 0x8000f007).lrw8(
		"led",
		[this]() { return m_led; },
		[this](u8 data)
		{
			logerror("led 0x%02x\n", data);
			m_led = data;
		}).umask64(0xff);

	//map(0x800e0000, 0x800fffff).m() // dram config

	//map(0x90000000, 0x90ffffff).m(); // ISA I/O ports
	map(0x90000020, 0x90000027).rw(m_pic[0], FUNC(pic8259_device::read), FUNC(pic8259_device::write)).umask64(0xffff);
	map(0x90000040, 0x90000047).rw(m_pit[0], FUNC(pit8254_device::read), FUNC(pit8254_device::write)).umask64(0xffffffff);
	//map(0x90000061, 0x90000061).noprw(); // "port B"
	map(0x90000070, 0x90000077).lw8("rtc_index", [this](address_space &space, offs_t offset, u8 data) { m_rtc->write(space, 0, data & 0x7f); }).umask64(0xff);
	map(0x900000a0, 0x900000a7).rw(m_pic[1], FUNC(pic8259_device::read), FUNC(pic8259_device::write)).umask64(0xffff);
	//map(0x90000461, 0x00000461).nopw(); // extended nmi status/control register?

	//map(0x91000000, 0x91ffffff).m(); // i82357 integrated system peripheral
	//map(0x92000000, 0x92ffffff).m(); // EISA I/O ports?
	//map(0x93000000, 0x93ffffff).m(); // EISA memory

	//map(0xf0000000, 0xf0000fff).m(); // interrupt source registers
	map(0xf0000000, 0xf0000007).r(m_mct_adr, FUNC(jazz_mct_adr_device::isr_r)).umask64(0xffff);
	map(0xf0000000, 0xf0000007).rw(m_mct_adr, FUNC(jazz_mct_adr_device::imr_r), FUNC(jazz_mct_adr_device::imr_w)).umask64(0xffff0000);

	map(0xfff00000, 0xfff3ffff).r(m_flash, FUNC(amd_28f020_device::read)); // mirror?
}

void jazz_state::jazz_be_map(address_map &map)
{
	jazz_common_map(map);

	map(0x80006000, 0x80006007).lrw8("ace0",
		[this](address_space &space, offs_t offset) { return m_ace[0]->ins8250_r(space, 7 - offset); },
		[this](address_space &space, offs_t offset, u8 data) { m_ace[0]->ins8250_w(space, 7 - offset, data); });

	map(0x80007000, 0x80007007).lrw8("ace1",
		[this](address_space &space, offs_t offset) { return m_ace[1]->ins8250_r(space, 7 - offset); },
		[this](address_space &space, offs_t offset, u8 data) { m_ace[1]->ins8250_w(space, 7 - offset, data); });
}

void jazz_state::jazz_le_map(address_map &map)
{
	jazz_common_map(map);
}

void jazz_state::ram(address_map &map)
{
	map(0x00000000, 0x007fffff).ram().share(RAM_TAG);
}

static void jazz_scsi_devices(device_slot_interface &device)
{
	device.option_add("harddisk", NSCSI_HARDDISK);
	device.option_add("cdrom", NSCSI_CDROM);
}

static void jazz_scsi_adapter(device_t *device)
{
	ncr53c94_device &adapter = downcast<ncr53c94_device &>(*device);

	adapter.set_clock(24_MHz_XTAL);

	adapter.irq_handler_cb().set(":mct_adr", FUNC(jazz_mct_adr_device::irq<5>));;
	//adapter.drq_handler_cb().set(m_mct_adr, FUNC(interpro_ioga_device::drq_scsi));
}

void jazz_state::jazz(machine_config &config)
{
	m_maincpu->set_addrmap(AS_PROGRAM, &jazz_state::jazz_common_map);

	RAM(config, m_ram);
	m_ram->set_default_size("16M");
	m_ram->set_extra_options("32M,64M,128M,256M");
	m_ram->set_default_value(0);

	RAM(config, m_vram);
	m_vram->set_default_size("2M");
	m_vram->set_default_value(0);

	// FIXME: may require big and little endian variants
	JAZZ_MCT_ADR(config, m_mct_adr, 0);
	m_mct_adr->set_ram(RAM_TAG);
	m_mct_adr->out_int0_cb().set_inputline(m_maincpu, INPUT_LINE_IRQ4);
	m_mct_adr->out_int1_cb().set_inputline(m_maincpu, INPUT_LINE_IRQ1);

	// scsi bus and devices
	NSCSI_BUS(config, m_scsibus, 0);

	nscsi_connector &harddisk(NSCSI_CONNECTOR(config, "scsi:0", 0));
	jazz_scsi_devices(harddisk);
	harddisk.set_default_option("harddisk");

	nscsi_connector &cdrom(NSCSI_CONNECTOR(config, "scsi:6", 0));
	jazz_scsi_devices(cdrom);
	cdrom.set_default_option("cdrom");

	jazz_scsi_devices(NSCSI_CONNECTOR(config, "scsi:1", 0));
	jazz_scsi_devices(NSCSI_CONNECTOR(config, "scsi:2", 0));
	jazz_scsi_devices(NSCSI_CONNECTOR(config, "scsi:3", 0));
	jazz_scsi_devices(NSCSI_CONNECTOR(config, "scsi:4", 0));
	jazz_scsi_devices(NSCSI_CONNECTOR(config, "scsi:5", 0));

	// scsi host adapter
	nscsi_connector &adapter(NSCSI_CONNECTOR(config, "scsi:7", 0));
	adapter.option_add_internal("host", NCR53C94);
	adapter.set_default_option("host");
	adapter.set_fixed(true);
	adapter.set_option_machine_config("host", jazz_scsi_adapter);

	N82077AA(config, m_fdc, 24_MHz_XTAL);
	m_fdc->intrq_wr_callback().set(m_mct_adr, FUNC(jazz_mct_adr_device::irq<1>));

	MC146818(config, m_rtc, 32.768_kHz_XTAL);

	NVRAM(config, m_nvram, nvram_device::DEFAULT_ALL_0);

	AMD_28F020(config, m_flash);

	// pc keyboard connector
	pc_kbdc_device &kbdc(PC_KBDC(config, "pc_kbdc", 0));
	kbdc.out_clock_cb().set(m_kbdc, FUNC(at_keyboard_controller_device::keyboard_clock_w));
	kbdc.out_data_cb().set(m_kbdc, FUNC(at_keyboard_controller_device::keyboard_data_w));

	// keyboard port
	pc_kbdc_slot_device &kbd(PC_KBDC_SLOT(config, "kbd", 0));
	pc_at_keyboards(kbd);
	kbd.set_default_option(STR_KBD_IBM_PC_AT_84);
	kbd.set_pc_kbdc_slot(&kbdc);

	// at keyboard controller
	AT_KEYBOARD_CONTROLLER(config, m_kbdc, 12_MHz_XTAL);
	m_kbdc->system_reset_cb().set_inputline(m_maincpu, INPUT_LINE_RESET);
	//m_kbdc->gate_a20_cb().set_inputline()
	m_kbdc->keyboard_clock_cb().set(kbdc, FUNC(pc_kbdc_device::clock_write_from_mb));
	m_kbdc->keyboard_data_cb().set(kbdc, FUNC(pc_kbdc_device::data_write_from_mb));
	m_kbdc->input_buffer_full_cb().set(m_mct_adr, FUNC(jazz_mct_adr_device::irq<6>));; // keyboard interrupt
	//m_kbdc->output_buffer_empty_cb().set(m_mct_adr, FUNC(jazz_mct_adr_device::irq<7>)); // mouse interrupt

	SCREEN(config, m_screen, SCREEN_TYPE_RASTER);
	m_screen->set_raw(78643200, 1280, 0, 1280, 1024, 0, 1024);
	m_screen->set_screen_update("g364", FUNC(g364_device::screen_update));
	//m_screen->screen_vblank().set(m_mct_adr, FUNC(jazz_mct_adr_device::irq<3>)); // maybe?

	G364(config, m_ramdac, 5_MHz_XTAL); // FIXME: guessed clock
	m_ramdac->set_screen(m_screen);
	m_ramdac->set_vram(m_vram);

	// WD16C552 (two 16550 + pc_lpt)
	NS16550(config, m_ace[0], 8_MHz_XTAL); // apparently?
	rs232_port_device &serial0(RS232_PORT(config, "serial0", default_rs232_devices, nullptr));

	m_ace[0]->out_dtr_callback().set(serial0, FUNC(rs232_port_device::write_dtr));
	m_ace[0]->out_rts_callback().set(serial0, FUNC(rs232_port_device::write_rts));
	m_ace[0]->out_tx_callback().set(serial0, FUNC(rs232_port_device::write_txd));
	m_ace[0]->out_int_callback().set(m_mct_adr, FUNC(jazz_mct_adr_device::irq<8>));

	serial0.cts_handler().set(m_ace[0], FUNC(ns16550_device::cts_w));
	serial0.dcd_handler().set(m_ace[0], FUNC(ns16550_device::dcd_w));
	serial0.dsr_handler().set(m_ace[0], FUNC(ns16550_device::dsr_w));
	serial0.ri_handler().set(m_ace[0], FUNC(ns16550_device::ri_w));
	serial0.rxd_handler().set(m_ace[0], FUNC(ns16550_device::rx_w));

	NS16550(config, m_ace[1], 8_MHz_XTAL);
	rs232_port_device &serial1(RS232_PORT(config, "serial1", default_rs232_devices, nullptr));

	m_ace[1]->out_dtr_callback().set(serial1, FUNC(rs232_port_device::write_dtr));
	m_ace[1]->out_rts_callback().set(serial1, FUNC(rs232_port_device::write_rts));
	m_ace[1]->out_tx_callback().set(serial1, FUNC(rs232_port_device::write_txd));
	m_ace[1]->out_int_callback().set(m_mct_adr, FUNC(jazz_mct_adr_device::irq<9>));

	serial1.cts_handler().set(m_ace[1], FUNC(ns16550_device::cts_w));
	serial1.dcd_handler().set(m_ace[1], FUNC(ns16550_device::dcd_w));
	serial1.dsr_handler().set(m_ace[1], FUNC(ns16550_device::dsr_w));
	serial1.ri_handler().set(m_ace[1], FUNC(ns16550_device::ri_w));
	serial1.rxd_handler().set(m_ace[1], FUNC(ns16550_device::rx_w));

	PC_LPT(config, m_lpt, 0);
	m_lpt->irq_handler().set(m_mct_adr, FUNC(jazz_mct_adr_device::irq<0>));

	// i82357 (two 82C59 + two 82C54 + two 82C37)
	PIC8259(config, m_pic[0], 0);
	m_pic[0]->out_int_callback().set_inputline(m_maincpu, INPUT_LINE_IRQ2);
	m_pic[0]->in_sp_callback().set_constant(1);
	m_pic[0]->read_slave_ack_callback().set(FUNC(jazz_state::pic_slave_ack));

	PIC8259(config, m_pic[1], 0);
	m_pic[1]->out_int_callback().set(m_pic[0], FUNC(pic8259_device::ir2_w));
	m_pic[1]->in_sp_callback().set_constant(0);

	PIT8254(config, m_pit[0], 0);
	m_pit[0]->set_clk<0>(14.318181_MHz_XTAL / 12); // irq0
	m_pit[0]->set_clk<1>(14.318181_MHz_XTAL / 12); // refresh
	m_pit[0]->set_clk<2>(14.318181_MHz_XTAL / 12); // speaker
	m_pit[0]->out_handler<0>().set(m_pic[0], FUNC(pic8259_device::ir0_w));
	m_pit[0]->out_handler<1>().set(m_pit[1], FUNC(pit8254_device::write_gate2));
	//m_pit[0]->out_handler<2>() // TODO: speaker
	// TODO: port 61 -> m_pit[0]->write_gate2()

	PIT8254(config, m_pit[1], 0);
	m_pit[1]->set_clk<0>(14.318181_MHz_XTAL / 48); // nmi
	// counter 1 does not exist
	m_pit[1]->set_clk<2>(8_MHz_XTAL); // cpu speed
	//m_pit[1]->out_handler<0>().set(); // TODO: nmi

	// sound, interrupt 2

	DP83932C(config, m_network, 20_MHz_XTAL);
	m_network->out_int_cb().set(m_mct_adr, FUNC(jazz_mct_adr_device::irq<4>));
	m_network->set_ram(RAM_TAG);
}

void jazz_state::mmr4000be(machine_config &config)
{
	R4000BE(config, m_maincpu, 50_MHz_XTAL);

	jazz(config);

}

void jazz_state::mmr4000le(machine_config &config)
{
	R4000LE(config, m_maincpu, 50_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &jazz_state::jazz_le_map);

	jazz(config);
}

u8 jazz_state::pic_slave_ack(offs_t offset)
{
	if (offset == 2)
		return m_pic[1]->acknowledge();

	return 0;
}

ROM_START(mmr4000be)
	ROM_REGION64_BE(0x40000, "flash", 0)
	ROM_SYSTEM_BIOS(0, "riscos", "R4000 RISC/os PROM")
	ROMX_LOAD("riscos.bin", 0x00000, 0x40000, CRC(cea6bc8f) SHA1(3e47b4ad5d1a0c7aac649e6aef3df1bf86fc938b), ROM_BIOS(0))

	// borrowed from ps2.cpp
	ROM_REGION(0x800, "at_keybc", 0)
	ROM_LOAD("72x8455.zm82", 0x000, 0x800, CRC(7da223d3) SHA1(54c52ff6c6a2310f79b2c7e6d1259be9de868f0e))
ROM_END

ROM_START(mmr4000le)
	ROM_REGION64_LE(0x40000, "flash", 0)
	ROM_SYSTEM_BIOS(0, "ntprom", "R4000 Windows NT PROM")
	ROMX_LOAD("ntprom.bin", 0x00000, 0x40000, CRC(d91018d7) SHA1(316de17820192c89b8ee6d9936ab8364a739ca53), ROM_BIOS(0))

	// borrowed from ps2.cpp
	ROM_REGION(0x800, "at_keybc", 0)
	ROM_LOAD("72x8455.zm82", 0x000, 0x800, CRC(7da223d3) SHA1(54c52ff6c6a2310f79b2c7e6d1259be9de868f0e))
ROM_END

/*    YEAR   NAME       PARENT  COMPAT  MACHINE    INPUT  CLASS       INIT         COMPANY  FULLNAME                 FLAGS */
COMP( 1992,  mmr4000be, 0,      0,      mmr4000be, 0,     jazz_state, init_common, "MIPS",  "Magnum R4000 (big)",    MACHINE_IS_SKELETON)
COMP( 1992,  mmr4000le, 0,      0,      mmr4000le, 0,     jazz_state, init_common, "MIPS",  "Magnum R4000 (little)", MACHINE_IS_SKELETON)
