// license:BSD-3-Clause
// copyright-holders:Patrick Mackinlay

/*
 * BBN Advanced Computers Inc. TC2000 TC/TCS TCS interface card.
 *
 * Sources:
 *  - TC2000 Hardware Architecture, Revision 2/14/90, BBN Advanced Computers Inc.
 *
 * TODO:
 *  - skeleton only
 */

#include "emu.h"
#include "tctcs.h"

#define VERBOSE (LOG_GENERAL)
#include "logmacro.h"

enum ctl_mask : u8
{
	CTL_R1   = 0x01, // relay 1
	CTL_R2   = 0x02, // relay 2
	CTL_TB   = 0x04, // TCS bus enabled (inverted on read)
	CTL_PL   = 0x08, // main power LED
	CTL_UPS  = 0x10, // on UPS power (read)
	CTL_TBAB = 0x10, // TCS bus A*/B (write)
	CTL_WT   = 0x20, // watchdog timer (read)
	CTL_IL   = 0x20, // indicator LED (write)
	CTL_REV  = 0x40, // revision level
	CTL_SEC  = 0x80, // secure (0=secure)
};

DEFINE_DEVICE_TYPE(ISA16_TCTCS, isa16_tctcs_device, "tctcs", "BBN ACI TC-2000 TC/TCS card")

ROM_START(tctcs)
	ROM_REGION(0x10000, "tctcs", 0)
	ROM_LOAD("4617347g01__a700_revb.u25", 0x00000, 0x10000, CRC(dad060cf) SHA1(191a0efb7b71d784b75be34cc0ebb1c4dd5e2f2a))
ROM_END

static INPUT_PORTS_START(tctcs)
#if 0
	PORT_START("SW2")
	PORT_DIPNAME(0xe0, 0x00, "EPROM Size") //PORT_DIPLOCATION("SW2:6,7,8")
	PORT_DIPSETTING(0xe0, "2 kilobytes")
	PORT_DIPSETTING(0xc0, "4 kilobytes")
	PORT_DIPSETTING(0xa0, "8 kilobytes")
	PORT_DIPSETTING(0x80, "16 kilobytes")
	PORT_DIPSETTING(0x60, "32 kilobytes")

	PORT_DIPNAME(0x7f, 0x64, "EPROM Base") //PORT_DIPLOCATION("SW1:2,3,SW2:1,2,3,4,5")
	PORT_DIPSETTING(0x00, "0x00000")
	PORT_DIPSETTING(0x64, "0xc8000")
	PORT_DIPSETTING(0x7f, "0xfe000")

	PORT_DIPNAME(0x01, 0x01, "EPROM Code Version") //PORT_DIPLOCATION("SW1:1")
	PORT_DIPSETTING(0x00, "0")
	PORT_DIPSETTING(0x01, "1")

	PORT_DIPNAME(0x1f, 0x16, "I/O Base")
	PORT_DIPSETTING(0x16, "0x2c0")
#endif
INPUT_PORTS_END

isa16_tctcs_device::isa16_tctcs_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock)
	: device_t(mconfig, ISA16_TCTCS, tag, owner, clock)
	, device_isa16_card_interface(mconfig, *this)
	, m_duart(*this, "duart")
	, m_pit(*this, "pit")
#if 0
	, m_iobase(*this, "IO_BASE")
	, m_irqdrq(*this, "IRQ_DRQ")
	, m_romopts(*this, "ROM_OPTS")
	, m_test(*this, "TEST")
#endif
	, m_installed(false)
{
}

const tiny_rom_entry *isa16_tctcs_device::device_rom_region() const
{
	return ROM_NAME(tctcs);
}

void isa16_tctcs_device::device_add_mconfig(machine_config &config)
{
	SCN2681N40(config, m_duart, 0); // clock from 8254

	// 125kbit/sec, asynch, 1 start, 9 data, 1 stop, ~11,363 bytes/sec
	PIT8254(config, m_pit);
	m_pit->set_clk<0>(8_MHz_XTAL);
	m_pit->set_clk<1>(8_MHz_XTAL);
	m_pit->set_clk<2>(8_MHz_XTAL);
	m_pit->out_handler<0>().set(m_duart, FUNC(scn2681n40_device::clk_w)); // X2
	m_pit->out_handler<1>().set(m_duart, FUNC(scn2681n40_device::ip_w<3>)); // TxCA
	m_pit->out_handler<2>().set(m_duart, FUNC(scn2681n40_device::ip_w<4>)); // RxCA
}

ioport_constructor isa16_tctcs_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(tctcs);
}

void isa16_tctcs_device::device_start()
{
	set_isa_device();

	m_watchdog = timer_alloc(FUNC(isa16_tctcs_device::watchdog), this);

	m_ctlr = CTL_REV;
	m_ctlw = 0;
}

void isa16_tctcs_device::device_reset()
{
	if (!m_installed)
	{
		if (m_isa->is_option_rom_space_available(0xc8000, 0x800))
			m_isa->install_rom(this, 0xc8000, 0xc8000 | 0x07ff, "tctcs");

		m_isa->install_device(0x2c0, 0x2df, *this, &isa16_tctcs_device::map_isa);

#if 0
		u16 const base = m_iobase->read();
		m_isa->install_device(base, base | 0xf, *this, &isa16_tctcs_device::map_isa);

		m_isa_irq = m_irqdrq->read() & 0xf;
		m_isa_drq = (m_irqdrq->read() >> 4) & 0x7;

		if (m_romopts->read() & 1)
		{
			offs_t const rom_base = (m_romopts->read() & 0xfe) << 12;

			if (m_isa->is_option_rom_space_available(rom_base, 0x2000))
				m_isa->install_rom(this, rom_base, rom_base | 0x01fff, "host");
		}

		m_isa->set_dma_channel(m_isa_drq, this, true);
#endif
		m_installed = true;
	}

	m_watchdog->adjust(attotime::never);

	m_ctlw &= ~(CTL_IL | CTL_TBAB | CTL_TB);

	//m_isa_irq_asserted = false;
}

void isa16_tctcs_device::map_isa(address_map &map)
{
	map(0x00, 0x00).rw(FUNC(isa16_tctcs_device::ctl_r), FUNC(isa16_tctcs_device::ctl_w));
	map(0x01, 0x01).w(FUNC(isa16_tctcs_device::wdp_w));
	map(0x02, 0x02).w(FUNC(isa16_tctcs_device::wdc_w));
	map(0x04, 0x07).rw(m_pit, FUNC(pit8254_device::read), FUNC(pit8254_device::write));
	map(0x10, 0x1f).m(m_duart, FUNC(scn2681n40_device::map));
}


u8 isa16_tctcs_device::ctl_r()
{
	u8 data = m_ctlr | (((m_ctlw & (CTL_PL | CTL_TB | CTL_R2 | CTL_R1)) ^ CTL_TB));

	if (m_watchdog->enabled())
		data |= CTL_WT;

	return data;
}

void isa16_tctcs_device::ctl_w(u8 data)
{
	LOG("ctl_w 0x%02x (%s)\n", data, machine().describe_context());

	m_ctlw = data;
}
void isa16_tctcs_device::wdp_w(u8 data)
{
	LOG("wdp_w 0x%02x (%s)\n", data, machine().describe_context());

	m_watchdog->adjust(attotime::from_seconds(20));
}
void isa16_tctcs_device::wdc_w(u8 data)
{
	LOG("wdc_w 0x%02x (%s)\n", data, machine().describe_context());

	if (BIT(data, 0))
		m_watchdog->adjust(attotime::from_seconds(20));
	else
		m_watchdog->adjust(attotime::never);
}
void isa16_tctcs_device::watchdog(int param)
{
	LOG("watchdog event\n");
}

#if 0
void isa16_tctcs_device::update_isa_irq(int state)
{
	if (bool(state) != m_isa_irq_asserted)
	{
		LOG("update_isa_irq %d\n", state);

		switch (m_isa_irq)
		{
		case 3: m_isa->irq3_w(state); break;
		case 4: m_isa->irq4_w(state); break;
		case 5: m_isa->irq5_w(state); break;
		case 6: m_isa->irq6_w(state); break;
		case 7: m_isa->irq7_w(state); break;
		case 9: m_isa->irq2_w(state); break;
		case 10: m_isa->irq10_w(state); break;
		case 11: m_isa->irq11_w(state); break;
		case 12: m_isa->irq12_w(state); break;
		case 14: m_isa->irq14_w(state); break;
		case 15: m_isa->irq15_w(state); break;

		default:
			fatalerror("%s: invalid isa irq %d\n", tag(), m_isa_irq);
		}

		m_isa_irq_asserted = bool(state);
	}
}
#endif
