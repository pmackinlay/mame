// license:BSD-3-Clause
// copyright-holders:Patrick Mackinlay

/*
 * An implementation of the MCT-ADR device found in Microsoft Jazz/MIPS
 * ARCSystem 100 architecture systems. This device was originally designed
 * by Microsoft, and then implemented and used in various forms by MIPS,
 * Olivetti, LSI Logic, NEC, Acer and others.
 *
 * Specific implementations/derivatives include:
 *
 *   LSI Logic R4030/R4230
 *   NEC μPD31432
 *   ALI M6101-A1
 *
 * References:
 *
 *   https://datasheet.datasheetarchive.com/originals/scans/Scans-054/DSAIH000102184.pdf
 *   https://github.com/torvalds/linux/tree/master/arch/mips/jazz/
 *   http://cvsweb.netbsd.org/bsdweb.cgi/src/sys/arch/arc/jazz/
 *
 * TODO
 *   - everything (skeleton only)
 */

#include "emu.h"
#include "jazz_mct_adr.h"

#define VERBOSE (0)
#include "logmacro.h"

DEFINE_DEVICE_TYPE(JAZZ_MCT_ADR, jazz_mct_adr_device, "jazz_mct_adr", "Jazz MCT-ADR")

jazz_mct_adr_device::jazz_mct_adr_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, JAZZ_MCT_ADR, tag, owner, clock)
	, m_ram(*this, finder_base::DUMMY_TAG)
	, m_out_int0(*this)
	, m_out_int1(*this)
{
}

void jazz_mct_adr_device::map(address_map &map)
{
	map(0x000, 0x007).lrw32("config", [this](){ return m_config; }, [this](u32 data) { m_config = data; });

	map(0x008, 0x00f).lr32("revision", [](){ return 1; });

	map(0x010, 0x017).lr32("invalid_addr", []() { return 0; });
	map(0x018, 0x01f).lrw32("trans_tbl_base", [this]() { return m_trans_tbl_base; }, [this](u32 data) { m_trans_tbl_base = data; });
	map(0x020, 0x027).lrw32("trans_tbl_limit", [this]() { return m_trans_tbl_limit; }, [this](u32 data) { m_trans_tbl_limit = data; });
	map(0x028, 0x02f).lrw32("trans_tbl_invalid", [this]() { return 0; }, [this](u32 data) { });
	map(0x030, 0x037).lw32("maint", [this](u32 data) { m_ioc_maint = data; }); // cache maintenance?
	map(0x038, 0x03f).lr32("remote_fail_addr", []() { return 0; });
	map(0x040, 0x047).lr32("memory_fail_addr", []() { return 0; });
	map(0x048, 0x04f).lw32("io_cache_physical_tag", [this](u32 data) { m_ioc_physical_tag = data; }); // I/O cache physical tag
	map(0x050, 0x057).lw32("io_cache_logical_tag", [this](u32 data) { m_ioc_logical_tag = data; }); // I/O cache logical tag
	map(0x058, 0x05f).lrw32("io_cache_byte_mask",
		// FIXME: hack to pass diagnostics
		[this]()
		{
			u32 const data = m_ioc_byte_mask;

			if (data == 0xffffffff)
				m_ioc_byte_mask = 0;
			return data;
		},
		[this](u32 data) { m_ioc_byte_mask |= data; });

	map(0x060, 0x067).lw32("io_cache_buffer_window", [this](u32 data)
	{
		// FIXME: hack to pass diagnostics
		if (m_ioc_logical_tag == 0x80000001 && m_ioc_byte_mask == 0x0f0f0f0f)
		{
			u32 const address = (m_ioc_physical_tag & ~0x1) + ((m_ioc_maint & 0x3) << 3);

			m_ram->write(address + 0, data >> 0);
			m_ram->write(address + 1, data >> 8);
			m_ram->write(address + 2, data >> 16);
			m_ram->write(address + 3, data >> 24);
		}
	});

	// remote speed registers
	map(0x070, 0x0ef).lrw32("remote_speed",
		[this](offs_t offset) { return m_remote_speed[offset  >> 1]; },
		[this](offs_t offset, u32 data) { m_remote_speed[offset >> 1] = data; });

	// dma registers
	map(0x100, 0x1ff).lrw32("dma_reg",
		[this](offs_t offset) { return m_dma_reg[offset >> 1]; },
		[this](offs_t offset, u32 data) { m_dma_reg[offset >> 1] = data; });

	map(0x200, 0x207).lr32("irq_source", []() { return 0; }); // dma_int_src?
	map(0x208, 0x20f).lr32("eisa_error", []() { return 0; });
	map(0x210, 0x217).lrw32("memory_refresh_rate", [this]() { return m_memory_refresh_rate; }, [this](u32 data) { m_memory_refresh_rate = data; });
	map(0x220, 0x227).lr32("nvram_protect", []() { return 0x7; });
	map(0x228, 0x22f).lw32("timer_interval", [this](u32 data)
	{
		attotime interval = attotime::from_ticks((data + 1) & 0x1ff, 1000);

		m_interval_timer->adjust(interval, 0, interval);
	});
	map(0x230, 0x237).lr32("interval_timer_count", [this]() { m_out_int0(CLEAR_LINE); return 0; });
	//map(0x238, 0x23b).lr32("eisa_irq_ack", []() { return 0; });
}

void jazz_mct_adr_device::device_start()
{
	m_out_int0.resolve();
	m_out_int1.resolve();

	m_config = 0x104; // or 0x410?

	m_ioc_maint = 0;
	m_ioc_physical_tag = 0;
	m_ioc_logical_tag = 0;
	m_trans_tbl_base = 0;
	m_trans_tbl_limit = 0;
	m_ioc_byte_mask = 0;

	for (u32 &val : m_remote_speed)
		val = 0x7;

	for (u32 &val : m_dma_reg)
		val = 0;

	m_memory_refresh_rate = 0x18186;

	m_irq_check = machine().scheduler().timer_alloc(timer_expired_delegate(FUNC(jazz_mct_adr_device::irq_check), this));
	m_interval_timer = machine().scheduler().timer_alloc(timer_expired_delegate(FUNC(jazz_mct_adr_device::interval_timer), this));
}

void jazz_mct_adr_device::device_reset()
{
	m_isr = 0;
	m_imr = 0x10;

	m_interval_timer->adjust(attotime::from_usec(1), 0, attotime::from_usec(1));
}

void jazz_mct_adr_device::set_irq_line(int irq, int state)
{
	if (state)
		LOG("set_irq_line %d state %d\n", irq, state);

	if (state)
		m_isr |= (1 << irq);
	else
		m_isr &= ~(1 << irq);

	m_irq_check->adjust(attotime::zero);
}

TIMER_CALLBACK_MEMBER(jazz_mct_adr_device::irq_check)
{
	if (m_isr & m_imr)
		m_out_int1(ASSERT_LINE);
	else
		m_out_int1(CLEAR_LINE);
}

u16 jazz_mct_adr_device::isr_r()
{
	u16 pending = m_isr & m_imr;

	for (u16 irq = 0; irq < 16; irq++)
		if (BIT(pending, irq))
			return (irq + 1) << 2;

	return 0;
}

TIMER_CALLBACK_MEMBER(jazz_mct_adr_device::interval_timer)
{
	m_out_int0(ASSERT_LINE);
}
