// license:BSD-3-Clause
// copyright-holders:Patrick Mackinlay

/*
 * Data General AViiON M88k systems.
 *
 * Sources:
 *  - https://archive.org/details/Aviion530Docs/40020761
 *
 * TODO:
 *  - everything
 */

#include "emu.h"

// processors and memory
#include "cpu/m88000/m88000.h"
#include "machine/mc88200.h"
#include "machine/28fxxx.h"

// i/o devices
#include "machine/timekpr.h"
#include "machine/mc68681.h"
#include "machine/53c7xx.h"
#include "machine/scn_pci.h"
#include "machine/scnxx562.h"
#include "machine/input_merger.h"

// busses and connectors
#include "machine/nscsi_bus.h"
#include "bus/nscsi/cd.h"
#include "bus/nscsi/hd.h"
#include "bus/rs232/rs232.h"
#include "bus/pc_kbd/pc_kbdc.h"
#include "bus/pc_kbd/keyboards.h"

#include "sound/spkrdev.h"
#include "speaker.h"

#include "debugger.h"

#define LOG_GENERAL (1U << 0)
#define LOG_ECC     (1U << 1)

#define VERBOSE (LOG_GENERAL|LOG_ECC)
#include "logmacro.h"

namespace {

class aviion88k_state : public driver_device
{
public:
	aviion88k_state(machine_config const &mconfig, device_type type, char const *tag)
		: driver_device(mconfig, type, tag)
		, m_cpu(*this, "cpu")
		, m_cmmu(*this, "cmmu%u", 0U)
		, m_prom(*this, "prom%u", 0U)
		, m_novram(*this, "novram")
		, m_uart(*this, "uart")
		, m_kbdc(*this, "kbdc")
		, m_kbdc_txc(*this, "kbdc_txc")
		, m_kbdc_dsc(*this, "kbdc_dsc")
		, m_duart(*this, "duart%u", 0U)
		, m_async(*this, { "console_port", "seriala", "mouse_port", "serialb" })
		, m_duscc(*this, "duscc")
		, m_scsibus(*this, "scsi")
		, m_scsi(*this, "scsi:7:ncr53c700")
		, m_speaker(*this, "speaker")
		, m_leds(*this, "CR%u", 1U)
		, m_mbus(*this, "mbus")
	{
	}

	// machine config
	void aviion_4600(machine_config &config);

	void init();

protected:
	// driver_device overrides
	virtual void machine_start() override;
	virtual void machine_reset() override;

	// address maps
	void cpu_map(address_map &map);
	void lsio_map(address_map &map) {}

	template <u32 IST> void interrupt(int state);
	template <u32 EXIST> void interrupt_ex(int state);
	void interrupt_check();

	void pit_timer(int param)
	{
		LOG("pit_timer<%d> expired\n", param);
		m_pit_cmd[param] &= ~8;
		switch (param)
		{
		case 0: interrupt_ex<EXIST_PIT0OF>(1); break;
		case 1: interrupt_ex<EXIST_PIT1OF>(1); break;
		case 2: interrupt_ex<EXIST_PIT2OF>(1); break;
		case 3: interrupt_ex<EXIST_PIT3OF>(1); break;
		}
	}
	void spken_timer(int param) { LOG("spken_timer expired\n", param); }
	void rtc_timer(int param)
	{
		LOG("rtc_timer expired\n");
		interrupt_ex<EXIST_RTCOF>(1);
	}

	template <unsigned N> u32 ien_r() { return m_ien[N]; }
	template <unsigned N> void ien_w(u32 data) { logerror("ien %d 0x%08x\n", N, data); m_ien[N] = data; interrupt_check(); }
	void ien_all_w(u32 data)
	{
		m_ien[0] = data;
		m_ien[1] = data;
		m_ien[2] = data;
		m_ien[3] = data;

		interrupt_check();
	}
	template <unsigned N> u32 exien_r() { return m_exien[N]; }
	template <unsigned N> void exien_w(u32 data) { m_exien[N] = data; interrupt_check(); }
	void exien_all_w(u32 data)
	{
		m_exien[0] = data;
		m_exien[1] = data;
		m_exien[2] = data;
		m_exien[3] = data;

		interrupt_check();
	}

	template <unsigned N> u32 pit_cnt_r()
	{
		u32 const data = m_pit_cnt[N];

		if (BIT(m_pit_cmd[N], 2))
		{
			if (m_pit_cnt[N] == 0xffffff00)
				m_pit_cnt[N] = 0;
			else
				m_pit_cnt[N] += 0x11111100;

			return data;
		}
		else
			return m_pit[N]->enabled() ? m_pit[N]->elapsed().as_ticks(m_cpu->clock()) : data;
	}
	template <unsigned N> u32 pit_sts_r() { return m_pit_cmd[N]; }
	template <unsigned N> void pit_cnt_w(u32 data)
	{
		LOG("pit_cnt_w<%d> 0x%08x\n", N, data);

		m_pit_cnt[N] = data & 0xffffff00U;

		if (BIT(m_pit_cmd[N], 3))
			m_pit[N]->adjust(attotime::from_ticks(-data, m_cpu->clock()), N);
	}
	template <unsigned N> void pit_cmd_w(u32 data)
	{
		LOG("pit_cmd_w<%d> 0x%x\n", N, data & 15);
		m_pit_cmd[N] = data & 15;

		// reset
		if (BIT(data, 0))
			m_pit[N]->adjust(attotime::from_ticks(-m_pit_cnt[N], m_cpu->clock()), N);

		// interrupt acknowledge
		if (BIT(data, 1))
			interrupt_ex<EXIST_PIT0OF << N>(0);

		if (BIT(data, 2))
		{
			m_pit[N]->enable(false);
			m_pit_cnt[N] = 0;
		}
		else if (BIT(data, 3) && !m_pit[N]->enabled())
			// count enable
			m_pit[N]->enable(true);
		else if (!BIT(data, 3) && m_pit[N]->enabled())
			m_pit[N]->enable(false);
	}

	u32 ram_r(offs_t offset, u32 mem_mask);
	void ram_w(offs_t offset, u32 data, u32 mem_mask);
	u8 compute_ecc(u32 const data) const;
	unsigned check_ecc(u32 &data, u8 const ecc, bool const correct) const;

	u32 le_ram_r(offs_t offset, u32 mem_mask) { return swapendian_int32(ram_r(offset, mem_mask)); }
	void le_ram_w(offs_t offset, u32 data, u32 mem_mask) { ram_w(offset, swapendian_int32(data), swapendian_int32(mem_mask)); }

	u8 vme_a16_r(offs_t offset);
	void vme_a16_w(offs_t offset, u8 data);

private:
	// processors and memory
	required_device<mc88100_device> m_cpu;
	required_device_array<mc88200_device, 2> m_cmmu;
	required_device_array<intel_28f010_device, 4> m_prom;

	// i/o devices
	required_device<timekeeper_device> m_novram;
	required_device<scn_pci_device> m_uart;
	required_device<pc_kbdc_device> m_kbdc;
	required_device<input_merger_any_high_device> m_kbdc_txc;
	required_device<input_merger_any_low_device> m_kbdc_dsc;
	required_device_array<scn2681_device, 2> m_duart;
	required_device_array<rs232_port_device, 4> m_async;
	required_device<duscc68562_device> m_duscc;
	required_device<nscsi_bus_device> m_scsibus;
	required_device<ncr53c7xx_device> m_scsi;
	required_device<speaker_sound_device> m_speaker;

	output_finder<3> m_leds;

	memory_view m_mbus;

	enum ucs_mask : u16
	{
		UCS_WDA  = 0x0003, // watchdog action
		UCS_EWD  = 0x0004, // enable watchdog timer
		UCS_VTS  = 0x0018, // VMEbus data transfer timeout select
		UCS_ETO  = 0x0020, // enable VMEbus arbitration timeout
		UCS_VRM  = 0x0040, // VMEbus release mode
		UCS_FAIR = 0x0080, // fairness arbitration
		UCS_RNV  = 0x0100, // release never
		UCS_VRL  = 0x0600, // VMEbus request/grant level
		UCS_VAM  = 0x0800, // VMEbus arbitration mode
		UCS_BIR  = 0x1000, // broadcast interrupt request
		UCS_ASF  = 0x2000, // assert system failure (active low)
		UCS_PUP  = 0x4000, // power-up initialization indicator (active low)
	};
	u16 m_ucs;
	u16 m_psc;

	// interrupts
	enum ist_mask : u32
	{
		IST_SI0  = 0x00000001, // software interrupt 0
		IST_SI1  = 0x00000002, // software interrupt 1
		IST_SI2  = 0x00000004, // software interrupt 2
		IST_SI3  = 0x00000008, // software interrupt 3
		IST_VME1 = 0x00000010, // VME level 1 interrupt
		IST_VME2 = 0x00000040, // VME level 2 interrupt
		IST_SLP  = 0x00000080, // signal low priority interrupt
		IST_LM   = 0x00000100, // location monitor interrupt
		IST_VME3 = 0x00000400, // VME level 3 interrupt
		IST_VME4 = 0x00001000, // VME level 4 interrupt
		IST_VME5 = 0x00004000, // VME level 5 interrupt
		IST_SHP  = 0x00010000, // signal high priority interrupt
		IST_DI   = 0x00020000, // DUART interrupt
		IST_MEM  = 0x00040000, // memory error interrupt
		IST_VME6 = 0x00080000, // VME level 6 interrupt
		IST_SF   = 0x00100000, // system failure interrupt
		IST_KBD  = 0x00400000, // keyboard interrupt
		IST_VME7 = 0x00800000, // VME level 7 interrupt
		IST_SI4  = 0x01000000, // software interrupt 4
		IST_SI5  = 0x02000000, // software interrupt 5
		IST_SI6  = 0x04000000, // software interrupt 6
		IST_SI7  = 0x08000000, // software interrupt 7
		IST_DTI  = 0x10000000, // DUART timer interrupt
		IST_ATO  = 0x20000000, // VMEbus arbiter time out
		IST_ACF  = 0x40000000, // AC failure
		IST_ABT  = 0x80000000, // abort pushbutton
	};
	u32 m_ist;      // interrupt status
	u32 m_ien[4];   // interrupt enable
	enum exist_mask : u32
	{
		EXIST_PDMA   = 0x00000020, // parallel printer DMA
		EXIST_IOEXP2 = 0x00000040, // I/O expansion 2
		EXIST_IOEXP1 = 0x00000100, // I/O expansion 1
		EXIST_VDMA   = 0x00001000, // video DMA
		EXIST_DUART2 = 0x00002000, // DUART 2
		EXIST_ZBUF   = 0x00004000, // Z-buffer
		EXIST_VIDEO  = 0x00008000, // video
		EXIST_SCSI1  = 0x00010000, // SCSI 1
		EXIST_SCSI0  = 0x00020000, // SCSI 0
		EXIST_LAN1   = 0x00040000, // ethernet 1
		EXIST_LAN0   = 0x00080000, // ethernet 0
		EXIST_SCC    = 0x00100000, // synchronous controller
		EXIST_DMA0C  = 0x00200000, // DMA channel 0 complete
		EXIST_DMA1C  = 0x00400000, // DMA channel 1 complete
		EXIST_DMA2C  = 0x00800000, // DMA channel 2 complete
		EXIST_DMA3C  = 0x01000000, // DMA channel 3 complete
		EXIST_DMA4C  = 0x02000000, // DMA channel 4 complete
		EXIST_PIT0OF = 0x08000000, // PIT 0 overflow
		EXIST_PIT1OF = 0x10000000, // PIT 1 overflow
		EXIST_PIT2OF = 0x20000000, // PIT 2 overflow
		EXIST_PIT3OF = 0x40000000, // PIT 3 overflow
		EXIST_RTCOF  = 0x80000000, // real time clock overflow
	};
	u32 m_exist;    // extended interrupt status
	u32 m_exien[4]; // extended interrupt enable
	bool m_int_state;

	emu_timer *m_pit[4]{};

	u32 m_pit_cmd[4]{};
	u32 m_pit_cnt[4] = {};

	emu_timer *m_spken;

	enum mdr_mask : u16
	{
		MDR_MSS   = 0x0001, // monitor speed select (0=60Hz, 1=70Hz)
		MDR_ECE   = 0x0002, // ECC correction enable
		MDR_APEX2 = 0x0004, // manufacturing jumper (APEX_TEST2)?
		MDR_EMS   = 0x0018, // ECC mode select
		MDR_FDMA0 = 0x0020, // flush DMA channel 0
		MDR_FDMA2 = 0x0040, // flush DMA channel 2
		MDR_EWE   = 0x0080, // EPROM write enable
		MDR_MEA   = 0x0100, // multiple-bit error bank A
		MDR_MEB   = 0x0200, // multiple-bit error bank B
		MDR_SEA   = 0x0400, // single-bit error bank A
		MDR_SEB   = 0x0800, // single-bit error bank B
		MDR_MS    = 0xe000, // memory SIMM modules

		MDR_RSV   = 0x1064, // reserved bits
	};
	u16 m_mdr;
	enum dle_mask : u16
	{
		DLE_CBE = 0x007f, // diagnostic latch enable
		DLE_DLE = 0x8000, // check bits enable
	};
	u16 m_dle;
	u16 m_ecb;
	u16 m_eeal; // ecc error address lower
	u16 m_eeau; // ecc error address upper
	std::unique_ptr<u32[]> m_ram;
	std::unique_ptr<u8[]> m_ecc;

	unsigned const m_ram_size = 0x800000;

	u8 m_brdid;
	u8 m_gpcs[5];

	u16 m_basad;
	u8 m_ccs;
	u8 m_extad;
	u8 m_extam;
	u32 m_rvad;
	u8 m_vad[2048];
	u8 m_virl;
	u8 m_viv;
	u8 m_global[2];

	emu_timer *m_rtc;
	enum rtc_ctl_mask : u32
	{
		RTC_RESET  = 0x01,
		RTC_INTACK = 0x02,
		RTC_TEST   = 0x04,
	};
	u32 m_rtc_cnt;
	u32 m_rtc_ctl;
};

void aviion88k_state::machine_start()
{
	m_leds.resolve();

	for (emu_timer *&pit : m_pit)
		pit = timer_alloc(FUNC(aviion88k_state::pit_timer), this);

	m_spken = timer_alloc(FUNC(aviion88k_state::spken_timer), this);
	m_rtc = timer_alloc(FUNC(aviion88k_state::rtc_timer), this);

	m_ist = 0;
	m_exist = 0;
	m_int_state = false;

	m_mdr = MDR_MS | MDR_RSV | MDR_EMS | MDR_ECE;

	m_dle = 0;
	m_ecb = 0;
	m_eeal = 0;
	m_eeau = 0;

	// TODO: assume 8M RAM
	m_ram = std::make_unique<u32[]>(m_ram_size / 4);
	m_ecc = std::make_unique<u8[]>(m_ram_size / 4);

	m_basad = 0; // FIXME: dip switches
	m_global[0] = 0x1f;
	m_global[1] = 0x40;
}

void aviion88k_state::machine_reset()
{
	// disable mbus address decode
	m_mbus.select(0);

	m_ucs = UCS_VRL | UCS_VTS | UCS_WDA;

	// disable keyboard transmit clock
	m_uart->dcd_w(0);
	m_uart->dsr_w(1);
	m_kbdc_dsc->in_w<1>(0);
	m_kbdc_txc->in_w<1>(1);

	for (u32 &ien : m_ien)
		ien = 0;
	for (u32 &exien : m_exien)
		exien = 0;

	m_virl = 0;

	//m_mdr &= ~(MDR_EMS|MDR_ECE|MDR_MSS);
	m_gpcs[0] = 0xff;
	m_gpcs[1] = 0;
	m_gpcs[2] = 0;
	m_gpcs[3] = 0;
	m_gpcs[4] = 0;

	m_rtc_cnt = 0;
	m_rtc->adjust(attotime::from_ticks(0x1'0000'0000, m_cpu->clock()), 0, attotime::from_ticks(0x1'0000'0000, m_cpu->clock()));

	interrupt_check();
}

void aviion88k_state::init()
{
	// map the configured ram
	//m_mbus[1].install_ram(0x00000000, m_ram->mask(), m_ram->pointer());
	m_mbus[1].install_readwrite_handler(0, m_ram_size - 1,
		read32s_delegate(*this, FUNC(aviion88k_state::ram_r)),
		write32s_delegate(*this, FUNC(aviion88k_state::ram_w)));

	m_scsi->space(0).install_readwrite_handler(0, m_ram_size - 1,
		read32s_delegate(*this, FUNC(aviion88k_state::le_ram_r)),
		write32s_delegate(*this, FUNC(aviion88k_state::le_ram_w)));
}

// cmmu base = 0xfffii000, where ii = IDR
//             0xfff7x000 (at power up)
//             0xfff0x000 (normal)

// system writes 00 to 0xfff78000  x=8 (6 cmmu, cpu1 data cmmu0)
//               01 to 0xfff79000  x=9 (6 cmmu, cpu1 data cmmu1)
void aviion88k_state::cpu_map(address_map &map)
{
	map(0x00000000, 0xffc7ffff).view(m_mbus);

	/*
	 * utility space (4M) 0xffc0'0000-ffff'ffff
	 *
	 * madv=0: only utility space available, mapped to 0x0000'0000-ffc7'ffff
	 * madv=1: utility space 0xffc7'0000-ffc7'ffff
	 */
	// mbus address decode disabled
	m_mbus[0](0x00000000, 0x0007ffff).rw(m_prom[0], FUNC(intel_28f010_device::read), FUNC(intel_28f010_device::write)).mirror(0xffc00000).umask32(0xff000000);
	m_mbus[0](0x00000000, 0x0007ffff).rw(m_prom[1], FUNC(intel_28f010_device::read), FUNC(intel_28f010_device::write)).mirror(0xffc00000).umask32(0x00ff0000);
	m_mbus[0](0x00000000, 0x0007ffff).rw(m_prom[2], FUNC(intel_28f010_device::read), FUNC(intel_28f010_device::write)).mirror(0xffc00000).umask32(0x0000ff00);
	m_mbus[0](0x00000000, 0x0007ffff).rw(m_prom[3], FUNC(intel_28f010_device::read), FUNC(intel_28f010_device::write)).mirror(0xffc00000).umask32(0x000000ff);

	// mbus address decode enabled
	m_mbus[1](0xffc00000, 0xffc7ffff).rw(m_prom[0], FUNC(intel_28f010_device::read), FUNC(intel_28f010_device::write)).umask32(0xff000000);
	m_mbus[1](0xffc00000, 0xffc7ffff).rw(m_prom[1], FUNC(intel_28f010_device::read), FUNC(intel_28f010_device::write)).umask32(0x00ff0000);
	m_mbus[1](0xffc00000, 0xffc7ffff).rw(m_prom[2], FUNC(intel_28f010_device::read), FUNC(intel_28f010_device::write)).umask32(0x0000ff00);
	m_mbus[1](0xffc00000, 0xffc7ffff).rw(m_prom[3], FUNC(intel_28f010_device::read), FUNC(intel_28f010_device::write)).umask32(0x000000ff);

	map(0xfff8'0000, 0xfff8'1fff).rw(m_novram, FUNC(mk48t12_device::read), FUNC(mk48t12_device::write)).umask32(0x000000ff);
	map(0xfff8'2000, 0xfff8'203f).rw(m_duart[0], FUNC(scn2681_device::read), FUNC(scn2681_device::write)).umask32(0x000000ff);
	map(0xfff8'2040, 0xfff8'207f).rw(m_duart[1], FUNC(scn2681_device::read), FUNC(scn2681_device::write)).umask32(0x000000ff);
	map(0xfff8'2800, 0xfff8'280f).rw(m_uart, FUNC(scn2661a_device::read), FUNC(scn2661a_device::write)).umask32(0x000000ff);
	map(0xfff8'2810, 0xfff8'2813).lw32([this](u32 data) { LOG("dsc_w %d\n", BIT(data, 0)); m_kbdc_dsc->in_w<1>(BIT(data, 0)); }, "dsc_w");
	map(0xfff8'2820, 0xfff8'2823).lw32([this](u32 data) { LOG("etxc_w %d\n", BIT(data, 0)); m_uart->dsr_w(BIT(data, 0)); m_kbdc_txc->in_w<1>(BIT(data, 0)); }, "etxc_w");

	map(0xfff8'3100, 0xfff8'3103).lw32(
		[this](u32 data)
		{
			if (!BIT(data, 0))
				m_duart[0]->reset();
			if (!BIT(data, 1))
				m_duart[1]->reset();

			// reset the keyboard or the uart?
			if (!BIT(data, 3))
			{
				LOG("uart reset\n");
				//m_uart->reset();

				// disable keyboard transmit clock
				m_uart->dsr_w(1);
				m_kbdc_txc->in_w<1>(1);
			}
		}, "srst_w");
	map(0xfff8'3104, 0xfff8'3107).lw32([this](u32 data) { m_spken->adjust(attotime::from_msec(200)); }, "spken");

	map(0xfff8'4004, 0xfff8'4007).rw(FUNC(aviion88k_state::ien_r<0>), FUNC(aviion88k_state::ien_w<0>));
	map(0xfff8'4008, 0xfff8'400b).rw(FUNC(aviion88k_state::ien_r<1>), FUNC(aviion88k_state::ien_w<1>));
	map(0xfff8'4010, 0xfff8'4013).rw(FUNC(aviion88k_state::ien_r<2>), FUNC(aviion88k_state::ien_w<2>));
	map(0xfff8'4020, 0xfff8'4023).rw(FUNC(aviion88k_state::ien_r<3>), FUNC(aviion88k_state::ien_w<3>));
	map(0xfff8'403c, 0xfff8'403f).w(FUNC(aviion88k_state::ien_all_w));
	map(0xfff8'4040, 0xfff8'4043).lr32([this]() { return m_ist; }, "ist_r");
	map(0xfff8'4080, 0xfff8'4083).lw32(
		[this](u32 data)
		{
			if (BIT(data, 0))
				m_ist |= IST_SI0;
			if (BIT(data, 1))
				m_ist |= IST_SI1;
			if (BIT(data, 2))
				m_ist |= IST_SI2;
			if (BIT(data, 3))
				m_ist |= IST_SI3;
			if (BIT(data, 4))
				m_ist |= IST_SI4;
			if (BIT(data, 5))
				m_ist |= IST_SI5;
			if (BIT(data, 6))
				m_ist |= IST_SI6;
			if (BIT(data, 7))
				m_ist |= IST_SI7;

			interrupt_check();
		}, "setswi_w");
	map(0xfff8'4084, 0xfff8'4087).lw32(
		[this](u32 data)
		{
			if (BIT(data, 0))
				m_ist &= ~IST_SI0;
			if (BIT(data, 1))
				m_ist &= ~IST_SI1;
			if (BIT(data, 2))
				m_ist &= ~IST_SI2;
			if (BIT(data, 3))
				m_ist &= ~IST_SI3;
			if (BIT(data, 4))
				m_ist &= ~IST_SI4;
			if (BIT(data, 5))
				m_ist &= ~IST_SI5;
			if (BIT(data, 6))
				m_ist &= ~IST_SI6;
			if (BIT(data, 7))
				m_ist &= ~IST_SI7;

			interrupt_check();
		}, "clrswi_w");
	map(0xfff8'4088, 0xfff8'408b).lr32(
		[this]()
		{
			u32 data = 0;

			if (m_ist & IST_SF)
				data |= 1;
			if (m_ist & IST_ACF)
				data |= 2;
			if (m_ist & IST_ABT)
				data |= 4;

			return data;
		}, "istate_r");
	map(0xfff8'408c, 0xfff8'408f).lw32(
		[this](u32 data)
		{
			if (BIT(data, 0))
				m_ist &= ~IST_SF;
			if (BIT(data, 1))
				m_ist &= ~IST_ACF;
			if (BIT(data, 2))
				m_ist &= ~IST_ABT;

			interrupt_check();
		}, "clrint_w");

	map(0xfff8'5000, 0xfff8'5003).lrw32([this]() { return m_virl; }, "virl_r",
		[this](u32 data)
		{
			m_ist &= ~(IST_VME7 | IST_VME6 | IST_VME5 | IST_VME4 | IST_VME3 | IST_VME2 | IST_VME1);

			logerror("virl %d\n", data & 7);

			switch (data & 7)
			{
			case 1: m_ist |= IST_VME1; break;
			case 2: m_ist |= IST_VME2; break;
			case 3: m_ist |= IST_VME3; break;
			case 4: m_ist |= IST_VME4; break;
			case 5: m_ist |= IST_VME5; break;
			case 6: m_ist |= IST_VME6; break;
			case 7: m_ist |= IST_VME7; break;
			}

			m_virl = data;
			interrupt_check();
		}, "virl_w");
	map(0xfff8'5004, 0xfff8'501f).lr32(
		[this](offs_t offset)
		{
			logerror("viav %d\n", offset + 1);
			u8 const data = m_viv;

			switch (offset)
			{
			case 0: m_ist &= ~IST_VME1; break;
			case 1: m_ist &= ~IST_VME2; break;
			case 2: m_ist &= ~IST_VME3; break;
			case 3: m_ist &= ~IST_VME4; break;
			case 4: m_ist &= ~IST_VME5; break;
			case 5: m_ist &= ~IST_VME6; break;
			case 6: m_ist &= ~IST_VME7; break;
			}

			m_virl = 0;

			interrupt_check();
			return data;
		}, "viav_r");
	map(0xfff8'5020, 0xfff8'5023).lrw32([this]() { return m_viv; }, "viv_r", [this](u32 data) { m_viv = data; }, "viv_w");

	map(0xfff8'6001, 0xfff8'6001).lrw8([this]() { return m_global[0]; }, "global0_r",
		[this](u8 data)
		{
			m_global[0] = (data & 0xf0) | (m_global[0] & 0x0f);

			if (data & 0xf0)
				interrupt<IST_LM>(0);
		}, "global0_w");
	map(0xfff8'6003, 0xfff8'6003).lrw8([this]() { return m_global[1]; }, "global1_r",
		[this](u8 data)
		{
			m_global[1] = ((m_global[1] & 0x5f) | (data & 0xa0)) & (data | 0xfc);

			if (!BIT(data, 0))
				m_ist &= ~IST_SLP;
			if (!BIT(data, 1))
				m_ist &= ~IST_SHP;

			m_global[1] = (m_global[1] & 0x4c) | (data & ~0x4c);

			interrupt_check();
		}, "global1_w");
	map(0xfff8'6005, 0xfff8'6005).lrw8([this]() { return m_brdid; }, "brdid_r", [this](u8 data) { m_brdid = data; }, "brdid_w");
	map(0xfff8'6007, 0xfff8'6007).lrw8([this]() { return m_gpcs[0]; }, "gpcs0_r", [this](u8 data) { m_gpcs[0] = data; }, "gpcs0_w");
	map(0xfff8'6009, 0xfff8'6009).lrw8([this]() { return m_gpcs[1]; }, "gpcs1_r", [this](u8 data) { m_gpcs[1] = data; }, "gpcs1_w");
	map(0xfff8'600b, 0xfff8'600b).lrw8([this]() { return m_gpcs[2]; }, "gpcs2_r", [this](u8 data) { m_gpcs[2] = data; }, "gpcs2_w");
	map(0xfff8'600d, 0xfff8'600d).lrw8([this]() { return m_gpcs[3]; }, "gpcs3_r", [this](u8 data) { m_gpcs[3] = data; }, "gpcs3_w");
	map(0xfff8'600f, 0xfff8'600f).lrw8([this]() { return m_gpcs[4]; }, "gpcs4_r", [this](u8 data) { m_gpcs[4] = data; }, "gpcs4_w");

	map(0xfff8'7000, 0xfff8'7003).lrw32(
		[this]() { return m_ucs; }, "ucs_r",
		[this](u32 data) { m_ucs = (m_ucs & UCS_PUP) | (data & 0xffff);	}, "ucs_w");
	map(0xfff8'7004, 0xfff8'7007).lr32([this]() { return m_basad; }, "basad_r");

	map(0xfff8'8000, 0xfff8'8003).lrw32(
		[this]() { return m_ccs; }, "ccs_r",
		[this](u32 data)
		{
			LOG("madv %d vadv %d\n", BIT(data, 1), BIT(data, 0));
			if (BIT(data, 1))
				m_mbus.select(BIT(data, 1));

			m_ccs = data;
		}, "ccs_w");
	map(0xfff8'8010, 0xfff8'8013).lrw32(
		[this]() { return m_extad; }, "extad_r",
		[this](u32 data) { m_extad = data; }, "extad_w");
	map(0xfff8'8014, 0xfff8'8017).lrw32(
		[this]() { return m_extam; }, "extam_r",
		[this](u32 data) { m_extam = data; }, "extam_w");
	map(0xfff8'8018, 0xfff8'801b).lr32([this]() { LOG("whoami (%s)\n", machine().describe_context()); return 0xa1; }, "whoami_r");
	map(0xfff8'8028, 0xfff8'802b).lw32(
		[this](u32 data)
		{
			LOG("wvad vms=%d vpn=0x%04x vse=%d mbs=%d\n", BIT(data, 21), BIT(data, 22, 10), BIT(data, 1), BIT(data, 0));

			m_vad[BIT(data, 21, 11)] = BIT(data, 0, 2);
		}, "wvad_w");
	map(0xfff8'802c, 0xfff8'802f).lrw32(
		[this]()
		{
			u8 data = m_vad[BIT(m_rvad, 21, 11)];
			LOG("rvad vms=%d vpn=0x%04x\n", BIT(m_rvad, 21), BIT(m_rvad, 22, 10), BIT(data, 1), BIT(data, 0));

			return data;
		}, "rvad_r",
		[this](u32 data) { m_rvad = data; }, "rvad_w");

	map(0xfff8'df04, 0xfff8'df07).lrw16(
		[this]() { return m_psc; }, "psc_r",
		[this](u16 data) { LOG("psc_w 0x%x\n", data); m_psc = (m_psc & 0x3f) | (data & 0x3c0); }, "psc_w").umask32(0xffff);

	map(0xfff8'e040, 0xfff8'e043).lr32([this]() { return m_exist; }, "exist_r");
	map(0xfff8'e004, 0xfff8'e007).rw(FUNC(aviion88k_state::exien_r<0>), FUNC(aviion88k_state::exien_w<0>));
	map(0xfff8'e008, 0xfff8'e00b).rw(FUNC(aviion88k_state::exien_r<1>), FUNC(aviion88k_state::exien_w<1>));
	map(0xfff8'e010, 0xfff8'e013).rw(FUNC(aviion88k_state::exien_r<2>), FUNC(aviion88k_state::exien_w<2>));
	map(0xfff8'e020, 0xfff8'e023).rw(FUNC(aviion88k_state::exien_r<3>), FUNC(aviion88k_state::exien_w<3>));
	map(0xfff8'e03c, 0xfff8'e03f).w(FUNC(aviion88k_state::exien_all_w));

	map(0xfff8'f004, 0xfff8'f007).rw(FUNC(aviion88k_state::pit_cnt_r<0>), FUNC(aviion88k_state::pit_cnt_w<0>));
	map(0xfff8'f008, 0xfff8'f00b).rw(FUNC(aviion88k_state::pit_cnt_r<1>), FUNC(aviion88k_state::pit_cnt_w<1>));
	map(0xfff8'f010, 0xfff8'f013).rw(FUNC(aviion88k_state::pit_cnt_r<2>), FUNC(aviion88k_state::pit_cnt_w<2>));
	map(0xfff8'f020, 0xfff8'f023).rw(FUNC(aviion88k_state::pit_cnt_r<3>), FUNC(aviion88k_state::pit_cnt_w<3>));
	map(0xfff8'f044, 0xfff8'f047).rw(FUNC(aviion88k_state::pit_sts_r<0>), FUNC(aviion88k_state::pit_cmd_w<0>));
	map(0xfff8'f048, 0xfff8'f04b).rw(FUNC(aviion88k_state::pit_sts_r<1>), FUNC(aviion88k_state::pit_cmd_w<1>));
	map(0xfff8'f050, 0xfff8'f053).rw(FUNC(aviion88k_state::pit_sts_r<2>), FUNC(aviion88k_state::pit_cmd_w<2>));
	map(0xfff8'f060, 0xfff8'f063).rw(FUNC(aviion88k_state::pit_sts_r<3>), FUNC(aviion88k_state::pit_cmd_w<3>));

	map(0xfff8'f084, 0xfff8'f087).lrw32(
		[this]()
		{
			if (m_rtc_ctl & RTC_TEST)
			{
				u32 const data = m_rtc_cnt;

				if (m_rtc_cnt == 0xffffffffU)
					m_rtc_cnt = 0;
				else
					m_rtc_cnt += 0x11111111;

				return data;
			}
			else
				return -u32(m_rtc->remaining().as_ticks(m_cpu->clock()));
		}, "rtc_cnt_r",
		[this](u32 data)
		{
			m_rtc_cnt = data;

			m_rtc->adjust(attotime::from_ticks(-m_rtc_cnt, m_cpu->clock()), 0, attotime::from_ticks(0x1'0000'0000, m_cpu->clock()));
		}, "rtc_cnt_w");
	map(0xfff8'f088, 0xfff8'f08b).lrw32(
		[this]() { return m_rtc_ctl; }, "rtc_ctl_r",
		[this](u32 data)
		{
			if (data & RTC_TEST)
			{
				m_rtc->enable(false);
				m_rtc_cnt = 0;
			}
			else if (!m_rtc->enabled())
				m_rtc->adjust(attotime::from_ticks(-m_rtc_cnt, m_cpu->clock()), 0, attotime::from_ticks(0x1'0000'0000, m_cpu->clock()));

			if (data & RTC_INTACK)
				interrupt_ex<EXIST_RTCOF>(0);

			if (data & RTC_RESET)
				m_rtc_cnt = 0;

			m_rtc_ctl = data;
		}, "rtc_ctl_w");

	map(0xfff8'ff00, 0xfff8'ff03).lrw32(
		[this]() { return m_mdr; }, "mds_r",
		[this](u32 data) { logerror("mdc_w 0x%02x (%s)\n", data, machine().describe_context()); m_mdr = (m_mdr & ~0x9b) | u8(data); }, "mdc_w");
	map(0xfff8'ff04, 0xfff8'ff07).lrw32(
		[this]() { return m_ecb | 0x8080; }, "ecb_r",
		[this](u32 data) { logerror("dle_w 0x%04x (%s)\n", data, machine().describe_context()); m_dle = data; }, "dle_w");
	map(0xfff8'ff08, 0xfff8'ff0b).lr32([this]() { return m_eeal; }, "eeal_r");
	map(0xfff8'ff0c, 0xfff8'ff0f).lr32(
		[this]()
		{
			if (m_ecb)
			{
				m_mdr &= ~(MDR_SEB | MDR_SEA | MDR_MEB | MDR_MEA);
				m_ecb = 0;

				interrupt<IST_MEM>(0);
			}

			return m_eeau;
		}, "eeal_r");

	//map(0xfff8'ff10, 0xfff8'ff10)
	map(0xfffb'0000, 0xfffb'003f).rw(m_scsi, FUNC(ncr53c7xx_device::read), FUNC(ncr53c7xx_device::write));
	map(0xfffb'0040, 0xfffb'0040).lr8([]() { return 0x3; }, "iofuse0");

	map(0xfffb'0080, 0xfffb'00bf).noprw(); // scsi 2

	map(0xfffb'0104, 0xfffb'0107).ram();
	map(0xfffb'0110, 0xfffb'012f).rom().region("lanid", 0);

	map(0xfffb'0140, 0xfffb'016f).noprw(); // lan 2

	map(0xffff'0000, 0xffff'ffff).rw(FUNC(aviion88k_state::vme_a16_r), FUNC(aviion88k_state::vme_a16_w));
}

template <u32 IST> void aviion88k_state::interrupt(int state)
{
	if (state)
		m_ist |= IST;
	else
		m_ist &= ~IST;

	interrupt_check();
}

template <u32 EXIST> void aviion88k_state::interrupt_ex(int state)
{
	if (state)
		m_exist |= EXIST;
	else
		m_exist &= ~EXIST;

	interrupt_check();
}

void aviion88k_state::interrupt_check()
{
	bool int_state = (m_ist & m_ien[0]) || (m_exist & m_exien[0]);

	if (int_state != m_int_state)
	{
		logerror("interrupt %d\n", int_state);
		m_int_state = int_state;
		m_cpu->set_input_line(INPUT_LINE_IRQ0, m_int_state);
	}
}

u32 aviion88k_state::ram_r(offs_t offset, u32 mem_mask)
{
	if (offset < m_ram_size / 4)
	{
		u32 data = m_ram[offset];

		if (!machine().side_effects_disabled())
		{
			unsigned errors = 0;
			u8 ecc = 0;

			switch (m_mdr & MDR_EMS)
			{
			case 0x00:
			case 0x08:
				// normal ECC function
				ecc = m_ecc[offset];
				errors = check_ecc(data, ecc, m_mdr & MDR_ECE);
				break;
			case 0x10:
				// diagnostic detect
				ecc = 0x80 | (m_dle & DLE_CBE);
				errors = check_ecc(data, ecc, m_mdr & MDR_ECE);
				break;
			case 0x18:
				// pass-through
				ecc = m_ecc[offset];
				break;
			}

			switch (errors)
			{
			case 1:
				// correctable error
				if (!m_ecb)
				{
					if (offset & 1)
					{
						m_ecb = u16(ecc) << 8;
						m_mdr |= MDR_SEB;
					}
					else
					{
						m_ecb = ecc;
						m_mdr |= MDR_SEA;
					}
					m_eeal = u16(offset << 2);
					m_eeau = (m_eeau & ~0x7ff) | ((offset << 14) & 0x7ff);

					interrupt<IST_MEM>(1);
				}
				break;
			case 2:
				// uncorrectable error
				if (!m_ecb)
				{
					if (offset & 1)
					{
						m_ecb = u16(ecc) << 8;
						m_mdr |= MDR_MEB;
					}
					else
					{
						m_ecb = ecc;
						m_mdr |= MDR_MEA;
					}
					m_eeal = u16(offset << 2);
					m_eeau = (m_eeau & ~0x7ff) | ((offset << 14) & 0x7ff);

					interrupt<IST_MEM>(1);
				}
				break;
			}
		}

		return data;
	}
	else
		return 0;
}

void aviion88k_state::ram_w(offs_t offset, u32 data, u32 mem_mask)
{
	if (offset < m_ram_size / 4)
	{
		m_ram[offset] = (m_ram[offset] & ~mem_mask) | (data & mem_mask);

		switch (m_mdr & MDR_EMS)
		{
		case 0x00:
		case 0x10:
			// normal ECC function
			m_ecc[offset] = compute_ecc(m_ram[offset]);
			break;
		case 0x08:
			// diagnostic generate
			m_ecc[offset] = 0x80 | (m_dle & DLE_CBE);
			break;
		case 0x18:
			// initialize, no ECC
			m_ecc[offset] = compute_ecc(0);
			break;
		}
	}
}

static u8 const ecc_bits[] =
{
	0x4f, 0x4a, 0x52, 0x54,
	0x57, 0x58, 0x5b, 0x5d,
	0x23, 0x25, 0x26, 0x29,
	0x2a, 0x2c, 0x31, 0x34,

	0x0e, 0x0b, 0x13, 0x15,
	0x16, 0x19, 0x1a, 0x1c,
	0x62, 0x64, 0x67, 0x68,
	0x6b, 0x6d, 0x70, 0x75,
};

u8 aviion88k_state::compute_ecc(u32 const data) const
{
	u8 ecc = 0;

	for (unsigned i = 0; i < std::size(ecc_bits); i++)
		ecc ^= BIT(data, i) ? ecc_bits[i] : 0;

	return ecc ^ 0x8c;
}

unsigned aviion88k_state::check_ecc(u32 &data, u8 const ecc, bool const correct) const
{
	u8 const error = compute_ecc(data) ^ ecc;

	if (error)
	{
		for (unsigned i = 0; i < std::size(ecc_bits); i++)
		{
			if (error == ecc_bits[i])
			{
				LOGMASKED(LOG_ECC, "check_ecc single-bit error 0x%08x ecc 0x%02x error 0x%02x\n", data, ecc, error);

				// correct error
				if (correct)
					data ^= (1U << i);

				return 1;
			}
		}

		// multiple-bit error
		LOGMASKED(LOG_ECC, "check_ecc multiple-bit error 0x%08x ecc 0x%02x error 0x%02x\n", data, ecc, error);
		return 2;
	}
	else
		return 0;
}

/*
 * CPU writes to ffff'0005, which maps to VME A16, which probably maps to the GCS BRDID register
 * GRPAD[7-0] and BDAD[3-0] define base address of GCS registers
 *
 * write 16 bit address of GCS register and define as A16 via address modifiers AM[5-0]
 * 3 most significant nibbles of address ust be == BASAD loaded by GRPAD and BDAD switches on system board
 * VME logic compares the address with the value stored in BASAD. If equal, route to MBus. Bits A[3-0] select one of 8 GCS registers.
 * 
 * 
 */
u8 aviion88k_state::vme_a16_r(offs_t offset)
{
	if ((offset & 0xfff0) == m_basad)
	{
		return m_cpu->space(0).read_byte(0xfff8'6000 | (offset & 0xf));
	}
	else
	{
		// TODO: accesses to 00fx trigger location monitor - where else?
		m_global[0] &= ~(0x10 << ((offset >> 1) & 3));
		interrupt<IST_LM>(1);
	}

	return 0;
}

void aviion88k_state::vme_a16_w(offs_t offset, u8 data)
{
	logerror("vme_a16_w %x data %x\n", offset, data);

	if ((offset & 0xfff0) == m_basad)
	{
		switch (offset & 0xf)
		{
		case 0x3:
			{
				m_global[1] = (m_global[1] & 0x5f) | (data & 0xa3);

				if (BIT(data, 0))
					m_ist |= IST_SLP;
				if (BIT(data, 1))
				{
					logerror("setting shp\n");
					m_ist |= IST_SHP;
				}
				interrupt_check();
			}
			break;
		case 0x5: break;
		//case 0x7: m_gpcs[0] = data; break;
		//case 0x9: m_gpcs[1] = data; break;
		//case 0xa: m_gpcs[2] = data; break;
		//case 0xc: m_gpcs[3] = data; break;
		//case 0xe: m_gpcs[4] = data; break;

		default:
			m_cpu->space(0).write_byte(0xfff8'6000 | (offset & 0xf), data);
			break;
		}
	}
	else
	{
		m_global[0] &= ~(0x10 << ((offset >> 1) & 3));
		interrupt<IST_LM>(1);
	}
}

static void aviion88k_scsi_devices(device_slot_interface &device)
{
	device.option_add("harddisk", NSCSI_HARDDISK);
	device.option_add("cdrom", NSCSI_CDROM);
}

void aviion88k_state::aviion_4600(machine_config &config)
{
	MC88100(config, m_cpu, 33'333'333);
	m_cpu->set_addrmap(AS_PROGRAM, &aviion88k_state::cpu_map);

	MC88200(config, m_cmmu[0], 33'333'333, 0x78); // cpu0 cmmu d0
	m_cmmu[0]->set_mbus(m_cpu, AS_PROGRAM);
	m_cpu->set_cmmu_d(m_cmmu[0]);
	MC88200(config, m_cmmu[1], 33'333'333, 0x79); // cpu0 cmmu i0
	m_cmmu[1]->set_mbus(m_cpu, AS_PROGRAM);
	m_cpu->set_cmmu_i(m_cmmu[1]);

	INTEL_28F010(config, m_prom[0]);
	INTEL_28F010(config, m_prom[1]);
	INTEL_28F010(config, m_prom[2]);
	INTEL_28F010(config, m_prom[3]);

	// 8 SIMM slots (populated with pairs of 4M or 16M modules)
#if 0
	RAM(config, m_ram);
	m_ram->set_default_size("8M");
	m_ram->set_extra_options("16M,24M,32M,40M,48M,56M,64M,72M,80M,96M,104M,128M");
	m_ram->set_default_value(0);
#endif

	MK48T12(config, m_novram);
	m_novram->irq_cb().set(FUNC(aviion88k_state::interrupt_ex<EXIST_RTCOF>));

	INPUT_MERGER_ANY_HIGH(config, m_kbdc_txc);
	m_kbdc_txc->output_handler().set(m_uart, FUNC(scn2661a_device::txc_w));

	INPUT_MERGER_ANY_LOW(config, m_kbdc_dsc);
	m_kbdc_dsc->output_handler().set(m_kbdc, FUNC(pc_kbdc_device::clock_write_from_mb)).invert();

	// uart - keyboard interface
	SCN2661A(config, m_uart, 0);
	//m_uart->out_int_callback().set(FUNC(aviion88k_state::interrupt<IST_KBD>));
	m_uart->txd_handler().set(m_kbdc, FUNC(pc_kbdc_device::data_write_from_mb));
	//m_uart->dtr_handler().set(m_kbd_dsc, FUNC(input_merger_any_low_device::in_w<0>));
	m_uart->dtr_handler().set(
		[this](int state)
		{
			LOG("dtr %d\n", state);
			m_kbdc_dsc->in_w<0>(state);
		});

	// keyboard connector
	PC_KBDC(config, m_kbdc, pc_at_keyboards, nullptr);
	m_kbdc->out_clock_cb().set(m_uart, FUNC(scn2661a_device::rxc_w));
	m_kbdc->out_clock_cb().append(m_kbdc_txc, FUNC(input_merger_any_high_device::in_w<0>));
	m_kbdc->out_data_cb().set(m_uart, FUNC(scn2661a_device::rxd_w));

	// duart 1
	SCN2681(config, m_duart[0], 14.7456_MHz_XTAL / 4); // SCC2692
	m_duart[0]->irq_cb().set(FUNC(aviion88k_state::interrupt<IST_DI>));
	RS232_PORT(config, m_async[0], default_rs232_devices, "terminal"); // console: DCD,RXD,TXD,DTR,RTS,CTS
	RS232_PORT(config, m_async[1], default_rs232_devices, nullptr); // async A: DCD,RXD,TXD,DTR,DSR,RTS,CTS,RI

	m_duart[0]->a_tx_cb().set(m_async[0], FUNC(rs232_port_device::write_txd));
	m_duart[0]->b_tx_cb().set(m_async[1], FUNC(rs232_port_device::write_txd));
	m_async[0]->rxd_handler().set(m_duart[0], FUNC(scn2681_device::rx_a_w));
	m_async[1]->rxd_handler().set(m_duart[0], FUNC(scn2681_device::rx_b_w));

	// duart 2
	SCN2681(config, m_duart[1], 14.7456_MHz_XTAL / 4); // SCC2692
	m_duart[1]->irq_cb().set(FUNC(aviion88k_state::interrupt_ex<EXIST_DUART2>));
	RS232_PORT(config, m_async[2], default_rs232_devices, nullptr); // mouse: RTS,DTR,TXD,RXD
	RS232_PORT(config, m_async[3], default_rs232_devices, nullptr); // async B: DCD,RXD,TXD,DTR,DSR,RTS,CTS,RI

	m_duart[1]->a_tx_cb().set(m_async[2], FUNC(rs232_port_device::write_txd));
	m_duart[1]->b_tx_cb().set(m_async[3], FUNC(rs232_port_device::write_txd));
	m_async[2]->rxd_handler().set(m_duart[1], FUNC(scn2681_device::rx_a_w));
	m_async[3]->rxd_handler().set(m_duart[1], FUNC(scn2681_device::rx_b_w));

	m_duart[1]->outport_cb().set(
		[this](u8 data)
		{
			m_speaker->level_w(m_spken->enabled() && BIT(data, 3));
		});

	// duscc
	DUSCC68562(config, m_duscc, 14.7456_MHz_XTAL);
	m_duscc->out_int_callback().set(FUNC(aviion88k_state::interrupt_ex<EXIST_SCC>));

	// scsi bus and devices
	NSCSI_BUS(config, m_scsibus);
	NSCSI_CONNECTOR(config, "scsi:0", aviion88k_scsi_devices, "harddisk");
	NSCSI_CONNECTOR(config, "scsi:1", aviion88k_scsi_devices, nullptr);
	NSCSI_CONNECTOR(config, "scsi:2", aviion88k_scsi_devices, nullptr);
	NSCSI_CONNECTOR(config, "scsi:3", aviion88k_scsi_devices, nullptr);
	NSCSI_CONNECTOR(config, "scsi:4", aviion88k_scsi_devices, nullptr);
	NSCSI_CONNECTOR(config, "scsi:5", aviion88k_scsi_devices, nullptr);
	NSCSI_CONNECTOR(config, "scsi:6", aviion88k_scsi_devices, nullptr);

	// scsi host adapter (NCR53C700)
	NSCSI_CONNECTOR(config, "scsi:7").option_set("ncr53c700", NCR53C7XX).machine_config(
		[this](device_t *device)
		{
			ncr53c7xx_device &adapter = downcast<ncr53c7xx_device &>(*device);

			adapter.set_addrmap(AS_PROGRAM, *this, &aviion88k_state::lsio_map);
			adapter.set_clock(66'000'000);
			adapter.irq_handler().set(*this, FUNC(aviion88k_state::interrupt_ex<EXIST_SCSI0>));
		});

	// TODO: ethernet (AM79C900)

	// speaker
	SPEAKER(config, "mono").front_center();
	SPEAKER_SOUND(config, m_speaker);
	m_speaker->add_route(ALL_OUTPUTS, "mono", 0.50);
}

ROM_START(aviion_4600)
	ROM_REGION(0x20000, "prom0", 0)
	ROM_LOAD("11513__x02__92-05.bin", 0, 0x20000, CRC(7031d7d4) SHA1(c1ca7567b764b7f48e53b9bc8df40407464f9f67))
	ROM_REGION(0x20000, "prom1", 0)
	ROM_LOAD("11514__x02__92-06.bin", 0, 0x20000, CRC(4fcf85e6) SHA1(9afeec63cf8098d4518dc0712ba92614d44cd859))
	ROM_REGION(0x20000, "prom2", 0)
	ROM_LOAD("11515__x02__92-05.bin", 0, 0x20000, CRC(c9ce39d7) SHA1(fbdd3287b9f9eb6a621d7c10d900ccaff02660c5))
	ROM_REGION(0x20000, "prom3", 0)
	ROM_LOAD("11516__x02__92-05.bin", 0, 0x20000, CRC(71b6d338) SHA1(eb85bd16a25b6cd790272f007b8117fcf13b6b40))

	ROM_REGION32_BE(0x20, "lanid", 0)
	ROM_LOAD32_BYTE("lanid.bin", 0, 8, CRC(91c210c7) SHA1(edcf953b17ac968fa45f83d5c8e0c155c5c7e006))
ROM_END

} // anonymous namespace

/*   YEAR   NAME         PARENT  COMPAT  MACHINE      INPUT  CLASS            INIT  COMPANY         FULLNAME       FLAGS */
COMP(1991,  aviion_4600, 0,      0,      aviion_4600, 0,     aviion88k_state, init, "Data General", "AViiON 4600", MACHINE_IS_SKELETON)
