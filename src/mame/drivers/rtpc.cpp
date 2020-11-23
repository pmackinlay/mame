// license:BSD-3-Clause
// copyright-holders:Patrick Mackinlay

/*
 * IBM RT PC.
 *
 * Sources:
 *
 *   - http://www.cs.cmu.edu/afs/andrew.cmu.edu/usr/shadow/www/ibmrt.html
 *   - http://ps-2.kev009.com/ohlandl/6152/rt_index.html
 * 
 * TODO
 *   - everything
 *
 */

#include "emu.h"

// cpu and memory
#include "cpu/romp/romp.h"
#include "cpu/mcs51/mcs51.h"

// various hardware
#include "machine/rosetta.h"
#include "machine/rtpc_iocc.h"
#include "machine/am9517a.h"
#include "machine/pic8259.h"
#include "machine/i8255.h"
#include "machine/mc146818.h"
#include "machine/z80scc.h"
#include "machine/timer.h"

// isa bus
#include "bus/isa/isa.h"
#include "bus/isa/isa_cards.h"
#include "bus/isa/mda.h"

// busses and connectors
#include "bus/rs232/rs232.h"
#include "bus/rtpc/kbd_con.h"
#include "bus/rtpc/kbd.h"

#include "sound/spkrdev.h"

#include "imagedev/floppy.h"
#include "formats/pc_dsk.h"
#include "softlist.h"

#define VERBOSE 1
#include "logmacro.h"

#include "debugger.h"

#include "rtpc.lh"

/*
 * WIP
 *  - inside sub_80125c
 *  - 34b4 == 8012d8
 *  - 3512 == 801336
 *     10000  ffffffff 00018000 01ffffff   
 *  - 3586 - should trigger exception?
 *  - bp 359e
 *
 *  *4000 = 4000
 *  *4800 = 4800
 *  ...
 *  *8000 = 8000
 *  *8800 = 8800
 *  *9000 = 9000
 *  *9800 = 9800
 *  ...
 *  *f800 = f800
 * 
 *  *10800 = 10800
 * 
 *  enable translation
 *  *ef008000 == 8800? therefore, ef008000 should translate to 8800 (not 8000)
 *  *8800 == 0x9000? therefore, ef008000 should translate to 8800 (not 8000)
 *
 * [:mmu] reload segment 0x00010000 shift 11 index 16 mask 0x00000fff
 * [:mmu] reload hat base 0x00010000 index 0x10 entry 0x00118000
 * [:mmu] reload ipt count 0 entry 0x0001e010
 * [:mmu] reload address 0x1e010
 * [:mmu] reload complete f0 0x0001e010 f1 0x00000084 f2 0x00000000
 * [:mmu] mem_r 0xef008000 translated 0x00008000
 *
 * real_page = (f1 & 0xfff8) >> 3 == 0x10
 * real_addr = real_page << 11 | base & 7ff
 *
 * bp 3756: double-executing increment r5?
 * failing at 80604e - iocc test #7
 *
 * 806660 j 806826
 *
 * 8880 00e3
 *
 * go 806660
 * iar=806826
 * bp 806bb0
 *   kls: function call at 505 fails due to keyboard clock sense
 *    - why does jb int0 (offset b2 not read the sfr)?
 * 
 */
namespace {

class rtpc_state : public driver_device
{
public:
	rtpc_state(machine_config const &mconfig, device_type type, char const *tag)
		: driver_device(mconfig, type, tag)
		, m_cpu(*this, "cpu")
		, m_mcu(*this, "mcu")
		, m_mmu(*this, "mmu")
		, m_iocc(*this, "iocc")
		, m_dma(*this, "dma%d", 0U)
		, m_pic(*this, "pic%d", 0U)
		, m_ppi(*this, "ppi")
		, m_rtc(*this, "rtc")
		, m_scc(*this, "scc")
		, m_isa(*this, "isa")
		, m_kbd_con(*this, "kbd_con")
		, m_speaker(*this, "kbd_con:kbd:speaker")
		, m_softlist(*this, "softlist")
		, m_leds(*this, "led%d", 0U)
		, m_ipl(*this, "ipl")
	{
	}

protected:
	// driver_device overrides
	virtual void machine_start() override;
	virtual void machine_reset() override;

	// address maps
	template <bool Translate> void cpu_map(address_map &map);
	void io_map(address_map &map);

	void iocc_mem_map(address_map &map);
	void iocc_io_map(address_map &map);

	// machine config
	void common(machine_config &config);

public:
	void ibm6150(machine_config &config);
	void ibm6151(machine_config &config);

	void init_common();

	DECLARE_FLOPPY_FORMATS(floppy_formats);

protected:
	enum csr_mask : u32
	{
		CSR_EXR   = 0x80000000, // exception reported
		CSR_INTP  = 0x40000000, // interrupt pending
		CSR_EPOW  = 0x10000000, // EPOW?
		CSR_SRST  = 0x08000000, // soft reset
		CSR_SAT   = 0x04000000, // system attention
		CSR_PER   = 0x01000000, // pio error
		CSR_DE0   = 0x00800000, // dma error channel 0
		CSR_DE1   = 0x00400000, // dma error channel 1
		CSR_DE2   = 0x00200000, // dma error channel 2
		CSR_DE3   = 0x00100000, // dma error channel 3
		CSR_DE5   = 0x00080000, // dma error channel 5
		CSR_DE6   = 0x00040000, // dma error channel 6
		CSR_DE7   = 0x00020000, // dma error channel 7
		CSR_DE8   = 0x00010000, // dma error channel 8
		CSR_PD    = 0x00008000, // pio/dma
		CSR_PVIO  = 0x00004000, // protection violation
		CSR_INVOP = 0x00002000, // invalid operation
		CSR_IOCK  = 0x00001000, // i/o channel check
		CSR_DEXK  = 0x00000800, // dma exception
		CSR_CRC   = 0x00000400, // channel reset captured
		CSR_SBB   = 0x00000200, // system board busy
		CSR_PRP   = 0x00000100, // pio request pending

		CSR_RSV   = 0x220000ff, // reserved
	};
	void mcu_port1_w(u8 data);
	void mcu_port2_w(u8 data);
	void mcu_port3_w(u8 data);
	void ppi_portc_w(u8 data);

	u8 ccr_r() { return m_ccr; }
	template <unsigned Word> u16 csr_r() { return u16((m_csr | CSR_RSV) >> (16 * Word)); }

	void kls_cmd_w(offs_t offset, u16 data, u16 mem_mask);
	void ccr_w(u8 data) { LOG("ccr_w 0x%02x (%s)\n", data, machine().describe_context()); m_ccr = data; }
	void crra_w(u8 data);
	void crrb_w(u8 data);
	void csr_w(u16 data) { LOG("csr_w (%s)\n", machine().describe_context()); m_csr = 0; }
	void dia_w(u8 data);

	void mcu_timer(timer_device &timer, void *ptr, s32 param)
	{
		m_mcu_p3 ^= 0x10;
		m_mcu->set_input_line(MCS51_T0_LINE, BIT(m_mcu_p3, 4));
	}

	void speaker()
	{
		if (m_speaker)
			m_speaker->level_w(!BIT(m_mcu_p2, 7) ? m_mcu_p1 >> 6 : 0);
	}

	// devices
	required_device<romp_device> m_cpu;
	required_device<i8051_device> m_mcu;
	required_device<rosetta_device> m_mmu;
	required_device<rtpc_iocc_device> m_iocc;

	required_device_array<am9517a_device, 2> m_dma;
	required_device_array<pic8259_device, 2> m_pic;
	required_device<i8255_device> m_ppi;
	required_device<mc146818_device> m_rtc;
	optional_device<z80scc_device> m_scc;

	required_device<isa16_device> m_isa;
	required_device<rtpc_kbd_con_device> m_kbd_con;
	optional_device<speaker_sound_device> m_speaker;

	required_device<software_list_device> m_softlist;

	output_finder<2> m_leds;

	required_region_ptr<u32> m_ipl;

	u8 m_mcu_p0;
	u8 m_mcu_p1;
	u8 m_mcu_p2;
	u8 m_mcu_p3;

	u8 m_ppi_pb;
	u8 m_mcu_uart;

	u8 m_ccr;  // channel control register
	u32 m_csr; // channel status register

	u8 m_dbr; // dma buffer register
	u8 m_dmr; // dma mode register
	u8 m_ch8er; // dma channel 8 enable register
	u8 m_crra; // component reset register a
	u8 m_crrb; // component reset register b
	u16 m_tcw[512]; // dma translation control words
};

// 7-segment diagnostic led
static u8 const led_pattern[16] =
{
	0x3f, 0x06, 0x5b, 0x4f, 
	0x66, 0x6d, 0x7d, 0x07, 
	0x7f, 0x6f, 0x77, 0x7c, 
	0x39, 0x5e, 0x79, 0x00, // 0x71
};

static double const speaker_levels[4] = { 0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0 };

void rtpc_state::machine_start()
{
	m_leds.resolve();

	m_mcu_p0 = 0;
	m_mcu_p1 = 0;
	m_mcu_p2 = 0;
	m_mcu_p3 = 0;

	m_ppi_pb = 0;

	m_ccr = 0;

	m_crra = 0xff;
	m_crrb = 0xff;

	// HACK: branch over word/dword IOCC I/O space tests
	m_ipl[0x6660 >> 2] = 0x888000e3;
	m_ipl[0xfff8 >> 2] = 0x87b90000 | u16(m_ipl[0xfff8 >> 2]);
}

void rtpc_state::machine_reset()
{
	m_csr = 0;
}

void rtpc_state::init_common()
{
	//m_cpu->space(0).install_read_tap(0x0080'0000, 0x0080'ffff, "led_on", [this](offs_t offset, u32 &data, u32 mem_mask) { logerror("led on\n"); return data; });

	m_cpu->space(AS_IO).install_readwrite_tap(0x81'1000, 0x81'2fff, "led",
		[this](offs_t offset, u32 &data, u32 mem_mask)
		{
			m_leds[0] = 0;
			m_leds[1] = 0;

			return data;
		},
		[this](offs_t offset, u32 &data, u32 mem_mask)
		{
			m_leds[0] = led_pattern[(offset >> 0) & 15];
			m_leds[1] = led_pattern[(offset >> 4) & 15];
		});

	if (m_speaker)
		m_speaker->set_levels(4, speaker_levels);
}

/*
 * startup
 *   I/O base address register = 0x81
 *   crra = 0xff (reset all slots)
 *   
 * LEDs
 *   1. enable led by accessing ROM
 *   2. read r/c array - last byte of address gives LED value (blank with ff)
 *   3. disable led by writing to r/c array
 */

template <bool Translate> void rtpc_state::cpu_map(address_map &map)
{
	// system processor real memory address map
	// 0000'0000-00ff'ffff 16MB memory management unit
	//map(0x0000'0000, 0x00ff'ffff).rw(m_mmu, FUNC(rosetta_device::mem_r), FUNC(rosetta_device::mem_w));
	map(0x0000'0000, 0xefff'ffff).rw(m_mmu, FUNC(rosetta_device::mem_r<Translate>), FUNC(rosetta_device::mem_w<Translate>));

	// 0100'0000-efff'ffff -- not defined

	// system board I/O addresses (all in segment F)
	// f000'0000-f0ff'ffff 16MB i/o channel i/o map
	// f100'0000-f3ff'ffff 48MB reserved
	// f400'0000-f4ff'ffff 16MB i/o channel memory map
	// f400'0000-feff'ffff 158MB reserved
	// ff00'0000-ffff'ffff 16MB floating point accelerator

	//map(0x0000'0000, 0x0000'ffff).rom().region("ipl", 0).mirror(0xffff'0000);
	//map(0x0080'0000, 0x0080'ffff).rom().region("ipl", 0); // TODO: via MMU

	map(0xf000'0000, 0xf0ff'ffff).rw(m_iocc, FUNC(rtpc_iocc_device::processor_r<AS_IO>), FUNC(rtpc_iocc_device::processor_w<AS_IO>));
	map(0xf400'0000, 0xf4ff'ffff).rw(m_iocc, FUNC(rtpc_iocc_device::processor_r<AS_PROGRAM>), FUNC(rtpc_iocc_device::processor_w<AS_PROGRAM>));
	//map(0xff00'0000, 0xffff'ffff).rw(m_fpa);
}

void rtpc_state::io_map(address_map &map)
{
	// system processor i/o address map
	// 00'0000-7f'ffff 8MB not defined
	// 80'0000-80'ffff 64KB processor channel device initialization
	// 81'0000-81'ffff 64KB memory management unit
	// 82'0000-ff'ffff 8064KB not defined

	// TODO: assuming the whole space is decoded internal to the mmu
	map(0x00'0000, 0xff'ffff).rw(m_mmu, FUNC(rosetta_device::io_r), FUNC(rosetta_device::io_w));
}

void rtpc_state::iocc_mem_map(address_map &map)
{
	map(0xb'8000, 0xb'8fff).noprw(); // silence
}

void rtpc_state::iocc_io_map(address_map &map)
{
	//map(0x00'8000, 0x00'8003).rw(m_scc, FUNC(z80scc_device::dc_ab_r), FUNC(z80scc_device::dc_ab_w));
	//map(0x00'8020, 0x00'8023); // ch a ext reg
	//map(0x00'8040, 0x00'8043); // ch b ext reg
	//map(0x00'8060, 0x00'8063); // 8530 intack
	map(0x00'80e0, 0x00'80e3).lw8(
		[this](u8 data)
		{
			// delay 1usec per byte written
			m_cpu->eat_cycles(m_cpu->clock() / 1000000);
		}, "io_delay");

	map(0x00'8400, 0x00'8403).mirror(0x7c).rw(m_ppi, FUNC(i8255_device::read), FUNC(i8255_device::write));
	map(0x00'8400, 0x00'8401).mirror(0x78).w(FUNC(rtpc_state::kls_cmd_w));

	map(0x00'8800, 0x00'883f).rw(m_rtc, FUNC(mc146818_device::read_direct), FUNC(mc146818_device::write_direct));
	map(0x00'8840, 0x00'884f).mirror(0x10).rw(m_dma[0], FUNC(am9517a_device::read), FUNC(am9517a_device::write));
	map(0x00'8860, 0x00'886f).mirror(0x10).rw(m_dma[1], FUNC(am9517a_device::read), FUNC(am9517a_device::write));
	map(0x00'8880, 0x00'8881).mirror(0x1e).rw(m_pic[0], FUNC(pic8259_device::read), FUNC(pic8259_device::write));
	map(0x00'88a0, 0x00'88a1).mirror(0x1e).rw(m_pic[1], FUNC(pic8259_device::read), FUNC(pic8259_device::write));
	map(0x00'88c0, 0x00'88c0).mirror(0x1f).lrw8([this]() { return m_dbr; }, "dbr_r", [this](u8 data) { m_dbr = data; }, "dbr_w");
	map(0x00'88e0, 0x00'88e0).mirror(0x1f).lrw8([this]() { return m_dmr; }, "dmr_r", [this](u8 data) { m_dmr = data; }, "dmr_w");

	map(0x00'8c00, 0x00'8c00).mirror(0x03).lrw8([this]() { return m_ch8er; }, "ch8er_r", [this](u8 data) { m_ch8er = data; }, "ch8er_w");
	map(0x00'8c20, 0x00'8c20).mirror(0x03).rw(FUNC(rtpc_state::ccr_r), FUNC(rtpc_state::ccr_w)); // channel control register
	map(0x00'8c40, 0x00'8c40).mirror(0x03).lr8([this]() { return m_crra; }, "crra_r");
	map(0x00'8c40, 0x00'8c40).mirror(0x03).w(FUNC(rtpc_state::crra_w));
	map(0x00'8c60, 0x00'8c60).mirror(0x03).lr8([this]() { return m_crrb; }, "crrb_r");
	map(0x00'8c60, 0x00'8c60).mirror(0x03).w(FUNC(rtpc_state::crrb_w));

	map(0x00'8c80, 0x00'8c80).mirror(0x03).lr8([this]() { return 0xc4; }, "mcr"); // memory config reg
	map(0x00'8ca0, 0x00'8ca0).mirror(0x03).w(FUNC(rtpc_state::dia_w));

	map(0x01'0000, 0x01'03ff).lrw16(
		[this](offs_t offset, u16 mem_mask)
		{
			logerror("tcw_r offset 0x%x data 0x%04x mask 0x%04x\n", offset, m_tcw[offset], mem_mask);
			return m_tcw[offset];
		}, "tcw_r",
		[this](offs_t offset, u16 data, u16 mem_mask)
		{
			logerror("tcw_w offset 0x%x data 0x%04x mask 0x%04x\n", offset, data, mem_mask);

			COMBINE_DATA(&m_tcw[offset]);
		}, "tcw_r");
	//map(0x01'0400, 0x01'07ff); // tcw
	map(0x01'0800, 0x01'0801).mirror(0x7fc).rw(FUNC(rtpc_state::csr_r<1>), FUNC(rtpc_state::csr_w)); // channel status register
	map(0x01'0802, 0x01'0803).mirror(0x7fc).rw(FUNC(rtpc_state::csr_r<0>), FUNC(rtpc_state::csr_w)); // channel status register
}

FLOPPY_FORMATS_MEMBER(rtpc_state::floppy_formats)
	FLOPPY_PC_FORMAT
FLOPPY_FORMATS_END

void rtpc_state::mcu_port1_w(u8 data)
{
	// bit  function
	//  6   speaker volume 0
	//  7   speaker volume 1
	LOG("mcu_port1_w volume %d\n", data >> 6);
	m_mcu_p1 = (m_mcu_p1 & 0x3f) | (data & 0xc0);

	// speaker volume wraps to ppi port b.6 and b.5
	m_ppi_pb &= ~0x60;
	m_ppi_pb |= (data >> 1) & 0x60;

	speaker();
}

void rtpc_state::mcu_port2_w(u8 data)
{
	// bit  dst
	//  0   ppi port c.0 (iid0)
	//  1   ppi port c.1 (iid1)
	//  2   ppi port c.2 (iid2)
	//  3   i/o channel system reset (active low)
	//  4   ppi port c.4
	//  5   (input)
	//  6   ppi port c.6 (-ack)
	//  7   speaker frequency

	// interrupts
	// 0 informational
	// 1 received byte from keyboard
	// 2 received byte from uart
	// 3 returning byte requested by system
	// 4 block transfer ready
	// 5 unassigned
	// 6 self-test performed
	// 7 error condition

	if ((data ^ m_mcu_p2) & 7)
		LOG("mcu_port2_w interrupt %d\n", data & 7);

	if ((data ^ m_mcu_p2) & 8)
		LOG("mcu_port2_w system reset %d\n", data & 8);

	m_ppi->pc4_w(BIT(data, 4));
	m_ppi->pc6_w(BIT(data, 6));

	m_mcu_p2 &= ~0xdf;
	m_mcu_p2 |= data & 0xdf;

	// speaker frequency wraps to ppi port b.7
	m_ppi_pb &= ~0x80;
	m_ppi_pb |= ~data & 0x80;

	speaker();
}

void rtpc_state::mcu_port3_w(u8 data)
{
	// bit  i/o  function
	//  0    i   uart rx
	//  1    o   uart tx
	//  2    i   kbd clock in
	//  3    i   obf/int1
	//  4    i   32 kHz
	//  5    i   kbd data in
	//  6    o   kbd data out
	//  7    o   kbd clock out

	logerror("mcu_port3_w 0x%02x\n", data);

	m_kbd_con->data_write_from_mb(BIT(data, 6));
	m_kbd_con->clock_write_from_mb(BIT(data, 7));

	m_mcu_p3 = (m_mcu_p3 & ~0xc2) | (data & 0xc2);
}

void rtpc_state::ppi_portc_w(u8 data)
{
	logerror("ppi_portc_w 0x%02x\n", data);

	// bit 3 -> i/o channel (irq)
	m_pic[0]->ir5_w(BIT(data, 3));

	// bit 5 -> mcu p2.5 (ibf)
	if (BIT(data, 5))
		m_mcu_p2 |= 0x20;
	else
		m_mcu_p2 &= ~0x20;

	// bit 7 -> mcu p3.3(-int1) (-obf)
	if (BIT(data, 7))
		m_mcu_p3 |= 0x08;
	else
		m_mcu_p3 &= ~0x08;
	m_mcu->set_input_line(MCS51_INT1_LINE, !BIT(data, 7));
}

void rtpc_state::kls_cmd_w(offs_t offset, u16 data, u16 mem_mask)
{
	LOG("kls_cmd_w command 0x%02x data 0x%02x mask 0x%04x\n", u8(data), data >> 8, mem_mask);

	// 00cc cccc dddd dddd
	switch (mem_mask)
	{
	case 0xff00:
		m_ppi->write(0, data);
		break;
	case 0x00ff:
		m_ppi->write(1, data);
		break;
	case 0xffff:
		m_mcu_p1 = (m_mcu_p1 & ~0x3f) | (data & 0x3f);
		m_ppi->write(0, data >> 8);
		break;
	}
}

void rtpc_state::crra_w(u8 data)
{
	LOG("crra_w 0x%02x\n", data);

	// bit  function
	//  0   i/o slot 1
	//  1   i/o slot 2
	//  2   i/o slot 3
	//  3   i/o slot 4
	//  4   i/o slot 5
	//  5   i/o slot 6
	//  6   i/o slot 7
	//  7   i/o slot 8

	m_crra = data;
}

void rtpc_state::crrb_w(u8 data)
{
	LOG("crrb_w 0x%02x\n", data);

	// bit  function
	//  0   8530
	//  1   rs232 interface
	//  2   8051
	//  3   dmac1
	//  4   dmac2
	//  5   arbitor
	//  6   reserved
	//  7   reserved

	if (BIT(data, 0))
		m_scc->reset();
	// TODO: rs232 if
	m_mcu->set_input_line(INPUT_LINE_RESET, !BIT(data, 2));
	m_dma[0]->set_input_line(INPUT_LINE_RESET, BIT(data, 3));
	m_dma[1]->set_input_line(INPUT_LINE_RESET, BIT(data, 4));
	// TODO: arbitor

	m_crrb = data;
}

void rtpc_state::dia_w(u8 data)
{
	bool const state = BIT(data, 0);

	m_pic[0]->ir0_w(state);
	m_pic[0]->ir1_w(state);
	m_pic[0]->ir2_w(state);
	m_pic[0]->ir3_w(state);
	m_pic[0]->ir4_w(state);
	m_pic[0]->ir5_w(state);
	m_pic[0]->ir6_w(state);
	m_pic[0]->ir7_w(state);

	m_pic[1]->ir0_w(state);
	m_pic[1]->ir1_w(state);
	m_pic[1]->ir2_w(state);
	m_pic[1]->ir3_w(state);
	m_pic[1]->ir4_w(state);
	m_pic[1]->ir5_w(state);
	m_pic[1]->ir6_w(state);
	m_pic[1]->ir7_w(state);
}

void rtpc_state::common(machine_config &config)
{
	/*
	 * https://www-01.ibm.com/common/ssi/ShowDoc.wss?docURL=/common/ssi/rep_ca/6/897/ENUS186-006/index.html
	 * https://www-01.ibm.com/common/ssi/ShowDoc.wss?docURL=/common/ssi/rep_ca/1/897/ENUS187-021/index.html
	 * https://www-01.ibm.com/common/ssi/ShowDoc.wss?docURL=/common/ssi/rep_ca/0/897/ENUS188-120/index.html
	 *
	 *   Model  Chassis  CPU  RAM      HDD  Release   Price    Notes
	 *    010    6151    032  1M/4M    40M  Jan 1986  $11,700
	 *    015    6151    032  2M/4M    70M  Nov 1986  $10,050
	 *    020    6150    032  1M/4M    40M  Jan 1986  $14,945
	 *    025    6150    032  2M/4M    70M  Jan 1986  $17,940
	 *    A25    6150    032  2M/4M    70M  Jan 1986  $19,510  5080 attachment/no keyboard
	 *    115    6151    Adv  4M/16M   70M  Feb 1987  $10,600  AFPA
	 *    125    6150    Adv  4M/16M   70M  Feb 1987  $16,100  AFPA
	 *    B25    6150    Adv  4M/16M   70M  Feb 1987  $17,670  AFPA, 5080 attachment/no keyboard
	 *    130    6151    Enh  16M     114M  Jul 1988  $23,220  EAFPA
	 *    135    6150    Enh  16M     114M  Jul 1988  $30,595  EAFPA
	 *    B35    6150    Enh  16M     114M  Jul 1988  $32,165  EAFPA, 5080 attachment/no keyboard
	 */
	// 032, 170ns (23.5294 MHz crystal / 4 == 5.882350 MHz == 170ns)
	// Advanced, 100ns 4MB (6151) external (6150) (presume ~40MHz crystal/4)
	// Enhanced, 80ns, 16MB soldered, EAFPA standard, CMOS (49.400 MHz crystal/4 == 12.350MHz == 80.971ns)

	// AFPA is M68881 @ 20MHz
	// EAFPA is AD90221-2 ADSP-3210 (multiplier) + AD90222-2 ADSP-3221 (fp alu) + AD90220-2 ADSP-1401 (program sequencer)

	ROMP(config, m_cpu, 23'529'400 / 4);
	m_cpu->set_addrmap(0, &rtpc_state::cpu_map<false>);
	m_cpu->set_addrmap(1, &rtpc_state::cpu_map<true>);
	m_cpu->set_addrmap(2, &rtpc_state::io_map);

	ROSETTA(config, m_mmu, m_cpu->clock(), rosetta_device::MASTER, rosetta_device::RAM_16M);
	m_mmu->set_bus(m_cpu, AS_PROGRAM);
	m_mmu->set_rom("ipl");
	m_mmu->out_trap().set_inputline(m_cpu, INPUT_LINE_NMI);

	// keyboard, locator, speaker adapter
	// P8051AH 2075
	// 61X6310 8811
	// (c)IBM 1986
	// (c)INTEL '80
	I8051(config, m_mcu, 9.216_MHz_XTAL);
	m_mcu->port_in_cb<0>().set([this]() { return m_ppi->pa_r(); });
	m_mcu->port_out_cb<0>().set([this](u8 data) { logerror("mcu p0 0x%02x (%s)\n", data, machine().describe_context()); m_mcu_p0 = data; });
	m_mcu->port_in_cb<1>().set([this]() { return m_mcu_p1 & 0x1f; });
	m_mcu->port_out_cb<1>().set(FUNC(rtpc_state::mcu_port1_w));
	m_mcu->port_in_cb<2>().set([this]() { return m_mcu_p2; });
	m_mcu->port_out_cb<2>().set(FUNC(rtpc_state::mcu_port2_w));
	m_mcu->port_out_cb<3>().set(FUNC(rtpc_state::mcu_port3_w));
	m_mcu->port_in_cb<3>().set([this]() { return m_mcu_p3 & 0x3d; });
	m_mcu->serial_tx_cb().set(
		[this](u8 data)
		{
			if (BIT(m_mcu_p1, 5))
			{
				m_mcu_uart = data;
				m_mcu->set_input_line(MCS51_RX_LINE, 1);
			}
			else
				logerror("uart tx 0x%02x\n", data);
		});
	m_mcu->serial_rx_cb().set([this]() { return m_mcu_uart; });

	TIMER(config, "mcu_timer").configure_periodic(FUNC(rtpc_state::mcu_timer), attotime::from_hz(32768));

	RTPC_IOCC(config, m_iocc, 0);
	m_iocc->set_addrmap(AS_PROGRAM, &rtpc_state::iocc_mem_map);
	m_iocc->set_addrmap(AS_IO, &rtpc_state::iocc_io_map);

	// ISA bus
	ISA16(config, m_isa, 4'770'000); // 33% duty cycle
	// oscillator 14.31818 MHz, 50% duty cycle, not synchronous with system clock
	m_isa->set_memspace(m_iocc, AS_PROGRAM);
	m_isa->set_iospace(m_iocc, AS_IO);
	//m_isa->iochck_callback().set(FUNC(at_mb_device::iochck_w));

	// NEC
	// D8237AC-5
	// 8903HV101
	AM9517A(config, m_dma[0], 0);
	// chan  usage
	//  0    serial port A
	//  1    serial port B
	//  2    diskette drive, serial port A
	//  3    pc network, serial port B
	m_isa->drq0_callback().set(m_dma[0], FUNC(am9517a_device::dreq0_w));
	m_isa->drq1_callback().set(m_dma[0], FUNC(am9517a_device::dreq1_w));
	m_isa->drq2_callback().set(m_dma[0], FUNC(am9517a_device::dreq2_w));
	m_isa->drq3_callback().set(m_dma[0], FUNC(am9517a_device::dreq3_w));

	// NEC
	// D8237AC-5
	// 8903HV101
	AM9517A(config, m_dma[1], 0);
	// chan  usage
	//  5    reserved
	//  6    reserved
	//  7    reserved
	//  8    286 coprocessor
	m_isa->drq5_callback().set(m_dma[1], FUNC(am9517a_device::dreq1_w));
	m_isa->drq6_callback().set(m_dma[1], FUNC(am9517a_device::dreq2_w));
	m_isa->drq7_callback().set(m_dma[1], FUNC(am9517a_device::dreq3_w));

	// NEC
	// D8259AC
	PIC8259(config, m_pic[0]);
	m_pic[0]->out_int_callback().set_inputline(m_cpu, INPUT_LINE_IRQ3).invert();
	// irq  source
	//  0   iocc irq 0: dma tc
	//  1   iocc irq 10: (multiport async)
	//  2   iocc irq 9: pc network, (multiport async), enhanced color graphics adapter, 3278/79 emulation adapter
	//  3   iocc irq 3: serial port 2, pc network
	//  4   iocc irq 4: serial port 1
	//  5   iocc irq 1: kbd
	//  6   iocc irq 2: 8530
	//  7   iocc irq 7: parallel port 1, monochrome/printer
	m_isa->irq10_callback().set(m_pic[0], FUNC(pic8259_device::ir1_w));
	m_isa->irq2_callback().set(m_pic[0], FUNC(pic8259_device::ir2_w));
	m_isa->irq3_callback().set(m_pic[0], FUNC(pic8259_device::ir3_w));
	m_isa->irq4_callback().set(m_pic[0], FUNC(pic8259_device::ir4_w));
	m_isa->irq7_callback().set(m_pic[0], FUNC(pic8259_device::ir7_w));

	// NEC
	// D8259AC
	PIC8259(config, m_pic[1]);
	m_pic[1]->out_int_callback().set_inputline(m_cpu, INPUT_LINE_IRQ4).invert();
	// irq  source
	//  0   iocc irq 8: reserved
	//  1   iocc irq 11: (advanced monochrome graphics display) (multiport async)
	//  2   iocc irq 14: fixed disk
	//  3   iocc irq 12: (ibm rt pc streaming tape drive adapter)
	//  4   iocc irq 6: diskette drive
	//  5   iocc irq 5: parallel port 2
	//  6   iocc irq 15: (286 coprocessor)
	//  7   iocc irq 13: serial port ex ct1 irpt
	m_isa->irq11_callback().set(m_pic[1], FUNC(pic8259_device::ir1_w));
	m_isa->irq14_callback().set(m_pic[1], FUNC(pic8259_device::ir2_w));
	m_isa->irq12_callback().set(m_pic[1], FUNC(pic8259_device::ir3_w));
	m_isa->irq6_callback().set(m_pic[1], FUNC(pic8259_device::ir4_w));
	m_isa->irq5_callback().set(m_pic[1], FUNC(pic8259_device::ir5_w));
	m_isa->irq15_callback().set(m_pic[1], FUNC(pic8259_device::ir6_w));

	// P8255A-5
	// L6430434
	// (C)INTEL '81
	I8255A(config, m_ppi);
	// port A: read/write 8051
	// port B: input
	// port C lower: input
	// port C upper: 8051 handshake
	// port C & 0x20 -> irq
	m_ppi->in_pa_callback().set([this]() { return m_mcu_p0; });
	
	// TODO: bits 4-1 "non-adapter system board signals"
	// TODO: bit 0 "uart rxd signal"
	m_ppi->in_pb_callback().set([this]() { return m_ppi_pb; });

	m_ppi->out_pc_callback().set(FUNC(rtpc_state::ppi_portc_w));
	m_ppi->in_pc_callback().set([this]() { return m_mcu_p2 & 0x57; });

	RTPC_KBD_CON(config, m_kbd_con);
	m_kbd_con->option_add("kbd", RTPC_KBD);
	m_kbd_con->set_default_option("kbd");
	m_kbd_con->out_data_cb().set([this](int state) { if (state) m_mcu_p3 |= 0x20; else m_mcu_p3 &= ~0x20; });
	m_kbd_con->out_clock_cb().set(
		[this](int state)
		{
			if (state)
				m_mcu_p3 |= 0x04;
			else
				m_mcu_p3 &= ~0x04;
			m_mcu->set_input_line(MCS51_INT0_LINE, !state);
		});

	// MC146818AP
	// IL 0A46D8729
	MC146818(config, m_rtc, 32.768_kHz_XTAL);
	m_rtc->sqw().set(m_cpu, FUNC(romp_device::clk_w));
	m_rtc->irq().set_inputline(m_cpu, INPUT_LINE_IRQ1).invert();
	//m_rtc->set_epoch(1980);

	config.set_default_layout(layout_rtpc);

	// software list
	SOFTWARE_LIST(config, m_softlist).set_original("rtpc");
}

void rtpc_isa8_cards(device_slot_interface &device)
{
	device.option_add("mda", ISA8_MDA);
}

void rtpc_isa16_cards(device_slot_interface &device)
{
}

void rtpc_state::ibm6150(machine_config &config)
{
	common(config);

	SCC8530N(config, m_scc, 3'580'000);
	m_scc->configure_channels(3'072'000, 3'072'000, 3'072'000, 3'072'000);
	m_scc->out_int_callback().set(m_pic[0], FUNC(pic8259_device::ir6_w));

	// port: TXD, DTR, RTS, RI, RXD, DSR, CTS, DCD
	// external registers: DTR, DSR, RI
	// scc: TXD, RTS, RXD, CTS, DCD
	// scc: DTR/REQ, SYNC not connected

	rs232_port_device &port0(RS232_PORT(config, "serial0", default_rs232_devices, nullptr));
	port0.cts_handler().set(m_scc, FUNC(z80scc_device::ctsa_w));
	port0.dcd_handler().set(m_scc, FUNC(z80scc_device::dcda_w));
	port0.rxd_handler().set(m_scc, FUNC(z80scc_device::rxa_w));
	m_scc->out_rtsa_callback().set(port0, FUNC(rs232_port_device::write_rts));
	m_scc->out_txda_callback().set(port0, FUNC(rs232_port_device::write_txd));
	//m_scc->out_wreqa_callback().set(m_ioga, FUNC(interpro_ioga_device::drq_serial1)).invert();

	rs232_port_device &port1(RS232_PORT(config, "serial1", default_rs232_devices, nullptr));
	// TXD, DTR, RTS, RI, RXD, DSR, CTS, DCD
	port1.cts_handler().set(m_scc, FUNC(z80scc_device::ctsb_w));
	port1.dcd_handler().set(m_scc, FUNC(z80scc_device::dcdb_w));
	port1.rxd_handler().set(m_scc, FUNC(z80scc_device::rxb_w));
	m_scc->out_rtsb_callback().set(port1, FUNC(rs232_port_device::write_rts));
	m_scc->out_txdb_callback().set(port1, FUNC(rs232_port_device::write_txd));
	//m_scc->out_wreqb_callback().set(m_ioga, FUNC(interpro_ioga_device::drq_serial2)).invert();

	// ISA slots
	ISA16_SLOT(config, "isa1", 0, m_isa, rtpc_isa16_cards, nullptr, false); // slot 1: disk/diskette adapter
	ISA16_SLOT(config, "isa2", 0, m_isa, rtpc_isa16_cards, nullptr, false); // slot 2: option
	ISA8_SLOT(config,  "isa3", 0, m_isa, rtpc_isa8_cards,  "mda",   false); // slot 3: option
	ISA16_SLOT(config, "isa4", 0, m_isa, rtpc_isa16_cards, nullptr, false); // slot 4: option
	ISA16_SLOT(config, "isa5", 0, m_isa, rtpc_isa16_cards, nullptr, false); // slot 5: option
	ISA8_SLOT(config,  "isa6", 0, m_isa, rtpc_isa8_cards,  nullptr, false); // slot 6: option
	ISA16_SLOT(config, "isa7", 0, m_isa, rtpc_isa16_cards, nullptr, false); // slot 7: option
	ISA16_SLOT(config, "isa8", 0, m_isa, rtpc_isa16_cards, nullptr, false); // slot 8: coprocessor/option
}

void rtpc_state::ibm6151(machine_config &config)
{
	common(config);

	// ISA slots
	ISA8_SLOT(config,  "isa1", 0, m_isa, rtpc_isa8_cards,  "mda",   false); // slot 1: option
	ISA16_SLOT(config, "isa2", 0, m_isa, rtpc_isa16_cards, nullptr, false); // slot 2: option
	ISA16_SLOT(config, "isa3", 0, m_isa, rtpc_isa16_cards, nullptr, false); // slot 2: option
	ISA16_SLOT(config, "isa4", 0, m_isa, rtpc_isa16_cards, nullptr, false); // slot 4: option
	ISA16_SLOT(config, "isa5", 0, m_isa, rtpc_isa16_cards, nullptr, false); // slot 5: coprocessor/option
	ISA16_SLOT(config, "isa6", 0, m_isa, rtpc_isa16_cards, nullptr, false); // slot 6: disk/diskette adapter
}

ROM_START(ibm6150)
	ROM_REGION32_BE(0x10000, "ipl", 0)
	ROM_SYSTEM_BIOS(0, "ipl", "IPL")
	ROMX_LOAD("79x3456.bin", 0x00000, 0x4000, CRC(0a45a9ba) SHA1(02ca637c6a871c180dbfebf2ec68d8ec5a998c76), ROM_BIOS(0) | ROM_SKIP(3))
	ROMX_LOAD("79x3458.bin", 0x00001, 0x4000, CRC(7bd08ab6) SHA1(aabcfbb8fa1a5f8a08fb5cfd90ca6fe05258fde9), ROM_BIOS(0) | ROM_SKIP(3))
	ROMX_LOAD("79x3460.bin", 0x00002, 0x4000, CRC(897586e0) SHA1(528772635903f27235ebba2622b03386b84e4e17), ROM_BIOS(0) | ROM_SKIP(3))
	ROMX_LOAD("79x3462.bin", 0x00003, 0x4000, CRC(12aca906) SHA1(58f95b95768ef131d8d9d552506a9fe9c9c6077d), ROM_BIOS(0) | ROM_SKIP(3))

	ROM_REGION(0x1000, "mcu", 0)
	ROM_LOAD("61x6310_8051.bin", 0x0000, 0x1000, CRC(296c16c1) SHA1(83858109c39d5be37e49f24d1db4e2b15f38843e))
ROM_END

ROM_START(ibm6151)
	ROM_REGION32_BE(0x10000, "ipl", 0)
	ROM_SYSTEM_BIOS(0, "ipl", "IPL")
	ROMX_LOAD("79x3456.bin", 0x00000, 0x4000, CRC(0a45a9ba) SHA1(02ca637c6a871c180dbfebf2ec68d8ec5a998c76), ROM_BIOS(0) | ROM_SKIP(3))
	ROMX_LOAD("79x3458.bin", 0x00001, 0x4000, CRC(7bd08ab6) SHA1(aabcfbb8fa1a5f8a08fb5cfd90ca6fe05258fde9), ROM_BIOS(0) | ROM_SKIP(3))
	ROMX_LOAD("79x3460.bin", 0x00002, 0x4000, CRC(897586e0) SHA1(528772635903f27235ebba2622b03386b84e4e17), ROM_BIOS(0) | ROM_SKIP(3))
	ROMX_LOAD("79x3462.bin", 0x00003, 0x4000, CRC(12aca906) SHA1(58f95b95768ef131d8d9d552506a9fe9c9c6077d), ROM_BIOS(0) | ROM_SKIP(3))

	// Version: 073
	// Date: 85 289 (16 Oct 1985)
	// Checksum: 0xc8 0xe8
	ROM_REGION(0x1000, "mcu", 0)
	ROM_LOAD("61x6310_8051.bin", 0x0000, 0x1000, CRC(296c16c1) SHA1(83858109c39d5be37e49f24d1db4e2b15f38843e))
ROM_END

#define rom_rtpc010 rom_ibm6151
#define rom_rtpc015 rom_ibm6151
#define rom_rtpc020 rom_ibm6150
#define rom_rtpc025 rom_ibm6150
#define rom_rtpca25 rom_ibm6150

}

/*   YEAR   NAME      PARENT  COMPAT  MACHINE   INPUT  CLASS        INIT          COMPANY                             FULLNAME               FLAGS */
COMP(1986,  rtpc010,  0,      0,      ibm6151,  0,     rtpc_state,  init_common,  "International Business Machines",  "IBM RT PC Model 010", MACHINE_NOT_WORKING)
COMP(1986,  rtpc015,  0,      0,      ibm6151,  0,     rtpc_state,  init_common,  "International Business Machines",  "IBM RT PC Model 015", MACHINE_NOT_WORKING)
COMP(1986,  rtpc020,  0,      0,      ibm6150,  0,     rtpc_state,  init_common,  "International Business Machines",  "IBM RT PC Model 020", MACHINE_NOT_WORKING)
COMP(1986,  rtpc025,  0,      0,      ibm6150,  0,     rtpc_state,  init_common,  "International Business Machines",  "IBM RT PC Model 025", MACHINE_NOT_WORKING)
COMP(1986,  rtpca25,  0,      0,      ibm6150,  0,     rtpc_state,  init_common,  "International Business Machines",  "IBM RT PC Model A25", MACHINE_NOT_WORKING)
