// license:BSD-3-Clause
// copyright-holders:Patrick Mackinlay

/*
 * SCN2681 Dual asynchronous receiver/transmitter (DUART).
 *
 * Sources:
 *  -
 *
 * TODO:
 *  - counter/timer
 *  - flow control/output port
 *  - variants: scc2681n24, scc2681n28, mc68681, scc2691, mc68306, mc68307, 28c94, 68340, xr68c681
 *  - txrdy isn't set until after start bit time
 */

#include "emu.h"
#include "scn2681.h"

enum mr1_mask : u8
{
	MR1_BPC   = 0x03, // bits per character (0=5, 1=6, 2=7, 3=8)
	MR1_PTYPE = 0x04, // parity type (0=even, 1=odd)
	MR1_PMODE = 0x18, // parity mode (0=with parity, 1=force parity, 2=no parity, 3=multidrop)
	MR1_EMODE = 0x20, // error mode (0=char, 1=block)
	MR1_RXINT = 0x40, // rxint select (0=rxrdy, 1=ffull)
	MR1_RXRTS = 0x80, // rxrts control (1=yes)
};
enum pmode_mask : unsigned
{
	PMODE_WITH      = 0,
	PMODE_FORCE     = 1,
	PMODE_NONE      = 2,
	PMODE_MULTIDROP = 3,
};
enum mr2_mask : u8
{
	MR2_STOP  = 0x0f, // stop bit length
	MR2_TXCTS = 0x10, // cts enable tx (1=yes)
	MR2_TXRTS = 0x20, // txrts control (1=yes)
	MR2_CMODE = 0xc0, // channel mode (0=normal, 1=auto-echo, 2=local loop, 3=remote loop)
};
enum sr_mask : u16
{
	SR_RXRDY = 0x01, // receiver ready
	SR_FFULL = 0x02, // fifo full
	SR_TXRDY = 0x04, // transmitter ready
	SR_TXEMT = 0x08, // transmitter empty
	SR_OERR  = 0x10, // overrun error
	SR_PERR  = 0x20, // parity error
	SR_FERR  = 0x40, // framing error
	SR_RBRK  = 0x80, // received break
};
enum cr_mask : u8
{
	CR_ERX = 0x01, // enable rx
	CR_DRX = 0x02, // disable rx
	CR_ETX = 0x04, // enable tx
	CR_DTX = 0x08, // disable tx
	CR_CMD = 0x70, // miscellaneous commands
};
enum ipcr_mask : u8
{
	IPCR_CUR = 0x0f, // ip0-ip3 current state
	IPCR_CHG = 0xf0, // ip0-ip3 state change
};
enum acr_mask : u8
{
	ACR_DIP0 = 0x01, // delta ip0 interrupt
	ACR_DIP1 = 0x02, // delta ip1 interrupt
	ACR_DIP2 = 0x04, // delta ip2 interrupt
	ACR_DIP3 = 0x08, // delta ip3 interrupt
	ACR_CTMS = 0x70, // counter/timer mode and source
	ACR_BRGS = 0x80, // brg set select
};
enum isr_mask : u8
{
	ISR_TXRDYA = 0x01, // channel a transmitter ready
	ISR_RXINTA = 0x02, // channel a receiver ready or fifo full
	ISR_DBRKA  = 0x04, // channel a break change
	ISR_CRDY   = 0x08, // counter ready
	ISR_TXRDYB = 0x10, // channel b transmitter ready
	ISR_RXINTB = 0x20, // channel b receiver ready or fifo full
	ISR_DBRKB  = 0x40, // channel b break change
	ISR_DIP    = 0x80, // input port change
};
enum ct_mask : u8
{
	C_IP2    = 0x0, // counter IP2
	C_TXCA   = 0x1, // counter TxCA
	C_TXCB   = 0x2, // counter TxCB
	C_CLKD16 = 0x3, // counter clock/16
	T_IP2    = 0x4, // timer IP2
	T_IP2D16 = 0x5, // timer IP2/16
	T_CLK    = 0x6, // timer CLK
	T_CLKD16 = 0x7, // timer CLK/16
};

enum rx_state : u8
{
	RXS_DISABLED = 0,
	RXS_SEARCH   = 1,
	RXS_START    = 2,
	RXS_DATA1    = 3,
	RXS_DATA2    = 4,
	RXS_DATA3    = 5,
	RXS_DATA4    = 6,
	RXS_DATA5    = 7,
	RXS_DATA6    = 8,
	RXS_DATA7    = 9,
	RXS_DATA8    = 10,
	RXS_PARITY   = 11,
	RXS_STOP     = 12,
};
enum tx_state : u8
{
	TXS_DISABLED = 0,
	TXS_START    = 1,
	TXS_DATA1    = 2,
	TXS_DATA2    = 3,
	TXS_DATA3    = 4,
	TXS_DATA4    = 5,
	TXS_DATA5    = 6,
	TXS_DATA6    = 7,
	TXS_DATA7    = 8,
	TXS_DATA8    = 9,
	TXS_PARITY   = 10,
	TXS_STOP     = 11,
	TXS_DONE     = 12,
};

// 16x clock divisors for baud rate generator (published rates assume 3.6864MHz clock)
static u16 const brg[2][13] =
{
	//   50    110  134.5    200    300    600   1200   1050   2400   4800   7200   9600  38400
	{  4608,  2095,  1713,  1152,   768,   384,   192,   220,    96,    48,    32,    24,     6 },
	//   75    110  134.5    150    300    600   1200   2000   2400   4800   1800   9600  19200
	{  3072,  2095,  1713,  1536,   768,   384,   192,   115,    96,    48,   128,    24,    12 }
};

#define LOG_CMD     (1U << 1)
#define LOG_MODE    (1U << 2)
#define LOG_REGR    (1U << 3)
#define LOG_REGW    (1U << 4)
#define LOG_PORT    (1U << 5)
#define LOG_INT     (1U << 6)
#define LOG_RXS     (1U << 10)
#define LOG_TXS     (1U << 11)

#define VERBOSE (LOG_GENERAL|LOG_CMD|LOG_MODE|LOG_REGR|LOG_REGW|LOG_INT|LOG_TXS)
#include "logmacro.h"

DEFINE_DEVICE_TYPE(SCN2681N40, scn2681n40_device, "scn2681n40", "Dual asynchronous receiver/transmitter")

scn2681n40_device::scn2681n40_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock)
	: device_t(mconfig, SCN2681N40, tag, owner, clock)
	, m_intrn(*this)
	, m_txd(*this)
	, m_op(*this)
	, m_ch{}
{
}

void scn2681n40_device::device_start()
{
	save_item(STRUCT_MEMBER(m_ch, mr));
	save_item(STRUCT_MEMBER(m_ch, mrp));
	save_item(STRUCT_MEMBER(m_ch, sr));
	save_item(STRUCT_MEMBER(m_ch, csr));

	save_item(STRUCT_MEMBER(m_ch, rhr));
	save_item(STRUCT_MEMBER(m_ch, rsr));
	save_item(STRUCT_MEMBER(m_ch, rxd));
	save_item(STRUCT_MEMBER(m_ch, rpf));
	save_item(STRUCT_MEMBER(m_ch, rhc));
	save_item(STRUCT_MEMBER(m_ch, rxs));
	save_item(STRUCT_MEMBER(m_ch, rxp));

	save_item(STRUCT_MEMBER(m_ch, thr));
	save_item(STRUCT_MEMBER(m_ch, tsr));
	save_item(STRUCT_MEMBER(m_ch, tpf));
	save_item(STRUCT_MEMBER(m_ch, txs));
	save_item(STRUCT_MEMBER(m_ch, txp));

	save_item(NAME(m_ipcr));
	save_item(NAME(m_acr));
	save_item(NAME(m_isr));
	save_item(NAME(m_imr));
	save_item(NAME(m_ctr));
	save_item(NAME(m_ipr));
	save_item(NAME(m_opcr));
	save_item(NAME(m_opr));

	save_item(NAME(m_int_state));

	m_ct = timer_alloc(FUNC(scn2681n40_device::clock_ct), this);

	m_ch[0].rxc = timer_alloc(FUNC(scn2681n40_device::clock_rx<0>), this);
	m_ch[1].rxc = timer_alloc(FUNC(scn2681n40_device::clock_rx<1>), this);
	m_ch[0].txc = timer_alloc(FUNC(scn2681n40_device::clock_tx<0>), this);
	m_ch[1].txc = timer_alloc(FUNC(scn2681n40_device::clock_tx<1>), this);

	for (channel &ch : m_ch)
		ch.rxd = true;

	m_ipcr = 0;
	m_acr = 0;
	m_ctr = 0;
	m_ipr = 0;

	m_int_state = false;
}

void scn2681n40_device::device_reset()
{
	m_ct->adjust(attotime::never);

	for (channel &ch : m_ch)
	{
		ch.rxc->adjust(attotime::never);
		ch.txc->adjust(attotime::never);

		ch.rhr[0] = 0;
		ch.rhr[1] = 0;
		ch.rhr[2] = 0;
		ch.rhc = 0;

		ch.sr = 0;
		ch.mrp = false;

		ch.rxs = RXS_DISABLED;
		ch.txs = TXS_DISABLED;
	}

	m_isr = 0;
	m_imr = 0;
	m_opcr = 0;
	m_opr = 0;

	update_op();

	// TODO: counter/timer in timer mode

	// stop counter/timer
	// TxDA TxDB output high
	// clear test mode

	update_int();

	m_txd[0](1);
	m_txd[1](1);
}

void scn2681n40_device::map(address_map &map)
{
	map(0x0, 0x0).rw(FUNC(scn2681n40_device::mr_r<0>), FUNC(scn2681n40_device::mr_w<0>));
	map(0x1, 0x1).rw(FUNC(scn2681n40_device::sr_r<0>), FUNC(scn2681n40_device::csr_w<0>));
	map(0x2, 0x2).rw(FUNC(scn2681n40_device::brgt_r), FUNC(scn2681n40_device::cr_w<0>));
	map(0x3, 0x3).rw(FUNC(scn2681n40_device::rhr_r<0>), FUNC(scn2681n40_device::thr_w<0>));
	map(0x4, 0x4).rw(FUNC(scn2681n40_device::ipcr_r), FUNC(scn2681n40_device::acr_w));
	map(0x5, 0x5).rw(FUNC(scn2681n40_device::isr_r), FUNC(scn2681n40_device::imr_w));
	map(0x6, 0x6).rw(FUNC(scn2681n40_device::ctu_r), FUNC(scn2681n40_device::ctu_w));
	map(0x7, 0x7).rw(FUNC(scn2681n40_device::ctl_r), FUNC(scn2681n40_device::ctl_w));
	map(0x8, 0x8).rw(FUNC(scn2681n40_device::mr_r<1>), FUNC(scn2681n40_device::mr_w<1>));
	map(0x9, 0x9).rw(FUNC(scn2681n40_device::sr_r<1>), FUNC(scn2681n40_device::csr_w<1>));
	map(0xa, 0xa).rw(FUNC(scn2681n40_device::clkt_r), FUNC(scn2681n40_device::cr_w<1>));
	map(0xb, 0xb).rw(FUNC(scn2681n40_device::rhr_r<1>), FUNC(scn2681n40_device::thr_w<1>));

	map(0xd, 0xd).rw(FUNC(scn2681n40_device::ip_r), FUNC(scn2681n40_device::opcr_w));
	map(0xe, 0xe).rw(FUNC(scn2681n40_device::scc_r<1>), FUNC(scn2681n40_device::opbc_w<1>));
	map(0xf, 0xf).rw(FUNC(scn2681n40_device::scc_r<0>), FUNC(scn2681n40_device::opbc_w<0>));
}

// mode register read
template <unsigned C> u8 scn2681n40_device::mr_r()
{
	channel &ch = m_ch[C];

	u8 const data = ch.mr[ch.mrp];
	LOGMASKED(LOG_REGR, "mr%u%c_r 0x%02x (%s)\n", ch.mrp + 1U, 'a' + C, data, machine().describe_context());

	ch.mrp = true;

	return data;
}

// mode register write
template <unsigned C> void scn2681n40_device::mr_w(u8 data)
{
	channel &ch = m_ch[C];
	LOGMASKED(LOG_REGW, "mr%u%c_w 0x%02x (%s)\n", ch.mrp + 1U, 'a' + C, data, machine().describe_context());

	if (VERBOSE & LOG_MODE)
	{
		static const char *const parity[] = { "with even parity", "with odd parity", "force even parity", "force odd parity", "no parity", "no parity", "multidrop data", "multidrop address" };
		static const char *const cmode[] = { "normal", "auto-echo", "local loop", "remote loop" };

		if (!ch.mrp)
			LOGMASKED(LOG_MODE, "mr1%c bpc=%u, %s, error=%s, rxint=%s, rxrts=%u\n", 'a' + C,
				BIT(data, 0, 2) + 5, parity[BIT(data, 2, 3)], (data & MR1_EMODE) ? "block" : "char",
				(data & MR1_RXINT) ? "FFULL" : "RxRDY", BIT(data, 7));
		else
		{
			unsigned stop_length = (data & MR2_STOP) + 9;
			if (!(ch.mr[0] & MR1_BPC) || BIT(data, 3))
				stop_length += 8;

			LOGMASKED(LOG_MODE, "mr2%c stop=%g, txcts=%u, txrts=%u, mode=%s\n", 'a' + C,
				stop_length / 16.0, BIT(data, 4), BIT(data, 5), cmode[BIT(data, 6, 2)]);
		}

	}

	ch.mr[ch.mrp] = data;
	ch.mrp = true;
}

// status register read
template <unsigned C> u8 scn2681n40_device::sr_r()
{
	u8 const data = m_ch[C].sr;
	//LOGMASKED(LOG_REGR, "sr%c_r 0x%02x (%s)\n", 'a' + C, data, machine().describe_context());

	return data;
}

// clock select register write
template <unsigned C> void scn2681n40_device::csr_w(u8 data)
{
	LOGMASKED(LOG_REGW, "csr%c_w 0x%02x (%s)\n", 'a' + C, data, machine().describe_context());

	if (m_ch[C].csr != data)
	{
		m_ch[C].csr = data;

		log_brg(m_acr, 1U << C);
	}
}

// brg test read
u8 scn2681n40_device::brgt_r()
{
	u8 const data = 0;
	LOGMASKED(LOG_REGR, "brgt_r 0x%02x (%s)\n", data, machine().describe_context());

	return data;
}

// command register write
template <unsigned C> void scn2681n40_device::cr_w(u8 data)
{
	channel &ch = m_ch[C];

	LOGMASKED(LOG_REGW, "cr%c_w 0x%02x (%s)\n", 'a' + C, data, machine().describe_context());

	if ((data & CR_ERX) && (ch.rxs == RXS_DISABLED))
	{
		LOGMASKED(LOG_CMD, "channel %c enable receiver\n", 'a' + C);
		ch.rxs = RXS_START;

		if (BIT(ch.csr, 4, 4) < 13)
		{
			unsigned const divisor = brg[BIT(m_acr, 7)][BIT(ch.csr, 4, 4)];

			if (clock())
			{
				LOGMASKED(LOG_CMD, "channel %c receiver started %g baud\n", 'a' + C, attotime::from_ticks(divisor * 16, clock()).as_hz());

				ch.rxc->adjust(attotime::zero, 1, attotime::from_ticks(divisor, clock()));
			}
			else
				LOGMASKED(LOG_CMD, "channel %c receiver started with external clock divisor %u\n", 'a' + C, divisor);
		}
	}
	if ((data & CR_DRX) && (ch.rxs != RXS_DISABLED))
	{
		LOGMASKED(LOG_CMD, "channel %c disable receiver\n", 'a' + C);
		ch.rxs = RXS_DISABLED;

		ch.rxc->adjust(attotime::never);
	}
	if ((data & CR_ETX) && (ch.txs == TXS_DISABLED))
	{
		LOGMASKED(LOG_CMD, "channel %c enable transmitter\n", 'a' + C);
		ch.txs = TXS_START;
		ch.txp = 0;
		ch.sr |= SR_TXEMT | SR_TXRDY;
		m_isr |= (C ? ISR_TXRDYB : ISR_TXRDYA);

		if (BIT(ch.csr, 0, 4) < 13)
		{
			unsigned const divisor = brg[BIT(m_acr, 7)][BIT(ch.csr, 0, 4)];

			if (clock())
			{
				LOGMASKED(LOG_CMD, "channel %c transmitter started %g baud\n", 'a' + C, attotime::from_ticks(divisor * 16, clock()).as_hz());

				ch.txc->adjust(attotime::zero, 0, attotime::from_ticks(divisor, clock()));
			}
			else
				LOGMASKED(LOG_CMD, "channel %c transmitter started with external clock divisor %u\n", 'a' + C, divisor);
		}
	}
	if ((data & CR_DTX) && (ch.txs == TXS_DISABLED))
	{
		LOGMASKED(LOG_CMD, "channel %c disable transmitter\n", 'a' + C);
		// FIXME: defer until transmission complete
		ch.txs = TXS_DISABLED;
		ch.sr &= ~(SR_TXEMT | SR_TXRDY);
		m_isr &= ~(C ? ISR_TXRDYB : ISR_TXRDYA);

		ch.txc->adjust(attotime::never);
	}
	switch (BIT(data, 4, 3))
	{
	case 0:
		// no command
		break;
	case 1:
		// reset mr pointer
		LOGMASKED(LOG_CMD, "channel %c reset mr pointer\n", 'a' + C);
		ch.mrp = false;
		break;
	case 2:
		// reset receiver
		LOGMASKED(LOG_CMD, "channel %c reset receiver\n", 'a' + C);
		ch.rxs = RXS_DISABLED;
		ch.rsr = 0;
		ch.rhc = 0;
		ch.sr &= ~(SR_RBRK | SR_FERR | SR_PERR | SR_OERR | SR_FFULL | SR_RXRDY);
		m_isr &= ~(C ? ISR_RXINTB : ISR_RXINTA);

		ch.rxc->adjust(attotime::never);
		break;
	case 3:
		// reset transmitter
		LOGMASKED(LOG_CMD, "channel %c reset transmitter\n", 'a' + C);
		ch.txs = TXS_DISABLED;
		ch.sr &= ~(SR_TXEMT | SR_TXRDY);
		m_isr &= ~(C ? ISR_TXRDYB : ISR_TXRDYA);

		ch.txc->adjust(attotime::never);
		break;
	case 4:
		// reset error status
		LOGMASKED(LOG_CMD, "channel %c reset error status\n", 'a' + C);
		ch.sr &= ~(SR_RBRK | SR_FERR | SR_PERR | SR_OERR);
		break;
	case 5:
		// reset break change interrupt
		LOGMASKED(LOG_CMD, "channel %c reset break change interrupt\n", 'a' + C);
		m_isr &= ~(C ? ISR_DBRKB : ISR_DBRKA);
		break;
	case 6:
		// start break
		LOGMASKED(LOG_CMD, "channel %c start break\n", 'a' + C);
		break;
	case 7:
		// stop break
		LOGMASKED(LOG_CMD, "channel %c stop break\n", 'a' + C);
		break;
	}

	update_int();
}

// auxiliary control register write
void scn2681n40_device::acr_w(u8 data)
{
	LOGMASKED(LOG_REGW, "acr_w 0x%02x (%s)\n", data, machine().describe_context());

	if (((data ^ m_acr) & ACR_CTMS) && (VERBOSE & LOG_MODE))
	{
		static const char *const source[] = { "IP2", "TxCA (1x)", "TxCB (1x)", "CLK/16", "IP2", "IP2/16", "CLK", "CLK/16" };

		LOGMASKED(LOG_MODE, "%s mode source=%s\n", BIT(data, 6) ? "timer" : "counter", source[BIT(data, 4, 3)]);
	}

	if ((data ^ m_acr) & ACR_BRGS)
		log_brg(data, ~0U);

	m_acr = data;
}

// counter/timer upper read
u8 scn2681n40_device::ctu_r()
{
	u8 const data = m_ctr >> 8;
	LOGMASKED(LOG_REGR, "ctu_r 0x%02x (%s)\n", data, machine().describe_context());

	return data;
}

// counter/timer upper write
void scn2681n40_device::ctu_w(u8 data)
{
	LOGMASKED(LOG_REGW, "ctu_w 0x%02x (%s)\n", data, machine().describe_context());

	m_ctr = (u16(data) << 8) | (m_ctr & 0x00ffU);
}

// counter/timer lower read
u8 scn2681n40_device::ctl_r()
{
	u8 const data = m_ctr;
	LOGMASKED(LOG_REGR, "ctl_r 0x%02x (%s)\n", data, machine().describe_context());

	return data;
}

// counter/timer lower write
void scn2681n40_device::ctl_w(u8 data)
{
	LOGMASKED(LOG_REGW, "ctl_w 0x%02x (%s)\n", data, machine().describe_context());

	m_ctr = (m_ctr & 0xff00U) | data;
}

// 1x/16x test read
u8 scn2681n40_device::clkt_r()
{
	u8 const data = 0;
	LOGMASKED(LOG_REGR, "clkt_r 0x%02x (%s)\n", data, machine().describe_context());

	return data;
}

// start/stop counter command
template <unsigned N> u8 scn2681n40_device::scc_r()
{
	LOGMASKED(LOG_REGR, "scc_r counter %s (%s)\n", N ? "stop" : "start", machine().describe_context());

	return 0;
}

// clock counter/timer
void scn2681n40_device::clock_ct(int param)
{
}

// receive holding register read
template <unsigned C> u8 scn2681n40_device::rhr_r()
{
	channel &ch = m_ch[C];

	u8 const data = ch.rhr[0];
	LOGMASKED(LOG_REGR, "rhr%c_r 0x%02x (%s)\n", 'a' + C, data, machine().describe_context());

	if (ch.rhc)
	{
		// pop the fifo
		ch.rhr[0] = ch.rhr[1];
		ch.rhr[1] = ch.rhr[2];
		ch.rhr[2] = 0;
		ch.rhc--;
		ch.sr &= ~SR_FFULL;

		// clear rxrdy interrupt
		if (!(ch.mr[0] & MR1_RXINT))
		{
			m_isr &= ~(C ? ISR_RXINTB : ISR_RXINTA);

			update_int();
		}

		// update status
		if (!(ch.mr[0] & MR1_EMODE))
			ch.sr &= ~(SR_RBRK | SR_FERR | SR_PERR);

		if (ch.rhc)
		{
			ch.sr |= ch.rhr[0] >> 8;

			if (!(ch.mr[0] & MR1_RXINT))
			{
				m_isr |= (C ? ISR_RXINTB : ISR_RXINTA);

				update_int();
			}
		}
		else
			ch.sr &= ~SR_RXRDY;
	}
	else
		LOG("rx fifo underflow\n");

	return data;
}

// clock receiver
template <unsigned C> void scn2681n40_device::clock_rx(int param)
{
	/*
	 * Clock the receiver, sampling data at the middle of a bit-time, and changing
	 * state at the end of the bit-time. For 16x modes, these correspond to the 8th
	 * and 16th clock, while in the 1x mode, they correspond to the rising and
	 * falling edges of the clock respectively.
	 */
	channel &ch = m_ch[C];

	// the common case is a 16x clock
	if (BIT(ch.csr, 4, 4) != 0xf)
	{
		if (ch.rxp)
		{
			ch.rxp--;

			// only continue at half bit-times, or during start bit verification
			if (!(ch.rxp == 7) && !(ch.rxs == RXS_START && ch.rxp > 7))
				return;
		}
	}
	else
		// for 1x clock, use rising/falling edge to control state
		ch.rxp = param;

	if (ch.rxs > RXS_SEARCH)
		LOGMASKED(LOG_RXS, "channel %c rxs %u rxp %u\n", 'a' + C, ch.rxs, ch.rxp);

	switch (ch.rxs)
	{
	case RXS_DISABLED:
		break;
	case RXS_SEARCH:
		// search for start bit
		if (!ch.rxd)
		{
			LOGMASKED(LOG_RXS, "channel %c rxs start time %s\n", 'a' + C, machine().time().as_string());
			ch.rxs = RXS_START;
			ch.rxp = 14;
		}
		break;
	case RXS_START:
		// verify start bit
		if (ch.rxp)
		{
			if (ch.rxd)
			{
				LOGMASKED(LOG_RXS, "channel %c rxs start invalid time %s\n", 'a' + C, machine().time().as_string());
				ch.rxs = RXS_SEARCH;
			}
		}
		else
		{
			// initialise receive shift register and parity flag
			ch.rsr = 0;
			ch.rpf = BIT(ch.mr[0], 2);

			ch.rxs = RXS_DATA1;
		}
		break;
	case RXS_DATA1:
	case RXS_DATA2:
	case RXS_DATA3:
	case RXS_DATA4:
	case RXS_DATA5:
	case RXS_DATA6:
	case RXS_DATA7:
	case RXS_DATA8:
		if (ch.rxp)
		{
			// sample data bit
			LOGMASKED(LOG_RXS, "channel %c rxs data %u time %s\n", 'a' + C, ch.rxd, machine().time().as_string());
			if (ch.rxd)
				ch.rsr |= 1U << (ch.rxs - RXS_DATA1);

			// update parity flag
			if (!(ch.mr[0] & MR1_PMODE))
				ch.rpf += ch.rxd;
		}
		else
		{
			// check for last data bit
			if ((ch.rxs - RXS_DATA5) == (ch.mr[0] & MR1_BPC))
				ch.rxs = (BIT(ch.mr[0], 3, 2) == PMODE_NONE) ? RXS_STOP : RXS_PARITY;
			else
				ch.rxs++;
		}
		break;
	case RXS_PARITY:
		if (ch.rxp)
		{
			// sample and check parity bit
			LOGMASKED(LOG_RXS, "channel %c rxs parity %u expect %u time %s\n", 'a' + C, ch.rxd, ch.rpf, machine().time().as_string());
			if (ch.rxd != ch.rpf)
				ch.rsr |= SR_PERR << 8;
		}
		else
			ch.rxs++;
		break;
	case RXS_STOP:
		if (ch.rxp)
		{
			// check for stop bit
			// TODO: receive break
			// stop=0, data=0, parity=0 -> framing
			// if rxd remains low after stop=0, assume new start bit
			// if stop=0, data=0, parity=0 -> break
			// wait for high >= half-bit time before new search
			LOGMASKED(LOG_RXS, "channel %c rxs stop %u expect 1 time %s\n", 'a' + C, ch.rxd, machine().time().as_string());
			if (!ch.rxd)
				ch.rsr |= SR_FERR << 8;
		}
		else
		{
			// update status if rhr is empty
			if (ch.rhc == 0)
			{
				if (!(ch.mr[0] & MR1_EMODE))
					ch.sr &= ~(SR_RBRK | SR_FERR | SR_PERR);

				ch.sr |= ch.rsr >> 8;
			}

			// load receive holding register
			if (ch.rhc < 3)
			{
				LOGMASKED(LOG_RXS, "channel %c rxs rsr 0x%02x rhc %u flags 0x%02x\n", 'a' + C, u8(ch.rsr), ch.rhc, ch.rsr >> 8);
				ch.rhr[ch.rhc++] = ch.rsr;

				// update status
				ch.sr |= SR_RXRDY;
				if (ch.rhc == 3)
					ch.sr |= SR_FFULL;

				// update interrupt status
				if (!(ch.mr[0] & MR1_RXINT) || ch.rhc == 3)
					m_isr |= (C ? ISR_RXINTB : ISR_RXINTA);
			}
			else
			{
				// TODO: leave data in rsr until fifo is not full; mark overrun if new start bit received
				LOGMASKED(LOG_RXS, "channel %c rxs rhr overrun\n", 'a' + C);
			}

			ch.rxs = RXS_SEARCH;
			update_int();
		}
		break;
	}

	// reload prescaler
	if ((ch.rxs > RXS_SEARCH) && !ch.rxp)
		ch.rxp = 15;
}

// transmit holding register write
template <unsigned C> void scn2681n40_device::thr_w(u8 data)
{
	channel &ch = m_ch[C];

	LOGMASKED(LOG_REGW, "thr%c_w 0x%02x (%s)\n", 'a' + C, data, machine().describe_context());

	if (ch.txs == TXS_DISABLED || !(ch.sr & SR_TXRDY))
	{
		LOG("thr%c transmitter not %s\n", 'a' + C, (ch.txs == TXS_DISABLED) ? "ready" : "enabled");

		return;
	}

	ch.thr = data;
	ch.sr &= ~(SR_TXEMT | SR_TXRDY);
	m_isr &= ~(C ? ISR_TXRDYB : ISR_TXRDYA);

	update_int();
}

// clock transmitter
template <unsigned C> void scn2681n40_device::clock_tx(int param)
{
	/*
	 * Output at bit-time start, change state at bit-time end.
	 */
	channel &ch = m_ch[C];

	if (ch.txs > TXS_START)
	{
		//LOGMASKED(LOG_TXS, "channel %c txs %u txp %u\n", 'a' + C, ch.txs, ch.txp);
		if (ch.txp)
		{
			// deal with 1x clock
			if (BIT(ch.csr, 0, 4) == 0xf)
				ch.txp -= 16;
			else
				ch.txp--;

			return;
		}
	}

	switch (ch.txs)
	{
	case TXS_DISABLED:
		break;
	case TXS_START:
		if (!(ch.sr & SR_TXRDY))
		{
			// load transmit shift register and parity flag
			ch.tsr = ch.thr;
			ch.tpf = BIT(ch.mr[0], 2);

			// update status
			ch.sr |= SR_TXRDY;
			m_isr |= (C ? ISR_TXRDYB : ISR_TXRDYA);

			// output start bit
			LOGMASKED(LOG_TXS, "channel %c txs start time %s\n", 'a' + C, machine().time().as_string());
			m_txd[C](0);

			ch.txs = TXS_DATA1;
			update_int();
		}
		else
			ch.sr |= SR_TXEMT;
		break;
	case TXS_DATA1:
	case TXS_DATA2:
	case TXS_DATA3:
	case TXS_DATA4:
	case TXS_DATA5:
	case TXS_DATA6:
	case TXS_DATA7:
	case TXS_DATA8:
		// output data bit
		LOGMASKED(LOG_TXS, "channel %c txs data %u time %s\n", 'a' + C, BIT(ch.tsr, ch.txs - TXS_DATA1), machine().time().as_string());
		m_txd[C](BIT(ch.tsr, ch.txs - TXS_DATA1));

		// update parity flag
		if (!(ch.mr[0] & MR1_PMODE))
			ch.tpf += BIT(ch.tsr, ch.txs - TXS_DATA1);

		// check for last data bit
		if ((ch.txs - TXS_DATA5) == (ch.mr[0] & MR1_BPC))
			ch.txs = (BIT(ch.mr[0], 3, 2) == PMODE_NONE) ? TXS_STOP : TXS_PARITY;
		else
			ch.txs++;
		break;
	case TXS_PARITY:
		// output parity/multidrop bit
		LOGMASKED(LOG_TXS, "channel %c txs parity %u time %s\n", 'a' + C, ch.tpf, machine().time().as_string());
		m_txd[C](ch.tpf);

		ch.txs = TXS_STOP;
		break;
	case TXS_STOP:
		// output stop bit
		LOGMASKED(LOG_TXS, "channel %c txs stop time %s\n", 'a' + C, machine().time().as_string());
		m_txd[C](1);

		// compute stop bit length
		ch.txp = (ch.mr[1] & MR2_STOP) + 8;
		if (!(ch.mr[0] & MR1_BPC) || BIT(ch.mr[1], 3))
			ch.txp += 8;

		ch.txs = TXS_DONE;
		break;
	case TXS_DONE:
		LOGMASKED(LOG_TXS, "channel %c txs done time %s\n", 'a' + C, machine().time().as_string());
		ch.txs = TXS_START;
		break;
	}

	// reload prescaler
	if (!ch.txp)
		ch.txp = 15;
}

// interrupt status register read
u8 scn2681n40_device::isr_r()
{
	u8 const data = m_isr;
	LOGMASKED(LOG_REGR, "isr_r 0x%02x (%s)\n", data, machine().describe_context());

	return data;
}

// interrupt mask register write
void scn2681n40_device::imr_w(u8 data)
{
	LOGMASKED(LOG_REGW, "imr_w 0x%02x (%s)\n", data, machine().describe_context());

	m_imr = data;

	update_int();
}

// update interrupt line
void scn2681n40_device::update_int()
{
	bool const int_state = m_isr & m_imr;
	if (m_int_state != int_state)
	{
		LOGMASKED(LOG_INT, "interrupt %s\n", int_state ? "asserted" : "cleared");

		m_int_state = int_state;
		m_intrn(!m_int_state);
	}
}

// output port control register write
void scn2681n40_device::opcr_w(u8 data)
{
	LOGMASKED(LOG_REGW, "opcr_w 0x%02x (%s)\n", data, machine().describe_context());

	m_opcr = data;
}

// set/reset output port bits command
template <unsigned N> void scn2681n40_device::opbc_w(u8 data)
{
	LOGMASKED(LOG_REGW, "opbc_w 0x%02x %s (%s)\n", data, N ? "clr" : "set", machine().describe_context());

	if (N)
		m_opr |= data;
	else
		m_opr &= ~data;

	update_op(data);
}

void scn2681n40_device::update_op(u8 mask)
{
	enum op_mask : u8
	{
		OP0_RTSAN  = 0x01,
		OP1_RTSBN  = 0x02,
		OP2_CLKA   = 0x04,
		OP3_CLKB   = 0x08,
		OP4_RXRDYA = 0x10,
		OP5_RXRDYB = 0x20,
		OP6_TXRDYA = 0x40,
		OP7_TXRDYB = 0x80,
	};

	u8 data = m_opr;

	// opcr controls source for op2-op7
	// mr & cr control source for op0 and op1
	// when opr is source, data is inverted

	// output is /RTS, i.e. should be 0 when sending is permitted
	//
	// RTS is high when rx fifo is full and start bit sensed
	// op0 = nand(RTS, opr0)
	// assume opr0 = 1 (would normally assert /RTS)
	//
	// RTS OPR0  /RTS
	//  0   0    1
	//  1   0    1   : receiver full
	//
	//  0   1    1
	//  1   1    0   : receiver full
	//
	//


	// TODO: check rx fifo full and start bit sensed
	if ((m_ch[0].mr[0] & MR1_RXRTS) && false)
		data &= ~OP0_RTSAN;

	// TODO: check rx fifo full and start bit sensed
	if ((m_ch[1].mr[0] & MR1_RXRTS) && false)
		data &= ~OP1_RTSBN;

	m_op(0, ~data, mask);
}

// input port read
u8 scn2681n40_device::ip_r()
{
	u8 const data = 0x80U | m_ipr;
	LOGMASKED(LOG_REGR, "ip_r 0x%02x (%s)\n", data, machine().describe_context());

	return data;
}

// input port change register read
u8 scn2681n40_device::ipcr_r()
{
	u8 const data = m_ipcr;
	LOGMASKED(LOG_REGR, "ipcr_r 0x%02x (%s)\n", data, machine().describe_context());

	m_ipcr &= ~IPCR_CHG;
	m_isr &= ~ISR_DIP;

	update_int();

	return data;
}

// input port write
template <unsigned N> void scn2681n40_device::ip_w(int state)
{
	LOGMASKED(LOG_PORT, "ip%u_w %u (%s)\n", N, state, machine().describe_context());

	// TODO: mc68681 does not have IP6, pin use is different

	unsigned const chn = N < 2 ? N : N > 4;
	channel &ch = m_ch[chn];

	// handle non-general purpose inputs first
	switch (N)
	{
	case 0: // ctsan
	case 1: // ctsbn
		break;
	case 2:
		// counter/timer source (assume rising edge)
		//if (!BIT(m_ipr, N) && state && (BIT(m_acr, 4, 3) == C_IP2 || BIT(m_acr, 4, 3) == T_IP2 || BIT(m_acr, 4, 3) == T_IP2D16))
		//  ; // TODO: ct clock
		break;
	case 3:
	case 5:
		// TxC, falling edge
		if (BIT(m_ipr, N) && !state && BIT(ch.csr, 0, 4) >= 0xe)
			ch.txc->adjust(attotime::zero, state);
		break;
	case 4:
	case 6:
		switch (BIT(ch.csr, 4, 4))
		{
		case 0xe:
			// RxC, 16x, rising edge
			if (!BIT(m_ipr, N) && state)
				ch.rxc->adjust(attotime::zero, state);
			break;
		case 0xf:
			// RxC, 1x, both edges
			if (BIT(m_ipr, N) != state)
				ch.rxc->adjust(attotime::zero, state);
			break;
		}
		break;
	}

	// update state
	if (state)
		m_ipr |= 1U << N;
	else
		m_ipr &= ~(1U << N);

	// report change
	u8 const change = (m_ipr ^ m_ipcr) & IPCR_CUR;
	if (change)
	{
		m_ipcr = (m_ipcr & IPCR_CHG) | (change << 4) | (m_ipr & IPCR_CUR);

		if ((m_ipcr >> 4) & m_acr)
			m_isr |= ISR_DIP;

		update_int();
	}
}

template void scn2681n40_device::ip_w<0>(int state);
template void scn2681n40_device::ip_w<1>(int state);
template void scn2681n40_device::ip_w<2>(int state);
template void scn2681n40_device::ip_w<3>(int state);
template void scn2681n40_device::ip_w<4>(int state);
template void scn2681n40_device::ip_w<5>(int state);
template void scn2681n40_device::ip_w<6>(int state);

void scn2681n40_device::log_brg(u8 acr, unsigned mask)
{
	if (VERBOSE & LOG_MODE)
	{
		static const char *const brg[2][16] = {
			{ "50", "110", "134.5", "200", "300", "600", "1200", "1050", "2400", "4800", "7200", "9600", "38400", "timer", "ip%u (16x)", "ip%u (1x)" },
			{ "75", "110", "134.5", "150", "300", "600", "1200", "2000", "2400", "4800", "1800", "9600", "19200", "timer", "ip%u (16x)", "ip%u (1x)" }
		};

		for (unsigned i = 0; i < std::size(m_ch); i++)
		{
			if (BIT(mask, i))
			{
				u8 const csr = m_ch[i].csr;

				LOGMASKED(LOG_MODE, "csr%c tx clock source=%s rx clock source=%s\n", 'a' + i,
					(BIT(csr, 0, 4) < 0xe) ? brg[BIT(acr, 7)][BIT(csr, 0, 4)] : util::string_format(brg[BIT(acr, 7)][BIT(csr, 0, 4)], i ? 5 : 3),
					(BIT(csr, 4, 4) < 0xe) ? brg[BIT(acr, 7)][BIT(csr, 4, 4)] : util::string_format(brg[BIT(acr, 7)][BIT(csr, 4, 4)], i ? 6 : 4));
			}
		}
	}
}
