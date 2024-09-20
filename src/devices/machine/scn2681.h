// license:BSD-3-Clause
// copyright-holders:Patrick Mackinlay

#ifndef MAME_MACHINE_SCN2681_H
#define MAME_MACHINE_SCN2681_H

#pragma once

class scn2681n40_device : public device_t
{
public:
	// output lines
	auto intrn() { return m_intrn.bind(); }
	template <unsigned C> auto txd() { return m_txd[C].bind(); }
	auto op() { return m_op.bind(); }

	scn2681n40_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock);

	// input lines
	void clk_w(int state) { assert(clock() == 0); }
	template <unsigned C> void rxd_w(int state) { m_ch[C].rxd = state; }
	template <unsigned N> void ip_w(int state);

	void map(address_map &map);

protected:
	virtual void device_start() override;
	virtual void device_reset() override;

	// register read handlers
	template <unsigned C> u8 mr_r();
	template <unsigned C> u8 sr_r();
	u8 brgt_r();
	template <unsigned C> u8 rhr_r();
	u8 ipcr_r();
	u8 isr_r();
	u8 ctu_r();
	u8 ctl_r();
	u8 clkt_r();
	u8 ip_r();
	template <unsigned N> u8 scc_r();

	// register write handlers
	template <unsigned C> void mr_w(u8 data);
	template <unsigned C> void csr_w(u8 data);
	template <unsigned C> void cr_w(u8 data);
	template <unsigned C> void thr_w(u8 data);
	void acr_w(u8 data);
	void imr_w(u8 data);
	void ctu_w(u8 data);
	void ctl_w(u8 data);
	void opcr_w(u8 data);
	template <unsigned N> void opbc_w(u8 data);

	// counter/timer and uart clock handlers
	void clock_ct(int param);
	template <unsigned C> void clock_rx(int param);
	template <unsigned C> void clock_tx(int param);

	// interrupts
	void update_int();
	void update_op(u8 mask = 0xffU);

	// logging
	void log_brg(u8 acr, unsigned mask);

private:
	devcb_write_line m_intrn;
	devcb_write_line::array<2> m_txd;
	devcb_write8 m_op;

	emu_timer *m_ct;

	struct channel
	{
		emu_timer *rxc;
		emu_timer *txc;

		u8 mr[2];   // mode register
		bool mrp;   // mode register pointer
		u8 sr;      // status register
		u8 csr;     // clock select register

		u16 rhr[3]; // receiver holding register
		u16 rsr;    // receiver shift register
		bool rxd;   // receiver line data
		bool rpf;   // receiver parity flag
		u8 rhc;     // receiver holding count
		u8 rxs;     // receiver state
		u8 rxp;     // receiver clock prescale

		u8 thr;     // transmitter holding register
		u8 tsr;     // transmitter shift register
		bool tpf;   // transmitter parity flag
		u8 txs;     // transmitter state
		u8 txp;     // transmitter clock prescale
	}
	m_ch[2];

	u8 m_ipcr; // input port change register
	u8 m_acr;  // auxiliary control register
	u8 m_isr;  // interrupt status register
	u8 m_imr;  // interrupt mask register
	u16 m_ctr; // counter/timer register
	u8 m_ipr;  // input port register
	u8 m_opcr; // output port configuration register
	u8 m_opr;  // output port register

	bool m_int_state;
};

DECLARE_DEVICE_TYPE(SCN2681N40, scn2681n40_device)

#endif // MAME_MACHINE_SCN2681_H
