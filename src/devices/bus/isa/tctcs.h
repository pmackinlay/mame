// license:BSD-3-Clause
// copyright-holders:Patrick Mackinlay

#ifndef MAME_BUS_ISA_TCTCS_H
#define MAME_BUS_ISA_TCTCS_H

#pragma once

#include "machine/scn2681.h"
#include "machine/pit8253.h"

#include "bus/isa/isa.h"

class isa16_tctcs_device
	: public device_t
	, public device_isa16_card_interface
{
public:
	isa16_tctcs_device(machine_config const &mconfig, char const *const tag, device_t *owner, u32 clock);

protected:
	virtual const tiny_rom_entry *device_rom_region() const override;
	virtual void device_add_mconfig(machine_config &config) override;
	virtual ioport_constructor device_input_ports() const override;
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	void map_isa(address_map &map);

	u8 ctl_r();
	void ctl_w(u8 data);
	void wdp_w(u8 data);
	void wdc_w(u8 data);

	void watchdog(int param);

	required_device<scn2681n40_device> m_duart;
	required_device<pit8254_device> m_pit;

	emu_timer *m_watchdog;

	bool m_installed;

	u8 m_ctlr;
	u8 m_ctlw;
};

DECLARE_DEVICE_TYPE(ISA16_TCTCS, isa16_tctcs_device)

#endif // MAME_BUS_ISA_TCTCS_H
