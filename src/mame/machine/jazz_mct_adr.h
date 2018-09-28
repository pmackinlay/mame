// license:BSD-3-Clause
// copyright-holders:Patrick Mackinlay

#ifndef MAME_MACHINE_JAZZ_MCT_ADR_H
#define MAME_MACHINE_JAZZ_MCT_ADR_H

#pragma once

#include "machine/ram.h"

class jazz_mct_adr_device : public device_t
{
public:
	jazz_mct_adr_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// configuration
	auto out_int0_cb() { return m_out_int0.bind(); }
	auto out_int1_cb() { return m_out_int1.bind(); }
	template <typename T> void set_ram(T &&tag) { m_ram.set_tag(std::forward<T>(tag)); }

	template <unsigned IRQ> DECLARE_WRITE_LINE_MEMBER(irq) { set_irq_line(IRQ, state); }
	virtual void map(address_map &map);

	u16 isr_r();
	u16 imr_r() { return m_imr; }
	void imr_w(u16 data) { m_imr = data; }

protected:
	// device_t overrides
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	void set_irq_line(int number, int state);

	TIMER_CALLBACK_MEMBER(irq_check);
	TIMER_CALLBACK_MEMBER(interval_timer);

	required_device<ram_device> m_ram;

	devcb_write_line m_out_int0;
	devcb_write_line m_out_int1;

	emu_timer *m_irq_check;
	emu_timer *m_interval_timer;

	u16 m_isr; // local bus interrupt source
	u16 m_imr; // local bus interrupt mask

	u32 m_config;

	u32 m_trans_tbl_base;
	u32 m_trans_tbl_limit;
	u32 m_ioc_maint;
	u32 m_ioc_physical_tag;
	u32 m_ioc_logical_tag;
	u32 m_ioc_byte_mask;
	u32 m_remote_speed[16];
	u32 m_dma_reg[32];
	u32 m_memory_refresh_rate;
};

// device type definition
DECLARE_DEVICE_TYPE(JAZZ_MCT_ADR, jazz_mct_adr_device)

#endif // MAME_MACHINE_JAZZ_MCT_ADR_H
