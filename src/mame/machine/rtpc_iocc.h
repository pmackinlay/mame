// license:BSD-3-Clause
// copyright-holders:Patrick Mackinlay

#ifndef MAME_MACHINE_RTPC_IOCC_H
#define MAME_MACHINE_RTPC_IOCC_H

#pragma once

class rtpc_iocc_device
	: public device_t
	, public device_memory_interface
{
public:
	rtpc_iocc_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock);

	template <unsigned Space> u32 processor_r(offs_t offset, u32 mem_mask) { return space(Space).read_dword(offset << 2, mem_mask); }
	template <unsigned Space> void processor_w(offs_t offset, u32 data, u32 mem_mask) { space(Space).write_dword(offset << 2, data, mem_mask); }

	u32 mem_r(offs_t offset, u32 mem_mask) { return space(0).read_dword(offset << 2, mem_mask); }
	void mem_w(offs_t offset, u32 data, u32 mem_mask) { space(0).write_dword(offset << 2, data, mem_mask); }

	// TODO: check i/o map access authority
	u32 io_r(offs_t offset, u32 mem_mask);
	void io_w(offs_t offset, u32 data, u32 mem_mask);

protected:
	// device_t overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// device_memory_interface overrides
	virtual space_config_vector memory_space_config() const override;
	//virtual bool memory_translate(int spacenum, int intention, offs_t &address) override;

private:
	address_space_config m_mem_config;
	address_space_config m_io_config;
};

DECLARE_DEVICE_TYPE(RTPC_IOCC, rtpc_iocc_device)

#endif // MAME_MACHINE_RTPC_IOCC_H
