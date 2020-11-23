// license:BSD-3-Clause
// copyright-holders:Patrick Mackinlay

/*
 * IBM RT PC I/O Channel Controller
 * 
 * Sources:
 *
 * TODO
 *   - everything
 */

// processor channel address: 1111xxxx xxxxxxxx xxxxxxxx xxxxxxxx
//                            chan select
// only responds to f0 and f4 (mem and io space)
//
// first byte of incoming 32-bit address is i/o channel select 1111xxxx
// remaining 3 bytes make 24-bit address
// iocc always does word

#include "emu.h"
#include "rtpc_iocc.h"

#define VERBOSE 1
#include "logmacro.h"

DEFINE_DEVICE_TYPE(RTPC_IOCC, rtpc_iocc_device, "rtpc_iocc", "RT PC I/O Channel Controller")

rtpc_iocc_device::rtpc_iocc_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock)
	: device_t(mconfig, RTPC_IOCC, tag, owner, clock)
	, device_memory_interface(mconfig, *this)
	, m_mem_config("memory", ENDIANNESS_BIG, 16, 24, 0)
	, m_io_config("io", ENDIANNESS_BIG, 16, 24, 0)
{
}

device_memory_interface::space_config_vector rtpc_iocc_device::memory_space_config() const
{
	return space_config_vector{
		std::make_pair(AS_PROGRAM, &m_mem_config),
		std::make_pair(AS_IO, &m_io_config)
	};
}

void rtpc_iocc_device::device_start()
{
}

void rtpc_iocc_device::device_reset()
{
}

u32 rtpc_iocc_device::io_r(offs_t offset, u32 mem_mask)
{
	offs_t const address = offset << 2;
	bool const word = BIT(address, 16);
	u32 data = 0;

	switch (mem_mask)
	{
	case 0x0000ffffU:
		if (!word)
		{
			// word read from byte handler
			data = (data << 8) | space(AS_IO).read_byte(address + 2);
			data = (data << 8) | space(AS_IO).read_byte(address + 2);
		}
		else
			data = space(AS_IO).read_word(address + 2);
		break;
	case 0xffff0000U:
		if (!word)
		{
			// word read from byte handler
			data = (data << 8) | space(AS_IO).read_byte(address + 0);
			data = (data << 8) | space(AS_IO).read_byte(address + 0);
		}
		else
			data = space(AS_IO).read_word(address + 0);

		data <<= 16;
		break;
	case 0xffffffffU:
		if (word)
		{
			// dword read from word handler
			data = (data << 16) | space(AS_IO).read_word(address + 2);
			data = (data << 16) | space(AS_IO).read_word(address + 2);
		}
		else
		{
			// dword read from byte handler
			data = (data << 8) | space(AS_IO).read_byte(address + 3);
			data = (data << 8) | space(AS_IO).read_byte(address + 3);
			data = (data << 8) | space(AS_IO).read_byte(address + 3);
			data = (data << 8) | space(AS_IO).read_byte(address + 3);
		}
		break;
	default:
		data = space(AS_IO).read_dword(address, mem_mask);
		break;
	}

	return data;
}

void rtpc_iocc_device::io_w(offs_t offset, u32 data, u32 mem_mask)
{
	offs_t const address = offset << 2;
	bool const word = BIT(address, 16);

	switch (mem_mask)
	{
	case 0x0000ffffU:
		if (!word)
		{
			LOG("word to byte offset 0x%x data 0x%x mask 0x%x\n", offset, data, mem_mask);
			// word write to byte handler
			space(AS_IO).write_byte(address + 2, u8(data >> 8));
			space(AS_IO).write_byte(address + 2, u8(data >> 0));
		}
		else
			space(AS_IO).write_word(address + 2, u16(data));
		break;
	case 0xffff0000U:
		if (!word)
		{
			LOG("word to byte offset 0x%x data 0x%x mask 0x%x\n", offset, data, mem_mask);
			// word write to byte handler
			space(AS_IO).write_byte(address + 0, u8(data >> 24));
			space(AS_IO).write_byte(address + 0, u8(data >> 16));
			return;
		}
		else
			space(AS_IO).write_word(address + 0, u16(data >> 16));
		break;
	case 0xffffffffU:
		if (word)
		{
			LOG("dword to word offset 0x%x data 0x%x mask 0x%x\n", offset, data, mem_mask);
			// dword write to word handler
			space(AS_IO).write_word(address + 2, u16(data >> 16));
			space(AS_IO).write_word(address + 2, u16(data >> 0));
		}
		else
		{
			LOG("dword to byte offset 0x%x data 0x%x mask 0x%x\n", offset, data, mem_mask);
			// dword write to byte handler
			space(AS_IO).write_byte(address + 3, u8(data >> 24));
			space(AS_IO).write_byte(address + 3, u8(data >> 16));
			space(AS_IO).write_byte(address + 3, u8(data >> 8));
			space(AS_IO).write_byte(address + 3, u8(data >> 0));
			return;
		}
		break;
	default:
		space(AS_IO).write_dword(address, data, mem_mask);
		break;
	}
}
