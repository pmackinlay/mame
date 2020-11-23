// license:BSD-3-Clause
// copyright-holders:Patrick Mackinlay

/*
 * IBM Rosetta MMU.
 *
 * Sources:
 *   - http://bitsavers.org/pdf/ibm/pc/rt/6489893_RT_PC_Technical_Reference_Volume_1_Nov85.pdf
 *
 * TODO:
 *   - everything
 */

/*
 * processor channel interface - connects processor channel to address translation
 * address translation - virtual -> real address, includes TLB (2-way set associative with 16 classes), includes reload on miss
 * memory control - rom, ram and ecc control, including refresh
 *
 * - tlb translate/invalidate
 * - ram/rom mapping
 * - control registers
 */

/*
 * address translation
 *   check translate bit on processor channel
 *   "virtual equal to real" mode exists (selected va bypass translate, protect and lock bit processing)
 *
 *   32 bit address from cpu by using 4 high-order bits to select segment register, then concatenating 12 bits with 28 bits of va
 *   translation then converts 40 bits to real
 *   protection on 2k or 4k pages (memory and fetch protect) per segment
 *   lock bits per page
 *   segment may be present/not present - only accept if present (otherwise pass to another mmu)
 *   isolation allows processor or i/o per segment
 *
 */

/*
 * After configuration is complete, the bus should have the iocc and bus "miss" handlers installed
 * Install ourselves according to "master" mode (maybe just segment 0?)
 * When segment registers are updated, install or restore handlers
 *
 */
#include "emu.h"
#include "rosetta.h"

#define LOG_GENERAL (1U << 0)
#define LOG_TLB     (1U << 1)
#define LOG_RELOAD  (1U << 2)
#define LOG_ECC     (1U << 3)

//#define VERBOSE (LOG_GENERAL|LOG_TLB|LOG_RELOAD|LOG_ECC)
#include "logmacro.h"

static char const *const control_names[] = { "IOBA", "MER", "MEAR", "TRAR", "TID", "TCR", "RAMS", "ROMS", "RMDR" };

static u8 const ecc_bits[] =
{
	0xa8, 0x68, 0xa4, 0x64, 0xa2, 0x62, 0xa1, 0x61,
	0x98, 0x58, 0x94, 0x54, 0x92, 0x52, 0x91, 0x51,
	0x8a, 0x89, 0x4a, 0x49, 0x2a, 0x29, 0x1a, 0x19,
	0x86, 0x85, 0x46, 0x45, 0x26, 0x25, 0x16, 0x15,
};

DEFINE_DEVICE_TYPE(ROSETTA, rosetta_device, "rosetta", "IBM Rosetta")

rosetta_device::rosetta_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock, mode initial_mode, ram_size ram)
	: device_t(mconfig, ROSETTA, tag, owner, clock)
	, device_memory_interface(mconfig, *this)
	, m_ram_size(ram)
	, m_bus(*this, finder_base::DUMMY_TAG, -1, 32)
	, m_out_trap(*this)
	, m_memory_config("memory", ENDIANNESS_BIG, 32, 24, 0)
	, m_control_config("control", ENDIANNESS_BIG, 32, 32, -2, address_map_constructor(FUNC(rosetta_device::internal_map), this))
	, m_rom(*this, finder_base::DUMMY_TAG)
	, m_mode(initial_mode)
{
}

void rosetta_device::device_start()
{
	m_out_trap.resolve_safe();

	for (u32 &control : m_control)
		control = 0;

	for (u32 &segment : m_segment)
		segment = 0;

	space(AS_PROGRAM).install_rom(0x000000, 0x00ffff, 0xff0000, m_rom);

	if (m_ram_size)
	{
		m_ram = std::make_unique<u32[]>(1U << (m_ram_size + 17));
		m_ecc = std::make_unique<u8[]>(1U << (m_ram_size + 17));
	}
	else
		fatalerror("invalid ram size configuration\n");

	save_pointer(NAME(m_ram), 1U << (m_ram_size + 17));

	space(AS_PROGRAM).cache(m_mem);
}

void rosetta_device::device_reset()
{
	m_mear_lock = UNLOCKED;
	m_rmdr_lock = false;

	m_out_trap(1);
}

device_memory_interface::space_config_vector rosetta_device::memory_space_config() const
{
	return space_config_vector{
		std::make_pair(AS_PROGRAM, &m_memory_config),
		std::make_pair(AS_IO, &m_control_config),
	};
}

void rosetta_device::internal_map(address_map &map)
{
	map(0x0000, 0x000f).rw(FUNC(rosetta_device::segment_r), FUNC(rosetta_device::segment_w));
	map(0x0010, 0x0018).rw(FUNC(rosetta_device::control_r), FUNC(rosetta_device::control_w));

	map(0x0020, 0x007f).rw(FUNC(rosetta_device::tlb_r), FUNC(rosetta_device::tlb_w));

	map(0x0080, 0x0080).w(FUNC(rosetta_device::tlb_inv_all));
	map(0x0081, 0x0081).w(FUNC(rosetta_device::tlb_inv_segment)); // invalidate TLB entries in specified segment
	map(0x0082, 0x0082).w(FUNC(rosetta_device::tlb_inv_address)); // invalidate TLB entry for specified effective address
	//map(0x0083, 0x0083); // load real address

	map(0x1000, 0x2fff).rw(FUNC(rosetta_device::rc_r), FUNC(rosetta_device::rc_w));
}

u32 rosetta_device::io_r(offs_t offset)
{
	u32 data = 0;

	// special case for i/o base address
	if (offset == 0x80'8000)
		data = m_control[IOBA];
	else if (u8(offset >> 16) == u8(m_control[IOBA]))
		data = space(AS_IO).read_dword(offset & 0x00'ffffU);
	else
		logerror("io_w unrecognized address 0x%x data 0x%x\n", offset, data);

	//LOG("io_r offset 0x%x data 0x%x (%s)\n", offset, data, machine().describe_context());

	return data;
}

void rosetta_device::io_w(offs_t offset, u32 data)
{
	// special case for i/o base address
	if (offset == 0x80'8000)
		m_control[IOBA] = data;
	else if (u8(offset >> 16) == u8(m_control[IOBA]))
		space(AS_IO).write_dword(offset & 0x00'ffffU, data);
	else
		logerror("io_w unrecognized address 0x%x data 0x%x\n", offset, data);
}

template <bool Translate> u32 rosetta_device::mem_r(offs_t offset, u32 mem_mask)
{
	if (m_mode == MASTER)
		m_mode = MASTER_ROM;

	u32 const address = offset << 2;

	u32 real_address = address & 0x00ff'ffffU;
	if (Translate)
	{
		translate_result const result = translate(address, false, false);

		if (result.status == SUCCESS)
		{
			//if (!machine().side_effects_disabled())
			//  LOG("mem_r 0x%08x translated 0x%08x\n", address, result.real_address);

			real_address = result.real_address;
		}
		// else -> exception
	}

	return m_mem.read_dword(real_address, mem_mask);
}

template <bool Translate> void rosetta_device::mem_w(offs_t offset, u32 data, u32 mem_mask)
{
	if (m_mode == MASTER)
		m_mode = MASTER_RAM;

	// main handler for cpu memory space access
	u32 const address = offset << 2;
	//u32 const segment = m_segment[address >> 28];

	u32 real_address = address & 0x00ff'ffffU;
	if (Translate)
	{
		translate_result const result = translate(address, true, false);

		if (result.status == SUCCESS)
		{
			//if (!machine().side_effects_disabled())
			//  LOG("mem_w 0x%08x translated 0x%08x\n", address, result.real_address);

			real_address = result.real_address;
		}
		// else -> exception
	}

	// deal with: virtual/real, segment present/absent, 2k/4k page
	m_mem.write_dword(real_address, data, mem_mask);
}

template u32 rosetta_device::mem_r<false>(offs_t offset, u32 mem_mask);
template u32 rosetta_device::mem_r<true>(offs_t offset, u32 mem_mask);
template void rosetta_device::mem_w<false>(offs_t offset, u32 data, u32 mem_mask);
template void rosetta_device::mem_w<true>(offs_t offset, u32 data, u32 mem_mask);

rosetta_device::translate_result rosetta_device::translate(u32 effective_address, bool store, bool io_device)
{
	// select segment
	u64 const segment = m_segment[effective_address >> 28];
	if (!(segment & SEGMENT_P))
		return { ABSENT, 0U };

	// segment protection
	// TODO: all translated requests, including v=r
	if ((io_device && (segment & SEGMENT_I)) || (!io_device && (segment & SEGMENT_R)))
	{
		// TODO: segment protection exception
		return { PROTECTION, 0U };
	}

	// compute virtual address
	u64 const virtual_address = ((segment & SEGMENT_ID) << 26) | (effective_address & 0x0fff'ffffU);

	// tlb operation
	unsigned const tlb_index = (effective_address >> ((m_control[TCR] & TCR_S) ? 12 : 11)) & 15;
	tlb_entry *tlb = m_tlb[tlb_index];

	unsigned tlb_set = 0;
	unsigned tlb_hit = 0;

	for (unsigned i = 0; i < 2; i++)
	{
		if (m_control[TCR] & TCR_S)
		{
			// 4k page
			if ((tlb[i].field1 & TLB_V) && ((tlb[i].field0 & TLB_AT4K) >> 5) == (virtual_address >> 16))
			{
				tlb_set = i;
				tlb_hit++;
			}
		}
		else
		{
			// 2k page
			if ((tlb[i].field1 & TLB_V) && ((tlb[i].field0 & TLB_AT2K) >> 4) == (virtual_address >> 15))
			{
				tlb_set = i;
				tlb_hit++;
			}
		}
	}

	u32 real_page = 0;
	switch (tlb_hit)
	{
	case 0:
		// tlb miss
		// TODO: refill or return exception/failure
		// TODO: select lru set
		tlb_set = 0;
		if (!reload(effective_address, tlb[tlb_set]))
			return { EXCEPTION, 0U };

		// fall through
	case 1:
		// single tlb set hit
		real_page = (m_control[TCR] & TCR_S)
			? (tlb[tlb_set].field1 & TLB_RPN4K) >> 4
			: (tlb[tlb_set].field1 & TLB_RPN2K) >> 3;
		break;

	case 2:
		// double tlb set hit
		m_control[MER] |= MER_S;

		// load mear with effective address unless locked, then lock
		set_mear(effective_address, LOCKED);

		// TODO: machine check
		if (!(m_control[TCR] & TCR_D))
			return { EXCEPTION, 0U };

		// if store, address is logical OR of both real page numbers
		// TODO: what about load?
		real_page =
			((m_control[TCR] & TCR_S) ? (tlb[0].field1 & TLB_RPN4K) >> 4 : (tlb[0].field1 & TLB_RPN2K) >> 3) |
			((m_control[TCR] & TCR_S) ? (tlb[1].field1 & TLB_RPN4K) >> 4 : (tlb[1].field1 & TLB_RPN2K) >> 3);
		tlb_set = 0;
	}

	// TODO: tlb mru/lru

	u32 const real_address = (m_control[TCR] & TCR_S)
		? (real_page << 12) | (effective_address & 0x0fffU)
		: (real_page << 11) | (effective_address & 0x07ffU);

	// access based on source (system processor or I/O) and segment access protect bits
	// nonspecial: read/write for each page of real memory
	// special:
	// only applies to translated: protection and lockbits disabled for v=r mode

	if (segment & SEGMENT_S)
	{
		// lockbit processing
		// TODO: disabled for v=r

		// check transaction identifier
		if ((m_control[TID] & 0xff) != ((tlb[tlb_set].field2 & TLB_TID) >> 16))
			return { PROTECTION, 0U };

		// select line lockbit from bits 21..24 of ea (2k) and bits 20..23 (4k)
		// 4k: bits 20..23 == bits 11..8 == 0x0000'0f00 == shift 8
		// 2k: bits 21..24 == bits 10..7 == 0x0000'0780 == shift 7

		// check write bit, lock bit and operation
		bool const lockbit = BIT(tlb[tlb_set].field2, 15 - ((effective_address >> ((m_control[TCR] & TCR_S) ? 8 : 7)) & 15));
		if (tlb[tlb_set].field2 & TLB_W)
		{
			if (!lockbit && store)
				return { PROTECTION, 0U };
		}
		else
		{
			if (!lockbit || store)
				return { PROTECTION, 0U };
		}
	}
	else
	{
		// check protect using key from tlb entry and segment

		// memory protection processing
		// TODO: virtual requests to nonspecial segments
		// TODO: disabled for v=r

		// 1 bit protection key from segment
		// 2 bit key in tlb entry
		// load/store mode (test and set is store)

		switch (tlb[tlb_set].field1 & TLB_KEY)
		{
		case 0:
			// key 0 fetch-protected
			if (segment & SEGMENT_K)
				return { PROTECTION, 0U };
			break;
		case 1:
			// key 0 read/write
			if ((segment & SEGMENT_K) && store)
				return { PROTECTION, 0U };
			break;
		case 2:
			// public read/write
			break;
		case 3:
			// public read-only
			if (store)
				return { PROTECTION, 0U };
			break;
		}
	}

	// access memory

	// update reference and change bits
	rc_w(real_page, store ? RC_R | RC_C : RC_R);

	return { SUCCESS, real_address };
}

bool rosetta_device::reload(u32 const effective_address, rosetta_device::tlb_entry &tlb_entry)
{
	u64 const segment = m_segment[effective_address >> 28];
	u64 const virtual_address = ((segment & SEGMENT_ID) << 26) | (effective_address & 0x0fff'ffffU);

	if (!machine().side_effects_disabled())
		LOGMASKED(LOG_RELOAD, "reload effective 0x%08x segment 0x%08x virtual 0x%08x\n", effective_address, segment, virtual_address);

	// TODO: lots of this is constant except when control registers change

	// compute hat base address
	unsigned const ram_size = (m_control[RAMS] & RAMS_SIZE) > 7 ? (m_control[RAMS] & RAMS_SIZE) - 7 : 0;
	u32 const hat_base = (m_control[TCR] & TCR_HIB) << (ram_size + (m_control[TCR] & TCR_S ? 8 : 9));

	// compute hat index
	u32 const mask = (1U << (ram_size + (m_control[TCR] & TCR_S ? 4 : 5))) - 1;
	unsigned const shift = (m_control[TCR] & TCR_S) ? 12 : 11;
	unsigned const index = (((segment & SEGMENT_ID) >> 2) ^ (effective_address >> shift)) & mask;
	if (!machine().side_effects_disabled())
		LOGMASKED(LOG_RELOAD, "reload shift %d index %d mask 0x%08x\n", shift, index, mask);

	// fetch hat entry
	u32 hat_entry = m_ram[(hat_base + (index << 4) + 4) >> 2];
	if (!machine().side_effects_disabled())
		LOGMASKED(LOG_RELOAD, "reload hat base 0x%x index 0x%x entry 0x%08x\n", hat_base, index, hat_entry);
	if (hat_entry & HAT_E)
		return false; // TODO: page fault

	// compute ipt entry pointer
	u32 pointer = ((hat_entry & HAT_HATP) >> 16);
	u32 const address = (m_control[TCR] & TCR_S) ? (virtual_address >> 11) & HAT_AT4K : (virtual_address >> 11) & HAT_AT2K;
	unsigned count = 0;

	// ipt search
	while (count < 1024)
	{
		// fetch ipt entry
		u32 const ipt_entry = m_ram[(hat_base + pointer * 16 + 0) >> 2];

		if ((ipt_entry & ((m_control[TCR] & TCR_S) ? HAT_AT4K : HAT_AT2K)) == address)
		{
			// reload tlb
			if (m_control[TCR] & TCR_S)
			{
				tlb_entry.field0 = (ipt_entry & TLB_AT4K);
				tlb_entry.field1 = (pointer << 3) | TLB_V | (ipt_entry >> 30);
			}
			else
			{
				tlb_entry.field0 = (ipt_entry & TLB_AT2K);
				tlb_entry.field1 = (pointer << 3) | TLB_V | (ipt_entry >> 30);

				if (!machine().side_effects_disabled())
					LOGMASKED(LOG_RELOAD, "reload real page 0x%08x\n", (hat_entry & HAT_HATP) >> 5);
			}

			if (segment & SEGMENT_S)
				tlb_entry.field2 = m_ram[(hat_base + pointer * 16 + 8) >> 2];

			if (!machine().side_effects_disabled())
				LOGMASKED(LOG_RELOAD, "reload complete count %d f0 0x%08x f1 0x%08x f2 0x%08x\n",
					count, tlb_entry.field0, tlb_entry.field1, tlb_entry.field2);
			return true;
		}

		// terminate long ipt search
		if ((m_control[TCR] & TCR_I) && (count == 127))
		{
			m_control[MER] |= MER_I;
			set_mear(effective_address, LOCKED);

			// TODO: exception
			LOGMASKED(LOG_RELOAD, "reload long search abort\n");
			return false;
		}

		// fetch next hat entry
		hat_entry = m_ram[(hat_base + pointer * 16 + 4) >> 2];
		if (hat_entry & HAT_L)
		{
			// TODO: page fault
			LOGMASKED(LOG_RELOAD, "reload fault\n");
			return false;
		}

		// compute next ipt entry pointer
		pointer = hat_entry & HAT_IPTP;
		count++;
	}

	fatalerror("endless loop in reload()\n");
}

void rosetta_device::set_mear(u32 const address, mear_state lock)
{
	if (m_mear_lock == LOCKED)
		return;

	if (m_mear_lock == MEMORY && lock == MEMORY)
		return;

	m_control[MEAR] = address;
	m_mear_lock = lock;
}

void rosetta_device::set_rmdr(u8 const ecc, bool lock)
{
	if (m_rmdr_lock)
		return;

	m_control[RMDR] = (m_control[RMDR] & ~RMDR_CHECK) | (ecc << 8);
	m_rmdr_lock = lock;
}

u32 rosetta_device::segment_r(offs_t offset)
{
	return m_segment[offset];
}

void rosetta_device::segment_w(offs_t offset, u32 data)
{
	LOG("segment_w 0x%x data 0x%x (%s)\n", offset, data, machine().describe_context());

	m_segment[offset] = data;
}

u32 rosetta_device::control_r(offs_t offset)
{
	u32 data = 0;

	switch (offset)
	{
	case MEAR:
		m_mear_lock = UNLOCKED;
		data = m_control[offset];
		break;

	case RMDR:
		m_mear_lock = UNLOCKED;
		m_rmdr_lock = false;
		data = m_control[offset];
		break;

	default:
		data = m_control[offset];
		break;
	}

	LOG("control_r %s data 0x%x (%s)\n", control_names[offset], data, machine().describe_context());

	return data;
}

void rosetta_device::control_w(offs_t offset, u32 data)
{
	LOG("control_w %s data 0x%x (%s)\n", control_names[offset], data, machine().describe_context());

	switch (offset)
	{
	case MEAR:
		m_mear_lock = UNLOCKED;
		m_control[offset] = data;
		break;

	case RAMS:
	case ROMS:
		m_control[offset] = data;
		remap();
		break;

	case RMDR:
		// only alternate check bits are writeable
		m_control[offset] = (m_control[offset] & ~RMDR_ALT) | (data & RMDR_ALT);
		break;

	default:
		m_control[offset] = data;
		break;
	}
}

u32 rosetta_device::tlb_r(offs_t offset)
{
	unsigned const tlb_set = BIT(offset, 4);
	u32 data = 0;

	switch (offset & 0x60)
	{
	case 0x00: data = m_tlb[offset & 0xf][tlb_set].field0; break;
	case 0x20: data = m_tlb[offset & 0xf][tlb_set].field1; break;
	case 0x40: data = m_tlb[offset & 0xf][tlb_set].field2; break;
	}

	LOG("tlb_r offset %x data %x\n", offset, data);

	return data;
}

void rosetta_device::tlb_w(offs_t offset, u32 data)
{
	unsigned const tlb_set = BIT(offset, 4);

	switch (offset & 0x60)
	{
	case 0x00: m_tlb[offset & 0xf][tlb_set].field0 = data; break;
	case 0x20: m_tlb[offset & 0xf][tlb_set].field1 = data; break;
	case 0x40: m_tlb[offset & 0xf][tlb_set].field2 = data; break;
	}

	LOG("tlb_w offset %x data %x\n", offset, data);
}

u32 rosetta_device::rc_r(offs_t offset)
{
	unsigned const shift = (offset & 3) * 2;

	return (m_rc[offset >> 2] >> shift) & 3;
}

void rosetta_device::rc_w(offs_t offset, u32 data)
{
	unsigned const shift = (offset & 3) * 2;

	m_rc[offset >> 2] &= ~(3 << shift);
	m_rc[offset >> 2] |= (data & 3) << shift;
}

template <bool ECC> void rosetta_device::ram_w(offs_t offset, u32 data, u32 mem_mask)
{
	m_ram[offset] = (m_ram[offset] & ~mem_mask) | (data & mem_mask);

	if (ECC)
		m_ecc[offset] = (m_control[TCR] & TCR_D) ? (m_control[RMDR] & RMDR_ALT) : compute_ecc(m_ram[offset]);
}

template <bool ECC> u32 rosetta_device::ram_r(offs_t offset, u32 mem_mask)
{
	u32 data = m_ram[offset];

	if (ECC && !machine().side_effects_disabled())
	{
		u8 const ecc = m_ecc[offset];

		switch (check_ecc(data, ecc))
		{
		case 0:
			// no error
			break;
		case 1:
			// correctable error
			if ((m_control[TCR] & TCR_C) || (m_control[TCR] & TCR_D))
			{
				if (!(m_control[MER] & MER_U))
					m_control[MER] |= MER_C;
				set_mear(offset, MEMORY);
				set_rmdr(ecc, true);

				if ((m_control[TCR] & TCR_C) && !(m_control[TCR] & TCR_D))
				{
					m_out_trap(0);
					m_out_trap(1);
				}
			}
			break;
		case 2:
			// uncorrectable error
			m_control[MER] |= MER_U;
			set_mear(offset, MEMORY);
			set_rmdr(ecc, true);

			if (!(m_control[TCR] & TCR_D))
			{
				m_out_trap(0);
				m_out_trap(1);
			}
			break;
		}
	}

	return data & mem_mask;
}

u8 rosetta_device::compute_ecc(u32 const data) const
{
	u8 result = 0;

	for (unsigned i = 0; i < ARRAY_LENGTH(ecc_bits); i++)
		result ^= BIT(data, 31 - i) ? ecc_bits[i] : 0;

	return result;
}

unsigned rosetta_device::check_ecc(u32 &data, u8 const ecc) const
{
	u8 const error = compute_ecc(data) ^ ecc;

	if (error)
	{
		for (unsigned i = 0; i < ARRAY_LENGTH(ecc_bits); i++)
		{
			if (error == ecc_bits[i])
			{
				LOGMASKED(LOG_ECC, "check_ecc single-bit error 0x%08x ecc 0x%02x error 0x%02x\n", data, ecc, error);

				// correct error
				data ^= (0x8000'0000U >> i);

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

void rosetta_device::remap()
{
	// unmap everything
	space(AS_PROGRAM).unmap_readwrite(0x000000, 0xffffff);

	// map rom
	if (m_control[ROMS] & ROMS_SIZE)
	{
		unsigned const shift = (m_control[ROMS] & ROMS_SIZE) > 7 ? (m_control[ROMS] & ROMS_SIZE) - 7 : 0;
		unsigned const size = 0x10000U << shift;
		unsigned const factor = (m_control[ROMS] & ROMS_START) >> (4 + shift);

		LOG("installing rom at 0x%06x-0x%06x\n", size * factor, size * factor + size - 1);
		if (size > m_rom.bytes())
			// FIXME: we assume the rom is mirrored if the configured size is larger than the region
			space(AS_PROGRAM).install_rom(size * factor, (size * factor + size - 1) & 0xffff, 0xff0000, m_rom);
		else
			space(AS_PROGRAM).install_rom(size * factor, (size * factor + size - 1), m_rom);
	}

	// map ram
	if (m_control[RAMS] & RAMS_SIZE)
	{
		unsigned const shift = (m_control[RAMS] & RAMS_SIZE) > 7 ? (m_control[RAMS] & RAMS_SIZE) - 7 : 0;
		unsigned const size = 0x10000U << shift;
		unsigned const factor = (m_control[RAMS] & RAMS_START) >> (4 + shift);

		LOG("installing ram at 0x%06x-0x%06x\n", size * factor, size * factor + size - 1);
		//space(AS_PROGRAM).install_ram(size * factor, size * factor + size - 1, m_ram.get());
		space(AS_PROGRAM).install_readwrite_handler(size * factor, size * factor + size - 1, read32s_delegate(*this, FUNC(rosetta_device::ram_r<true>)), write32s_delegate(*this, FUNC(rosetta_device::ram_w<true>)));
	}
}

void rosetta_device::tlb_inv_all(u32 data)
{
	LOGMASKED(LOG_TLB, "tlb_inv_all (%s)\n", machine().describe_context());

	for (unsigned i = 0; i < 16; i++)
	{
		m_tlb[i][0].field1 &= ~TLB_V;
		m_tlb[i][1].field1 &= ~TLB_V;
	}
}

void rosetta_device::tlb_inv_segment(u32 data)
{
	LOGMASKED(LOG_TLB, "tlb_inv_segment %x (%s)\n", data & 15, machine().describe_context());

	unsigned const identifier = (m_segment[data & 15] & SEGMENT_ID) >> 2;

	for (unsigned i = 0; i < 16; i++)
	{
		if (((m_tlb[i][0].field0 & TLB_SEG) >> 17) == identifier)
			m_tlb[i][0].field1 &= ~TLB_V;

		if (((m_tlb[i][1].field0 & TLB_SEG) >> 17) == identifier)
			m_tlb[i][1].field1 &= ~TLB_V;
	}
}

void rosetta_device::tlb_inv_address(u32 data)
{
	LOGMASKED(LOG_TLB, "tlb_inv_address 0x%08x unimplemented\n", data);
}
