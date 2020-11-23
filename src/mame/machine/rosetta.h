// license:BSD-3-Clause
// copyright-holders:Patrick Mackinlay

#ifndef MAME_MACHINE_ROSETTA_H
#define MAME_MACHINE_ROSETTA_H

#pragma once

class rosetta_device
	: public device_t
	, public device_memory_interface
{
public:
	enum mode : unsigned
	{
		MASTER = 0,
		MASTER_ROM = 1,
		MASTER_RAM = 2,
		STANDARD = 3,
	};

	enum ram_size : unsigned
	{
		RAM_NONE = 0,
		RAM_1M   = 1,
		RAM_2M   = 2,
		RAM_4M   = 3,
		RAM_8M   = 4,
		RAM_16M  = 5,
	};

	rosetta_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock, mode initial_mode = STANDARD, ram_size ram = RAM_NONE);

	template <typename T> void set_bus(T &&tag, int spacenum) { m_bus.set_tag(std::forward<T>(tag), spacenum); }
	template <typename T> void set_rom(T &&tag) { m_rom.set_tag(std::forward<T>(tag)); }

	auto out_trap() { return m_out_trap.bind(); }

	enum result : unsigned
	{
		ABSENT = 0, // segment not present
		EXCEPTION,
		PROTECTION,
		SUCCESS
	};

	struct translate_result
	{
		result status;
		u32 real_address;
	};
	translate_result translate(u32 effective_address, bool store, bool io_device = false);

	u32 io_r(offs_t offset);
	void io_w(offs_t offset, u32 data);

	template <bool Translate> u32 mem_r(offs_t offset, u32 mem_mask);
	template <bool Translate> void mem_w(offs_t offset, u32 data, u32 mem_mask);

protected:
	// device_t overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// device_memory_interface overrides
	virtual space_config_vector memory_space_config() const override;

	void internal_map(address_map &map);

	u32 segment_r(offs_t offset);
	u32 control_r(offs_t offset);
	u32 tlb_r(offs_t offset);
	u32 rc_r(offs_t offset);

	void segment_w(offs_t offset, u32 data);
	void control_w(offs_t offset, u32 data);
	void tlb_w(offs_t offset, u32 data);
	void rc_w(offs_t offset, u32 data);

	template <bool ECC> void ram_w(offs_t offset, u32 data, u32 mem_mask);
	template <bool ECC> u32 ram_r(offs_t offset, u32 mem_mask);
	u8 compute_ecc(u32 const data) const;
	unsigned check_ecc(u32 &data, u8 const ecc) const;

	void tlb_inv_all(u32 data);
	void tlb_inv_segment(u32 data);
	void tlb_inv_address(u32 data);

	void remap();

	enum mear_state : unsigned
	{
		UNLOCKED,
		LOCKED,
		MEMORY,
	};
	void set_mear(u32 const address, mear_state lock);
	void set_rmdr(u8 const ecc, bool lock);

	struct tlb_entry
	{
		u32 field0; // address tag
		u32 field1; // real page, valid, key
		u32 field2; // write, transaction identifier, lockbits
	};
	bool reload(u32 const effective_address, tlb_entry &tlb_entry);

private:
	ram_size const m_ram_size;

	required_address_space m_bus;
	devcb_write_line m_out_trap;

	address_space_config m_memory_config;
	address_space_config m_control_config;

	memory_access<24, 2, 0, ENDIANNESS_BIG>::cache m_mem;

	required_region_ptr<u32> m_rom;
	std::unique_ptr<u32[]> m_ram;
	std::unique_ptr<u8[]> m_ecc;

	enum roms_mask : u32
	{
		ROMS_SIZE  = 0x0000'000f, // rom size
		ROMS_START = 0x0000'0ff0, // rom address
		ROMS_P     = 0x0000'1000, // parity enable
	};

	enum segment_mask : u32
	{
		SEGMENT_K  = 0x0000'0001, // key
		SEGMENT_S  = 0x0000'0002, // special
		SEGMENT_ID = 0x0000'3ffc, // identifier
		SEGMENT_I  = 0x0000'4000, // i/o access protect
		SEGMENT_R  = 0x0000'8000, // system processor access protect
		SEGMENT_P  = 0x0001'0000, // present
	};

	enum rams_mask : u32
	{
		RAMS_SIZE  = 0x0000'000f, // ram size
		RAMS_START = 0x0000'0ff0, // ram address
	};

	enum tcr_mask : u32
	{
		TCR_HIB = 0x0000'00ff, // hat/ipt base address
		TCR_S   = 0x0000'0100, // page size (1=4k pages)
		TCR_R   = 0x0000'0400, // enable interrupt on successful tlb load
		TCR_C   = 0x0000'0800, // enable interrupt on correctable ecc error
		TCR_I   = 0x0000'1000, // terminate long ipt search
		TCR_D   = 0x0000'2000, // enable ras diagnostic mode
		TCR_E   = 0x0000'4000, // interrupt on successful parity error retry
		TCR_V   = 0x0000'8000, // segment register zero virtual equal to real
	};
	enum mer_mask : u32
	{
		MER_D = 0x0000'0001, // data
		MER_P = 0x0000'0002, // protection
		MER_S = 0x0000'0004, // tlb specification
		MER_F = 0x0000'0008, // page fault
		MER_M = 0x0000'0010, // multiple exception
		MER_E = 0x0000'0020, // external device exception
		MER_I = 0x0000'0040, // ipt specification error
		MER_W = 0x0000'0080, // write to rom
							 // reserved
		MER_T = 0x0000'0200, // successful tlb reload
		MER_C = 0x0000'0400, // correctable ecc error
		MER_U = 0x0000'0800, // uncorrectable memory error
		MER_L = 0x0000'1000, // access type
		MER_O = 0x0000'2000, // invalid i/o address
		MER_B = 0x0000'4000, // invalid memory address
		MER_N = 0x0000'8000, // processor channel nakd
		MER_A = 0x0001'0000, // processor channel ackd
		MER_V = 0x0002'0000, // segment protection violation
	};
	enum rmdr_mask : u32
	{
		RMDR_CHECK = 0x0000'ff00, // array check bits
		RMDR_ALT   = 0x0000'00ff, // alternate check bits
	};

	enum tlb_mask : u32
	{
		// field 0
		TLB_SEG  = 0x1ffe'0000, // segment identifier
		TLB_AT2K = 0x1fff'fff0, // address tag (2k page)
		TLB_AT4K = 0x1fff'ffe0, // address tag (4k page)

		// field 1
		TLB_KEY   = 0x0000'0003, // key bits
		TLB_V     = 0x0000'0004, // valid bit
		TLB_RPN2K = 0x0000'fff8, // real page number (2k page)
		TLB_RPN4K = 0x0000'fff0, // real page number (4k page)

		// field 2
		TLB_LB    = 0x0000'ffff, // lock bits
		TLB_TID   = 0x00ff'0000, // transaction identifier
		TLB_W     = 0x0100'0000, // write bit

	};
	// 0000 0000 0000 0000  0000 0000 0000 0000
	// 0123 4567 89ab cdef  0123 4567 89ab cdef

	enum hat_mask : u32
	{
		HAT_AT2K = 0x1fff'ffff, // address tag (2k page)
		HAT_AT4K = 0x1fff'fffe, // address tag (4k page)
		HAT_KEY  = 0xc000'0000, // key

		HAT_IPTP = 0x0000'1fff, // ipt pointer
		HAT_L    = 0x0000'8000, // last
		HAT_HATP = 0x1fff'0000, // hat pointer
		HAT_E    = 0x8000'0000, // empty

		HAT_LB   = 0x0000'ffff, // lock bits
		HAT_TID  = 0x00ff'0000, // transaction identifier
		HAT_W    = 0x0100'0000, // write protect
	};

	enum rc_mask : u8
	{
		RC_C = 0x01, // change
		RC_R = 0x02, // reference
	};

	enum reg : unsigned
	{
		IOBA = 0, // i/o base address
		MER  = 1, // memory exception
		MEAR = 2, // memory exception address
		TRAR = 3, // translated real address
		TID  = 4, // transaction identifier
		TCR  = 5, // translation control
		RAMS = 6, // ram specification
		ROMS = 7, // rom specification
		RMDR = 8, // ras mode diagnostic
	};

	mode m_mode;

	u32 m_segment[16];
	u32 m_control[9];

	mear_state m_mear_lock;
	bool m_rmdr_lock;

	tlb_entry m_tlb[16][2];

	// reference/change bits
	u8 m_rc[2048];
};

DECLARE_DEVICE_TYPE(ROSETTA, rosetta_device)

#endif // MAME_MACHINE_ROSETTA_H
