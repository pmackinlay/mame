// license:BSD-3-Clause
// copyright-holders:Patrick Mackinlay

#ifndef MAME_CPU_MIPS_R4000_H
#define MAME_CPU_MIPS_R4000_H

#pragma once

DECLARE_DEVICE_TYPE(R4000, r4000_device)

enum {
	MIPS3_PC = STATE_GENPC,
	MIPS3_R0 = 1,
	MIPS3_R1,
	MIPS3_R2,
	MIPS3_R3,
	MIPS3_R4,
	MIPS3_R5,
	MIPS3_R6,
	MIPS3_R7,
	MIPS3_R8,
	MIPS3_R9,
	MIPS3_R10,
	MIPS3_R11,
	MIPS3_R12,
	MIPS3_R13,
	MIPS3_R14,
	MIPS3_R15,
	MIPS3_R16,
	MIPS3_R17,
	MIPS3_R18,
	MIPS3_R19,
	MIPS3_R20,
	MIPS3_R21,
	MIPS3_R22,
	MIPS3_R23,
	MIPS3_R24,
	MIPS3_R25,
	MIPS3_R26,
	MIPS3_R27,
	MIPS3_R28,
	MIPS3_R29,
	MIPS3_R30,
	MIPS3_R31,
	MIPS3_HI,
	MIPS3_LO,
	MIPS3_FPR0,
	MIPS3_FPS0,
	MIPS3_FPD0,
	MIPS3_FPR1,
	MIPS3_FPS1,
	MIPS3_FPD1,
	MIPS3_FPR2,
	MIPS3_FPS2,
	MIPS3_FPD2,
	MIPS3_FPR3,
	MIPS3_FPS3,
	MIPS3_FPD3,
	MIPS3_FPR4,
	MIPS3_FPS4,
	MIPS3_FPD4,
	MIPS3_FPR5,
	MIPS3_FPS5,
	MIPS3_FPD5,
	MIPS3_FPR6,
	MIPS3_FPS6,
	MIPS3_FPD6,
	MIPS3_FPR7,
	MIPS3_FPS7,
	MIPS3_FPD7,
	MIPS3_FPR8,
	MIPS3_FPS8,
	MIPS3_FPD8,
	MIPS3_FPR9,
	MIPS3_FPS9,
	MIPS3_FPD9,
	MIPS3_FPR10,
	MIPS3_FPS10,
	MIPS3_FPD10,
	MIPS3_FPR11,
	MIPS3_FPS11,
	MIPS3_FPD11,
	MIPS3_FPR12,
	MIPS3_FPS12,
	MIPS3_FPD12,
	MIPS3_FPR13,
	MIPS3_FPS13,
	MIPS3_FPD13,
	MIPS3_FPR14,
	MIPS3_FPS14,
	MIPS3_FPD14,
	MIPS3_FPR15,
	MIPS3_FPS15,
	MIPS3_FPD15,
	MIPS3_FPR16,
	MIPS3_FPS16,
	MIPS3_FPD16,
	MIPS3_FPR17,
	MIPS3_FPS17,
	MIPS3_FPD17,
	MIPS3_FPR18,
	MIPS3_FPS18,
	MIPS3_FPD18,
	MIPS3_FPR19,
	MIPS3_FPS19,
	MIPS3_FPD19,
	MIPS3_FPR20,
	MIPS3_FPS20,
	MIPS3_FPD20,
	MIPS3_FPR21,
	MIPS3_FPS21,
	MIPS3_FPD21,
	MIPS3_FPR22,
	MIPS3_FPS22,
	MIPS3_FPD22,
	MIPS3_FPR23,
	MIPS3_FPS23,
	MIPS3_FPD23,
	MIPS3_FPR24,
	MIPS3_FPS24,
	MIPS3_FPD24,
	MIPS3_FPR25,
	MIPS3_FPS25,
	MIPS3_FPD25,
	MIPS3_FPR26,
	MIPS3_FPS26,
	MIPS3_FPD26,
	MIPS3_FPR27,
	MIPS3_FPS27,
	MIPS3_FPD27,
	MIPS3_FPR28,
	MIPS3_FPS28,
	MIPS3_FPD28,
	MIPS3_FPR29,
	MIPS3_FPS29,
	MIPS3_FPD29,
	MIPS3_FPR30,
	MIPS3_FPS30,
	MIPS3_FPD30,
	MIPS3_FPR31,
	MIPS3_FPS31,
	MIPS3_FPD31,
	MIPS3_CCR1_31,
	MIPS3_SR,
	MIPS3_EPC,
	MIPS3_CAUSE,
	MIPS3_COUNT,
	MIPS3_COMPARE,
	MIPS3_INDEX,
	MIPS3_RANDOM,
	MIPS3_ENTRYHI,
	MIPS3_ENTRYLO0,
	MIPS3_ENTRYLO1,
	MIPS3_PAGEMASK,
	MIPS3_WIRED,
	MIPS3_BADVADDR,
	MIPS3_CONTEXT,
	MIPS3_XCONTEXT,
	MIPS3_R0H,
	MIPS3_R1H,
	MIPS3_R2H,
	MIPS3_R3H,
	MIPS3_R4H,
	MIPS3_R5H,
	MIPS3_R6H,
	MIPS3_R7H,
	MIPS3_R8H,
	MIPS3_R9H,
	MIPS3_R10H,
	MIPS3_R11H,
	MIPS3_R12H,
	MIPS3_R13H,
	MIPS3_R14H,
	MIPS3_R15H,
	MIPS3_R16H,
	MIPS3_R17H,
	MIPS3_R18H,
	MIPS3_R19H,
	MIPS3_R20H,
	MIPS3_R21H,
	MIPS3_R22H,
	MIPS3_R23H,
	MIPS3_R24H,
	MIPS3_R25H,
	MIPS3_R26H,
	MIPS3_R27H,
	MIPS3_R28H,
	MIPS3_R29H,
	MIPS3_R30H,
	MIPS3_R31H,
};

class r4000_device : public cpu_device
{
public:
	// construction/destruction
	r4000_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	void set_endianness(endianness_t endianness) { m_endianness = endianness; }
	void set_icache_size(size_t icache_size) {}
	void set_dcache_size(size_t dcache_size) {}

	TIMER_CALLBACK_MEMBER(compare_int_callback);

	enum cop0_reg : int
	{
		COP0_Index    = 0,
		COP0_Random   = 1,
		COP0_EntryLo0 = 2,
		COP0_EntryLo1 = 3,
		COP0_Context  = 4,
		COP0_PageMask = 5,
		COP0_Wired    = 6,
		COP0_BadVAddr = 8,
		COP0_Count    = 9,
		COP0_EntryHi  = 10,
		COP0_Compare  = 11,
		COP0_Status   = 12,
		COP0_Cause    = 13,
		COP0_EPC      = 14,
		COP0_PRId     = 15,
		COP0_Config   = 16,
		COP0_LLAddr   = 17,
		COP0_WatchLo  = 18,
		COP0_WatchHi  = 19,
		COP0_XContext = 20,
		COP0_ECC      = 26,
		COP0_CacheErr = 27,
		COP0_TagLo    = 28,
		COP0_TagHi    = 29,
		COP0_ErrorEPC = 30,
	};

	enum sr_mask : u32
	{
		SR_IE    = 0x00000001, // interrupt enable
		SR_EXL   = 0x00000002, // exception level
		SR_ERL   = 0x00000004, // error level
		SR_KSU   = 0x00000018, // kernel/supervisor/user mode
		SR_UX    = 0x00000020, // 64-bit addressing user mode
		SR_SX    = 0x00000040, // 64-bit addressing supervisor mode
		SR_KX    = 0x00000080, // 64-bit addressing kernel mode
		SR_IMSW0 = 0x00000100, // software interrupt 0 enable
		SR_IMSW1 = 0x00000200, // software interrupt 1 enable
		SR_IMEX0 = 0x00000400, // external interrupt 0 enable
		SR_IMEX1 = 0x00000800, // external interrupt 1 enable
		SR_IMEX2 = 0x00001000, // external interrupt 2 enable
		SR_IMEX3 = 0x00002000, // external interrupt 3 enable
		SR_IMEX4 = 0x00004000, // external interrupt 4 enable
		SR_IMEX5 = 0x00008000, // external interrupt 5 enable
		SR_DE    = 0x00010000, // disable cache parity/ecc exceptions
		SR_CE    = 0x00020000, // cache ecc check enable
		SR_CH    = 0x00040000, // cache hit
		SR_SR    = 0x00100000, // soft reset
		SR_TS    = 0x00200000, // tlb shutdown (read only)
		SR_BEV   = 0x00400000, // bootstrap exception vectors
		SR_RE    = 0x02000000, // reverse endian
		SR_FR    = 0x04000000, // enable additional floating-point registers
		SU_RP    = 0x08000000, // reduced power
		SR_CU0   = 0x10000000, // coprocessor usable 0
		SR_CU1   = 0x20000000, // coprocessor usable 1
		SR_CU2   = 0x40000000, // coprocessor usable 2
		SR_CU3   = 0x80000000, // coprocessor usable 3

		SR_IM    = 0x0000ff00, // interrupt mask
		SR_DS    = 0x01ff0000, // diagnostic status
	};

	enum sr_ksu_mode : u32
	{
		SR_KSU_KERNEL = 0x00000000,
		SR_KSU_SUPER  = 0x00000008,
		SR_KSU_USER   = 0x00000010,
	};

	enum exception_mask : u32
	{
		EXCEPTION_INT   = 0x00000000, // interrupt
		EXCEPTION_MOD   = 0x00000004, // tlb modification
		EXCEPTION_TLBL  = 0x00000008, // tlb load
		EXCEPTION_TLBS  = 0x0000000c, // tlb store
		EXCEPTION_ADEL  = 0x00000010, // address error load
		EXCEPTION_ADES  = 0x00000014, // address error store
		EXCEPTION_IBE   = 0x00000018, // bus error (instruction fetch)
		EXCEPTION_DBE   = 0x0000001c, // bus error (data reference: load or store)
		EXCEPTION_SYS   = 0x00000020, // syscall
		EXCEPTION_BP    = 0x00000024, // breakpoint
		EXCEPTION_RI    = 0x00000028, // reserved instruction
		EXCEPTION_CPU   = 0x0000002c, // coprocessor unusable
		EXCEPTION_OV    = 0x00000030, // arithmetic overflow
		EXCEPTION_TR    = 0x00000034, // trap
		EXCEPTION_VCEI  = 0x00000038, // virtual coherency exception instruction
		EXCEPTION_FPE   = 0x0000003c, // floating point
		EXCEPTION_WATCH = 0x0000005c, // reference to watchhi/watchlo address
		EXCEPTION_VCED  = 0x0000007c, // virtual coherency exception data

		EXCEPTION_CPU0  = 0x0000002c, // coprocessor 0 unusable
		EXCEPTION_CPU1  = 0x1000002c, // coprocessor 1 unusable
		EXCEPTION_CPU2  = 0x2000002c, // coprocessor 2 unusable
		EXCEPTION_CPU3  = 0x3000002c, // coprocessor 3 unusable
	};

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// device_execute_interface overrides
	virtual u32 execute_min_cycles() const override { return 1; }
	virtual u32 execute_max_cycles() const override { return 40; }
	virtual u32 execute_input_lines() const override { return 6; }
	virtual void execute_run() override;
	virtual void execute_set_input(int inputnum, int state) override;

	// device_memory_interface overrides
	virtual space_config_vector memory_space_config() const override;
	virtual bool memory_translate(int spacenum, int intention, offs_t &address) override;

	// our 64 bit version
	bool memory_translate(int spacenum, int intention, u64 &address);

	template <typename T, typename U> std::enable_if_t<std::is_convertible<U, std::function<void(T)>>::value, bool> load(u64 program_address, U &&apply);
	template <typename T, typename U> std::enable_if_t<std::is_convertible<U, T>::value, void> store(u64 program_address, U data, T mem_mask = ~T(0));
	bool fetch(u64 program_address, std::function<void(u32)> &&apply);
	std::string debug_unicode_string(u64 unicode_string_pointer);

	// device_state_interface overrides
	virtual void state_export(const device_state_entry &entry) override;
	virtual void state_string_export(const device_state_entry &entry, std::string &str) const override;

	// device_disasm_interface overrides
	virtual std::unique_ptr<util::disasm_interface> create_disassembler() override;

	void invalid_instruction(u32 op);

	struct internal_mips3_state {
		/* core registers */
		u64        pc;
		int        icount;
		u64 r[32];
		u64 lo;
		u64 hi;

		/* COP registers */
		u64        cpr[3][32];
		u64        ccr[3][32];

		u64        count_zero_time;
		u32        compare_armed;
	};

	/* core state */
	internal_mips3_state m_core;

	address_space_config m_program_config_le;
	address_space_config m_program_config_be;

	/* internal stuff */
	enum branch_state_t : unsigned
	{
		NONE      = 0,
		DELAY     = 1, // delay slot instruction active
		BRANCH    = 2, // branch instruction active
		EXCEPTION = 3, // exception triggered
		NULLIFY   = 4, // next instruction nullified
	}
	m_branch_state;
	u64 m_branch_target;

	uint8_t         m_cf[4][8];
	u32        m_ll_value;
	u64        m_lld_value;

	emu_timer *     m_compare_int_timer;

	/* memory accesses */
	endianness_t m_endianness;

	u32        m_debugger_temp;

	void generate_exception(u32 exception, u16 const vector = 0x180);

	void check_irqs();

	void mips3com_update_cycle_counting();
	void mips3com_tlbr();
	void mips3com_tlbwi();
	void mips3com_tlbwr();
	void mips3com_tlbp();

	struct tlb_entry_t
	{
		u64 mask;
		u64 vpn;
		u64 pfn[2];

		u8 low_bit;
	}
	m_tlb[48];

	bool m_64;
	void mode_check()
	{
		//bool const old_mode = m_64;

		if (m_core.cpr[0][COP0_Status] & (SR_EXL | SR_ERL))
			m_64 = m_core.cpr[0][COP0_Status] & SR_KX;
		else
			switch (m_core.cpr[0][COP0_Status] & SR_KSU)
			{
			case SR_KSU_KERNEL: m_64 = m_core.cpr[0][COP0_Status] & SR_KX; break;
			case SR_KSU_SUPER: m_64 = m_core.cpr[0][COP0_Status] & SR_SX; break;
			case SR_KSU_USER: m_64 = m_core.cpr[0][COP0_Status] & SR_UX; break;
			}

		//if (m_64 != old_mode)
		//  logerror("mode change to %d-bit addressing\n", m_64 ? 64 : 32);
	};

	enum tlb_mask : u64
	{
		TLB_MASK = 0x0000'0000'01ff'e000,
	};
	enum tlb_eh : u64
	{
		EH_ASID    = 0x0000'0000'0000'00ff, // address space id
		EH_G       = 0x0000'0000'0000'1000, // global (tlb only)
		EH_VPN2_32 = 0x0000'0000'ffff'e000, // virtual page number (32-bit mode)
		EH_VPN2_64 = 0x0000'00ff'ffff'e000, // virtual page number (64-bit mode)
		EH_R       = 0xc000'0000'0000'0000, // region (64-bit mode)
	};
	enum tlb_el : u64
	{
		EL_G   = 0x0000'0000'0000'0001, // global (entrylo only)
		EL_V   = 0x0000'0000'0000'0002, // valid
		EL_D   = 0x0000'0000'0000'0004, // dirty
		EL_C   = 0x0000'0000'0000'0038, // coherency
		EL_PFN = 0x0000'0000'3fff'ffc0, // page frame number
	};

	enum context_mask : u64
	{
		CONTEXT_PTEBASE = 0xffff'ffff'ff80'0000,
		CONTEXT_BADVPN2 = 0x0000'0000'007f'fff0,
	};

	enum xcontext_mask : u64
	{
		XCONTEXT_PTEBASE = 0xffff'fffe'0000'0000, // page table entry base
		XCONTEXT_R       = 0x0000'0001'8000'0000, // region
		XCONTEXT_BADVPN2 = 0x0000'0000'7fff'fff0, // bad virtual page number / 2
	};

	enum pagemask_mask : u32
	{
		PAGEMASK = 0x01ff'e000,
	};

private:
	u32 compute_config_register();
	u32 compute_prid_register();

	void tlb_write_common(u8 const index);

	u64 get_cop0_reg(int index);
	void set_cop0_reg(int index, u64 val);
	u64 get_cop0_creg(int index);
	void set_cop0_creg(int index, u64 val);
	void handle_cop0(u32 op);

	u32 get_cop1_reg32(int index);
	u64 get_cop1_reg64(int index);
	void set_cop1_reg32(int index, u32 val);
	void set_cop1_reg64(int index, u64 val);
	u64 get_cop1_creg(int index);
	void set_cop1_creg(int index, u64 val);
	void handle_cop1_fr0(u32 op);
	void handle_cop1_fr1(u32 op);

	void lwl(u32 op);
	void lwr(u32 op);
	void ldl(u32 op);
	void ldr(u32 op);
	void swl(u32 op);
	void swr(u32 op);
	void sdl(u32 op);
	void sdr(u32 op);
};

#endif // MAME_CPU_MIPS_R4000_H
