// license:BSD-3-Clause
// copyright-holders:Patrick Mackinlay

/*
 * This is a stripped-down MIPS3 CPU derived from the main mips3 code:
 *
 *   - only supports the original MIPS R4000
 *   - no drc support
 *   - reworked address translation logic
 *   - configurable endianness
 *   - no cache support
 */

#include "emu.h"
#include "debugger.h"
#include "r4000.h"
#include "mips3dsm.h"
#include <cmath>

#define LOG_GENERAL   (1U << 0)
#define LOG_TLB       (1U << 1)
#define LOG_EXCEPTION (1U << 2)

#define VERBOSE (LOG_GENERAL)

#include "logmacro.h"

#define USE_ABI_REG_NAMES 1

#define RSREG           ((op >> 21) & 31)
#define RTREG           ((op >> 16) & 31)
#define RDREG           ((op >> 11) & 31)
#define SHIFT           ((op >> 6) & 31)

#define FRREG           ((op >> 21) & 31)
#define FTREG           ((op >> 16) & 31)
#define FSREG           ((op >> 11) & 31)
#define FDREG           ((op >> 6) & 31)

#define IS_SINGLE(o)    (((o) & (1 << 21)) == 0)
#define IS_DOUBLE(o)    (((o) & (1 << 21)) != 0)
#define IS_FLOAT(o)     (((o) & (1 << 23)) == 0)
#define IS_INTEGRAL(o)  (((o) & (1 << 23)) != 0)

/***************************************************************************
    HELPER MACROS
***************************************************************************/

#define RSVAL32     u32(m_core.r[RSREG])
#define RTVAL32     u32(m_core.r[RTREG])

#define FRVALS_FR0  (((float *)&m_core.cpr[1][FRREG & 0x1E])[BYTE_XOR_LE(FRREG & 1)])
#define FTVALS_FR0  (((float *)&m_core.cpr[1][FTREG & 0x1E])[BYTE_XOR_LE(FTREG & 1)])
#define FSVALS_FR0  (((float *)&m_core.cpr[1][FSREG & 0x1E])[BYTE_XOR_LE(FSREG & 1)])
#define FDVALS_FR0  (((float *)&m_core.cpr[1][FDREG & 0x1E])[BYTE_XOR_LE(FDREG & 1)])
#define FSVALW_FR0  (((u32 *)&m_core.cpr[1][FSREG & 0x1E])[BYTE_XOR_LE(FSREG & 1)])
#define FDVALW_FR0  (((u32 *)&m_core.cpr[1][FDREG & 0x1E])[BYTE_XOR_LE(FDREG & 1)])

#define FRVALD_FR0  (*(double *)&m_core.cpr[1][FRREG & 0x1E])
#define FTVALD_FR0  (*(double *)&m_core.cpr[1][FTREG & 0x1E])
#define FSVALD_FR0  (*(double *)&m_core.cpr[1][FSREG & 0x1E])
#define FDVALD_FR0  (*(double *)&m_core.cpr[1][FDREG & 0x1E])
#define FSVALL_FR0  (*(u64 *)&m_core.cpr[1][FSREG & 0x1E])
#define FDVALL_FR0  (*(u64 *)&m_core.cpr[1][FDREG & 0x1E])

#define FRVALS_FR1  (((float *)&m_core.cpr[1][FRREG])[BYTE_XOR_LE(0)])
#define FTVALS_FR1  (((float *)&m_core.cpr[1][FTREG])[BYTE_XOR_LE(0)])
#define FSVALS_FR1  (((float *)&m_core.cpr[1][FSREG])[BYTE_XOR_LE(0)])
#define FDVALS_FR1  (((float *)&m_core.cpr[1][FDREG])[BYTE_XOR_LE(0)])
#define FSVALW_FR1  (((u32 *)&m_core.cpr[1][FSREG])[BYTE_XOR_LE(0)])
#define FDVALW_FR1  (((u32 *)&m_core.cpr[1][FDREG])[BYTE_XOR_LE(0)])

#define FRVALD_FR1  (*(double *)&m_core.cpr[1][FRREG])
#define FTVALD_FR1  (*(double *)&m_core.cpr[1][FTREG])
#define FSVALD_FR1  (*(double *)&m_core.cpr[1][FSREG])
#define FDVALD_FR1  (*(double *)&m_core.cpr[1][FDREG])
#define FSVALL_FR1  (*(u64 *)&m_core.cpr[1][FSREG])
#define FDVALL_FR1  (*(u64 *)&m_core.cpr[1][FDREG])

#define ADDR(r, o) (m_64 ? (r + s16(o)) : s64(s32(u32(r) + s16(o))))

#define ADDPC(x)        do { m_branch_state = BRANCH; m_branch_target = ADDR(m_core.pc, ((x) << 2) + 4); } while (0)

#define SR          m_core.cpr[0][COP0_Status]
#define CAUSE       m_core.cpr[0][COP0_Cause]

#define GET_FCC(n)  (m_cf[1][n])
#define SET_FCC(n,v) (m_cf[1][n] = (v))

static const u8 fcc_shift[8] = { 23, 25, 26, 27, 28, 29, 30, 31 };

DEFINE_DEVICE_TYPE(R4000, r4000_device, "r4000", "MIPS R4000")

r4000_device::r4000_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: cpu_device(mconfig, R4000, tag, owner, clock)
	, m_program_config_le("program", ENDIANNESS_LITTLE, 64, 32)
	, m_program_config_be("program", ENDIANNESS_BIG, 64, 32)
	, m_ll_value(0)
	, m_lld_value(0)
	, m_endianness(ENDIANNESS_BIG)
	, m_debugger_temp(0)
{
}

device_memory_interface::space_config_vector r4000_device::memory_space_config() const
{
	return space_config_vector{
		std::make_pair(AS_PROGRAM, (m_endianness == ENDIANNESS_BIG) ? &m_program_config_be : &m_program_config_le),
		//std::make_pair(1, &m_icache_config),
		//std::make_pair(2, &m_dcache_config)
	};
}

void r4000_device::generate_exception(u32 exception, u16 const vector)
{
	LOGMASKED(LOG_EXCEPTION, "generate_exception 0x%08x\n", exception);

	if (!(SR & SR_EXL))
	{
		m_core.cpr[0][COP0_EPC] = m_core.pc;

		CAUSE = (CAUSE & 0x0000ff00) | exception;

		// if in a branch delay slot, restart at the branch instruction
		if (m_branch_state == DELAY)
		{
			m_core.cpr[0][COP0_EPC] -= 4;
			CAUSE |= 0x80000000;
		}

		SR |= SR_EXL;

		m_64 = m_core.cpr[0][COP0_Status] & SR_KX;
	}
	else
		CAUSE = (CAUSE & 0x8000ff00) | exception;

	m_branch_state = EXCEPTION;
	m_core.pc = ((SR & SR_BEV) ? 0xffffffff'bfc00200 : 0xffffffff'80000000) + vector;

	if (exception != EXCEPTION_INT)
		debugger_exception_hook(exception);
}

void r4000_device::invalid_instruction(u32 op)
{
	fatalerror("Invalid instruction! %08x\n", op);
	generate_exception(EXCEPTION_RI);
}



/***************************************************************************
    IRQ HANDLING
***************************************************************************/

void r4000_device::check_irqs()
{
	if ((CAUSE & SR & 0xff00) && (SR & SR_IE) && !(SR & (SR_EXL | SR_ERL)))
		generate_exception(EXCEPTION_INT);
}

/***************************************************************************
    CORE CALLBACKS
***************************************************************************/

void r4000_device::device_start()
{
	/* initialize based on the config */
	m_core = {};

	/* allocate a timer for the compare interrupt */
	m_compare_int_timer = machine().scheduler().timer_alloc(timer_expired_delegate(FUNC(r4000_device::compare_int_callback), this));

	/* register for save states */
	save_item(NAME(m_core.pc));
	save_item(NAME(m_core.r));
	save_item(NAME(m_core.cpr));
	save_item(NAME(m_core.ccr));

	save_item(NAME(m_core.count_zero_time));
	// TODO: tlb

	// Register state with debugger
	state_add( MIPS3_PC,           "PC", m_core.pc).formatstr("%016X");
	state_add( MIPS3_SR,           "SR", m_core.cpr[0][COP0_Status]).formatstr("%08X");
	state_add( MIPS3_EPC,          "EPC", m_core.cpr[0][COP0_EPC]).formatstr("%016X");
	state_add( MIPS3_CAUSE,        "Cause", m_core.cpr[0][COP0_Cause]).formatstr("%08X");
	state_add( MIPS3_COUNT,        "Count", m_debugger_temp).callexport().formatstr("%08X");
	state_add( MIPS3_COMPARE,      "Compare", m_core.cpr[0][COP0_Compare]).formatstr("%08X");
	state_add( MIPS3_INDEX,        "Index", m_core.cpr[0][COP0_Index]).formatstr("%08X");
	state_add( MIPS3_RANDOM,       "Random", m_core.cpr[0][COP0_Random]).formatstr("%08X");
	state_add( MIPS3_ENTRYHI,      "EntryHi", m_core.cpr[0][COP0_EntryHi]).formatstr("%016X");
	state_add( MIPS3_ENTRYLO0,     "EntryLo0", m_core.cpr[0][COP0_EntryLo0]).formatstr("%016X");
	state_add( MIPS3_ENTRYLO1,     "EntryLo1", m_core.cpr[0][COP0_EntryLo1]).formatstr("%016X");
	state_add( MIPS3_PAGEMASK,     "PageMask", m_core.cpr[0][COP0_PageMask]).formatstr("%016X");
	state_add( MIPS3_WIRED,        "Wired", m_core.cpr[0][COP0_Wired]).formatstr("%08X");
	state_add( MIPS3_BADVADDR,     "BadVAddr", m_core.cpr[0][COP0_BadVAddr]).formatstr("%016X");

	state_add( MIPS3_CONTEXT,      "Context", m_core.cpr[0][COP0_Context]).formatstr("%016X");
	state_add( MIPS3_XCONTEXT,     "XContext", m_core.cpr[0][COP0_XContext]).formatstr("%016X");

#if USE_ABI_REG_NAMES
	state_add( MIPS3_R0,           "zero", m_core.r[0]).callimport().formatstr("%016X");   // Can't change R0
	state_add( MIPS3_R1,           "at", m_core.r[1]).formatstr("%016X");
	state_add( MIPS3_R2,           "v0", m_core.r[2]).formatstr("%016X");
	state_add( MIPS3_R3,           "v1", m_core.r[3]).formatstr("%016X");
	state_add( MIPS3_R4,           "a0", m_core.r[4]).formatstr("%016X");
	state_add( MIPS3_R5,           "a1", m_core.r[5]).formatstr("%016X");
	state_add( MIPS3_R6,           "a2", m_core.r[6]).formatstr("%016X");
	state_add( MIPS3_R7,           "a3", m_core.r[7]).formatstr("%016X");
	state_add( MIPS3_R8,           "t0", m_core.r[8]).formatstr("%016X");
	state_add( MIPS3_R9,           "t1", m_core.r[9]).formatstr("%016X");
	state_add( MIPS3_R10,          "t2", m_core.r[10]).formatstr("%016X");
	state_add( MIPS3_R11,          "t3", m_core.r[11]).formatstr("%016X");
	state_add( MIPS3_R12,          "t4", m_core.r[12]).formatstr("%016X");
	state_add( MIPS3_R13,          "t5", m_core.r[13]).formatstr("%016X");
	state_add( MIPS3_R14,          "t6", m_core.r[14]).formatstr("%016X");
	state_add( MIPS3_R15,          "t7", m_core.r[15]).formatstr("%016X");
	state_add( MIPS3_R16,          "s0", m_core.r[16]).formatstr("%016X");
	state_add( MIPS3_R17,          "s1", m_core.r[17]).formatstr("%016X");
	state_add( MIPS3_R18,          "s2", m_core.r[18]).formatstr("%016X");
	state_add( MIPS3_R19,          "s3", m_core.r[19]).formatstr("%016X");
	state_add( MIPS3_R20,          "s4", m_core.r[20]).formatstr("%016X");
	state_add( MIPS3_R21,          "s5", m_core.r[21]).formatstr("%016X");
	state_add( MIPS3_R22,          "s6", m_core.r[22]).formatstr("%016X");
	state_add( MIPS3_R23,          "s7", m_core.r[23]).formatstr("%016X");
	state_add( MIPS3_R24,          "t8", m_core.r[24]).formatstr("%016X");
	state_add( MIPS3_R25,          "t9", m_core.r[25]).formatstr("%016X");
	state_add( MIPS3_R26,          "k0", m_core.r[26]).formatstr("%016X");
	state_add( MIPS3_R27,          "k1", m_core.r[27]).formatstr("%016X");
	state_add( MIPS3_R28,          "gp", m_core.r[28]).formatstr("%016X");
	state_add( MIPS3_R29,          "sp", m_core.r[29]).formatstr("%016X");
	state_add( MIPS3_R30,          "fp", m_core.r[30]).formatstr("%016X");
	state_add( MIPS3_R31,          "ra", m_core.r[31]).formatstr("%016X");
#else
	state_add( MIPS3_R0,           "R0", m_core.r[0]).callimport().formatstr("%016X");   // Can't change R0
	state_add( MIPS3_R1,           "R1", m_core.r[1]).formatstr("%016X");
	state_add( MIPS3_R2,           "R2", m_core.r[2]).formatstr("%016X");
	state_add( MIPS3_R3,           "R3", m_core.r[3]).formatstr("%016X");
	state_add( MIPS3_R4,           "R4", m_core.r[4]).formatstr("%016X");
	state_add( MIPS3_R5,           "R5", m_core.r[5]).formatstr("%016X");
	state_add( MIPS3_R6,           "R6", m_core.r[6]).formatstr("%016X");
	state_add( MIPS3_R7,           "R7", m_core.r[7]).formatstr("%016X");
	state_add( MIPS3_R8,           "R8", m_core.r[8]).formatstr("%016X");
	state_add( MIPS3_R9,           "R9", m_core.r[9]).formatstr("%016X");
	state_add( MIPS3_R10,          "R10", m_core.r[10]).formatstr("%016X");
	state_add( MIPS3_R11,          "R11", m_core.r[11]).formatstr("%016X");
	state_add( MIPS3_R12,          "R12", m_core.r[12]).formatstr("%016X");
	state_add( MIPS3_R13,          "R13", m_core.r[13]).formatstr("%016X");
	state_add( MIPS3_R14,          "R14", m_core.r[14]).formatstr("%016X");
	state_add( MIPS3_R15,          "R15", m_core.r[15]).formatstr("%016X");
	state_add( MIPS3_R16,          "R16", m_core.r[16]).formatstr("%016X");
	state_add( MIPS3_R17,          "R17", m_core.r[17]).formatstr("%016X");
	state_add( MIPS3_R18,          "R18", m_core.r[18]).formatstr("%016X");
	state_add( MIPS3_R19,          "R19", m_core.r[19]).formatstr("%016X");
	state_add( MIPS3_R20,          "R20", m_core.r[20]).formatstr("%016X");
	state_add( MIPS3_R21,          "R21", m_core.r[21]).formatstr("%016X");
	state_add( MIPS3_R22,          "R22", m_core.r[22]).formatstr("%016X");
	state_add( MIPS3_R23,          "R23", m_core.r[23]).formatstr("%016X");
	state_add( MIPS3_R24,          "R24", m_core.r[24]).formatstr("%016X");
	state_add( MIPS3_R25,          "R25", m_core.r[25]).formatstr("%016X");
	state_add( MIPS3_R26,          "R26", m_core.r[26]).formatstr("%016X");
	state_add( MIPS3_R27,          "R27", m_core.r[27]).formatstr("%016X");
	state_add( MIPS3_R28,          "R28", m_core.r[28]).formatstr("%016X");
	state_add( MIPS3_R29,          "R29", m_core.r[29]).formatstr("%016X");
	state_add( MIPS3_R30,          "R30", m_core.r[30]).formatstr("%016X");
	state_add( MIPS3_R31,          "R31", m_core.r[31]).formatstr("%016X");
#endif
	state_add( MIPS3_HI,           "HI", m_core.hi).formatstr("%016X");
	state_add( MIPS3_LO,           "LO", m_core.lo).formatstr("%016X");

	state_add( MIPS3_CCR1_31,      "CCR31", m_core.ccr[1][31]).formatstr("%08X");

	state_add( MIPS3_FPR0,         "FPR0", m_core.cpr[1][0]).formatstr("%016X");
	state_add( MIPS3_FPS0,         "FPS0", m_core.cpr[1][0]).formatstr("%17s");
	state_add( MIPS3_FPD0,         "FPD0", m_core.cpr[1][0]).formatstr("%17s");
	state_add( MIPS3_FPR1,         "FPR1", m_core.cpr[1][1]).formatstr("%016X");
	state_add( MIPS3_FPS1,         "FPS1", m_core.cpr[1][1]).formatstr("%17s");
	state_add( MIPS3_FPD1,         "FPD1", m_core.cpr[1][1]).formatstr("%17s");
	state_add( MIPS3_FPR2,         "FPR2", m_core.cpr[1][2]).formatstr("%016X");
	state_add( MIPS3_FPS2,         "FPS2", m_core.cpr[1][2]).formatstr("%17s");
	state_add( MIPS3_FPD2,         "FPD2", m_core.cpr[1][2]).formatstr("%17s");
	state_add( MIPS3_FPR3,         "FPR3", m_core.cpr[1][3]).formatstr("%016X");
	state_add( MIPS3_FPS3,         "FPS3", m_core.cpr[1][3]).formatstr("%17s");
	state_add( MIPS3_FPD3,         "FPD3", m_core.cpr[1][3]).formatstr("%17s");
	state_add( MIPS3_FPR4,         "FPR4", m_core.cpr[1][4]).formatstr("%016X");
	state_add( MIPS3_FPS4,         "FPS4", m_core.cpr[1][4]).formatstr("%17s");
	state_add( MIPS3_FPD4,         "FPD4", m_core.cpr[1][4]).formatstr("%17s");
	state_add( MIPS3_FPR5,         "FPR5", m_core.cpr[1][5]).formatstr("%016X");
	state_add( MIPS3_FPS5,         "FPS5", m_core.cpr[1][5]).formatstr("%17s");
	state_add( MIPS3_FPD5,         "FPD5", m_core.cpr[1][5]).formatstr("%17s");
	state_add( MIPS3_FPR6,         "FPR6", m_core.cpr[1][6]).formatstr("%016X");
	state_add( MIPS3_FPS6,         "FPS6", m_core.cpr[1][6]).formatstr("%17s");
	state_add( MIPS3_FPD6,         "FPD6", m_core.cpr[1][6]).formatstr("%17s");
	state_add( MIPS3_FPR7,         "FPR7", m_core.cpr[1][7]).formatstr("%016X");
	state_add( MIPS3_FPS7,         "FPS7", m_core.cpr[1][7]).formatstr("%17s");
	state_add( MIPS3_FPD7,         "FPD7", m_core.cpr[1][7]).formatstr("%17s");
	state_add( MIPS3_FPR8,         "FPR8", m_core.cpr[1][8]).formatstr("%016X");
	state_add( MIPS3_FPS8,         "FPS8", m_core.cpr[1][8]).formatstr("%17s");
	state_add( MIPS3_FPD8,         "FPD8", m_core.cpr[1][8]).formatstr("%17s");
	state_add( MIPS3_FPR9,         "FPR9", m_core.cpr[1][9]).formatstr("%016X");
	state_add( MIPS3_FPS9,         "FPS9", m_core.cpr[1][9]).formatstr("%17s");
	state_add( MIPS3_FPD9,         "FPD9", m_core.cpr[1][9]).formatstr("%17s");
	state_add( MIPS3_FPR10,        "FPR10", m_core.cpr[1][10]).formatstr("%016X");
	state_add( MIPS3_FPS10,        "FPS10", m_core.cpr[1][10]).formatstr("%17s");
	state_add( MIPS3_FPD10,        "FPD10", m_core.cpr[1][10]).formatstr("%17s");
	state_add( MIPS3_FPR11,        "FPR11", m_core.cpr[1][11]).formatstr("%016X");
	state_add( MIPS3_FPS11,        "FPS11", m_core.cpr[1][11]).formatstr("%17s");
	state_add( MIPS3_FPD11,        "FPD11", m_core.cpr[1][11]).formatstr("%17s");
	state_add( MIPS3_FPR12,        "FPR12", m_core.cpr[1][12]).formatstr("%016X");
	state_add( MIPS3_FPS12,        "FPS12", m_core.cpr[1][12]).formatstr("%17s");
	state_add( MIPS3_FPD12,        "FPD12", m_core.cpr[1][12]).formatstr("%17s");
	state_add( MIPS3_FPR13,        "FPR13", m_core.cpr[1][13]).formatstr("%016X");
	state_add( MIPS3_FPS13,        "FPS13", m_core.cpr[1][13]).formatstr("%17s");
	state_add( MIPS3_FPD13,        "FPD13", m_core.cpr[1][13]).formatstr("%17s");
	state_add( MIPS3_FPR14,        "FPR14", m_core.cpr[1][14]).formatstr("%016X");
	state_add( MIPS3_FPS14,        "FPS14", m_core.cpr[1][14]).formatstr("%17s");
	state_add( MIPS3_FPD14,        "FPD14", m_core.cpr[1][14]).formatstr("%17s");
	state_add( MIPS3_FPR15,        "FPR15", m_core.cpr[1][15]).formatstr("%016X");
	state_add( MIPS3_FPS15,        "FPS15", m_core.cpr[1][15]).formatstr("%17s");
	state_add( MIPS3_FPD15,        "FPD15", m_core.cpr[1][15]).formatstr("%17s");
	state_add( MIPS3_FPR16,        "FPR16", m_core.cpr[1][16]).formatstr("%016X");
	state_add( MIPS3_FPS16,        "FPS16", m_core.cpr[1][16]).formatstr("%17s");
	state_add( MIPS3_FPD16,        "FPD16", m_core.cpr[1][16]).formatstr("%17s");
	state_add( MIPS3_FPR17,        "FPR17", m_core.cpr[1][17]).formatstr("%016X");
	state_add( MIPS3_FPS17,        "FPS17", m_core.cpr[1][17]).formatstr("%17s");
	state_add( MIPS3_FPD17,        "FPD17", m_core.cpr[1][17]).formatstr("%17s");
	state_add( MIPS3_FPR18,        "FPR18", m_core.cpr[1][18]).formatstr("%016X");
	state_add( MIPS3_FPS18,        "FPS18", m_core.cpr[1][18]).formatstr("%17s");
	state_add( MIPS3_FPD18,        "FPD18", m_core.cpr[1][18]).formatstr("%17s");
	state_add( MIPS3_FPR19,        "FPR19", m_core.cpr[1][19]).formatstr("%016X");
	state_add( MIPS3_FPS19,        "FPS19", m_core.cpr[1][19]).formatstr("%17s");
	state_add( MIPS3_FPD19,        "FPD19", m_core.cpr[1][19]).formatstr("%17s");
	state_add( MIPS3_FPR20,        "FPR20", m_core.cpr[1][20]).formatstr("%016X");
	state_add( MIPS3_FPS20,        "FPS20", m_core.cpr[1][20]).formatstr("%17s");
	state_add( MIPS3_FPD20,        "FPD20", m_core.cpr[1][20]).formatstr("%17s");
	state_add( MIPS3_FPR21,        "FPR21", m_core.cpr[1][21]).formatstr("%016X");
	state_add( MIPS3_FPS21,        "FPS21", m_core.cpr[1][21]).formatstr("%17s");
	state_add( MIPS3_FPD21,        "FPD21", m_core.cpr[1][21]).formatstr("%17s");
	state_add( MIPS3_FPR22,        "FPR22", m_core.cpr[1][22]).formatstr("%016X");
	state_add( MIPS3_FPS22,        "FPS22", m_core.cpr[1][22]).formatstr("%17s");
	state_add( MIPS3_FPD22,        "FPD22", m_core.cpr[1][22]).formatstr("%17s");
	state_add( MIPS3_FPR23,        "FPR23", m_core.cpr[1][23]).formatstr("%016X");
	state_add( MIPS3_FPS23,        "FPS23", m_core.cpr[1][23]).formatstr("%17s");
	state_add( MIPS3_FPD23,        "FPD23", m_core.cpr[1][23]).formatstr("%17s");
	state_add( MIPS3_FPR24,        "FPR24", m_core.cpr[1][24]).formatstr("%016X");
	state_add( MIPS3_FPS24,        "FPS24", m_core.cpr[1][24]).formatstr("%17s");
	state_add( MIPS3_FPD24,        "FPD24", m_core.cpr[1][24]).formatstr("%17s");
	state_add( MIPS3_FPR25,        "FPR25", m_core.cpr[1][25]).formatstr("%016X");
	state_add( MIPS3_FPS25,        "FPS25", m_core.cpr[1][25]).formatstr("%17s");
	state_add( MIPS3_FPD25,        "FPD25", m_core.cpr[1][25]).formatstr("%17s");
	state_add( MIPS3_FPR26,        "FPR26", m_core.cpr[1][26]).formatstr("%016X");
	state_add( MIPS3_FPS26,        "FPS26", m_core.cpr[1][26]).formatstr("%17s");
	state_add( MIPS3_FPD26,        "FPD26", m_core.cpr[1][26]).formatstr("%17s");
	state_add( MIPS3_FPR27,        "FPR27", m_core.cpr[1][27]).formatstr("%016X");
	state_add( MIPS3_FPS27,        "FPS27", m_core.cpr[1][27]).formatstr("%17s");
	state_add( MIPS3_FPD27,        "FPD27", m_core.cpr[1][27]).formatstr("%17s");
	state_add( MIPS3_FPR28,        "FPR28", m_core.cpr[1][28]).formatstr("%016X");
	state_add( MIPS3_FPS28,        "FPS28", m_core.cpr[1][28]).formatstr("%17s");
	state_add( MIPS3_FPD28,        "FPD28", m_core.cpr[1][28]).formatstr("%17s");
	state_add( MIPS3_FPR29,        "FPR29", m_core.cpr[1][29]).formatstr("%016X");
	state_add( MIPS3_FPS29,        "FPS29", m_core.cpr[1][29]).formatstr("%17s");
	state_add( MIPS3_FPD29,        "FPD29", m_core.cpr[1][29]).formatstr("%17s");
	state_add( MIPS3_FPR30,        "FPR30", m_core.cpr[1][30]).formatstr("%016X");
	state_add( MIPS3_FPS30,        "FPS30", m_core.cpr[1][30]).formatstr("%17s");
	state_add( MIPS3_FPD30,        "FPD30", m_core.cpr[1][30]).formatstr("%17s");
	state_add( MIPS3_FPR31,        "FPR31", m_core.cpr[1][31]).formatstr("%016X");
	state_add( MIPS3_FPS31,        "FPS31", m_core.cpr[1][31]).formatstr("%17s");
	state_add( MIPS3_FPD31,        "FPD31", m_core.cpr[1][31]).formatstr("%17s");

	state_add( STATE_GENPCBASE, "CURPC", m_core.pc).noshow();
	state_add( STATE_GENSP, "CURSP", m_core.r[31]).noshow();
	//state_add( STATE_GENFLAGS, "CURFLAGS", m_debugger_temp).formatstr("%1s").noshow();

	set_icountptr(m_core.icount);
}

void r4000_device::state_export(const device_state_entry &entry)
{
	switch (entry.index())
	{
		case MIPS3_COUNT:
			m_debugger_temp = (total_cycles() - m_core.count_zero_time) / 2;
			break;
	}
}


void r4000_device::state_string_export(const device_state_entry &entry, std::string &str) const
{
	switch (entry.index())
	{
		case MIPS3_FPS0:
			str = string_format("!%16g", *(float *)&m_core.cpr[1][0]);
			break;

		case MIPS3_FPD0:
			str = string_format("!%16g", *(double *)&m_core.cpr[1][0]);
			break;

		case MIPS3_FPS1:
			str = string_format("!%16g", *(float *)&m_core.cpr[1][1]);
			break;

		case MIPS3_FPD1:
			str = string_format("!%16g", *(double *)&m_core.cpr[1][1]);
			break;

		case MIPS3_FPS2:
			str = string_format("!%16g", *(float *)&m_core.cpr[1][2]);
			break;

		case MIPS3_FPD2:
			str = string_format("!%16g", *(double *)&m_core.cpr[1][2]);
			break;

		case MIPS3_FPS3:
			str = string_format("!%16g", *(float *)&m_core.cpr[1][3]);
			break;

		case MIPS3_FPD3:
			str = string_format("!%16g", *(double *)&m_core.cpr[1][3]);
			break;

		case MIPS3_FPS4:
			str = string_format("!%16g", *(float *)&m_core.cpr[1][4]);
			break;

		case MIPS3_FPD4:
			str = string_format("!%16g", *(double *)&m_core.cpr[1][4]);
			break;

		case MIPS3_FPS5:
			str = string_format("!%16g", *(float *)&m_core.cpr[1][5]);
			break;

		case MIPS3_FPD5:
			str = string_format("!%16g", *(double *)&m_core.cpr[1][5]);
			break;

		case MIPS3_FPS6:
			str = string_format("!%16g", *(float *)&m_core.cpr[1][6]);
			break;

		case MIPS3_FPD6:
			str = string_format("!%16g", *(double *)&m_core.cpr[1][6]);
			break;

		case MIPS3_FPS7:
			str = string_format("!%16g", *(float *)&m_core.cpr[1][7]);
			break;

		case MIPS3_FPD7:
			str = string_format("!%16g", *(double *)&m_core.cpr[1][7]);
			break;

		case MIPS3_FPS8:
			str = string_format("!%16g", *(float *)&m_core.cpr[1][8]);
			break;

		case MIPS3_FPD8:
			str = string_format("!%16g", *(double *)&m_core.cpr[1][8]);
			break;

		case MIPS3_FPS9:
			str = string_format("!%16g", *(float *)&m_core.cpr[1][9]);
			break;

		case MIPS3_FPD9:
			str = string_format("!%16g", *(double *)&m_core.cpr[1][9]);
			break;

		case MIPS3_FPS10:
			str = string_format("!%16g", *(float *)&m_core.cpr[1][10]);
			break;

		case MIPS3_FPD10:
			str = string_format("!%16g", *(double *)&m_core.cpr[1][10]);
			break;

		case MIPS3_FPS11:
			str = string_format("!%16g", *(float *)&m_core.cpr[1][11]);
			break;

		case MIPS3_FPD11:
			str = string_format("!%16g", *(double *)&m_core.cpr[1][11]);
			break;

		case MIPS3_FPS12:
			str = string_format("!%16g", *(float *)&m_core.cpr[1][12]);
			break;

		case MIPS3_FPD12:
			str = string_format("!%16g", *(double *)&m_core.cpr[1][12]);
			break;

		case MIPS3_FPS13:
			str = string_format("!%16g", *(float *)&m_core.cpr[1][13]);
			break;

		case MIPS3_FPD13:
			str = string_format("!%16g", *(double *)&m_core.cpr[1][13]);
			break;

		case MIPS3_FPS14:
			str = string_format("!%16g", *(float *)&m_core.cpr[1][14]);
			break;

		case MIPS3_FPD14:
			str = string_format("!%16g", *(double *)&m_core.cpr[1][14]);
			break;

		case MIPS3_FPS15:
			str = string_format("!%16g", *(float *)&m_core.cpr[1][15]);
			break;

		case MIPS3_FPD15:
			str = string_format("!%16g", *(double *)&m_core.cpr[1][15]);
			break;

		case MIPS3_FPS16:
			str = string_format("!%16g", *(float *)&m_core.cpr[1][16]);
			break;

		case MIPS3_FPD16:
			str = string_format("!%16g", *(double *)&m_core.cpr[1][16]);
			break;

		case MIPS3_FPS17:
			str = string_format("!%16g", *(float *)&m_core.cpr[1][17]);
			break;

		case MIPS3_FPD17:
			str = string_format("!%16g", *(double *)&m_core.cpr[1][17]);
			break;

		case MIPS3_FPS18:
			str = string_format("!%16g", *(float *)&m_core.cpr[1][18]);
			break;

		case MIPS3_FPD18:
			str = string_format("!%16g", *(double *)&m_core.cpr[1][18]);
			break;

		case MIPS3_FPS19:
			str = string_format("!%16g", *(float *)&m_core.cpr[1][19]);
			break;

		case MIPS3_FPD19:
			str = string_format("!%16g", *(double *)&m_core.cpr[1][19]);
			break;

		case MIPS3_FPS20:
			str = string_format("!%16g", *(float *)&m_core.cpr[1][20]);
			break;

		case MIPS3_FPD20:
			str = string_format("!%16g", *(double *)&m_core.cpr[1][20]);
			break;

		case MIPS3_FPS21:
			str = string_format("!%16g", *(float *)&m_core.cpr[1][21]);
			break;

		case MIPS3_FPD21:
			str = string_format("!%16g", *(double *)&m_core.cpr[1][21]);
			break;

		case MIPS3_FPS22:
			str = string_format("!%16g", *(float *)&m_core.cpr[1][22]);
			break;

		case MIPS3_FPD22:
			str = string_format("!%16g", *(double *)&m_core.cpr[1][22]);
			break;

		case MIPS3_FPS23:
			str = string_format("!%16g", *(float *)&m_core.cpr[1][23]);
			break;

		case MIPS3_FPD23:
			str = string_format("!%16g", *(double *)&m_core.cpr[1][23]);
			break;

		case MIPS3_FPS24:
			str = string_format("!%16g", *(float *)&m_core.cpr[1][24]);
			break;

		case MIPS3_FPD24:
			str = string_format("!%16g", *(double *)&m_core.cpr[1][24]);
			break;

		case MIPS3_FPS25:
			str = string_format("!%16g", *(float *)&m_core.cpr[1][25]);
			break;

		case MIPS3_FPD25:
			str = string_format("!%16g", *(double *)&m_core.cpr[1][25]);
			break;

		case MIPS3_FPS26:
			str = string_format("!%16g", *(float *)&m_core.cpr[1][26]);
			break;

		case MIPS3_FPD26:
			str = string_format("!%16g", *(double *)&m_core.cpr[1][26]);
			break;

		case MIPS3_FPS27:
			str = string_format("!%16g", *(float *)&m_core.cpr[1][27]);
			break;

		case MIPS3_FPD27:
			str = string_format("!%16g", *(double *)&m_core.cpr[1][27]);
			break;

		case MIPS3_FPS28:
			str = string_format("!%16g", *(float *)&m_core.cpr[1][28]);
			break;

		case MIPS3_FPD28:
			str = string_format("!%16g", *(double *)&m_core.cpr[1][28]);
			break;

		case MIPS3_FPS29:
			str = string_format("!%16g", *(float *)&m_core.cpr[1][29]);
			break;

		case MIPS3_FPD29:
			str = string_format("!%16g", *(double *)&m_core.cpr[1][29]);
			break;

		case MIPS3_FPS30:
			str = string_format("!%16g", *(float *)&m_core.cpr[1][30]);
			break;

		case MIPS3_FPD30:
			str = string_format("!%16g", *(double *)&m_core.cpr[1][30]);
			break;

		case MIPS3_FPS31:
			str = string_format("!%16g", *(float *)&m_core.cpr[1][31]);
			break;

		case MIPS3_FPD31:
			str = string_format("!%16g", *(double *)&m_core.cpr[1][31]);
			break;

		case STATE_GENFLAGS:
			str = " ";
			break;
	}
}


void r4000_device::device_reset()
{
	/* common reset */
	memset(m_cf, 0, sizeof(m_cf));

	/* initialize the state */
	m_branch_state = NONE;
	m_core.pc = 0xffffffff'bfc00000;

	m_core.cpr[0][COP0_Status] = SR_BEV | SR_ERL;
	m_core.cpr[0][COP0_Wired] = 0;
	m_core.cpr[0][COP0_Compare] = 0xffffffff;
	m_core.cpr[0][COP0_Count] = 0;
	m_core.cpr[0][COP0_Config] = compute_config_register();
	m_core.cpr[0][COP0_PRId] = compute_prid_register();
	m_core.count_zero_time = total_cycles();

	m_64 = false;
}

std::unique_ptr<util::disasm_interface> r4000_device::create_disassembler()
{
	return std::make_unique<mips3_disassembler>();
}

/***************************************************************************
    COP0 (SYSTEM) EXECUTION HANDLING
***************************************************************************/

u64 r4000_device::get_cop0_reg(int index)
{
	switch (index)
	{
	case COP0_Count:
		return u32((total_cycles() - m_core.count_zero_time) / 2);

	case COP0_Random:
		{
			u8 const wired = m_core.cpr[0][COP0_Wired] & 0x3f;

			if (wired < 48)
				return ((total_cycles() - m_core.count_zero_time) % (48 - wired) + wired) & 0x3f;
			else
				return 47;
		}
	}

	return m_core.cpr[0][index];
}

void r4000_device::set_cop0_reg(int index, u64 val)
{
	switch (index)
	{
		case COP0_Cause:
			CAUSE = (CAUSE & ~0x300) | (val & 0x300);
			if (val & 0x300)
				m_core.icount = 0; // FIXME: force immediate software interrupt detection
			break;

		case COP0_Status:
		{
			/* update interrupts and cycle counting */
			u32 diff = m_core.cpr[0][index] ^ val;
			//if (val & 0xe0)
			//  logerror("64-bit addressing mode PC=0x%08x SR=0x%08x data=0x%08X\n", m_core.pc, m_core.cpr[0][index], val);
			m_core.cpr[0][index] = val;
			if (diff & 0x8000)
				mips3com_update_cycle_counting();
			mode_check();

			if (diff & 0x300)
				m_core.icount = 0; // FIXME: force immediate software interrupt detection

			if (val & SR_RE)
				logerror("warning: reverse endian enabled (%s)\n", machine().describe_context());
			break;
		}

		case COP0_Count:
			m_core.cpr[0][index] = val;
			m_core.count_zero_time = total_cycles() - ((u64)(u32)val * 2);
			mips3com_update_cycle_counting();
			break;

		case COP0_Compare:
			m_core.compare_armed = 1;
			CAUSE &= ~0x8000;
			m_core.cpr[0][index] = val & 0xffffffff;
			mips3com_update_cycle_counting();
			break;

		case COP0_PRId:
			break;

		case COP0_Config:
			m_core.cpr[0][index] = (m_core.cpr[0][index] & ~7) | (val & 7);
			break;

		case COP0_EntryHi:
			// TODO: force read-only fields
			m_core.cpr[0][index] = val;
			break;

		default:
			m_core.cpr[0][index] = val;
			break;
	}
}

inline u64 r4000_device::get_cop0_creg(int index)
{
	return m_core.ccr[0][index];
}

inline void r4000_device::set_cop0_creg(int index, u64 val)
{
	m_core.ccr[0][index] = val;
}

void r4000_device::handle_cop0(u32 op)
{
	if ((SR & SR_KSU) && !(SR & SR_CU0) && !(SR & (SR_EXL | SR_ERL)))
	{
		generate_exception(EXCEPTION_CPU0);
		return;
	}

	switch (RSREG)
	{
		case 0x00: // MFCz
			if (RTREG)
				m_core.r[RTREG] = s32(get_cop0_reg(RDREG));
			break;
		case 0x01: // DMFCz
			if (RTREG)
				m_core.r[RTREG] = get_cop0_reg(RDREG);
			break;
		case 0x02:  /* CFCz */      if (RTREG) m_core.r[RTREG] = s32(get_cop0_creg(RDREG));       break;
		case 0x04:  /* MTCz */      set_cop0_reg(RDREG, s64(s32(m_core.r[RTREG])));                           break;
		case 0x05:  /* DMTCz */     set_cop0_reg(RDREG, m_core.r[RTREG]);                           break;
		case 0x06:  /* CTCz */      set_cop0_creg(RDREG, u32(m_core.r[RTREG]));                          break;
		case 0x08:  /* BC */
			switch (RTREG)
			{
				case 0x00:  /* BCzF */  if (!m_cf[0]) ADDPC(s16(op));               break;
				case 0x01:  /* BCzF */  if (m_cf[0]) ADDPC(s16(op));                break;
				case 0x02:  /* BCzFL */ invalid_instruction(op);                            break;
				case 0x03:  /* BCzTL */ invalid_instruction(op);                            break;
				default:    invalid_instruction(op);                                        break;
			}
			break;
		case 0x10:
		case 0x11:
		case 0x12:
		case 0x13:
		case 0x14:
		case 0x15:
		case 0x16:
		case 0x17:
		case 0x18:
		case 0x19:
		case 0x1a:
		case 0x1b:
		case 0x1c:
		case 0x1d:
		case 0x1e:
		case 0x1f:  /* COP */
			switch (op & 0x01ffffff)
			{
				case 0x01:  /* TLBR */
					mips3com_tlbr();
					break;

				case 0x02:  /* TLBWI */
					mips3com_tlbwi();
					break;

				case 0x06:  /* TLBWR */
					mips3com_tlbwr();
					break;

				case 0x08:  /* TLBP */
					mips3com_tlbp();
					break;

				case 0x10:  /* RFE */   invalid_instruction(op);                            break;
				case 0x18:  /* ERET */
					if (SR & SR_ERL)
					{
						logerror("eret from error\n");
						m_branch_state = EXCEPTION;
						m_core.pc = m_core.cpr[0][COP0_ErrorEPC];
						SR &= ~SR_ERL;
					}
					else
					{
						m_branch_state = EXCEPTION;
						m_core.pc = m_core.cpr[0][COP0_EPC];
						SR &= ~SR_EXL;
					}
					m_lld_value ^= 0xffffffff;
					m_ll_value ^= 0xffffffff;
					mode_check();
					break;
				case 0x20:  /* WAIT */                                                      break;
				default:    invalid_instruction(op);                                        break;
			}
			break;
		default:    invalid_instruction(op);                                                break;
	}
}


/***************************************************************************
    COP1 (FPU) EXECUTION HANDLING
***************************************************************************/

inline u32 r4000_device::get_cop1_reg32(int index)
{
	if (!(SR & SR_FR))
		return ((u32 *)&m_core.cpr[1][index & 0x1E])[index & 1];
	else
		return m_core.cpr[1][index];
}

inline u64 r4000_device::get_cop1_reg64(int index)
{
	if (!(SR & SR_FR))
		index &= 0x1E;
	return m_core.cpr[1][index];
}

inline void r4000_device::set_cop1_reg32(int index, u32 val)
{
	if (!(SR & SR_FR))
		((u32 *)&m_core.cpr[1][index & 0x1E])[index & 1] = val;
	else
		m_core.cpr[1][index] = val;
}

inline void r4000_device::set_cop1_reg64(int index, u64 val)
{
	if (!(SR & SR_FR))
		index &= 0x1E;
	m_core.cpr[1][index] = val;
}

inline u64 r4000_device::get_cop1_creg(int index)
{
	if (index == 31)
	{
		u32 result = m_core.ccr[1][31] & ~0xfe800000;
		int i;

		for (i = 0; i < 8; i++)
			if (m_cf[1][i])
				result |= 1 << fcc_shift[i];
		return result;
	}
	return m_core.ccr[1][index];
}

inline void r4000_device::set_cop1_creg(int index, u64 val)
{
	m_core.ccr[1][index] = val;
	if (index == 31)
	{
		int i;

		for (i = 0; i < 8; i++)
			m_cf[1][i] = (val >> fcc_shift[i]) & 1;
	}
}

void r4000_device::handle_cop1_fr0(u32 op)
{
	double dtemp;

	/* note: additional condition codes available on R5000 only */

	if (!(SR & SR_CU1))
	{
		generate_exception(EXCEPTION_CPU1);
		return;
	}

	switch (RSREG)
	{
		case 0x00:  /* MFCz */      if (RTREG) m_core.r[RTREG] = (s32)get_cop1_reg32(RDREG);      break;
		case 0x01:  /* DMFCz */     if (RTREG) m_core.r[RTREG] = get_cop1_reg64(RDREG);             break;
		case 0x02:  /* CFCz */      if (RTREG) m_core.r[RTREG] = (s32)get_cop1_creg(RDREG);       break;
		case 0x04:  /* MTCz */      set_cop1_reg32(RDREG, u32(m_core.r[RTREG]));                         break;
		case 0x05:  /* DMTCz */     set_cop1_reg64(RDREG, m_core.r[RTREG]);                         break;
		case 0x06:  /* CTCz */      set_cop1_creg(RDREG, u32(m_core.r[RTREG]));                          break;
		case 0x08:  /* BC */
			switch ((op >> 16) & 3)
			{
				case 0x00:  /* BCzF */  if (!GET_FCC((op >> 18) & 7)) ADDPC(s16(op));   break;
				case 0x01:  /* BCzT */  if (GET_FCC((op >> 18) & 7)) ADDPC(s16(op));    break;
				case 0x02:  /* BCzFL */ if (!GET_FCC((op >> 18) & 7)) ADDPC(s16(op)); else m_branch_state = NULLIFY;  break;
				case 0x03:  /* BCzTL */ if (GET_FCC((op >> 18) & 7)) ADDPC(s16(op)); else m_branch_state = NULLIFY;   break;
			}
			break;
		default:
			switch (op & 0x3f)
			{
				case 0x00:
					if (IS_SINGLE(op))  /* ADD.S */
						FDVALS_FR0 = FSVALS_FR0 + FTVALS_FR0;
					else                /* ADD.D */
						FDVALD_FR0 = FSVALD_FR0 + FTVALD_FR0;
					break;

				case 0x01:
					if (IS_SINGLE(op))  /* SUB.S */
						FDVALS_FR0 = FSVALS_FR0 - FTVALS_FR0;
					else                /* SUB.D */
						FDVALD_FR0 = FSVALD_FR0 - FTVALD_FR0;
					break;

				case 0x02:
					if (IS_SINGLE(op))  /* MUL.S */
						FDVALS_FR0 = FSVALS_FR0 * FTVALS_FR0;
					else                /* MUL.D */
						FDVALD_FR0 = FSVALD_FR0 * FTVALD_FR0;
					break;

				case 0x03:
					if (IS_SINGLE(op))  /* DIV.S */
						FDVALS_FR0 = FSVALS_FR0 / FTVALS_FR0;
					else                /* DIV.D */
						FDVALD_FR0 = FSVALD_FR0 / FTVALD_FR0;
					break;

				case 0x04:
					if (IS_SINGLE(op))  /* SQRT.S */
						FDVALS_FR0 = sqrt(FSVALS_FR0);
					else                /* SQRT.D */
						FDVALD_FR0 = sqrt(FSVALD_FR0);
					break;

				case 0x05:
					if (IS_SINGLE(op))  /* ABS.S */
						FDVALS_FR0 = fabs(FSVALS_FR0);
					else                /* ABS.D */
						FDVALD_FR0 = fabs(FSVALD_FR0);
					break;

				case 0x06:
					if (IS_SINGLE(op))  /* MOV.S */
						FDVALS_FR0 = FSVALS_FR0;
					else                /* MOV.D */
						FDVALD_FR0 = FSVALD_FR0;
					break;

				case 0x07:
					if (IS_SINGLE(op))  /* NEG.S */
						FDVALS_FR0 = -FSVALS_FR0;
					else                /* NEG.D */
						FDVALD_FR0 = -FSVALD_FR0;
					break;

				case 0x08:
					if (IS_SINGLE(op))  /* ROUND.L.S */
					{
						double temp = FSVALS_FR0;
						if (temp < 0)
							temp = ceil(temp - 0.5);
						else
							temp = floor(temp + 0.5);
						FDVALL_FR0 = (s64)temp;
					}
					else                /* ROUND.L.D */
					{
						double temp = FSVALD_FR0;
						if (temp < 0)
							temp = ceil(temp - 0.5);
						else
							temp = floor(temp + 0.5);
						FDVALL_FR0 = (s64)temp;
					}
					break;

				case 0x09:
					if (IS_SINGLE(op))  /* TRUNC.L.S */
					{
						double temp = FSVALS_FR0;
						if (temp < 0)
							temp = ceil(temp);
						else
							temp = floor(temp);
						FDVALL_FR0 = (s64)temp;
					}
					else                /* TRUNC.L.D */
					{
						double temp = FSVALD_FR0;
						if (temp < 0)
							temp = ceil(temp);
						else
							temp = floor(temp);
						FDVALL_FR0 = (s64)temp;
					}
					break;

				case 0x0a:
					if (IS_SINGLE(op))  /* CEIL.L.S */
						dtemp = ceil(FSVALS_FR0);
					else                /* CEIL.L.D */
						dtemp = ceil(FSVALD_FR0);
					FDVALL_FR0 = (s64)dtemp;
					break;

				case 0x0b:
					if (IS_SINGLE(op))  /* FLOOR.L.S */
						dtemp = floor(FSVALS_FR0);
					else                /* FLOOR.L.D */
						dtemp = floor(FSVALD_FR0);
					FDVALL_FR0 = (s64)dtemp;
					break;

				case 0x0c:
					if (IS_SINGLE(op))  /* ROUND.W.S */
					{
						dtemp = FSVALS_FR0;
						if (dtemp < 0)
							dtemp = ceil(dtemp - 0.5);
						else
							dtemp = floor(dtemp + 0.5);
						FDVALW_FR0 = (s32)dtemp;
					}
					else                /* ROUND.W.D */
					{
						dtemp = FSVALD_FR0;
						if (dtemp < 0)
							dtemp = ceil(dtemp - 0.5);
						else
							dtemp = floor(dtemp + 0.5);
						FDVALW_FR0 = (s32)dtemp;
					}
					break;

				case 0x0d:
					if (IS_SINGLE(op))  /* TRUNC.W.S */
					{
						dtemp = FSVALS_FR0;
						if (dtemp < 0)
							dtemp = ceil(dtemp);
						else
							dtemp = floor(dtemp);
						FDVALW_FR0 = (s32)dtemp;
					}
					else                /* TRUNC.W.D */
					{
						dtemp = FSVALD_FR0;
						if (dtemp < 0)
							dtemp = ceil(dtemp);
						else
							dtemp = floor(dtemp);
						FDVALW_FR0 = (s32)dtemp;
					}
					break;

				case 0x0e:
					if (IS_SINGLE(op))  /* CEIL.W.S */
						dtemp = ceil(FSVALS_FR0);
					else                /* CEIL.W.D */
						dtemp = ceil(FSVALD_FR0);
					FDVALW_FR0 = (s32)dtemp;
					break;

				case 0x0f:
					if (IS_SINGLE(op))  /* FLOOR.W.S */
						dtemp = floor(FSVALS_FR0);
					else                /* FLOOR.W.D */
						dtemp = floor(FSVALD_FR0);
					FDVALW_FR0 = (s32)dtemp;
					break;

				case 0x11:  /* R5000 */
					if (GET_FCC((op >> 18) & 7) == ((op >> 16) & 1))
					{
						if (IS_SINGLE(op))  /* MOVT/F.S */
							FDVALS_FR0 = FSVALS_FR0;
						else                /* MOVT/F.D */
							FDVALD_FR0 = FSVALD_FR0;
					}
					break;

				case 0x12:  /* R5000 */
					if (m_core.r[RTREG] == 0)
					{
						if (IS_SINGLE(op))  /* MOVZ.S */
							FDVALS_FR0 = FSVALS_FR0;
						else                /* MOVZ.D */
							FDVALD_FR0 = FSVALD_FR0;
					}
					break;

				case 0x13:  /* R5000 */
					if (m_core.r[RTREG] != 0)
					{
						if (IS_SINGLE(op))  /* MOVN.S */
							FDVALS_FR0 = FSVALS_FR0;
						else                /* MOVN.D */
							FDVALD_FR0 = FSVALD_FR0;
					}
					break;

				case 0x15:  /* R5000 */
					if (IS_SINGLE(op))  /* RECIP.S */
						FDVALS_FR0 = 1.0f / FSVALS_FR0;
					else                /* RECIP.D */
						FDVALD_FR0 = 1.0 / FSVALD_FR0;
					break;

				case 0x16:  /* R5000 */
					if (IS_SINGLE(op))  /* RSQRT.S */
						FDVALS_FR0 = 1.0f / sqrt(FSVALS_FR0);
					else                /* RSQRT.D */
						FDVALD_FR0 = 1.0 / sqrt(FSVALD_FR0);
					break;

				case 0x20:
					if (IS_INTEGRAL(op))
					{
						if (IS_SINGLE(op))  /* CVT.S.W */
							FDVALS_FR0 = (s32)FSVALW_FR0;
						else                /* CVT.S.L */
							FDVALS_FR0 = (s64)FSVALL_FR0;
					}
					else                    /* CVT.S.D */
						FDVALS_FR0 = FSVALD_FR0;
					break;

				case 0x21:
					if (IS_INTEGRAL(op))
					{
						if (IS_SINGLE(op))  /* CVT.D.W */
							FDVALD_FR0 = (s32)FSVALW_FR0;
						else                /* CVT.D.L */
							FDVALD_FR0 = (s64)FSVALL_FR0;
					}
					else                    /* CVT.D.S */
						FDVALD_FR0 = FSVALS_FR0;
					break;

				case 0x24:
					if (IS_SINGLE(op))  /* CVT.W.S */
						FDVALW_FR0 = (s32)FSVALS_FR0;
					else
						FDVALW_FR0 = (s32)FSVALD_FR0;
					break;

				case 0x25:
					if (IS_SINGLE(op))  /* CVT.L.S */
						FDVALL_FR0 = (s64)FSVALS_FR0;
					else                /* CVT.L.D */
						FDVALL_FR0 = (s64)FSVALD_FR0;
					break;

				case 0x30:
				case 0x38:
					if (IS_SINGLE(op))  /* C.F.S */
						SET_FCC((op >> 8) & 7, 0);
					else                /* C.F.D */
						SET_FCC((op >> 8) & 7, 0);
					break;

				case 0x31:
				case 0x39:
					if (IS_SINGLE(op))  /* C.UN.S */
						SET_FCC((op >> 8) & 7, 0);
					else                /* C.UN.D */
						SET_FCC((op >> 8) & 7, 0);
					break;

				case 0x32:
				case 0x3a:
					if (IS_SINGLE(op))  /* C.EQ.S */
						SET_FCC((op >> 8) & 7, (FSVALS_FR0 == FTVALS_FR0));
					else                /* C.EQ.D */
						SET_FCC((op >> 8) & 7, (FSVALD_FR0 == FTVALD_FR0));
					break;

				case 0x33:
				case 0x3b:
					if (IS_SINGLE(op))  /* C.UEQ.S */
						SET_FCC((op >> 8) & 7, (FSVALS_FR0 == FTVALS_FR0));
					else                /* C.UEQ.D */
						SET_FCC((op >> 8) & 7, (FSVALD_FR0 == FTVALD_FR0));
					break;

				case 0x34:
				case 0x3c:
					if (IS_SINGLE(op))  /* C.OLT.S */
						SET_FCC((op >> 8) & 7, (FSVALS_FR0 < FTVALS_FR0));
					else                /* C.OLT.D */
						SET_FCC((op >> 8) & 7, (FSVALD_FR0 < FTVALD_FR0));
					break;

				case 0x35:
				case 0x3d:
					if (IS_SINGLE(op))  /* C.ULT.S */
						SET_FCC((op >> 8) & 7, (FSVALS_FR0 < FTVALS_FR0));
					else                /* C.ULT.D */
						SET_FCC((op >> 8) & 7, (FSVALD_FR0 < FTVALD_FR0));
					break;

				case 0x36:
				case 0x3e:
					if (IS_SINGLE(op))  /* C.OLE.S */
						SET_FCC((op >> 8) & 7, (FSVALS_FR0 <= FTVALS_FR0));
					else                /* C.OLE.D */
						SET_FCC((op >> 8) & 7, (FSVALD_FR0 <= FTVALD_FR0));
					break;

				case 0x37:
				case 0x3f:
					if (IS_SINGLE(op))  /* C.ULE.S */
						SET_FCC((op >> 8) & 7, (FSVALS_FR0 <= FTVALS_FR0));
					else                /* C.ULE.D */
						SET_FCC((op >> 8) & 7, (FSVALD_FR0 <= FTVALD_FR0));
					break;

				default:
					invalid_instruction(op);
					break;
			}
			break;
	}
}

void r4000_device::handle_cop1_fr1(u32 op)
{
	double dtemp;

	/* note: additional condition codes available on R5000 only */

	if (!(SR & SR_CU1))
	{
		generate_exception(EXCEPTION_CPU1);
		return;
	}

	switch (RSREG)
	{
		case 0x00:  /* MFCz */      if (RTREG) m_core.r[RTREG] = (s32)get_cop1_reg32(RDREG);      break;
		case 0x01:  /* DMFCz */     if (RTREG) m_core.r[RTREG] = get_cop1_reg64(RDREG);             break;
		case 0x02:  /* CFCz */      if (RTREG) m_core.r[RTREG] = (s32)get_cop1_creg(RDREG);       break;
		case 0x04:  /* MTCz */      set_cop1_reg32(RDREG, u32(m_core.r[RTREG]));                         break;
		case 0x05:  /* DMTCz */     set_cop1_reg64(RDREG, m_core.r[RTREG]);                         break;
		case 0x06:  /* CTCz */      set_cop1_creg(RDREG, u32(m_core.r[RTREG]));                          break;
		case 0x08:  /* BC */
			switch ((op >> 16) & 3)
			{
				case 0x00:  /* BCzF */  if (!GET_FCC((op >> 18) & 7)) ADDPC(s16(op));   break;
				case 0x01:  /* BCzT */  if (GET_FCC((op >> 18) & 7)) ADDPC(s16(op));    break;
				case 0x02:  /* BCzFL */ if (!GET_FCC((op >> 18) & 7)) ADDPC(s16(op)); else m_branch_state = NULLIFY;  break;
				case 0x03:  /* BCzTL */ if (GET_FCC((op >> 18) & 7)) ADDPC(s16(op)); else m_branch_state = NULLIFY;   break;
			}
			break;
		default:
			switch (op & 0x3f)
			{
				case 0x00:
					if (IS_SINGLE(op))  /* ADD.S */
						FDVALS_FR1 = FSVALS_FR1 + FTVALS_FR1;
					else                /* ADD.D */
						FDVALD_FR1 = FSVALD_FR1 + FTVALD_FR1;
					break;

				case 0x01:
					if (IS_SINGLE(op))  /* SUB.S */
						FDVALS_FR1 = FSVALS_FR1 - FTVALS_FR1;
					else                /* SUB.D */
						FDVALD_FR1 = FSVALD_FR1 - FTVALD_FR1;
					break;

				case 0x02:
					if (IS_SINGLE(op))  /* MUL.S */
						FDVALS_FR1 = FSVALS_FR1 * FTVALS_FR1;
					else                /* MUL.D */
						FDVALD_FR1 = FSVALD_FR1 * FTVALD_FR1;
					break;

				case 0x03:
					if (IS_SINGLE(op))  /* DIV.S */
						FDVALS_FR1 = FSVALS_FR1 / FTVALS_FR1;
					else                /* DIV.D */
						FDVALD_FR1 = FSVALD_FR1 / FTVALD_FR1;
					break;

				case 0x04:
					if (IS_SINGLE(op))  /* SQRT.S */
						FDVALS_FR1 = sqrt(FSVALS_FR1);
					else                /* SQRT.D */
						FDVALD_FR1 = sqrt(FSVALD_FR1);
					break;

				case 0x05:
					if (IS_SINGLE(op))  /* ABS.S */
						FDVALS_FR1 = fabs(FSVALS_FR1);
					else                /* ABS.D */
						FDVALD_FR1 = fabs(FSVALD_FR1);
					break;

				case 0x06:
					if (IS_SINGLE(op))  /* MOV.S */
						FDVALS_FR1 = FSVALS_FR1;
					else                /* MOV.D */
						FDVALD_FR1 = FSVALD_FR1;
					break;

				case 0x07:
					if (IS_SINGLE(op))  /* NEG.S */
						FDVALS_FR1 = -FSVALS_FR1;
					else                /* NEG.D */
						FDVALD_FR1 = -FSVALD_FR1;
					break;

				case 0x08:
					if (IS_SINGLE(op))  /* ROUND.L.S */
					{
						double temp = FSVALS_FR1;
						if (temp < 0)
							temp = ceil(temp - 0.5);
						else
							temp = floor(temp + 0.5);
						FDVALL_FR1 = (s64)temp;
					}
					else                /* ROUND.L.D */
					{
						double temp = FSVALD_FR1;
						if (temp < 0)
							temp = ceil(temp - 0.5);
						else
							temp = floor(temp + 0.5);
						FDVALL_FR1 = (s64)temp;
					}
					break;

				case 0x09:
					if (IS_SINGLE(op))  /* TRUNC.L.S */
					{
						double temp = FSVALS_FR1;
						if (temp < 0)
							temp = ceil(temp);
						else
							temp = floor(temp);
						FDVALL_FR1 = (s64)temp;
					}
					else                /* TRUNC.L.D */
					{
						double temp = FSVALD_FR1;
						if (temp < 0)
							temp = ceil(temp);
						else
							temp = floor(temp);
						FDVALL_FR1 = (s64)temp;
					}
					break;

				case 0x0a:
					if (IS_SINGLE(op))  /* CEIL.L.S */
						dtemp = ceil(FSVALS_FR1);
					else                /* CEIL.L.D */
						dtemp = ceil(FSVALD_FR1);
					FDVALL_FR1 = (s64)dtemp;
					break;

				case 0x0b:
					if (IS_SINGLE(op))  /* FLOOR.L.S */
						dtemp = floor(FSVALS_FR1);
					else                /* FLOOR.L.D */
						dtemp = floor(FSVALD_FR1);
					FDVALL_FR1 = (s64)dtemp;
					break;

				case 0x0c:
					if (IS_SINGLE(op))  /* ROUND.W.S */
					{
						dtemp = FSVALS_FR1;
						if (dtemp < 0)
							dtemp = ceil(dtemp - 0.5);
						else
							dtemp = floor(dtemp + 0.5);
						FDVALW_FR1 = (s32)dtemp;
					}
					else                /* ROUND.W.D */
					{
						dtemp = FSVALD_FR1;
						if (dtemp < 0)
							dtemp = ceil(dtemp - 0.5);
						else
							dtemp = floor(dtemp + 0.5);
						FDVALW_FR1 = (s32)dtemp;
					}
					break;

				case 0x0d:
					if (IS_SINGLE(op))  /* TRUNC.W.S */
					{
						dtemp = FSVALS_FR1;
						if (dtemp < 0)
							dtemp = ceil(dtemp);
						else
							dtemp = floor(dtemp);
						FDVALW_FR1 = (s32)dtemp;
					}
					else                /* TRUNC.W.D */
					{
						dtemp = FSVALD_FR1;
						if (dtemp < 0)
							dtemp = ceil(dtemp);
						else
							dtemp = floor(dtemp);
						FDVALW_FR1 = (s32)dtemp;
					}
					break;

				case 0x0e:
					if (IS_SINGLE(op))  /* CEIL.W.S */
						dtemp = ceil(FSVALS_FR1);
					else                /* CEIL.W.D */
						dtemp = ceil(FSVALD_FR1);
					FDVALW_FR1 = (s32)dtemp;
					break;

				case 0x0f:
					if (IS_SINGLE(op))  /* FLOOR.W.S */
						dtemp = floor(FSVALS_FR1);
					else                /* FLOOR.W.D */
						dtemp = floor(FSVALD_FR1);
					FDVALW_FR1 = (s32)dtemp;
					break;

				case 0x11:  /* R5000 */
					if (GET_FCC((op >> 18) & 7) == ((op >> 16) & 1))
					{
						if (IS_SINGLE(op))  /* MOVT/F.S */
							FDVALS_FR1 = FSVALS_FR1;
						else                /* MOVT/F.D */
							FDVALD_FR1 = FSVALD_FR1;
					}
					break;

				case 0x12:  /* R5000 */
					if (m_core.r[RTREG] == 0)
					{
						if (IS_SINGLE(op))  /* MOVZ.S */
							FDVALS_FR1 = FSVALS_FR1;
						else                /* MOVZ.D */
							FDVALD_FR1 = FSVALD_FR1;
					}
					break;

				case 0x13:  /* R5000 */
					if (m_core.r[RTREG] != 0)
					{
						if (IS_SINGLE(op))  /* MOVN.S */
							FDVALS_FR1 = FSVALS_FR1;
						else                /* MOVN.D */
							FDVALD_FR1 = FSVALD_FR1;
					}
					break;

				case 0x15:  /* R5000 */
					if (IS_SINGLE(op))  /* RECIP.S */
						FDVALS_FR1 = 1.0f / FSVALS_FR1;
					else                /* RECIP.D */
						FDVALD_FR1 = 1.0 / FSVALD_FR1;
					break;

				case 0x16:  /* R5000 */
					if (IS_SINGLE(op))  /* RSQRT.S */
						FDVALS_FR1 = 1.0f / sqrt(FSVALS_FR1);
					else                /* RSQRT.D */
						FDVALD_FR1 = 1.0 / sqrt(FSVALD_FR1);
					break;

				case 0x20:
					if (IS_INTEGRAL(op))
					{
						if (IS_SINGLE(op))  /* CVT.S.W */
							FDVALS_FR1 = (s32)FSVALW_FR1;
						else                /* CVT.S.L */
							FDVALS_FR1 = (s64)FSVALL_FR1;
					}
					else                    /* CVT.S.D */
						FDVALS_FR1 = FSVALD_FR1;
					break;

				case 0x21:
					if (IS_INTEGRAL(op))
					{
						if (IS_SINGLE(op))  /* CVT.D.W */
							FDVALD_FR1 = (s32)FSVALW_FR1;
						else                /* CVT.D.L */
							FDVALD_FR1 = (s64)FSVALL_FR1;
					}
					else                    /* CVT.D.S */
						FDVALD_FR1 = FSVALS_FR1;
					break;

				case 0x24:
if (IS_SINGLE(op))  /* CVT.W.S */
FDVALW_FR1 = (s32)FSVALS_FR1;
else
FDVALW_FR1 = (s32)FSVALD_FR1;
break;

				case 0x25:
					if (IS_SINGLE(op))  /* CVT.L.S */
						FDVALL_FR1 = (s64)FSVALS_FR1;
					else                /* CVT.L.D */
						FDVALL_FR1 = (s64)FSVALD_FR1;
					break;

				case 0x30:
				case 0x38:
					if (IS_SINGLE(op))  /* C.F.S */
						SET_FCC((op >> 8) & 7, 0);
					else                /* C.F.D */
						SET_FCC((op >> 8) & 7, 0);
					break;

				case 0x31:
				case 0x39:
					if (IS_SINGLE(op))  /* C.UN.S */
						SET_FCC((op >> 8) & 7, 0);
					else                /* C.UN.D */
						SET_FCC((op >> 8) & 7, 0);
					break;

				case 0x32:
				case 0x3a:
					if (IS_SINGLE(op))  /* C.EQ.S */
						SET_FCC((op >> 8) & 7, (FSVALS_FR1 == FTVALS_FR1));
					else                /* C.EQ.D */
						SET_FCC((op >> 8) & 7, (FSVALD_FR1 == FTVALD_FR1));
					break;

				case 0x33:
				case 0x3b:
					if (IS_SINGLE(op))  /* C.UEQ.S */
						SET_FCC((op >> 8) & 7, (FSVALS_FR1 == FTVALS_FR1));
					else                /* C.UEQ.D */
						SET_FCC((op >> 8) & 7, (FSVALD_FR1 == FTVALD_FR1));
					break;

				case 0x34:
				case 0x3c:
					if (IS_SINGLE(op))  /* C.OLT.S */
						SET_FCC((op >> 8) & 7, (FSVALS_FR1 < FTVALS_FR1));
					else                /* C.OLT.D */
						SET_FCC((op >> 8) & 7, (FSVALD_FR1 < FTVALD_FR1));
					break;

				case 0x35:
				case 0x3d:
					if (IS_SINGLE(op))  /* C.ULT.S */
						SET_FCC((op >> 8) & 7, (FSVALS_FR1 < FTVALS_FR1));
					else                /* C.ULT.D */
						SET_FCC((op >> 8) & 7, (FSVALD_FR1 < FTVALD_FR1));
					break;

				case 0x36:
				case 0x3e:
					if (IS_SINGLE(op))  /* C.OLE.S */
						SET_FCC((op >> 8) & 7, (FSVALS_FR1 <= FTVALS_FR1));
					else                /* C.OLE.D */
						SET_FCC((op >> 8) & 7, (FSVALD_FR1 <= FTVALD_FR1));
					break;

				case 0x37:
				case 0x3f:
					if (IS_SINGLE(op))  /* C.ULE.S */
						SET_FCC((op >> 8) & 7, (FSVALS_FR1 <= FTVALS_FR1));
					else                /* C.ULE.D */
						SET_FCC((op >> 8) & 7, (FSVALD_FR1 <= FTVALD_FR1));
					break;

				default:
					fprintf(stderr, "cop1 %X\n", op);
					break;
			}
			break;
	}
}

void r4000_device::execute_run()
{
	mips3com_update_cycle_counting();
	check_irqs();

	do
	{
		debugger_instruction_hook(m_core.pc);

		//if (u32(m_core.pc) == 0x80063c6c)
		//  logerror("IoCreateSymbolicLink(\"%s\", \"%s\")\n", debug_unicode_string(m_core.r[4]), debug_unicode_string(m_core.r[5]));

		fetch(m_core.pc, [this](u32 const op)
		{
			switch (op & 0xfc000000)
			{
			case 0x00000000: // SPECIAL
				switch (op & 0x0000003f)
				{
				case 0x00000000: // SLL
					m_core.r[RDREG] = s64(s32(m_core.r[RTREG] << SHIFT));
					break;

				case 0x00000002: // SRL
					m_core.r[RDREG] = u32(m_core.r[RTREG]) >> SHIFT;
					break;
				case 0x00000003: // SRA
					m_core.r[RDREG] = s64(s32(m_core.r[RTREG]) >> SHIFT);
					break;
				case 0x00000004: // SLLV
					m_core.r[RDREG] = s64(s32(m_core.r[RTREG] << (m_core.r[RSREG] & 31)));
					break;
				case 0x00000006: // SRLV
					m_core.r[RDREG] = u32(m_core.r[RTREG]) >> (m_core.r[RSREG] & 31);
					break;
				case 0x00000007: // SRAV
					m_core.r[RDREG] = s64(s32(m_core.r[RTREG]) >> (m_core.r[RSREG] & 31));
					break;
				case 0x00000008: // JR
					m_branch_state = BRANCH;
					m_branch_target = ADDR(m_core.r[RSREG], 0);
					break;
				case 0x00000009: // JALR
					m_branch_state = BRANCH;
					m_branch_target = ADDR(m_core.r[RSREG], 0);
					m_core.r[RDREG] = ADDR(m_core.pc, 8);
					break;

				case 0x0000000c: // SYSCALL
					generate_exception(EXCEPTION_SYS);
					break;
				case 0x0000000d: // BREAK
					generate_exception(EXCEPTION_BP);
					break;

				case 0x0000000f: // SYNC
					break;
				case 0x00000010: // MFHI
					m_core.r[RDREG] = m_core.hi;
					break;
				case 0x00000011: // MTHI
					m_core.hi = m_core.r[RSREG];
					break;
				case 0x00000012: // MFLO
					m_core.r[RDREG] = m_core.lo;
					break;
				case 0x00000013: // MTLO
					m_core.lo = m_core.r[RSREG];
					break;
				case 0x00000014: // DSLLV
					m_core.r[RDREG] = m_core.r[RTREG] << (m_core.r[RSREG] & 63);
					break;

				case 0x00000016: // DSRLV
					m_core.r[RDREG] = m_core.r[RTREG] >> (m_core.r[RSREG] & 63);
					break;
				case 0x00000017: // DSRAV
					m_core.r[RDREG] = s64(m_core.r[RTREG]) >> (m_core.r[RSREG] & 63);
					break;
				case 0x00000018: // MULT
					{
						u64 const product = mul_32x32(s32(m_core.r[RSREG]), s32(m_core.r[RTREG]));

						m_core.lo = s64(s32(product));
						m_core.hi = s64(s32(product >> 32));
						m_core.icount -= 3;
					}
					break;
				case 0x00000019: // MULTU
					{
						u64 const product = mulu_32x32(u32(m_core.r[RSREG]), u32(m_core.r[RTREG]));

						m_core.lo = s64(s32(product));
						m_core.hi = s64(s32(product >> 32));
						m_core.icount -= 3;
					}
					break;
				case 0x0000001a: // DIV
					if (m_core.r[RTREG])
					{
						m_core.lo = s64(s32(m_core.r[RSREG]) / s32(m_core.r[RTREG]));
						m_core.hi = s64(s32(m_core.r[RSREG]) % s32(m_core.r[RTREG]));
					}
					m_core.icount -= 35;
					break;
				case 0x0000001b: // DIVU
					if (m_core.r[RTREG])
					{
						m_core.lo = s64(u32(m_core.r[RSREG]) / u32(m_core.r[RTREG]));
						m_core.hi = s64(u32(m_core.r[RSREG]) % u32(m_core.r[RTREG]));
					}
					m_core.icount -= 35;
					break;
				case 0x0000001c: // DMULT
					{
						u64 const a_hi = u32(m_core.r[RSREG] >> 32);
						u64 const b_hi = u32(m_core.r[RTREG] >> 32);
						u64 const a_lo = u32(m_core.r[RSREG]);
						u64 const b_lo = u32(m_core.r[RTREG]);

						u64 const p1 = a_lo * b_lo;
						u64 const p2 = a_hi * b_lo;
						u64 const p3 = a_lo * b_hi;
						u64 const p4 = a_hi * b_hi;
						u64 const carry = u32(((p1 >> 32) + u32(p2) + u32(p3)) >> 32);

						m_core.lo = p1 + (p2 << 32) + (p3 << 32);
						m_core.hi = p4 + (p2 >> 32) + (p3 >> 32) + carry;

						// adjust for sign
						if (m_core.r[RSREG] < 0)
							m_core.hi -= m_core.r[RTREG];
						if (m_core.r[RTREG] < 0)
							m_core.hi -= m_core.r[RSREG];

						m_core.icount -= 7;
					}
					break;
				case 0x0000001d: // DMULTU
					{
						u64 const a_hi = u32(m_core.r[RSREG] >> 32);
						u64 const b_hi = u32(m_core.r[RTREG] >> 32);
						u64 const a_lo = u32(m_core.r[RSREG]);
						u64 const b_lo = u32(m_core.r[RTREG]);

						u64 const p1 = a_lo * b_lo;
						u64 const p2 = a_hi * b_lo;
						u64 const p3 = a_lo * b_hi;
						u64 const p4 = a_hi * b_hi;
						u64 const carry = u32(((p1 >> 32) + u32(p2) + u32(p3)) >> 32);

						m_core.lo = p1 + (p2 << 32) + (p3 << 32);
						m_core.hi = p4 + (p2 >> 32) + (p3 >> 32) + carry;

						m_core.icount -= 7;
					}
					break;
				case 0x0000001e: // DDIV
					if (m_core.r[RTREG])
					{
						m_core.lo = s64(m_core.r[RSREG]) / s64(m_core.r[RTREG]);
						m_core.hi = s64(m_core.r[RSREG]) % s64(m_core.r[RTREG]);
					}
					m_core.icount -= 67;
					break;
				case 0x0000001f: // DDIVU
					if (m_core.r[RTREG])
					{
						m_core.lo = m_core.r[RSREG] / m_core.r[RTREG];
						m_core.hi = m_core.r[RSREG] % m_core.r[RTREG];
					}
					m_core.icount -= 67;
					break;
				case 0x00000020: // ADD
								 //if (RSVAL32 > ~RTVAL32)
								 // generate_exception(EXCEPTION_OV);
								 //else
					m_core.r[RDREG] = s64(s32(u32(m_core.r[RSREG]) + u32(m_core.r[RTREG])));
					break;
				case 0x00000021: // ADDU
					m_core.r[RDREG] = s64(s32(u32(m_core.r[RSREG]) + u32(m_core.r[RTREG])));
					break;
				case 0x00000022: // SUB
								 //if (RSVAL32 < RTVAL32) generate_exception(EXCEPTION_OV);
								 //else
					m_core.r[RDREG] = s64(s32(u32(m_core.r[RSREG]) - u32(m_core.r[RTREG])));
					break;
				case 0x00000023: // SUBU
					m_core.r[RDREG] = s64(s32(u32(m_core.r[RSREG]) - u32(m_core.r[RTREG])));
					break;
				case 0x00000024: // AND
					m_core.r[RDREG] = m_core.r[RSREG] & m_core.r[RTREG];
					break;
				case 0x00000025: // OR
					m_core.r[RDREG] = m_core.r[RSREG] | m_core.r[RTREG];
					break;
				case 0x00000026: // XOR
					m_core.r[RDREG] = m_core.r[RSREG] ^ m_core.r[RTREG];
					break;
				case 0x00000027: // NOR
					m_core.r[RDREG] = ~(m_core.r[RSREG] | m_core.r[RTREG]);
					break;

				case 0x0000002a: // SLT
					m_core.r[RDREG] = s64(m_core.r[RSREG]) < s64(m_core.r[RTREG]);
					break;
				case 0x0000002b: // SLTU
					m_core.r[RDREG] = m_core.r[RSREG] < m_core.r[RTREG];
					break;
				case 0x0000002c: // DADD
								 //if (m_core.r[RSREG] > ~m_core.r[RTREG]) generate_exception(EXCEPTION_OV);
								 //else
					m_core.r[RDREG] = m_core.r[RSREG] + m_core.r[RTREG];
					break;
				case 0x0000002d: // DADDU
					m_core.r[RDREG] = m_core.r[RSREG] + m_core.r[RTREG];
					break;
				case 0x0000002e: // DSUB
								 //if (m_core.r[RSREG] < m_core.r[RTREG]) generate_exception(EXCEPTION_OV);
								 //else
					m_core.r[RDREG] = m_core.r[RSREG] - m_core.r[RTREG];
					break;
				case 0x0000002f: // DSUBU
					m_core.r[RDREG] = m_core.r[RSREG] - m_core.r[RTREG];
					break;
				case 0x00000030: // TGE
					if (s64(m_core.r[RSREG]) >= s64(m_core.r[RTREG]))
						generate_exception(EXCEPTION_TR);
					break;
				case 0x00000031: // TGEU
					if (m_core.r[RSREG] >= m_core.r[RTREG])
						generate_exception(EXCEPTION_TR);
					break;
				case 0x00000032: // TLT
					if (s64(m_core.r[RSREG]) < s64(m_core.r[RTREG]))
						generate_exception(EXCEPTION_TR);
					break;
				case 0x00000033: // TLTU
					if (m_core.r[RSREG] < m_core.r[RTREG])
						generate_exception(EXCEPTION_TR);
					break;
				case 0x00000034: // TEQ
					if (m_core.r[RSREG] == m_core.r[RTREG])
						generate_exception(EXCEPTION_TR);
					break;
				case 0x00000036: // TNE
					if (m_core.r[RSREG] != m_core.r[RTREG])
						generate_exception(EXCEPTION_TR);
					break;
				case 0x00000038: // DSLL
					m_core.r[RDREG] = m_core.r[RTREG] << SHIFT;
					break;
				case 0x0000003a: // DSRL
					m_core.r[RDREG] = m_core.r[RTREG] >> SHIFT;
					break;
				case 0x0000003b: // DSRA
					m_core.r[RDREG] = s64(m_core.r[RTREG]) >> SHIFT;
					break;
				case 0x0000003c: // DSLL32
					m_core.r[RDREG] = m_core.r[RTREG] << (SHIFT + 32);
					break;
				case 0x0000003e: // DSRL32
					m_core.r[RDREG] = m_core.r[RTREG] >> (SHIFT + 32);
					break;
				case 0x0000003f: // DSRA32
					m_core.r[RDREG] = s64(m_core.r[RTREG]) >> (SHIFT + 32);
					break;

				default:
					generate_exception(EXCEPTION_RI);
					break;
				}
				break;

			case 0x04000000: // REGIMM
				switch (op & 0x001f0000)
				{
				case 0x00000000: // BLTZ
					if (s64(m_core.r[RSREG]) < 0)
						ADDPC(s16(op));
					break;
				case 0x00010000: // BGEZ
					if (s64(m_core.r[RSREG]) >= 0)
						ADDPC(s16(op));
					break;
				case 0x00020000: // BLTZL
					if (s64(m_core.r[RSREG]) < 0)
						ADDPC(s16(op));
					else
						m_branch_state = NULLIFY;
					break;
				case 0x00030000: // BGEZL
					if (s64(m_core.r[RSREG]) >= 0)
						ADDPC(s16(op));
					else
						m_branch_state = NULLIFY;
					break;
				case 0x00080000: // TGEI
					if (s64(m_core.r[RSREG]) >= s16(op))
						generate_exception(EXCEPTION_TR);
					break;
				case 0x00090000: // TGEIU
					if (m_core.r[RSREG] >= u16(op))
						generate_exception(EXCEPTION_TR);
					break;
				case 0x000a0000: // TLTI
					if (s64(m_core.r[RSREG]) < s16(op))
						generate_exception(EXCEPTION_TR);
					break;
				case 0x000b0000: // TLTIU
					if (m_core.r[RSREG] >= u16(op))
						generate_exception(EXCEPTION_TR);
					break;
				case 0x000c0000: // TEQI
					if (m_core.r[RSREG] == u16(op))
						generate_exception(EXCEPTION_TR);
					break;
				case 0x000e0000: // TNEI
					if (m_core.r[RSREG] != u16(op))
						generate_exception(EXCEPTION_TR);
					break;
				case 0x00100000: // BLTZAL
					if (s64(m_core.r[RSREG]) < 0)
						ADDPC(s16(op));
					m_core.r[31] = ADDR(m_core.pc, 8);
					break;
				case 0x00110000: // BGEZAL
					if (s64(m_core.r[RSREG]) >= 0)
						ADDPC(s16(op));
					m_core.r[31] = ADDR(m_core.pc, 8);
					break;
				case 0x00120000: // BLTZALL
					if (s64(m_core.r[RSREG]) < 0)
						ADDPC(s16(op));
					else
						m_branch_state = NULLIFY;
					m_core.r[31] = ADDR(m_core.pc, 8);
					break;
				case 0x00130000: // BGEZALL
					if (s64(m_core.r[RSREG]) >= 0)
						ADDPC(s16(op));
					else
						m_branch_state = NULLIFY;
					m_core.r[31] = ADDR(m_core.pc, 8);
					break;

				default:
					generate_exception(EXCEPTION_RI);
					break;
				}
				break;

			case 0x08000000: // J
				m_branch_state = BRANCH;
				m_branch_target = (ADDR(m_core.pc, 4) & ~0x0fffffffULL) | ((op & 0x03ffffffULL) << 2);
				break;
			case 0x0C000000: // JAL
				m_branch_state = BRANCH;
				m_branch_target = (ADDR(m_core.pc, 4) & ~0x0fffffffULL) | ((op & 0x03ffffffULL) << 2);
				m_core.r[31] = ADDR(m_core.pc, 8);
				break;
			case 0x10000000: // BEQ
				if (m_core.r[RSREG] == m_core.r[RTREG])
					ADDPC(s16(op));
				break;
			case 0x14000000: // BNE
				if (m_core.r[RSREG] != m_core.r[RTREG])
					ADDPC(s16(op));
				break;
			case 0x18000000: // BLEZ
				if (s64(m_core.r[RSREG]) <= 0)
					ADDPC(s16(op));
				break;
			case 0x1C000000: // BGTZ
				if (s64(m_core.r[RSREG]) > 0)
					ADDPC(s16(op));
				break;

			case 0x20000000: // ADDI
				//if (RSVAL32 > ~s16(op)) generate_exception(EXCEPTION_OV);
				//else
					m_core.r[RTREG] = s64(s32(u32(m_core.r[RSREG]) + s16(op)));
				break;
			case 0x24000000: // ADDIU
				m_core.r[RTREG] = s64(s32(u32(m_core.r[RSREG]) + s16(op)));
				break;
			case 0x28000000: // SLTI
				m_core.r[RTREG] = s64(m_core.r[RSREG]) < s64(s16(op));
				break;
			case 0x2c000000: // SLTIU
				m_core.r[RTREG] = u64(m_core.r[RSREG]) < u64(s16(op));
				break;
			case 0x30000000: // ANDI
				m_core.r[RTREG] = m_core.r[RSREG] & u16(op);
				break;
			case 0x34000000: // ORI
				m_core.r[RTREG] = m_core.r[RSREG] | u16(op);
				break;
			case 0x38000000: // XORI
				m_core.r[RTREG] = m_core.r[RSREG] ^ u16(op);
				break;
			case 0x3c000000: // LUI
				m_core.r[RTREG] = s64(s32(u16(op) << 16));
				break;

			case 0x40000000: // COP0
				handle_cop0(op);
				break;
			case 0x44000000: // COP1
				if (!(SR & SR_FR))
					handle_cop1_fr0(op);
				else
					handle_cop1_fr1(op);
				break;
			case 0x48000000: // COP2 rs
				// TODO: throw unusable/reserved
				break;
			case 0x50000000: // BEQL
				if (m_core.r[RSREG] == m_core.r[RTREG])
					ADDPC(s16(op));
				else
					m_branch_state = NULLIFY;
				break;
			case 0x54000000: // BNEL
				if (m_core.r[RSREG] != m_core.r[RTREG])
					ADDPC(s16(op));
				else
					m_branch_state = NULLIFY;
				break;
			case 0x58000000: // BLEZL
				if (s64(m_core.r[RSREG]) <= 0)
					ADDPC(s16(op));
				else
					m_branch_state = NULLIFY;
				break;
			case 0x5c000000: // BGTZL
				if (s64(m_core.r[RSREG]) > 0)
					ADDPC(s16(op));
				else
					m_branch_state = NULLIFY;
				break;

			case 0x60000000: // DADDI
				//if ((s64)m_core.r[RSREG] > ~s16(op)) generate_exception(EXCEPTION_OV);
				//else
					m_core.r[RTREG] = m_core.r[RSREG] + s64(s16(op));
				break;
			case 0x64000000: // DADDIU
				m_core.r[RTREG] = m_core.r[RSREG] + s64(s16(op));
				break;
			case 0x68000000: // LDL
				if (m_64 || !(SR & SR_KSU) || (SR & SR_EXL) || (SR & SR_ERL))
					ldl(op);
				else
					generate_exception(EXCEPTION_RI);
				break;
			case 0x6c000000: // LDR
				if (m_64 || !(SR & SR_KSU) || (SR & SR_EXL) || (SR & SR_ERL))
					ldr(op);
				else
					generate_exception(EXCEPTION_RI);
				break;

			case 0x80000000: // LB
				load<s8>(ADDR(m_core.r[RSREG], op),
					[this, op](s8 data)
					{
						m_core.r[RTREG] = data;
					});
				break;
			case 0x84000000: // LH
				load<s16>(ADDR(m_core.r[RSREG], op),
					[this, op](s16 data)
					{
						m_core.r[RTREG] = data;
					});
				break;
			case 0x88000000: // LWL
				lwl(op);
				break;
			case 0x8c000000: // LW
				load<s32>(ADDR(m_core.r[RSREG], op),
					[this, op](s32 data)
					{
						m_core.r[RTREG] = data;
					});
				break;
			case 0x90000000: // LBU
				load<s8>(ADDR(m_core.r[RSREG], op),
					[this, op](u8 data)
					{
						m_core.r[RTREG] = data;
					});
				break;
			case 0x94000000: // LHU
				load<u16>(ADDR(m_core.r[RSREG], op),
					[this, op](u16 data)
					{
						m_core.r[RTREG] = data;
					});
				break;
			case 0x98000000: // LWR
				lwr(op);
				break;
			case 0x9c000000: // LWU
				load<u32>(ADDR(m_core.r[RSREG], op),
					[this, op](u32 data)
					{
						m_core.r[RTREG] = data;
					});
				break;

			case 0xa0000000: // SB
				store<u8>(ADDR(m_core.r[RSREG], op), u8(m_core.r[RTREG]));
				break;
			case 0xa4000000: // SH
				store<u16>(ADDR(m_core.r[RSREG], op), u16(m_core.r[RTREG]));
				break;
			case 0xa8000000: // SWL
				swl(op);
				break;
			case 0xac000000: // SW
				store<u32>(ADDR(m_core.r[RSREG], op), u32(m_core.r[RTREG]));
				break;
			case 0xb0000000: // SDL
				if (m_64 || !(SR & SR_KSU) || (SR & SR_EXL) || (SR & SR_ERL))
					sdl(op);
				else
					generate_exception(EXCEPTION_RI);
				break;
			case 0xb4000000: // SDR
				if (m_64 || !(SR & SR_KSU) || (SR & SR_EXL) || (SR & SR_ERL))
					sdr(op);
				else
					generate_exception(EXCEPTION_RI);
				break;
			case 0xb8000000: // SWR
				swr(op);
				break;
			case 0xbc000000: // CACHE
				// no-op
				break;

			case 0xc0000000: // LL
				load<u32>(ADDR(m_core.r[RSREG], op),
					[this, op](u32 data)
					{
						m_core.r[RTREG] = data;
						m_ll_value = RTVAL32;
					});
				break;
			case 0xc4000000: // LWC1
				if (SR & SR_CU1)
					load<u32>(ADDR(m_core.r[RSREG], op),
						[this, op](u32 data)
						{
							set_cop1_reg32(RTREG, data);
						});
				else
					generate_exception(EXCEPTION_CPU1);
				break;
			case 0xc8000000: // LWC2
				load<u32>(ADDR(m_core.r[RSREG], op),
					[this, op](u32 data)
					{
						//set_cop2_reg(RTREG, data);
					});
				break;
			case 0xd0000000: // LLD
				load<u64>(ADDR(m_core.r[RSREG], op),
					[this, op](u64 data)
					{
						m_core.r[RTREG] = data;
						m_lld_value = data;
					});
				break;
			case 0xd4000000: // LDC1
				if (SR & SR_CU1)
					load<u64>(ADDR(m_core.r[RSREG], op),
						[this, op](u64 data)
						{
							set_cop1_reg64(RTREG, data);
						});
				else
					generate_exception(EXCEPTION_CPU1);
				break;
			case 0xd8000000: // LDC2
				load<u64>(ADDR(m_core.r[RSREG], op),
					[this, op](u64 data)
					{
						//set_cop2_reg(RTREG, data);
					});
				break;
			case 0xdc000000: // LD
				load<u64>(ADDR(m_core.r[RSREG], op),
					[this, op](u64 data)
					{
						m_core.r[RTREG] = data;
					});
				break;

			case 0xe0000000: // SC
				load<u32>(ADDR(m_core.r[RSREG], op),
					[this, op](u32 data)
					{
						// TODO: test lladr
						if (data == m_ll_value)
						{
							store<u32>(ADDR(m_core.r[RSREG], op), u32(m_core.r[RTREG]));
							m_core.r[RTREG] = 1;
						}
						else
							m_core.r[RTREG] = 0;
					});
				break;
			case 0xe4000000: // SWC1
				if (SR & SR_CU1)
					store<u32>(ADDR(m_core.r[RSREG], op), get_cop1_reg32(RTREG));
				else
					generate_exception(EXCEPTION_CPU1);
				break;
			case 0xe8000000: // SWC2
				//store<u32>(ADDR(m_core.r[RSREG], op), get_cop2_reg(RTREG));
				break;
			case 0xf0000000: // SCD
				load<u64>(ADDR(m_core.r[RSREG], op),
					[this, op](u64 data)
					{
						// TODO: test lladr
						if (data == m_lld_value)
						{
							store<u64>(ADDR(m_core.r[RSREG], op), m_core.r[RTREG]);
							m_core.r[RTREG] = 1;
						}
						else
							m_core.r[RTREG] = 0;
					});
				break;
			case 0xf4000000: // SDC1
				if (SR & SR_CU1)
					store<u64>(ADDR(m_core.r[RSREG], op), get_cop1_reg64(RTREG));
				else
					generate_exception(EXCEPTION_CPU1);
				break;
			case 0xf8000000: // SDC2
				//store<u64>(ADDR(m_core.r[RSREG], op), get_cop2_reg(RTREG));
				break;
			case 0xfc000000: // SD
				store<u64>(ADDR(m_core.r[RSREG], op), m_core.r[RTREG]);
				break;

			default:
				invalid_instruction(op);
				break;
			}

			// zero register zero
			m_core.r[0] = 0;

			// update pc and branch state
			switch (m_branch_state)
			{
			case NONE:
				m_core.pc += 4;
				break;

			case DELAY:
				m_branch_state = NONE;
				m_core.pc = m_branch_target;
				break;

			case BRANCH:
				m_branch_state = DELAY;
				m_core.pc += 4;
				break;

			case EXCEPTION:
				m_branch_state = NONE;
				break;

			case NULLIFY:
				m_branch_state = NONE;
				m_core.pc += 8;
			}
		});

		m_core.icount--;

	} while (m_core.icount > 0);
}

void r4000_device::lwl(u32 const op)
{
	unsigned const reverse = (SR & SR_RE) && ((SR & SR_KSU) == SR_KSU_USER) ? 7 : 0;
	u64 const offset = u64(ADDR(m_core.r[RSREG], op)) ^ reverse;
	unsigned const shift = ((offset & 3) ^ ENDIAN_VALUE_LE_BE(m_endianness, 3, 0)) << 3;

	load<u32>(offset & ~3, [this, op, shift](u32 const data)
	{
		m_core.r[RTREG] = s32((m_core.r[RTREG] & ~u32(0xffffffffUL << shift)) | (data << shift));
	});
}

void r4000_device::lwr(u32 const op)
{
	unsigned const reverse = (SR & SR_RE) && ((SR & SR_KSU) == SR_KSU_USER) ? 7 : 0;
	u64 const offset = u64(ADDR(m_core.r[RSREG], op)) ^ reverse;
	unsigned const shift = ((offset & 0x3) ^ ENDIAN_VALUE_LE_BE(m_endianness, 0, 3)) << 3;

	load<u32>(offset & ~3, [this, op, shift](u32 const data)
	{
		m_core.r[RTREG] = s32((m_core.r[RTREG] & ~u32(0xffffffffUL >> shift)) | (data >> shift));
	});
}

void r4000_device::swl(u32 const op)
{
	unsigned const reverse = (SR & SR_RE) && ((SR & SR_KSU) == SR_KSU_USER) ? 7 : 0;
	u64 const offset = u64(ADDR(m_core.r[RSREG], op)) ^ reverse;
	unsigned const shift = ((offset & 3) ^ ENDIAN_VALUE_LE_BE(m_endianness, 3, 0)) << 3;

	store<u32>(offset & ~3, u32(m_core.r[RTREG]) >> shift, 0xffffffffUL >> shift);
}

void r4000_device::swr(u32 const op)
{
	unsigned const reverse = (SR & SR_RE) && ((SR & SR_KSU) == SR_KSU_USER) ? 7 : 0;
	u64 const offset = u64(ADDR(m_core.r[RSREG], op)) ^ reverse;
	unsigned const shift = ((offset & 3) ^ ENDIAN_VALUE_LE_BE(m_endianness, 0, 3)) << 3;

	store<u32>(offset & ~3, u32(m_core.r[RTREG]) << shift, 0xffffffffUL << shift);
}

void r4000_device::ldl(u32 const op)
{
	unsigned const reverse = (SR & SR_RE) && ((SR & SR_KSU) == SR_KSU_USER) ? 7 : 0;
	u64 const offset = u64(ADDR(m_core.r[RSREG], op)) ^ reverse;
	unsigned const shift = ((offset & 7) ^ ENDIAN_VALUE_LE_BE(m_endianness, 7, 0)) << 3;

	load<u64>(offset & ~7, [this, op, shift](u64 const data)
	{
		m_core.r[RTREG] = (m_core.r[RTREG] & ~u64(0xffff'ffff'ffff'ffffULL << shift)) | (data << shift);
	});
}

void r4000_device::ldr(u32 const op)
{
	unsigned const reverse = (SR & SR_RE) && ((SR & SR_KSU) == SR_KSU_USER) ? 7 : 0;
	u64 const offset = u64(ADDR(m_core.r[RSREG], op)) ^ reverse;
	unsigned const shift = ((offset & 7) ^ ENDIAN_VALUE_LE_BE(m_endianness, 0, 7)) << 3;

	load<u64>(offset & ~7, [this, op, shift](u64 const data)
	{
		m_core.r[RTREG] = (m_core.r[RTREG] & ~u64(0xffff'ffff'ffff'ffffULL >> shift)) | (data >> shift);
	});
}

void r4000_device::sdl(u32 const op)
{
	unsigned const reverse = (SR & SR_RE) && ((SR & SR_KSU) == SR_KSU_USER) ? 7 : 0;
	u64 const offset = u64(ADDR(m_core.r[RSREG], op)) ^ reverse;
	unsigned const shift = ((offset & 7) ^ ENDIAN_VALUE_LE_BE(m_endianness, 7, 0)) << 3;

	store<u64>(offset & ~7, m_core.r[RTREG] >> shift, 0xffff'ffff'ffff'ffffULL >> shift);
}

void r4000_device::sdr(u32 const op)
{
	unsigned const reverse = (SR & SR_RE) && ((SR & SR_KSU) == SR_KSU_USER) ? 7 : 0;
	u64 const offset = u64(ADDR(m_core.r[RSREG], op)) ^ reverse;
	unsigned const shift = ((offset & 7) ^ ENDIAN_VALUE_LE_BE(m_endianness, 0, 7)) << 3;

	store<u64>(offset & ~7, m_core.r[RTREG] << shift, 0xffff'ffff'ffff'ffffULL << shift);
}

void r4000_device::execute_set_input(int inputnum, int state)
{
	if (state)
		m_core.cpr[0][COP0_Cause] |= 0x400 << inputnum;
	else
		m_core.cpr[0][COP0_Cause] &= ~(0x400 << inputnum);
}

void r4000_device::mips3com_update_cycle_counting()
{
	/* modify the timer to go off */
	if (m_core.compare_armed)
	{
		u32 count = (total_cycles() - m_core.count_zero_time) / 2;
		u32 compare = m_core.cpr[0][COP0_Compare];
		u32 delta = compare - count;
		m_core.compare_armed = 0;
		attotime newtime = cycles_to_attotime((u64)delta * 2);
		m_compare_int_timer->adjust(newtime);
		return;
	}
}

void r4000_device::mips3com_tlbr()
{
	u8 const index = m_core.cpr[0][COP0_Index] & 0x3f;

	if (index < ARRAY_LENGTH(m_tlb))
	{
		tlb_entry_t const &entry = m_tlb[index];

		m_core.cpr[0][COP0_PageMask] = entry.mask;
		m_core.cpr[0][COP0_EntryHi] = entry.vpn;
		m_core.cpr[0][COP0_EntryLo0] = entry.pfn[0];
		m_core.cpr[0][COP0_EntryLo1] = entry.pfn[1];
	}
}

void r4000_device::mips3com_tlbwi()
{
	tlb_write_common(m_core.cpr[0][COP0_Index] & 0x3f);
}

void r4000_device::mips3com_tlbwr()
{
	u8 const wired = m_core.cpr[0][COP0_Wired] & 0x3f;
	u8 const unwired = ARRAY_LENGTH(m_tlb) - wired;
	u8 index = ARRAY_LENGTH(m_tlb) - 1;

	if (unwired > 0)
		index = ((total_cycles() - m_core.count_zero_time) % unwired + wired) & 0x3f;
	LOGMASKED(LOG_TLB, "tlbwr %02d\n", index);
	tlb_write_common(index);
}

void r4000_device::mips3com_tlbp()
{
	m_core.cpr[0][COP0_Index] = 0x80000000;
	for (u8 index = 0; index < ARRAY_LENGTH(m_tlb); index++)
	{
		tlb_entry_t const &entry = m_tlb[index];

		u64 const mask = (m_64 ? EH_R | (EH_VPN2_64 & ~entry.mask) : (EH_VPN2_32 & ~entry.mask))
			| ((entry.vpn & EH_G) ? 0 : EH_ASID);

		if ((entry.vpn & mask) == (m_core.cpr[0][COP0_EntryHi] & mask))
		{
			m_core.cpr[0][COP0_Index] = index;
			break;
		}
	}

	if (m_core.cpr[0][COP0_Index] == 0x80000000)
		LOGMASKED(LOG_TLB, "tlbp miss 0x%08x\n", m_core.cpr[0][COP0_EntryHi]);
	else
		LOGMASKED(LOG_TLB, "tlbp hit 0x%08x index %02d\n", m_core.cpr[0][COP0_EntryHi], m_core.cpr[0][COP0_Index]);
}



/***************************************************************************
INTERNAL HELPERS
***************************************************************************/

/*-------------------------------------------------
compare_int_callback - callback that fires
whenever a compare interrupt is generated
-------------------------------------------------*/

TIMER_CALLBACK_MEMBER(r4000_device::compare_int_callback)
{
	m_compare_int_timer->adjust(attotime::never);
	set_input_line(5, ASSERT_LINE);
}


/*-------------------------------------------------
compute_config_register - compute the value
of the config register
-------------------------------------------------*/

u32 r4000_device::compute_config_register()
{
	/* set the cache line size to 32 bytes */
	u32 configreg = 0;

	/* set the endianness bit */
	if (m_endianness == ENDIANNESS_BIG)
		configreg |= 0x00008000;

	return configreg;
}


/*-------------------------------------------------
compute_prid_register - compute the value
of the PRId register
-------------------------------------------------*/

u32 r4000_device::compute_prid_register()
{
	// NOTE: chips prior to 3.0 have an xtlb bug
	return 0x0430;
}

void r4000_device::tlb_write_common(u8 const index)
{
	if (index < ARRAY_LENGTH(m_tlb))
	{
		tlb_entry_t &entry = m_tlb[index];

		entry.mask = m_core.cpr[0][COP0_PageMask];
		entry.vpn = m_core.cpr[0][COP0_EntryHi];
		if ((m_core.cpr[0][COP0_EntryLo0] & EL_G) && (m_core.cpr[0][COP0_EntryLo1] & EL_G))
			entry.vpn |= EH_G;
		entry.pfn[0] = m_core.cpr[0][COP0_EntryLo0];
		entry.pfn[1] = m_core.cpr[0][COP0_EntryLo1];

		entry.low_bit = 32 - count_leading_zeros((entry.mask >> 1) | 0xfff);
#if 1
		LOGMASKED(LOG_TLB, "tlb write index %02d mask 0x%016x vpn2 0x%016x %c asid 0x%02x pfn0 0x%016x %c%c pfn1 0x%016x %c%c (%s)\n",
			index, entry.mask,
			entry.vpn, entry.vpn & EH_G ? 'G' : '-', entry.vpn & EH_ASID,
			entry.pfn[0] & EL_PFN, entry.pfn[0] & EL_D ? 'D' : '-', entry.pfn[0] & EL_V ? 'V' : '-',
			entry.pfn[1] & EL_PFN, entry.pfn[1] & EL_D ? 'D' : '-', entry.pfn[1] & EL_V ? 'V' : '-',
			machine().describe_context());
#endif
	}
}

bool r4000_device::memory_translate(int spacenum, int intention, offs_t &address)
{
	u64 placeholder = s32(address);

	bool result = memory_translate(spacenum, intention, placeholder);

	if (result)
		address = placeholder;

	return result;
}

bool r4000_device::memory_translate(int spacenum, int intention, u64 &address)
{
	enum result_t { UNMAPPED, TLB, XTLB, ERROR } result = UNMAPPED;

	if (!(SR & SR_KSU) || (SR & SR_EXL) || (SR & SR_ERL))
	{
		if (SR & SR_KX)
		{
			// 64-bit kernel mode
			// 0x0000'0000'0000'0000 - 0x0000'00ff'ffff'ffff (xkuseg, mapped)
			// 0x4000'0000'0000'0000 - 0x4000'00ff'ffff'ffff (xksseg, mapped)
			// 0x8000'0000'0000'0000 - 0xbfff'ffff'ffff'ffff (xkphys, unmapped)
			// 0xc000'0000'0000'0000 - 0xc000'00ff'7fff'ffff (xkseg, mapped)
			// 0xffff'ffff'8000'0000 - 0xffff'ffff'9fff'ffff (ckseg0, unmapped, cached)
			// 0xffff'ffff'a000'0000 - 0xffff'ffff'bfff'ffff (ckseg1, unmapped, uncached)
			// 0xffff'ffff'c000'0000 - 0xffff'ffff'dfff'ffff (cksseg, mapped)
			// 0xffff'ffff'e000'0000 - 0xffff'ffff'ffff'ffff (ckseg3, mapped)

			// 0x0000'0100'0000'0000 - 0x3fff'ffff'ffff'ffff error
			// 0x4000'0100'0000'0000 - 0x7fff'ffff'ffff'ffff error
			// 0xc000'00ff'8000'0000 - 0xffff'ffff'7fff'ffff error

			// 0x8000'0000'0000'0000 - 0xbfff'ffff'ffff'ffff (xkphys, unmapped)
			// 0xffff'ffff'8000'0000 - 0xffff'ffff'9fff'ffff (ckseg0, unmapped, cached)
			// 0xffff'ffff'a000'0000 - 0xffff'ffff'bfff'ffff (ckseg1, unmapped, uncached)

			if (address & 0xffff'ff00'0000'0000)
				if ((address & 0xffff'ff00'0000'0000) == 0x4000'0000'0000'0000)
					result = XTLB; // xksseg
				else
					if ((address & 0xc000'0000'0000'0000) == 0x8000'0000'0000'0000)
						address &= 0x0000'000f'ffff'ffff; // xkphys
					else
						if ((address & 0xffff'ff00'0000'0000) == 0xc000'0000'0000'0000)
							if ((address & 0x0000'00ff'8000'0000) == 0x0000'00ff'8000'0000)
								result = ERROR; // exception
							else
								result = XTLB; // xkseg
						else
							switch (address & 0xffff'ffff'e000'0000)
							{
							case 0xffff'ffff'8000'0000: address -= 0x8000'0000; break; // ckseg0
							case 0xffff'ffff'a000'0000: address -= 0xa000'0000; break; // ckseg1
							case 0xffff'ffff'c000'0000: result = XTLB; break; // cksseg
							case 0xffff'ffff'e000'0000: result = XTLB; break; // ckseg3
							default: result = ERROR; break; // exception
							}
			else
				if (SR & SR_ERL)
					address &= 0x0000'0000'ffff'ffff;
				else
					result = XTLB; // xkuseg
		}
		else
		{
			// 32-bit kernel mode
			// 0x0000'0000 - 0x7fff'ffff (kuseg, mapped)
			// 0x8000'0000 - 0x9fff'ffff (kseg0, unmapped, cached)
			// 0xa000'0000 - 0xbfff'ffff (kseg1, unmapped, uncached)
			// 0xc000'0000 - 0xdfff'ffff (ksseg, mapped)
			// 0xe000'0000 - 0xffff'ffff (kseg3, mapped)

			if (address & 0xffff'ffff'8000'0000)
				switch (address & 0xffff'ffff'e000'0000)
				{
				case 0xffff'ffff'8000'0000: address -= 0x8000'0000; break; // kseg0
				case 0xffff'ffff'a000'0000: address -= 0xa000'0000; break; // kseg1
				case 0xffff'ffff'c000'0000: result = TLB; break; // ksseg
				case 0xffff'ffff'e000'0000: result = TLB; break; // kseg3
				default: result = ERROR; break; // exception
				}
			else
				if (SR & SR_ERL)
					address &= 0x0000'0000'ffff'ffff;
				else
					result = TLB; // kuseg
		}
	}
	else if ((SR & SR_KSU) == SR_KSU_SUPER)
	{
		if (SR & SR_SX)
		{
			// 64-bit supervisor mode
			// 0x0000'0000'0000'0000 - 0x0000'00ff'ffff'ffff (xsuseg, mapped)
			// 0x4000'0000'0000'0000 - 0x4000'00ff'ffff'ffff (xsseg, mapped)
			// 0xffff'ffff'c000'0000 - 0xffff'ffff'dfff'ffff (csseg, mapped)

			if (address & 0xffff'ff00'0000'0000)
				if ((address & 0xffff'ff00'0000'0000) == 0x4000'0000'0000'0000)
					result = XTLB; // xsseg
				else
					if ((address & 0xffff'ffff'e000'0000) == 0xffff'ffff'c000'0000)
						result = XTLB; // csseg
					else
						result = ERROR; // exception
			else
				result = XTLB; // xsuseg
		}
		else
		{
			// 32-bit supervisor mode
			// 0x0000'0000-0x7fff'ffff (suseg, mapped)
			// 0xc000'0000-0xdfff'ffff (ssseg, mapped)

			if (address & 0xffff'ffff'8000'0000)
				if ((address & 0xffff'ffff'e000'0000) == 0xffff'ffff'c000'0000)
					result = TLB; // sseg
				else
					result = ERROR; // exception
			else
				result = TLB; // suseg
		}
	}
	else
	{
		if (SR & SR_UX)
		{
			// 64-bit user mode
			// 0x0000'0000'0000'0000-0x0000'00ff'ffff'ffff (xuseg, mapped)

			if (address & 0xffff'ff00'0000'0000)
				result = ERROR; // exception
			else
				result = XTLB; // xuseg
		}
		else
		{
			// 32-bit user mode
			// 0x0000'0000-0x7fff'ffff (useg, mapped)

			if (address & 0xffff'ffff'8000'0000)
				result = ERROR; // exception
			else
				result = TLB; // useg
		}
	}

	/*
	 * 32-bit modes
	 * user:   0x0000'0000-0x7fff'ffff (useg, mapped)
	 *
	 * super:  0x0000'0000-0x7fff'ffff (suseg, mapped)
	 *         0xc000'0000-0xdfff'ffff (ssseg, mapped)
	 *
	 * kernel: 0x0000'0000-0x7fff'ffff (kuseg, mapped)
	 *         0x8000'0000-0x9fff'ffff (kseg0, unmapped, cached)
	 *         0xa000'0000-0xbfff'ffff (kseg1, unmapped, uncached)
	 *         0xc000'0000-0xdfff'ffff (ksseg, mapped)
	 *         0xe000'0000-0xffff'ffff (kseg3, mapped)
	 *
	 * 64-bit modes
	 * user:   0x0000'0000'0000'0000-0x0000'00ff'ffff'ffff (xuseg, mapped)
	 *
	 * super:  0x0000'0000'0000'0000-0x0000'00ff'ffff'ffff (xsuseg, mapped)
	 *         0x4000'0000'0000'0000-0x4000'00ff'ffff'ffff (xsseg, mapped)
	 *         0xffff'ffff'c000'0000-0xffff'ffff'dfff'ffff (csseg, mapped)
	 *
	 * kernel: 0x0000'0000'0000'0000-0x0000'00ff'ffff'ffff (xkuseg, mapped)
	 *         0x4000'0000'0000'0000-0x4000'00ff'ffff'ffff (xksseg, mapped)
	 *         0x8000'0000'0000'0000-0xbfff'ffff'ffff'ffff (xkphys, unmapped)
	 *         0xc000'0000'0000'0000-0xc000'00ff'7fff'ffff (xkseg, mapped)
	 *         0xffff'ffff'8000'0000-0xffff'ffff'9fff'ffff (ckseg0, unmapped, cached)
	 *         0xffff'ffff'a000'0000-0xffff'ffff'bfff'ffff (ckseg1, unmapped, uncached)
	 *         0xffff'ffff'c000'0000-0xffff'ffff'dfff'ffff (cksseg, mapped)
	 *         0xffff'ffff'e000'0000-0xffff'ffff'ffff'ffff (ckseg3, mapped)
	 */

	 if (result == UNMAPPED)
		 return true;

	 if (result == ERROR)
	 {
		 if (!machine().side_effects_disabled() && !(intention & TRANSLATE_DEBUG_MASK))
		 {
			 logerror("memory_translate bad address 0x%016x (%s)\n", address, machine().describe_context());

			 // TODO: check this
			 if (!(SR & SR_EXL))
				 m_core.cpr[0][COP0_BadVAddr] = address;

			 generate_exception((intention & TRANSLATE_WRITE) ? EXCEPTION_ADES : EXCEPTION_ADEL);
		 }

		 return false;
	 }

	// key is a combination of VPN2 and ASID
	u64 const key = (address & (m_64 ? (EH_R | EH_VPN2_64) : EH_VPN2_32)) | (m_core.cpr[0][COP0_EntryHi] & EH_ASID);

	bool invalid = false;
	bool modify = false;
	for (unsigned index = 0; index < ARRAY_LENGTH(m_tlb); index++)
	{
		tlb_entry_t const &entry = m_tlb[index];

		// test vpn and asid
		u64 const mask = (m_64 ? EH_R | (EH_VPN2_64 & ~entry.mask) : (EH_VPN2_32 & ~entry.mask))
			| ((entry.vpn & EH_G) ? 0 : EH_ASID);

		if ((entry.vpn & mask) != (key & mask))
			continue;

		u64 const pfn = entry.pfn[BIT(address, entry.low_bit)];

		if (intention & TRANSLATE_DEBUG_MASK)
			logerror("matched index %d mask 0x%016x vpn 0x%016x key 0x%016x address 0x%016x\n", index, entry.mask, entry.vpn, key, address);

		// test valid
		if (!(pfn & EL_V))
		{
			invalid = true;
			break;
		}

		// test dirty
		if ((intention & TRANSLATE_WRITE) && !(pfn & EL_D))
		{
			modify = true;
			break;
		}

		// translate the address
		address &= (entry.mask >> 1) | 0xfff;
		address |= ((pfn & EL_PFN) << 6) & ~(entry.mask >> 1);
		return true;
	}

	if (!machine().side_effects_disabled() && !(intention & TRANSLATE_DEBUG_MASK))
	{
		if (VERBOSE & LOG_TLB)
		{
			if (modify)
				LOGMASKED(LOG_TLB, "tlb modify asid %d address 0x%016x (%s)\n",
				m_core.cpr[0][COP0_EntryHi] & EH_ASID, address, machine().describe_context());
			else
				LOGMASKED(LOG_TLB, "tlb miss %c asid %d address 0x%016x (%s)\n",
				(intention & TRANSLATE_WRITE) ? 'w' : 'r', m_core.cpr[0][COP0_EntryHi] & EH_ASID, address, machine().describe_context());
		}

		// load tlb exception registers
		m_core.cpr[0][COP0_BadVAddr] = address;
		m_core.cpr[0][COP0_EntryHi] = key;
		m_core.cpr[0][COP0_Context] = (m_core.cpr[0][COP0_Context] & CONTEXT_PTEBASE) | ((address >> 9) & CONTEXT_BADVPN2);
		m_core.cpr[0][COP0_XContext] = (m_core.cpr[0][COP0_XContext] & XCONTEXT_PTEBASE) | ((address >> 31) & XCONTEXT_R) | ((address >> 9) & XCONTEXT_BADVPN2);

		if (invalid || modify || (SR & SR_EXL))
			generate_exception(modify ? EXCEPTION_MOD : (intention & TRANSLATE_WRITE) ? EXCEPTION_TLBS : EXCEPTION_TLBL);
		else
			generate_exception((intention & TRANSLATE_WRITE) ? EXCEPTION_TLBS : EXCEPTION_TLBL, (result == TLB) ? 0x000 : 0x080);
	}

	return false;
}

template <typename T, typename U> std::enable_if_t<std::is_convertible<U, std::function<void(T)>>::value, bool> r4000_device::load(u64 program_address, U &&apply)
{
	u64 translated_address = program_address;

	if (memory_translate(0, TRANSLATE_READ, translated_address))
	{
		switch (sizeof(T))
		{
		case 1: apply(T(space(0).read_byte(translated_address))); break;
		case 2: apply(T(space(0).read_word(translated_address))); break;
		case 4: apply(T(space(0).read_dword(translated_address))); break;
		case 8: apply(T(space(0).read_qword(translated_address))); break;
		}

		return true;
	}
	else
		return false;
}

template <typename T, typename U> std::enable_if_t<std::is_convertible<U, T>::value, void> r4000_device::store(u64 program_address, U data, T mem_mask)
{
	u64 translated_address = program_address;

	if (memory_translate(0, TRANSLATE_WRITE, translated_address))
	{
		switch (sizeof(T))
		{
		case 1: space(0).write_byte(translated_address, T(data)); break;
		case 2: space(0).write_word(translated_address, T(data), mem_mask); break;
		case 4: space(0).write_dword(translated_address, T(data), mem_mask); break;
		case 8: space(0).write_qword(translated_address, T(data), mem_mask); break;
		}
	}
}

bool r4000_device::fetch(u64 program_address, std::function<void(u32)> &&apply)
{
	u64 translated_address = program_address;

	if (memory_translate(0, TRANSLATE_FETCH, translated_address))
	{
		apply(space(0).read_dword(translated_address));

		return true;
	}
	else
		return false;
}

std::string r4000_device::debug_unicode_string(u64 unicode_string_pointer)
{
	auto const suppressor(machine().disable_side_effects());

	std::wstring result(L"");

	if (!load<u16>(unicode_string_pointer,
		[this, unicode_string_pointer, &result](u16 const length)
		{
			if (length)
				if (!load<u32>(unicode_string_pointer + 4,
					[this, length, &result](s32 buffer)
					{
						for (int i = 0; i < length; i += 2)
							load<u16>(buffer + i, [&result](wchar_t const character) { result += character; });
					}))
					result.assign(L"[unmapped]");
		}))
		result.assign(L"[unmapped]");

	return utf8_from_wstring(result);
}
