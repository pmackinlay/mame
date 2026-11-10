// license:BSD-3-Clause
// copyright-holders:Patrick Mackinlay
// thanks-to:Happy

/*
 * Silicon Graphics GE5 and HQ1 devices.
 *
 * This board handles the interface to the host, and mainly consists of the HQ1
 * instruction sequencer and the WTL3132 floating-point accelerator. The board
 * also contains instruction and data RAM for the HQ1, and a FIFO for host
 * communication.
 *
 * The undocumented HQ1 microcode instruction format is relatively well decoded
 * now, but the exact timing and function of some operations remains unknown.
 *
 * TODO:
 *   - implement host dma
 *   - verify some operations
 *   - implement single stepping
 *   - redo disassembly
 *   - save state
 */
/*
 * focus 1
 * gex ec
 * go 15c7
 *  - ff1 not set?
 *  - how do we unstall before dma?
 *
 * [:maincpu] asid 24 syscall execve("/usr/gfx/gfxinit", [ "/usr/gfx/gfxinit" ], [ "PATH=/usr/sbin:/usr/bsd:/usr/bin:/bin:/etc:/usr/etc:/usr/bin/X11", "TZ=PST8PDT" ]) (':maincpu' (0040F654))
 * [:maincpu] asid 25 syscall open("/usr/gfx/ucode/ge5_re1.bin") (':maincpu' (00405364))
 *  - seems to complete successfully ending with GE_FINISH0
 *
 * [:maincpu] asid 28 syscall execve("/usr/bin/X11/Xsgi", [ "/usr/bin/X11/Xsgi", "-gamma", "1.7", "-bs", "-c", "-pseudomap", "4sight" ], [ "PATH=/usr/sbin:/usr/bsd:/usr/bin:/bin:/etc:/usr/etc:/usr/bin/X11", "TZ=PST8PDT" ]) (':maincpu' (0040F654))
 * [:maincpu] asid 32 syscall open("/usr/lib/X11/dyDDX/re2.ddx") (':maincpu' (00479404))
 * 
 */

#include "emu.h"
#include "debugger.h"
#include "sgi_ge5.h"

#define LOG_TOKEN     (1U << 1)
#define LOG_MEMORY    (1U << 2)
#define LOG_DMA       (1U << 3)

//#define VERBOSE       (LOG_GENERAL|LOG_TOKEN|LOG_DMA)
#include "logmacro.h"

static char const *const token_diag[] =
{
	"DIAG_DATA",          "DIAG_INIT",          "DIAG_DRAMTEST",      "DIAG_DMA_IG",
	"DIAG_DMA_IB",        "DIAG_DMA_GB",        "DIAG_CHARPOS",       "DIAG_WRITEFULLPIX",
	"DIAG_DRAWLINE",      "DIAG_DK3_FIFO",      "DIAG_DK3_FINFLGS",   "DIAG_DRAW4SPANS",
	"DIAG_DRAWFLATSPAN",  "DIAG_DRAWSPAN",      "DIAG_LIFECHECK",     "DIAG_LOADRE",
	"DIAG_READPIXDMA",    "DIAG_READPIXELS",    "DIAG_SCREENCLEAR",   "DIAG_WRITEPIXDMA",
	"DIAG_FASTCLEAR20",   "DIAG_DRAWCHAR",      "DIAG_STRINGINIT",    "DIAG_STRINGEND",
	"DIAG_FASTCHAR",      "DIAG_DRAWLONGSPANS", nullptr,              nullptr,

	// turbo option
	nullptr,              nullptr,              "DIAG_DSPLOAD",       "DIAG_DSPRAMDATA",
	"DIAG_DSPRAMADDR",    "DIAG_DSPFIFO",       "DIAG_DSPSPAN",       "DIAG_DSPRD",
	"DIAG_DSPWR",         "DIAG_DSPINTRAM",     "DIAG_DSPSCOPE",      "DIAG_RESCOPE",
};

static char const *const token_puc[] =
{
	"PUC_DATA",           "PUC_INIT",           nullptr,              nullptr,
	nullptr,              nullptr,              nullptr,              nullptr,
	nullptr,              nullptr,              "PUC_COLOR",          "PUC_FINISH",
	"PUC_PNT2I",          "PUC_RECTI2D",        "PUC_CMOV2I",         "PUC_DRAWCHAR",
	"PUC_HAND",           "PUC_FBOPT",          "PUC_ZBOPT",          "PUC_TOPSCAN",
	"DIAG_READPIXELS",    "DIAG_WRITEFULLPIX",  "DIAG_LOADRE",        "DIAG_CHARPOS",
};

static char const *const token_gl[] =
{
	"GE_DATA",            "GE_INIT",            nullptr,              nullptr,
	nullptr,              nullptr,              nullptr,              nullptr,
	nullptr,              nullptr,              "GE_COLOR",           "GE_FINISH0",
	"GE_PNT2I",           "GE_SBOXI",           "GE_CMOV2I",          "GE_DRAWCHAR",
	"GE_HAND",            "GE_FBOPT",           "GE_ZBOPT",           "GE_TOPSCAN",
	"GE_READPIXELS",      "GE_WRITEPIXELS",     "GE_LOADRE",          "GE_GETCPOS",
	"GE_PICKMODE",        "GE_PIXTYPE",         "GE_PIXWRITEMASK",    "GE_POPNAME",
	"GE_PUSHNAME",        "GE_READBLOCK",       "GE_RECTREAD",        "GE_READBUF",
	"GE_READPIXDMA",      "GE_AUXWRITEMASK",    "GE_READRGB",         "GE_RECTCOPY",
	"GE_RGBCOLOR",        "GE_RGBSHADERANGE",   "GE_RWMODE",          "GE_SCREENCLEAR",
	"GE_SHADEMODEL",      "GE_SHADERANGE",      "GE_WRITEBLOCK",      "GE_RECTWRITE",
	"GE_WRITEPIXDMA",     "GE_BEGINBBOX",       "GE_ZBUFFER",         "GE_ZCLEAR",
	"GE_ZOOMFACTOR",      "GE_READSOURCE",      "GE_DRAWMODE",        "GE_CZCLEAR",
	"GE_HQMSAV",          "GE_ZFUNCTION",       "GE_SETPIECES",       "GE_FLATMODE",
	"GE_LMCOLOR",         "GE_LOADAMBIENT",     "GE_DEPTHFN",         "GE_LOADDIFFUSE",
	"GE_LOADMATRIX",      "GE_MULTMATRIX",      "GE_PUSHMATRIX",      "GE_POPMATRIX",
	"GE_LOADSPECULAR",    "GE_LOADEMISSION",    "GE_LOADASUM",        "GE_LOADLCOLOR",
	nullptr,              nullptr,              "GE_CURVEIT",         "GE_LOADVIEWP",
	"GE_POLYGON",         "GE_ENDPOLYGON",      "GE_TRANSLATEI",      "GE_TRANSLATE",
	"GE_LINESTYLE",       "GE_LINEWIDTH",       "GE_VERTEX2I",        "GE_VERTEX2",
	"GE_VERTEX3I",        "GE_VERTEX3",         "GE_VERTEX4I",        "GE_VERTEX4",
	"GE_RVERTEX2I",       "GE_RVERTEX2",        "GE_RVERTEX3I",       "GE_RVERTEX3",
	"GE_CLOSEDLINE",      "GE_ENDCLOSEDLINE",   "GE_LSREPEAT",        "GE_ANTIALIAS",
	"GE_COLORF",          "GE_PNT2",            "GE_PNT3I",           "GE_PNT3",
	"GE_PNT4I",           "GE_PNT4",            nullptr,              nullptr,
	"GE_MOVE2I",          "GE_MOVE2",           "GE_MOVE3I",          "GE_MOVE3",
	"GE_MOVE4I",          "GE_MOVE4",           "GE_RMOVE2I",         "GE_RMOVE2",
	"GE_RMOVE3I",         "GE_RMOVE3",          "GE_DRAW2I",          "GE_DRAW2",
	"GE_DRAW3I",          "GE_DRAW3",           "GE_DRAW4I",          "GE_DRAW4",
	"GE_RDRAW2I",         "GE_RDRAW2",          "GE_RDRAW3I",         "GE_RDRAW3",
	"GE_ENABLWID",        "GE_LOADGE",          nullptr,              nullptr,
	nullptr,              nullptr,              nullptr,              nullptr,
	nullptr,              "GE_FRONTFACE",       "GE_BACKFACE",        "GE_CONCAVE",
	"GE_PATTERN",         "GE_SETPATTERN",      "GE_LOADNORMAL",      "GE_MULTNORMAL",
	"GE_INITSTACK",       "GE_MMODE",           "GE_NORMAL",          "GE_ABNORMAL",
	"GE_LIGHTATTR1",      "GE_LIGHTATTR2",      "GE_LIGHTATTR3",      "GE_BINDLIGHT",
	"GE_LIGHTDATA4",      "GE_LIGHTMEMPTR",     "GE_LIGHTDIRECTION",  "GE_LIGHTPOSITION",
	"GE_LIGHTMOVEDATA",   "GE_BEGINMESH",       "GE_ENDMESH",         "GE_SWAPMESH",
	"GE_SBOXF",           "GE_SBOXFI",          "GE_FATPOLY",         "GE_ENDOLDPOLYGON",
	"GE_SBOX",            "GE_CURRENTWID",      nullptr,              nullptr,
	"GE_DEPTHCUE",        "GE_CMOV2",           "GE_CMOV3I",          "GE_CMOV3",
	"GE_CMOV4I",          "GE_CMOV4",           "GE_ENABDITH",        "GE_ENABWID",
	nullptr,              nullptr,              "GE_SETMATRIX",       "GE_COMPOSEMATRIX",
	"GE_LOADTOPMATRIX",   "GE_COPYMATRIX",      "GE_FEEDBACK",        "GE_ENDFEEDBACK",
	"GE_PASSTHROUGH",     "GE_FMOVE",           "GE_FDRAW",           "GE_FLINE",
	"GE_SCRMASK",         "GE_ZSOURCE",         "GE_SUBPIXEL",        "GE_SMOOTHPOINT",
	"GE_RASTEROP",        "GE_RESETLS",         nullptr,              nullptr,
	nullptr,              nullptr,              "GE_SETSURFSCALE",    "GE_SETV",
	"GE_PUSHV",           "GE_SURFP1",          "GE_SURFNTURF",       "GE_SETVHI",
	"GE_STRIP",           "GE_1LOAD1",          "GE_1LOAD3",          "GE_1LOAD4",
	"GE_SURFMODE",        "GE_DSPRD",           "GE_DSPWR",           "GE_DSPNEXT",
	"GE_DSPDUMMY",        "GE_PICKTYPE",        "GE_VERSION",         "GE_ENDBBOX",
	"GE_ENDPICKMODE",     "GE_INITNAMES",       "GE_LOADNAME",        nullptr,
	nullptr,              nullptr,              nullptr,              nullptr,
	nullptr,              nullptr,              nullptr,              nullptr,
	nullptr,              nullptr,              nullptr,              nullptr,
	nullptr,              nullptr,              nullptr,              nullptr,
	nullptr,              nullptr,              nullptr,              nullptr,
	nullptr,              nullptr,              nullptr,              nullptr,
	nullptr,              nullptr,              nullptr,              nullptr,
	"GE_CTX0",            nullptr,              nullptr,              nullptr,
	nullptr,              nullptr,              nullptr,              nullptr,
	nullptr,              nullptr,              nullptr,              nullptr,
	nullptr,              nullptr,              nullptr,              "GE_CTX1",
};

DEFINE_DEVICE_TYPE(SGI_GE5, sgi_ge5_device, "ge5", "SGI Geometry Engine 5")

/*
 * This constant has bits set where the corresponding primary control value
 * represents a conditional branch instruction. Bit 36 normally indicates the
 * presence of a secondary instruction word, but for conditional branches, a
 * secondary word containing the branch destination is implied even when bit
 * 36 is clear.
 */
static const u16 cond_branch = 0x6c6c;

sgi_ge5_device::sgi_ge5_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock)
	: cpu_device(mconfig, SGI_GE5, tag, owner, clock)
	, m_code_config("code", ENDIANNESS_BIG, 64, 15, -3, address_map_constructor(FUNC(sgi_ge5_device::code_map), this))
	, m_data_config("data", ENDIANNESS_BIG, 32, 13, -2, address_map_constructor(FUNC(sgi_ge5_device::data_map), this))
	, m_fpu(*this, "fpu")
	, m_int_cb(*this)
	, m_drq_cb(*this)
	, m_fifo_empty(*this, 0)
	, m_fifo_read(*this, 0)
	, m_re_r(*this, 0)
	, m_re_w(*this)
	, m_icount(0)
{
}

void sgi_ge5_device::device_add_mconfig(machine_config &config)
{
	WTL3132(config, m_fpu, clock());
	m_fpu->out_fpcn().set([this](int state) { m_fpu_c = bool(state); });
	m_fpu->out_port_x().set([this](u32 data) { m_fpu_data = data; });
}


void sgi_ge5_device::code_map(address_map &map)
{
	map(0x0000, 0x3fff).ram().share("code");
}

void sgi_ge5_device::data_map(address_map &map)
{
	map(0x0000, 0x1fff).ram().share("data");
}

void sgi_ge5_device::device_start()
{
	// TODO: save state
	state_add(STATE_GENPC,     "GENPC", m_pc).noshow();
	state_add(STATE_GENPCBASE, "CURPC", m_pc).noshow();
	state_add(0, "PC", m_pc).formatstr("%04X");

	state_add(1, "MEMPTR", m_memptr).formatstr("%04X");
	state_add(2, "REPTR",  m_reptr).formatstr("%04X");
	state_add(3, "BUS",    m_bus).formatstr("%08X");
	state_add(4, "DMACNT", m_dma_count).formatstr("%04X");
	state_add(5, "FF0", m_finish[0]).formatstr("%1X");
	state_add(6, "FF1", m_finish[1]).formatstr("%1X");

	m_fpu->state_add(*this, 7);

	set_icountptr(m_icount);
}

void sgi_ge5_device::device_reset()
{
	m_pc = 0;
	m_sp = 0;
	m_reptr = 0;
	m_memptr = 0;
	m_memptr_temp = 0;

	set_int(false);
	set_drq(false);

	m_state = STALL;
	m_burst = false;
}

device_memory_interface::space_config_vector sgi_ge5_device::memory_space_config() const
{
	return space_config_vector
	{
		std::make_pair(0, &m_code_config),
		std::make_pair(1, &m_data_config),
	};
}

std::unique_ptr<util::disasm_interface> sgi_ge5_device::create_disassembler()
{
	return std::make_unique<sgi_ge5_disassembler>();
}

void sgi_ge5_device::execute_run()
{
	while (m_icount > 0)
	{
		switch (m_state)
		{
		case STALL:
			m_icount = 0;
			break;

		case DECODE:
			debugger_instruction_hook(m_pc);

			// decode instruction
			decode();

			// execute secondary operation
			if (m_decode.secondary)
				secondary();

			// increment memptr
			if (m_decode.inc_memptr)
				m_memptr = (m_memptr + 1) & 0x3fff;

			// increment reptr
			if (m_decode.inc_reptr)
				m_reptr = (m_reptr + 1) & 0x3f;

			// update pc to next sequential instruction
			m_pc += m_decode.secondary ? 2 : 1;
			m_state = READ;
			break;

		case READ:
			m_state = CONTROL;

			// fetch source
			switch (m_decode.source)
			{
			case 0: // reptr
				if (m_reptr == 0x20 && !m_re_drq)
				{
					// re read stall
					m_state = READ;
					m_icount = 0;
				}
				else
					m_bus = m_re_r(m_reptr);
				break;
			case 1: // fifo or dma
				if (m_decode.control == 0xf)
				{
					if (!m_burst)
					{
						// dma read stall
						m_state = READ;
						m_icount = 0;

						set_drq(true);
					}
					else
						m_burst = false;
				}
				else if (m_fifo_empty())
				{
					// fifo read stall
					m_state = READ;
					m_icount = 0;
				}
				else
					m_bus = m_fifo_read();
				break;
			case 2: // memptr or register
				if (m_decode.secondary && m_decode.operation == 0x0f)
				{
					// FIXME: not sure if registers are read before or after increment
					switch (m_decode.immediate)
					{
					case 0: // store pcsave
						LOG("store pcsave (%s)\n", machine().describe_context());
						m_bus = m_pc;
						break;
					case 1: // store memptr
						m_bus = m_memptr;
						break;
					case 2: // store reptr
						m_bus = m_reptr;
						break;
					default:
						logerror("store unknown 0x%04x (%s)\n", m_decode.immediate, machine().describe_context());
						break;
					}
				}
				else
					m_bus = space(1).read_dword(m_memptr);
				break;
			case 3: // fpu
				m_decode.fpu |= (2ULL << wtl3132_device::S_IOCT);
				m_bus = m_fpu_data;
				break;
			}
			break;

		case CONTROL:
			m_state = WRITE;

			switch (m_decode.control)
			{
			case 0x0: // sequential execution
				break;
			case 0x1: // unconditional branch
				m_pc = m_decode.immediate;
				break;
			case 0x2: // branch fpu less than
				if (m_fpu_c_latch)
					m_pc = m_decode.immediate;
				break;
			case 0x3: // branch fpu greater or equal
				if (!m_fpu_c_latch)
					m_pc = m_decode.immediate;
				break;
			case 0x4: // unconditional call
				m_stack[m_sp] = m_pc;
				m_sp = (m_sp + 1) & 7;
				m_pc = m_decode.immediate;
				break;
			case 0x5: // call fpu less than
				if (m_fpu_c_latch)
				{
					m_stack[m_sp] = m_pc;
					m_sp = (m_sp + 1) & 7;
					m_pc = m_decode.immediate;
				}
				break;
			case 0x6: // call fpu greater or equal
				if (!m_fpu_c_latch)
				{
					m_stack[m_sp] = m_pc;
					m_sp = (m_sp + 1) & 7;
					m_pc = m_decode.immediate;
				}
				break;
			case 0x7: // return
				m_sp = (m_sp + 7) & 7;
				m_pc = m_stack[m_sp];
				break;
			case 0x8: // fetch
				m_pc = (m_bus >> 31) & 0x1fe;

				debugger_exception_hook(m_bus >> 32);

				if (VERBOSE & LOG_TOKEN)
				{
					auto const suppressor(machine().disable_side_effects());

					u8 const token = m_bus >> 32;
					char const *string = nullptr;

					/*
					 * Magic numbers stored at specific data memory locations
					 * are used to identify specific microcode programs. Other
					 * variations may exist but are not known at this time.
					 */
					if (space(1).read_dword(0x50b) == 0x004d0003)
					{
						if (token < std::size(token_puc))
							string = token_puc[token];
					}
					else if (space(1).read_dword(0x50d) == 0x004d0005
						|| space(1).read_dword(0x536) == 0x12345678
						|| space(1).read_dword(0x540) == 0x12345678)
					{
						if (token < std::size(token_gl))
							string = token_gl[token];
					}
					else if (token < std::size(token_diag))
						string = token_diag[token];

					if (string)
						LOGMASKED(LOG_TOKEN, "fetch 0x%02x (%s)\n", token, string);
					else
						LOGMASKED(LOG_TOKEN, "fetch 0x%02x (unknown)\n", token);
				}
				else
					LOG("fetch 0x%02x\n", m_bus >> 32);

				// neutralize previous instruction writeback
				m_fpu->neut_w(0);
				break;
			case 0x9: // branch indirect
				// TODO: verify value
				m_pc = m_bus;
				break;
			case 0xa: // branch less than
				if (BIT(m_bus, 31))
					m_pc = m_decode.immediate;
				break;
			case 0xb: // branch greater or equal
				if (!BIT(m_bus, 31))
					m_pc = m_decode.immediate;
				break;
			case 0xc: // stall
				LOG("stall (%s)\n", machine().describe_context());
				m_state = STALL;
				m_icount = 0;
				break;
			case 0xd: // call less than
				if (BIT(m_bus, 31))
				{
					m_stack[m_sp] = m_pc;
					m_sp = (m_sp + 1) & 7;
					m_pc = m_decode.immediate;
				}
				break;
			case 0xe: // call greater or equal
				if (!BIT(m_bus, 31))
				{
					m_stack[m_sp] = m_pc;
					m_sp = (m_sp + 1) & 7;
					m_pc = m_decode.immediate;
				}
				break;
			case 0xf: // repeatgez (dma cycle)
				if (--m_dma_count)
					m_pc -= m_decode.secondary ? 2 : 1;
				else
					LOGMASKED(LOG_DMA, "dma complete\n");
				break;
			}
			break;

		case WRITE:
			m_state = COMPLETE;

			// store destination
			switch (m_decode.destination)
			{
			case 0: // reptr
				if (m_reptr == 0x20 && !m_re_rdy && !m_re_drq)
				{
					// re write stall
					m_state = WRITE;
					m_icount = 0;
				}
				else if (m_reptr > 0x20 && !m_re_rdy)
				{
					// re unbuffered register write stall
					m_state = WRITE;
					m_icount = 0;
				}
				else
					m_re_w(m_reptr, m_bus);
				break;
			case 1: // TODO: dma?
				if (m_decode.control == 0xf)
				{
					if (!m_burst)
					{
						// dma write stall
						m_state = WRITE;
						m_icount = 0;

						set_drq(true);
					}
					else
						m_burst = false;
				}
				break;
			case 2: // memptr
				space(1).write_dword(m_memptr, m_bus);
				break;
			case 3: // fpu
				m_fpu->x_port_w(m_bus);
				m_decode.fpu |= (3ULL << wtl3132_device::S_IOCT);
				break;
			}
			break;

		case COMPLETE:
			m_state = DECODE;

			// restore memptr
			if (m_memptr_temp & 0x8000)
			{
				m_memptr = m_memptr_temp & 0x3fff;
				m_memptr_temp = 0;
			}

			// FIXME: fpu condition has additional 1 cycle latency
			m_fpu_c_latch = m_fpu_c;

			// fpu operation
			m_fpu->c_port_w(m_decode.fpu);
			m_fpu->clk_w(1);
			m_fpu->neut_w(1);

			m_icount--;
			break;
		}
	}
}

void sgi_ge5_device::decode()
{
	// fetch primary word
	u64 const primary = space(0).read_qword(m_pc + 0);

	// decode primary word
	m_decode.source = BIT(primary, 38, 2);
	m_decode.inc_reptr = BIT(primary, 37);
	m_decode.inc_memptr = BIT(primary, 35);
	m_decode.destination = BIT(primary, 33, 2);
	m_decode.control = BIT(primary, 29, 4);

	m_decode.secondary = BIT(primary, 36) || BIT(cond_branch, m_decode.control);

	// decode fpu instruction
	m_decode.fpu = ((primary & 0x1fff'f800ULL) << 5) | ((primary & 0x0000'07ffULL) << 2) | (2ULL << wtl3132_device::S_ENCN);
	if (m_cwen)
		m_decode.fpu |= wtl3132_device::M_CWEN;

	// decode secondary word
	if (m_decode.secondary)
	{
		u64 const secondary = space(0).read_qword(m_pc + 1);

		m_decode.operation = BIT(secondary, 34, 6);
		m_decode.immediate = BIT(secondary, 19, 15);
	}
}

void sgi_ge5_device::secondary()
{
	switch (m_decode.operation)
	{
	case 0x0f: // store register
		break;

	case 0x23: // load reptr
		m_reptr = m_bus & 0x3f;
		break;

	case 0x24: // load memptr; set reptr
		m_memptr = m_bus & 0x3fff;
		m_reptr = m_decode.immediate & 0x3f;
		break;

	case 0x27: // set reptr
		m_reptr = m_decode.immediate & 0x3f;
		break;

	case 0x2c: // load memptr
		m_memptr = m_bus & 0x3fff;
		break;

	case 0x2d: // set memptr or finish flag
		if (m_decode.immediate & 0x4000)
		{
			// set finish flag
			LOG("finish flag %d set (%s)\n", m_decode.immediate & 1, machine().describe_context());
			m_finish[m_decode.immediate & 1] = 1;
		}
		else
			m_memptr = m_decode.immediate & 0x3fff;
		break;

	case 0x2e: // set memptr_temp
		m_memptr_temp = m_memptr | 0x8000;
		m_memptr = m_decode.immediate & 0x3fff;
		break;

	case 0x2f: // nop?
		break;

	case 0x3f:
		switch (m_decode.immediate)
		{
		//case 0: // TODO: assert dma ready
		//	LOGMASKED(LOG_DMA, "dma ready\n");
		//	break;

		case 0x1000: // assert interrupt
			LOG("interrupt asserted\n");
			set_int(true);
			break;

		case 0x4000: // TODO: reset dma?
			LOGMASKED(LOG_DMA, "dma reset\n");
			break;

		case 0x4800: // load dma count
			m_dma_count = m_bus;
			LOGMASKED(LOG_DMA, "dma count %d (%d)\n", m_dma_count, machine().describe_context());
			break;

		default:
			//logerror("unknown dma 0x%04x (%s)\n", m_decode.immediate, machine().describe_context());
			set_drq(false);
			m_burst = false;
			break;
		}
		break;

	default:
		logerror("unknown secondary operation 0x%02x\n", m_decode.operation);
		break;
	}
}

void sgi_ge5_device::command_w(offs_t offset, u16 data, u16 mem_mask)
{
	switch (offset)
	{
	case 0x00: // clear stall
		LOG("clear stall\n");
		m_state = DECODE;
		debugger_exception_hook(0);
		break;

	case 0x10: // set single step
	case 0x20: // clear single step
	case 0x30: // execute single step
		break;

	case 0x50: // clear interrupt
		LOG("interrupt cleared\n");
		set_int(false);
		break;
	}
}

template u32 sgi_ge5_device::code_r<false>(offs_t offset);
template u32 sgi_ge5_device::code_r<true>(offs_t offset);
template void sgi_ge5_device::code_w<false>(offs_t offset, u32 data, u32 mem_mask);
template void sgi_ge5_device::code_w<true>(offs_t offset, u32 data, u32 mem_mask);

template <bool High> u32 sgi_ge5_device::code_r(offs_t offset)
{
	m_pc = offset | offs_t(m_mar & 0x7f) << 8;

	u64 const data = space(0).read_qword(m_pc);

	return High ? u32(data >> 32) : u32(data);
}

template <bool High> void sgi_ge5_device::code_w(offs_t offset, u32 data, u32 mem_mask)
{
	m_pc = offset | offs_t(m_mar & 0x7f) << 8;

	LOGMASKED(LOG_MEMORY, "code_w msb %d offset 0x%08x data 0x%08x mask 0x%08x (%s)\n", High, m_pc, data, mem_mask, machine().describe_context());

	if (High)
	{
		u64 const mask = u64(mem_mask & 0x000000ffU) << 32;

		space(0).write_qword(m_pc, u64(data) << 32, mask);
	}
	else
		space(0).write_qword(m_pc, data, mem_mask);
}

/*
 * FIXME: the original GR1 has a defect triggered when the host tries to access
 * data RAM while the HQ1 is reading from the fifo. The GRF1 gate array resolves
 * the issue by stalling the host until the HQ1 has finished accessing the fifo.
 *
 * Because the current emulated host CPU can't be stalled mid-access, we try to
 * avoid problems by allowing the access but inhibiting the side-effect of
 * updating the memptr which can disturb the running microcode. This workaround
 * is good enough to allow the gefifo diagnostic to complete successfully.
 */
u32 sgi_ge5_device::data_r(offs_t offset)
{
	u16 const memptr = offset | offs_t(m_mar & 0x3f) << 8;

	if (m_state == STALL)
		m_memptr = memptr;

	return space(1).read_dword(memptr);
}

void sgi_ge5_device::data_w(offs_t offset, u32 data, u32 mem_mask)
{
	u16 const memptr = offset | offs_t(m_mar & 0x3f) << 8;

	if (m_state == STALL)
		m_memptr = memptr;

	space(1).write_dword(memptr, data, mem_mask);
}

void sgi_ge5_device::burst_w(u32 data)
{
	m_bus = data;
	m_burst = true;

	set_drq(false);
}

offs_t sgi_ge5_disassembler::disassemble(std::ostream &stream, offs_t pc, data_buffer const &opcodes, data_buffer const &params)
{
	std::string src, dst;
	u16 immediate = 0;
	u32 flags = 0;

	u64 const primary = opcodes.r64(pc);

	// secondary word if explicit or control is conditional branch
	bool const secondary = BIT(primary, 36) || BIT(cond_branch, BIT(primary, 29, 4));

	if (secondary)
	{
		std::string prefix;

		u64 const data = opcodes.r64(pc + 1);
		u8 const operation = BIT(data, 34, 6);
		immediate = BIT(data, 19, 15);


		// operation: dbrrmm
		// d: db-src2 - 0=source is register, 1=source is bus?
		// b: bra-mux - 0=normal, 1=dma?
		// r: reptr2  - 0=load, 1=set, 2=temp?, 3=nop
		// m: memptr2 - 0=load, 1=set, 2=temp, 3=nop

		//      d b rr mm
		// 0x0f 0 0 11 11 store register
		// 0x23 1 0 00 11 load reptr
		// 0x24 1 0 01 00 load memptr, set reptr
		// 0x27 1 0 01 11 set reptr
		// 0x2c 1 0 11 00 load memptr
		// 0x2d 1 0 11 01 set memptr
		// 0x2e 1 0 11 10 set memptr_temp
		// 0x2f 1 0 11 11 nop?
		// 0x3f 1 1 11 11 dma

		switch (operation)
		{
		case 0x0f:
			switch (immediate)
			{
			case 0: prefix = std::string("STORE PCSAVE"); break;
			case 1: prefix = std::string("STORE MEMPTR"); break;
			case 2: prefix = std::string("STORE REPTR"); break;
			default: prefix = std::string("STORE ?"); break;
			}
			break;
		case 0x23: prefix = std::string("LOAD REPTR"); break;
		case 0x24: prefix = util::string_format("LOAD MEMPTR; SET REPTR,0x%04x", immediate); break;
		case 0x27: prefix = util::string_format("SET REPTR,0x%04x", immediate); break;
		case 0x2c: prefix = std::string("LOAD MEMPTR"); break;
		case 0x2d: prefix = (immediate & 0x4000)
			? util::string_format("SET FF%d", immediate & 1)
			: util::string_format("SET MEMPTR,0x%04x", immediate);
			break;
		case 0x2e: prefix = util::string_format("SET MEMPTR_TEMP,0x%04x", immediate); break;
		case 0x2f: break;
		case 0x3f:
			switch (immediate)
			{
			case 0x1000: prefix = std::string("SET INT"); break;
			case 0x4000: prefix = std::string("RESET DMA"); break;
			case 0x4800: prefix = std::string("LOAD DMACNT"); break;
			default: prefix = std::string("DMA? 0x%04x", immediate); break;
			}
			break;
		}

		if (!prefix.empty())
			stream << prefix << "; ";
	}

	u64 fpu_ctrl = 0; // ENCN=0, IOCT=0

	switch (BIT(primary, 29, 4))
	{
	case 2:
	case 3:
	case 5:
	case 6:
		fpu_ctrl |= 0x1'0000'0000; // ENCN=1
		break;
	}

	switch (BIT(primary, 38, 2))
	{
	case 0: src = std::string("RE"); break;
	case 1: src = std::string("FIFO"); break;
	case 2: src = std::string("MEM"); break; // TODO: register source
	case 3: src = std::string("FPU"); fpu_ctrl |= 0x8000'0000; break;
	}

	switch (BIT(primary, 33, 2))
	{
	case 0: dst = std::string("RE"); break;
	case 1: dst = std::string("BUS"); break;
	case 2: dst = std::string("MEM"); break;
	case 3: dst = std::string("FPU"); fpu_ctrl |= 0xc000'0000; break;
	}

	// logic for ioct:
	//u8 const ioct0 = dst==FPU
	//u8 const ioct1 = (dst=FPU) || (src=FPU);

	stream << wtl3132_device::disassemble(
		bitswap<34>(BIT(primary, 0, 28) | fpu_ctrl,
			28, 27, 26,          // f
			25, 24, 23, 22, 21,  // aadd
			20, 19, 18, 17, 16,  // badd
			15, 14, 13, 12, 11,  // cadd
			29,                  // cwen*
			31, 30,              // ioct*
			10, 9, 8, 7, 6,      // dadd
			5, 4, 3,             // abin
			2, 1,                // adst
			0,                   // mbin
			33, 32))             // encn*
		<< "; ";

	if (BIT(primary, 35))
		stream << std::string("MEMPTR++; ");

	if (BIT(primary, 37))
		stream << std::string("REPTR++; ");

	stream << util::string_format("R:%s; W:%s", src, dst);

	// branch
	switch (BIT(primary, 29, 4))
	{
	case 0x1: stream << util::string_format("; BRA 0x%04x", immediate); break;
	case 0x2: stream << util::string_format("; BLTF 0x%04x", immediate); break;
	case 0x3: stream << util::string_format("; BGEF 0x%04x", immediate); break;
	case 0x4: stream << util::string_format("; CALL 0x%04x", immediate); flags = STEP_OVER; break;
	case 0x5: stream << util::string_format("; CLTF 0x%04x", immediate); flags = STEP_OVER; break;
	case 0x6: stream << util::string_format("; CGEF 0x%04x", immediate); flags = STEP_OVER; break;
	case 0x7: stream << "; RET"; flags = STEP_OUT; break;
	case 0x8: stream << "; FETCH"; break;
	case 0x9: stream << util::string_format("; BRI 0x%04x", immediate); break;
	case 0xa: stream << util::string_format("; BLT 0x%04x", immediate); break;
	case 0xb: stream << util::string_format("; BGE 0x%04x", immediate); break;
	case 0xc: stream << "; STALL"; break;
	case 0xd: stream << util::string_format("; CLT 0x%04x", immediate); flags = STEP_OVER; break;
	case 0xe: stream << util::string_format("; CGE 0x%04x", immediate); flags = STEP_OVER; break;
	case 0xf: stream << "; REPEATGEZ"; flags = STEP_OVER; break;
	}

	return SUPPORTED | flags | (secondary ? 2 : 1);
}
