// license:BSD-3-Clause
// copyright-holders:Patrick Mackinlay

#include "emu.h"

#include "mc88200.h"

#define LOG_GENERAL (1U << 0)

#define VERBOSE (LOG_GENERAL)
#include "logmacro.h"

DEFINE_DEVICE_TYPE(MC88200, mc88200_device, "mc88200", "Motorola MC88200 Cache/Memory Management Unit")

mc88200_device::mc88200_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock, u8 id)
	: device_t(mconfig, MC88200, tag, owner, clock)
	, m_mbus(*this, finder_base::DUMMY_TAG, -1, 32)
	, m_id(u32(id) << 24)
{
}

enum idr_mask : u32
{
	IDR_VERSION = 0x001f0000,
	IDR_TYPE    = 0x00e00000,
	IDR_ID      = 0xff000000,
};

enum idr_type_mask : u32
{
	TYPE_MC88200 = 0x00a00000,
	TYPE_MC88204 = 0x00c00000,
};

enum ssr_mask : u32
{
	SSR_V  = 0x00000001, // valid
	SSR_BH = 0x00000002, // batc hit
	SSR_WP = 0x00000004, // write protection
	SSR_U  = 0x00000008, // used
	SSR_M  = 0x00000010, // modified
	SSR_CI = 0x00000040, // cache inhibit
	SSR_G  = 0x00000080, // global
	SSR_SP = 0x00000100, // supervisor privilege
	SSR_WT = 0x00000200, // writethrough
	SSR_BE = 0x00004000, // bus error
	SSR_CE = 0x00008000, // copyback error

	SSR_WM = 0x0000c3df,
};

enum sctr_mask : u32
{
	SCTR_PR = 0x00010000, // priority arbitration
	SCTR_SE = 0x00020000, // snoop enable
	SCTR_PE = 0x00040000, // parity enable
};

enum pfsr_mask : u32
{
	PFSR_OK = 0x00000000, // success (no fault)
	PFSR_BE = 0x00030000, // bus error
	PFSR_SF = 0x00040000, // segment fault
	PFSR_PF = 0x00050000, // page fault
	PFSR_SV = 0x00060000, // supervisor violation
	PFSR_WV = 0x00070000, // write violation

	PFSR_WM = 0x00070000,
};

enum apr_mask : u32
{
	APR_TE   = 0x00000001, // translation enable
	APR_CI   = 0x00000040, // cache inhibit
	APR_G    = 0x00000080, // global
	APR_WT   = 0x00000200, // writethrough
	APR_STBA = 0xfffff000, // segment table base address

	APR_WM   = 0xfffff2c1,
};

enum batc_mask : u32
{
	BATC_V   = 0x00000001, // valid
	BATC_WP  = 0x00000002, // write protect
	BATC_CI  = 0x00000004, // cache inhibit
	BATC_G   = 0x00000008, // global
	BATC_WT  = 0x00000010, // writethrough
	BATC_S   = 0x00000020, // supervisor
	BATC_PBA = 0x0007ffc0, // physical block address
	BATC_LBA = 0xfff80000, // logical block address
};

enum cssp_mask : u32
{
	CSSP_VV0 = 0x00003000, // line valid 0
	CSSP_VV1 = 0x0000c000, // line valid 1
	CSSP_VV2 = 0x00030000, // line valid 2
	CSSP_VV3 = 0x000c0000, // line valid 3
	CSSP_D0  = 0x00100000, // line disable 0
	CSSP_D1  = 0x00200000, // line disable 1
	CSSP_D2  = 0x00400000, // line disable 2
	CSSP_D3  = 0x00800000, // line disable 3
	CSSP_L0  = 0x01000000, // line 1 more recently used than line 0
	CSSP_L1  = 0x02000000, // line 2 more recently used than line 0
	CSSP_L2  = 0x04000000, // line 2 more recently used than line 1
	CSSP_L3  = 0x08000000, // line 3 more recently used than line 0
	CSSP_L4  = 0x10000000, // line 3 more recently used than line 1
	CSSP_L5  = 0x20000000, // line 3 more recently used than line 2

	CSSP_WM  = 0x3ffff000,
};

enum cssp_vv : u32
{
	VV_0 = 0x00000000, // exclusive unmodified
	VV_1 = 0x00001000, // exclusive modified
	VV_2 = 0x00002000, // shared unmodified
	VV_3 = 0x00003000, // invalid
};

enum patc_mask : u64
{
	PATC_WP  = 0x0000'00000001, // write protect
	PATC_M   = 0x0000'00000002, // modified
	PATC_CI  = 0x0000'00000004, // cache inhibit
	PATC_G   = 0x0000'00000008, // global
	PATC_WT  = 0x0000'00000010, // writethrough
	PATC_S   = 0x0000'00000020, // supervisor
	PATC_PFA = 0x0000'03ffffc0, // page frame address
	PATC_LPA = 0x3fff'fc000000, // logical page address

	PATC_V   = 0x8000'00000000, // valid (??)
};

enum segment_descriptor_mask : u32
{
	SGD_V    = 0x00000001, // valid
	SGD_WP   = 0x00000004, // write protect
	SGD_CI   = 0x00000040, // cache inhibit
	SGD_G    = 0x00000080, // global
	SGD_SP   = 0x00000100, // supervisor protection
	SGD_WT   = 0x00000200, // writethrough
	SGD_PTBA = 0xfffff000, // page table base address
};

enum page_descriptor_mask : u32
{
	PGD_V   = 0x00000001, // valid
	PGD_WP  = 0x00000004, // write protect
	PGD_U   = 0x00000008, // used
	PGD_M   = 0x00000010, // modified
	PGD_CI  = 0x00000040, // cache inhibit
	PGD_G   = 0x00000080, // global
	PGD_SP  = 0x00000100, // supervisor protection
	PGD_WT  = 0x00000200, // writethrough
	PGD_PFA = 0xfffff000, // page frame address
};

enum logical_address_mask : u32
{
	LA_OFS = 0x00000fff,
	LA_PAG = 0x003ff000,
	LA_SEG = 0xffc00000,
};

void mc88200_device::device_start()
{
	m_cache = std::make_unique<cache_set[]>(256);

	save_item(NAME(m_idr));
	save_item(NAME(m_scr));
	save_item(NAME(m_ssr));
	save_item(NAME(m_sar));
	save_item(NAME(m_sctr));
	save_item(NAME(m_pfsr));
	save_item(NAME(m_pfar));
	save_item(NAME(m_sapr));
	save_item(NAME(m_uapr));

	save_item(NAME(m_batc));
	save_item(NAME(m_patc));
	save_item(NAME(m_patc_next));

	m_idr = TYPE_MC88200;
}

void mc88200_device::device_reset()
{
	m_mbus->unmap_readwrite(0xfff00000U | ((m_idr & IDR_ID) >> 12), 0xfff00fffU | ((m_idr & IDR_ID) >> 12));

	idr_w(m_id);
	m_scr = 0;
	m_ssr = 0;
	m_sar = 0; // undefined
	m_sctr = 0;
	m_pfsr = 0;
	m_pfar = 0; // undefined
	m_sapr = 0x40;
	m_uapr = 0x40;

	for (unsigned i = 0; i < 8; i++)
		m_batc[i] = 0;
	m_batc[8] = 0xfff7ffb5;
	m_batc[9] = 0xfffffff5;

	for (u64 &patc : m_patc)
		patc = 0;
	m_patc_next = 0;

	m_bus_error = false;

	m_mbus->install_device(0xfff00000U | ((m_idr & IDR_ID) >> 12), 0xfff00fffU | ((m_idr & IDR_ID) >> 12), *this, &mc88200_device::map);
}

void mc88200_device::map(address_map &map)
{
	// system interface registers
	map(0x000, 0x003).rw(FUNC(mc88200_device::idr_r), FUNC(mc88200_device::idr_w));
	map(0x004, 0x007).rw(FUNC(mc88200_device::scr_r), FUNC(mc88200_device::scr_w));
	map(0x008, 0x00b).rw(FUNC(mc88200_device::ssr_r), FUNC(mc88200_device::ssr_w));
	map(0x00c, 0x00f).rw(FUNC(mc88200_device::sar_r), FUNC(mc88200_device::sar_w));
	map(0x104, 0x107).rw(FUNC(mc88200_device::sctr_r), FUNC(mc88200_device::sctr_w));

	// p bus fault registers
	map(0x108, 0x10b).rw(FUNC(mc88200_device::pfsr_r), FUNC(mc88200_device::pfsr_w));
	map(0x10c, 0x10f).rw(FUNC(mc88200_device::pfar_r), FUNC(mc88200_device::pfar_w));

	// area pointers
	map(0x200, 0x203).rw(FUNC(mc88200_device::sapr_r), FUNC(mc88200_device::sapr_w));
	map(0x204, 0x207).rw(FUNC(mc88200_device::uapr_r), FUNC(mc88200_device::uapr_w));

	// batc write ports
	map(0x400, 0x41f).w(FUNC(mc88200_device::bwp_w)).mirror(0x20);

	// cache diagnostic ports
	map(0x800, 0x80f).rw(FUNC(mc88200_device::cdp_r), FUNC(mc88200_device::cdp_w)).mirror(0x30);
	map(0x840, 0x84f).rw(FUNC(mc88200_device::ctp_r), FUNC(mc88200_device::ctp_w)).mirror(0x30);
	map(0x880, 0x883).rw(FUNC(mc88200_device::cssp_r), FUNC(mc88200_device::cssp_w)).mirror(0x30);
}

void mc88200_device::idr_w(u32 data)
{
	logerror("idr_w 0x%08x (%s)\n", data, machine().describe_context());

	if ((data ^ m_idr) & IDR_ID)
	{
		m_mbus->unmap_readwrite(0xfff00000U | ((m_idr & IDR_ID) >> 12), 0xfff00fffU | ((m_idr & IDR_ID) >> 12));

		m_idr = (m_idr & ~IDR_ID) | (data & IDR_ID);

		m_mbus->install_device(0xfff00000U | ((m_idr & IDR_ID) >> 12), 0xfff00fffU | ((m_idr & IDR_ID) >> 12), *this, &mc88200_device::map);
	}
}

void mc88200_device::scr_w(u32 data)
{
	logerror("scr_w 0x%08x (%s)\n", data, machine().describe_context());

	char const *const action[] = { "line", "page", "segment", "all" };

	switch (data & 0x3f)
	{
	case 0x00: case 0x01: case 0x02: case 0x03:
	case 0x04: case 0x05: case 0x06: case 0x07:
	case 0x08: case 0x09: case 0x0a: case 0x0b:
	case 0x0c: case 0x0d: case 0x0e: case 0x0f:
	case 0x10: case 0x11: case 0x12: case 0x13:
		logerror("no operation\n");
		break;
	case 0x14: case 0x15: case 0x16:
		logerror("data cache invalidate %s\n", action[data & 3]);
		break;
	case 0x17:
		logerror("data cache invalidate all\n");
		for (unsigned i = 0; i < 256; i++)
			m_cache[i].status |= (CSSP_VV3 | CSSP_VV2 | CSSP_VV1 | CSSP_VV0);
		break;
	case 0x18: case 0x19: case 0x1a: case 0x1b:
		logerror("data cache copyback to memory %s\n", action[data & 3]);
		break;
	case 0x1c: case 0x1d: case 0x1e:
		logerror("data cache copyback and invalidate %s\n", action[data & 3]);
		break;
	case 0x1f:
		logerror("data cache copyback and invalidate all\n");
		for (unsigned i = 0; i < 256; i++)
		{
			for (unsigned l = 0; l < 4; l++)
			{
				if (BIT(m_cache[i].status, 12 + l * 2, 2) == 1)
				{
					// copy back modified line
					u32 const copyback_address = m_cache[i].line[l].tag | (i << 4);

					m_mbus->write_dword(copyback_address | 0x0, m_cache[i].line[l].data[0]);
					m_mbus->write_dword(copyback_address | 0x4, m_cache[i].line[l].data[1]);
					m_mbus->write_dword(copyback_address | 0x8, m_cache[i].line[l].data[2]);
					m_mbus->write_dword(copyback_address | 0xc, m_cache[i].line[l].data[3]);
				}
			}

			m_cache[i].status |= (CSSP_VV3 | CSSP_VV2 | CSSP_VV1 | CSSP_VV0);
		}
		break;
	case 0x20: case 0x21: case 0x22: case 0x23:
	case 0x28: case 0x29: case 0x2a: case 0x2b:
		logerror("probe user address\n");
		break;
	case 0x24: case 0x25: case 0x26: case 0x27:
	case 0x2c: case 0x2d: case 0x2e: case 0x2f:
		logerror("probe supervisor address\n");
		break;
	case 0x30: case 0x38: // line
		// TODO:
		logerror("unemulated: invalidate user page address translation cache descriptors (line 0x%08x)\n", m_sar);
		break;
	case 0x31: case 0x39: // page
		logerror("invalidate user page address translation cache descriptors (page 0x%08x)\n", m_sar & ~LA_OFS);
		for (u64 &patc : m_patc)
			if (!(patc & PATC_S) && BIT(patc, 26, 20) == BIT(m_sar, 12, 20))
				patc &= ~PATC_V;
		break;
	case 0x32: case 0x3a: // segment
		logerror("invalidate user page address translation cache descriptors (segment 0x%08x)\n", m_sar & LA_SEG);
		for (u64 &patc : m_patc)
			if (!(patc & PATC_S) && BIT(patc, 36, 10) == BIT(m_sar, 22, 10))
				patc &= ~PATC_V;
		break;
	case 0x33: case 0x3b: // all
		logerror("invalidate user page address translation cache descriptors (all)\n");
		for (u64 &patc : m_patc)
			if (!(patc & PATC_S))
				patc &= ~PATC_V;
		break;
	case 0x34: case 0x3c: // line
		// TODO
		logerror("unemulated: invalidate supervisor page address translation cache descriptors (line 0x%08x)\n", m_sar);
		break;
	case 0x35: case 0x3d: // page
		logerror("invalidate supervisor page address translation cache descriptors (page 0x%08x)\n", m_sar & ~LA_OFS);
		for (u64 &patc : m_patc)
			if ((patc & PATC_S) && BIT(patc, 26, 20) == BIT(m_sar, 12, 20))
				patc &= ~PATC_V;
		break;
	case 0x36: case 0x3e: // segment
		logerror("invalidate supervisor page address translation cache descriptors (segment 0x%08x)\n", m_sar & LA_SEG);
		for (u64 &patc : m_patc)
			if ((patc & PATC_S) && BIT(patc, 36, 10) == BIT(m_sar, 22, 10))
				patc &= ~PATC_V;
		break;
	case 0x37: case 0x3f: // all
		logerror("invalidate supervisor page address translation cache descriptors (all)\n");
		for (u64 &patc : m_patc)
			if (patc & PATC_S)
				patc &= ~PATC_V;
		break;
	}

	m_scr = data & 0x3f;
}

void mc88200_device::ssr_w(u32 data) { logerror("ssr_w 0x%08x (%s)\n", data, machine().describe_context()); m_ssr = data & SSR_WM; }
void mc88200_device::sar_w(u32 data) { logerror("sar_w 0x%08x (%s)\n", data, machine().describe_context()); m_sar = data; }
void mc88200_device::sctr_w(u32 data) { logerror("sctr_w 0x%08x (%s)\n", data, machine().describe_context()); m_sctr = data; }

void mc88200_device::pfsr_w(u32 data) { logerror("pfsr_w 0x%08x (%s)\n", data, machine().describe_context()); m_pfsr = data & PFSR_WM; }
void mc88200_device::pfar_w(u32 data) { logerror("pfar_w 0x%08x (%s)\n", data, machine().describe_context()); m_pfar = data; }
void mc88200_device::sapr_w(u32 data) { logerror("sapr_w 0x%08x (%s)\n", data, machine().describe_context()); m_sapr = data & APR_WM; }
void mc88200_device::uapr_w(u32 data) { logerror("uapr_w 0x%08x (%s)\n", data, machine().describe_context()); m_uapr = data & APR_WM; }

void mc88200_device::bwp_w(offs_t offset, u32 data)
{
	LOG("bwp_w %x,0x%08x (%s)\n", offset, data, machine().describe_context());

	m_batc[offset] = data;
}

void mc88200_device::cdp_w(offs_t offset, u32 data)
{
	LOG("cdp_w set %d line %d word %d data 0x%08x (%s)\n",
		BIT(m_sar, 4, 8), offset, BIT(m_sar, 2, 2), data, machine().describe_context());

	m_cache[BIT(m_sar, 4, 8)].line[offset].data[BIT(m_sar, 2, 2)] = data;
}

void mc88200_device::ctp_w(offs_t offset, u32 data)
{
	LOG("ctp_w set %d line %d data 0x%08x (%s)\n",
		BIT(m_sar, 4, 8), offset, data, machine().describe_context());

	m_cache[BIT(m_sar, 4, 8)].line[offset].tag = data & ~LA_OFS;
}

void mc88200_device::cssp_w(u32 data)
{
	LOG("cssp_w 0x%08x (%s)\n", data, machine().describe_context());

	m_cache[BIT(m_sar, 4, 8)].status = data & CSSP_WM;
}

std::optional<mc88200_device::translate_result> mc88200_device::translate(u32 virtual_address, bool supervisor, bool write, bool debug)
{
	if (virtual_address == 0x00012000)
		logerror("v 0x%08x s %d w %d d %d\n", virtual_address, supervisor, write, debug);

	// select area descriptor
	u32 const apr = supervisor ? m_sapr : m_uapr;
	if (apr & APR_TE)
	{
		// check block address translation cache
		for (u32 const batc : m_batc)
		{
			if ((batc & BATC_V) && bool(batc & BATC_S) == supervisor && !((virtual_address ^ batc) & BATC_LBA))
			{
				if (virtual_address == 0x00012000)
					logerror("batc\n");

				if (!write || !(batc & BATC_WP))
					return translate_result(((batc & BATC_PBA) << 13) | (virtual_address & ~BATC_LBA), batc & BATC_CI, batc & BATC_G, batc & BATC_WT);
				else
				{
					// write violation
					m_pfsr = PFSR_WV;

					return std::nullopt;
				}
			}
		}

		// check page address translation cache
		for (u64 const patc : m_patc)
		{
			if ((patc & PATC_V) && (bool(patc & PATC_S) == supervisor && BIT(virtual_address, 12, 20) == BIT(patc, 26, 20)))
			{
				if (virtual_address == 0x00012000)
					logerror("patc\n");

				if (!write || !(patc & PATC_WP))
				{
					if (!write || (patc & PATC_M))
						return translate_result(((patc & PATC_PFA) << 6) | (virtual_address & LA_OFS), patc & PATC_CI, patc & PATC_G, patc & PATC_WT);

					break;
				}
				else
				{
					// write violation
					m_pfsr = PFSR_WV;

					return std::nullopt;
				}
			}
		}

		// table search
		if (virtual_address == 0x00012000)
			logerror("table search\n");

		// load and check segment descriptor
		u32 const sgd = m_mbus->read_dword((apr & APR_STBA) | ((virtual_address & LA_SEG) >> 20));
		if (m_bus_error)
		{
			m_pfsr = PFSR_BE;
			return std::nullopt;
		}
		if (!(sgd & SGD_V))
		{
			m_pfsr = PFSR_SF;
			m_pfar = (apr & APR_STBA) | ((virtual_address & LA_SEG) >> 20);

			return std::nullopt;
		}
		if ((sgd & SGD_SP) && !supervisor)
		{
			m_pfsr = PFSR_SV;
			m_pfar = (apr & APR_STBA) | ((virtual_address & LA_SEG) >> 20);

			return std::nullopt;
		}

		// load and check page descriptor
		u32 pgd = m_mbus->read_dword((sgd & SGD_PTBA) | ((virtual_address & LA_PAG) >> 10));
		if (m_bus_error)
		{
			m_pfsr = PFSR_BE;
			return std::nullopt;
		}
		if (!(pgd & PGD_V))
		{
			m_pfsr = PFSR_PF;
			m_pfar = (sgd & SGD_PTBA) | ((virtual_address & LA_PAG) >> 10);

			return std::nullopt;
		}
		if ((pgd & PGD_SP) && !supervisor)
		{
			m_pfsr = PFSR_SV;
			m_pfar = (sgd & SGD_PTBA) | ((virtual_address & LA_PAG) >> 10);

			return std::nullopt;
		}

		// check write protect
		if (write && ((sgd | pgd) & PGD_WP))
		{
			m_pfsr = PFSR_WV;
			return std::nullopt;
		}

		if (!debug)
		{
			// set page descriptor used and modified bits
			if (!(pgd & PGD_U) || (write && !(pgd & PGD_M)))
			{
				pgd |= (write ? PGD_M : 0) | PGD_U;

				m_mbus->write_dword((sgd & SGD_PTBA) | ((virtual_address & LA_PAG) >> 10), pgd);
			}

			// create patc entry (lpa,pfa,s,wt,g,ci,m,wp)
			m_patc[m_patc_next++] = PATC_V | (u64(virtual_address & ~LA_OFS) << 14) | ((pgd & PGD_PFA) >> 6) | (supervisor ? PATC_S : 0) | bitswap<u64>(apr | sgd | pgd, 9, 7, 6, 4, 2);
			if (m_patc_next == std::size(m_patc))
				m_patc_next = 0;
		}

		if (virtual_address == 0x00012000)
			logerror("descriptor\n");

		return translate_result((pgd & PGD_PFA) | (virtual_address & LA_OFS), (apr | sgd | pgd) & PGD_CI, (apr | sgd | pgd) & PGD_G, (apr | sgd | pgd) & PGD_WT);
	}
	else
		return translate_result(virtual_address, apr & APR_CI, apr & APR_G, apr & APR_WT);
}

// TODO: data shift for byte/word access
// TODO: update lru state on cache miss

template <typename T> std::optional<T> mc88200_device::cache_read(u32 physical_address)
{
	cache_set &cs = m_cache[BIT(physical_address, 4, 8)];

	for (unsigned i = 0; i < std::size(cs.line); i++)
	{
		// cache line hit: tag match, not disabled, not invalid
		if ((physical_address & ~LA_OFS) == cs.line[i].tag && !(cs.status & (CSSP_D0 << i)) && BIT(cs.status, 12 + i * 2, 2) != 3)
		{
			// update lru state
			switch (i)
			{
			case 0: cs.status &= ~(CSSP_L3 | CSSP_L1 | CSSP_L0); break;
			case 1: cs.status = (cs.status & ~(CSSP_L4 | CSSP_L2)) | CSSP_L0; break;
			case 2: cs.status = (cs.status & ~CSSP_L5) | (CSSP_L2 | CSSP_L1); break;
			case 3: cs.status |= (CSSP_L5 | CSSP_L4 | CSSP_L3); break;
			}

			// return data
			u32 const data = cs.line[i].data[BIT(physical_address, 2, 2)];

			switch (sizeof(T))
			{
			case 1: return T(data >> ((physical_address & 3) * 8));
			case 2: return T(data >> ((physical_address & 3) * 8));
			}

			return T(data);
		}
	}

	// select cache line for replacement
	std::optional<unsigned> const l = cache_replace(cs);

	if (l.has_value())
	{
		if (BIT(cs.status, 12 + l.value() * 2, 2) == 1)
		{
			// copy back modified line
			u32 const copyback_address = cs.line[l.value()].tag | (physical_address & 0xff0U);

			m_mbus->write_dword(copyback_address | 0x0, cs.line[l.value()].data[0]);
			m_mbus->write_dword(copyback_address | 0x4, cs.line[l.value()].data[1]);
			m_mbus->write_dword(copyback_address | 0x8, cs.line[l.value()].data[2]);
			m_mbus->write_dword(copyback_address | 0xc, cs.line[l.value()].data[3]);
		}

		// mark line invalid
		cs.status |= CSSP_VV0 << (l.value() * 2);

		// update tag
		cs.line[l.value()].tag = physical_address & ~LA_OFS;

		// read line from memory
		cs.line[l.value()].data[0] = m_mbus->read_dword((physical_address & 0xfffffff0U) | 0x0);
		cs.line[l.value()].data[1] = m_mbus->read_dword((physical_address & 0xfffffff0U) | 0x4);
		cs.line[l.value()].data[2] = m_mbus->read_dword((physical_address & 0xfffffff0U) | 0x8);
		cs.line[l.value()].data[3] = m_mbus->read_dword((physical_address & 0xfffffff0U) | 0xc);

		// mark line shared unmodified
		cs.status &= ~(VV_1 << (l.value() * 2));

		// TODO: update lru state

		// return data
		u32 const data = cs.line[l.value()].data[BIT(physical_address, 2, 2)];

		switch (sizeof(T))
		{
		case 1: return T(data >> ((physical_address & 3) * 8));
		case 2: return T(data >> ((physical_address & 3) * 8));
		}

		return T(data);
	}
	else
	{
		switch (sizeof(T))
		{
		case 1: return m_mbus->read_byte(physical_address);
		case 2: return m_mbus->read_word(physical_address);
		}

		return m_mbus->read_dword(physical_address);
	}
}

template <typename T> void mc88200_device::cache_write(u32 physical_address, T data, bool wt, bool g)
{
	cache_set &cs = m_cache[BIT(physical_address, 4, 8)];

	for (unsigned i = 0; i < std::size(cs.line); i++)
	{
		// cache line hit: tag match, not disabled, not invalid
		if ((physical_address & ~LA_OFS) == cs.line[i].tag && !(cs.status & (CSSP_D0 << i)) && BIT(cs.status, 12 + i * 2, 2) != 3)
		{
			switch (BIT(cs.status, 12 + i * 2, 2))
			{
			case 0:
				// exclusive unmodified
				cs.status |= VV_1 << (i * 2);
				break;
			case 1:
				// exclusive modified
				break;
			case 2:
				// shared unmodified
				if (!wt || g)
				{
					cs.status &= ~(CSSP_VV0 << (i * 2));

					if (!g)
						cs.status |= VV_1 << (i * 2);
				}
				break;
			}

			// write data to cache
			u32 &cache_data = cs.line[i].data[BIT(physical_address, 2, 2)];
			switch (sizeof(T))
			{
			case 1: cache_data = (cache_data & ~(0x000000ffU << ((physical_address & 3) * 8))) | (u32(data) << ((physical_address & 3) * 8)); break;
			case 2: cache_data = (cache_data & ~(0x0000ffffU << ((physical_address & 3) * 8))) | (u32(data) << ((physical_address & 3) * 8)); break;
			case 4: cache_data = data; break;
			}

			// update lru state
			switch (i)
			{
			case 0: cs.status &= ~(CSSP_L3 | CSSP_L1 | CSSP_L0); break;
			case 1: cs.status = (cs.status & ~(CSSP_L4 | CSSP_L2)) | CSSP_L0; break;
			case 2: cs.status = (cs.status & ~CSSP_L5) | (CSSP_L2 | CSSP_L1); break;
			case 3: cs.status |= (CSSP_L5 | CSSP_L4 | CSSP_L3); break;
			}

			// write data to memory
			if (wt || g)
			{
				switch (sizeof(T))
				{
				case 1: m_mbus->write_byte(physical_address, data); break;
				case 2: m_mbus->write_word(physical_address, data); break;
				case 4: m_mbus->write_dword(physical_address, data); break;
				}
			}

			return;
		}
	}

	// select cache line for replacement
	std::optional<unsigned> const l = cache_replace(cs);

	if (l.has_value())
	{
		if (BIT(cs.status, 12 + l.value() * 2, 2) == 1)
		{
			// copy back modified line
			u32 const copyback_address = cs.line[l.value()].tag | (physical_address & 0xff0U);

			m_mbus->write_dword(copyback_address | 0x0, cs.line[l.value()].data[0]);
			m_mbus->write_dword(copyback_address | 0x4, cs.line[l.value()].data[1]);
			m_mbus->write_dword(copyback_address | 0x8, cs.line[l.value()].data[2]);
			m_mbus->write_dword(copyback_address | 0xc, cs.line[l.value()].data[3]);
		}

		// mark line invalid
		cs.status |= CSSP_VV0 << (l.value() * 2);

		// read line from memory
		cs.line[l.value()].data[0] = m_mbus->read_dword((physical_address & 0xfffffff0U) | 0x0);
		cs.line[l.value()].data[1] = m_mbus->read_dword((physical_address & 0xfffffff0U) | 0x4);
		cs.line[l.value()].data[2] = m_mbus->read_dword((physical_address & 0xfffffff0U) | 0x8);
		cs.line[l.value()].data[3] = m_mbus->read_dword((physical_address & 0xfffffff0U) | 0xc);
	}

	// write data to memory
	switch (sizeof(T))
	{
	case 1: m_mbus->write_byte(physical_address, data); break;
	case 2: m_mbus->write_word(physical_address, data); break;
	case 4: m_mbus->write_dword(physical_address, data); break;
	}

	if (l.has_value())
	{
		// update tag
		cs.line[l.value()].tag = physical_address & ~LA_OFS;

		// write data into cache
		u32 &cache_data = cs.line[l.value()].data[BIT(physical_address, 2, 2)];
		switch (sizeof(T))
		{
		case 1: cache_data = (cache_data & ~(0x000000ffU << ((physical_address & 3) * 8))) | (u32(data) << ((physical_address & 3) * 8)); break;
		case 2: cache_data = (cache_data & ~(0x0000ffffU << ((physical_address & 3) * 8))) | (u32(data) << ((physical_address & 3) * 8)); break;
		case 4: cache_data = data; break;
		}

		// mark line exclusive unmodified
		cs.status &= ~(CSSP_VV0 << (l.value() * 2));

		// TODO: update lru state
	}
}

//                                 **          **
// 00 0xxx = 3             00, 01, 02, 03, 04, 05, 06, 07
// 1x x00x = 2  0x2. 0x3.  20, 21, 28, 29, 30, 31, 38, 39
// x1 x1x0 = 1  0x1. 0x3.  14, 16, 1c, 1e, 34, 36, 3c, 3e
// xx 1x11 = 0  0x..       0b, 0f, 1b, 1f, 2b, 2f, 3b, 3f

// 3: 00, 01,   , 03, 04,   , 06, 07
// 2: 20, 21,   , 29, 30,   , 38, 39
// 1: 14, 16,   , 1e, 34,   , 3c, 3e
// 0: 0b, 0f,   , 1f, 2b,   , 3b, 3f

/*
 *  lru   sequence
 *  0x00  0,1,2,3 0x1b
 *  0x20  0,1,3,2 0x1e
 *  0x04  0,2,1,3 0x27
 *  0x14  0,2,3,1 0x2d
 *  0x30  0,3,1,2 0x36
 *  0x34  0,3,2,1 0x39
 *
 *  0x01  1,0,2,3 0x4b
 *  0x21  1,0,3,2 0x4e
 *  0x03  1,2,0,3 0x63
 *  0x0b  1,2,3,0 0x6c
 *  0x29  1,3,0,2 0x72
 *  0x2b  1,3,2,0 0x78
 *
 *  0x06  2,0,1,3 0x87
 *  0x16  2,0,3,1 0x8d
 *  0x07  2,1,0,3 0x93
 *  0x0f  2,1,3,0 0x9c
 *  0x1e  2,3,0,1 0xb1
 *  0x1f  2,3,1,0 0xb4
 *
 *  0x38  3,0,1,2 0xc6
 *  0x3c  3,0,2,1 0xc9
 *  0x39  3,1,0,2 0xd2
 *  0x3b  3,1,2,0 0xd8
 *  0x3e  3,2,0,1 0xe1
 *  0x3f  3,2,1,0 0xe4
 */

std::optional<unsigned> mc88200_device::cache_replace(cache_set const &cs)
{
	// check for empty enabled lines
	if ((cs.status & (CSSP_D0 | CSSP_VV0)) == CSSP_VV0)
		return 0;
	if ((cs.status & (CSSP_D1 | CSSP_VV1)) == CSSP_VV1)
		return 1;
	if ((cs.status & (CSSP_D2 | CSSP_VV2)) == CSSP_VV2)
		return 2;
	if ((cs.status & (CSSP_D3 | CSSP_VV3)) == CSSP_VV3)
		return 3;

	/*
	 * This table encodes the cache line usage sequence for each combination of
	 * LRU flags. A zero value indicates an invalid flag combination, otherwise
	 * the two most-significant bits correspond to the most recently-used line
	 * and the two least-significant bits correspond to the least recently-used.
	 */
	static u8 const usage_table[] =
	{
		0x1b, 0x4b, 0x00, 0x63, 0x27, 0x00, 0x87, 0x93, // 00-07
		0x00, 0x00, 0x00, 0x6c, 0x00, 0x00, 0x00, 0x9c, // 08-0f
		0x00, 0x00, 0x00, 0x00, 0x2d, 0x00, 0x8d, 0x00, // 10-17
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb1, 0xb4, // 18-1f
		0x1e, 0x4e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 20-27
		0x00, 0x72, 0x00, 0x78, 0x00, 0x00, 0x00, 0x00, // 28-2f
		0x36, 0x00, 0x00, 0x00, 0x39, 0x00, 0x00, 0x00, // 30-37
		0xc6, 0xd2, 0x00, 0xd8, 0xc9, 0x00, 0xe1, 0xe4, // 38-3f
	};

	u8 const usage = usage_table[BIT(cs.status, 24, 6)];
	if (usage)
	{
		// find least-recently used enabled line
		for (unsigned i = 0; i < 4; i++)
		{
			unsigned const l = BIT(usage, i * 2, 2);

			if (!(cs.status & (CSSP_D0 << l)))
				return l;
		}
	}

	// invalid flags or no enabled lines
	return std::nullopt;
}

template <typename T> std::optional<T> mc88200_device::read(u32 virtual_address, bool supervisor, bool debug)
{
	std::optional<mc88200_device::translate_result> result = translate(virtual_address, supervisor, false, debug);
	if (!result.has_value())
		return std::nullopt;

	if (result.value().ci)
	{
		if (result.value().address == 0x3ffffff0)
			return std::nullopt;

		switch (sizeof(T))
		{
		case 1: return m_mbus->read_byte(result.value().address);
		case 2: return m_mbus->read_word(result.value().address);
		case 4: return m_mbus->read_dword(result.value().address);
		}
	}
	else
		return cache_read<T>(result.value().address);

	// can't happen
	abort();
}

template <typename T> bool mc88200_device::write(u32 virtual_address, T data, bool supervisor, bool debug)
{
	std::optional<mc88200_device::translate_result> result = translate(virtual_address, supervisor, true, debug);
	if (!result.has_value())
		return false;

	if (result.value().ci)
	{
		switch (sizeof(T))
		{
		case 1: m_mbus->write_byte(result.value().address, data); break;
		case 2: m_mbus->write_word(result.value().address, data); break;
		case 4: m_mbus->write_dword(result.value().address, data); break;
		}
	}
	else
		cache_write<T>(result.value().address, data, result.value().wt, result.value().g);

	return true;
}

template std::optional<u8> mc88200_device::read(u32 virtual_address, bool supervisor, bool debug);
template std::optional<u16> mc88200_device::read(u32 virtual_address, bool supervisor, bool debug);
template std::optional<u32> mc88200_device::read(u32 virtual_address, bool supervisor, bool debug);

template bool mc88200_device::write(u32 virtual_address, u8 data, bool supervisor, bool debug);
template bool mc88200_device::write(u32 virtual_address, u16 data, bool supervisor, bool debug);
template bool mc88200_device::write(u32 virtual_address, u32 data, bool supervisor, bool debug);
