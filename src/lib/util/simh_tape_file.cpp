// license:BSD-3-Clause
// copyright-holders:Mietek Bak

// best read together with SIMH magtape spec (rev 17 Jan 2022)
// http://simh.trailing-edge.com/docs/simh_magtape.pdf

#include "multibyte.h"
#include "simh_tape_file.h"

#include <cassert>
#include <cstring>
#include <stdexcept>

#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

//////////////////////////////////////////////////////////////////////////////

// constants and helpers

enum class simh_marker : u32 {
	TAPE_MARK   = 0x00000000, // filemark; TODO: SIMH doesn't define setmarks
	ERASE_GAP   = 0xfffffffe,
	EOM         = 0xffffffff
};

inline const bool is_simh_marker_half_gap_forward(const simh_marker marker)
{
	// this function is used when we're reading normally (from BOM to EOM); returns true for erase gap markers that have been half overwritten
	return (const u32)marker == 0xfffeffff;
}

inline const bool is_simh_marker_half_gap_reverse(const simh_marker marker)
{
	// this function is used when we're reading in reverse (from EOM to BOM); returns true for erase gap markers that have been half overwritten
	return (const u32)marker >= 0xffff0000 && (const u32)marker <= 0xfffffffd;
}

inline const bool is_simh_marker_eod_forward(const simh_marker marker)
{
	// this function is used when we're reading normally (from BOM to EOM); returns true for markers that we consider EOD
	return marker == simh_marker::ERASE_GAP
	    || is_simh_marker_half_gap_forward(marker)
	    || marker == simh_marker::EOM; // logical EOM
}

inline const bool is_simh_marker_eod_reverse(const simh_marker marker)
{
	// this function is used when we're reading in reverse (from EOM to BOM); returns true for markers that we consider EOD
	return marker == simh_marker::ERASE_GAP
	    || is_simh_marker_half_gap_reverse(marker)
	    || marker == simh_marker::EOM; // logical EOM
}

enum class simh_marker_class : u8 {
	GOOD_DATA_RECORD                = 0x0,
	PRIVATE_DATA_RECORD_1           = 0x1,
	PRIVATE_DATA_RECORD_2           = 0x2,
	PRIVATE_DATA_RECORD_3           = 0x3,
	PRIVATE_DATA_RECORD_4           = 0x4,
	PRIVATE_DATA_RECORD_5           = 0x5,
	PRIVATE_DATA_RECORD_6           = 0x6,
	PRIVATE_MARKER                  = 0x7,
	BAD_DATA_RECORD                 = 0x8,
	RESERVED_DATA_RECORD_9          = 0x9,
	RESERVED_DATA_RECORD_A          = 0xa,
	RESERVED_DATA_RECORD_B          = 0xb,
	RESERVED_DATA_RECORD_C          = 0xc,
	RESERVED_DATA_RECORD_D          = 0xd,
	TAPE_DESCRIPTION_DATA_RECORD    = 0xe,
	RESERVED_MARKER                 = 0xf
};

inline const simh_marker_class get_simh_marker_class(const simh_marker marker)
{
	return (const simh_marker_class)((const u32)marker >> 28);
}

inline const u32 get_simh_marker_value(const simh_marker marker)
{
	return (const u32)marker & 0x0fffffff;
}

//////////////////////////////////////////////////////////////////////////////

// construction

simh_tape_file::simh_tape_file(const std::string_view path, const bool create)
	: m_page_size(-1)
	, m_fd(-1)
	, m_buf(nullptr)
	, m_buf_len(0)
	, m_read_only(false)
	, m_pos(0)
{
	m_page_size = sysconf(_SC_PAGESIZE);
	if (m_page_size == -1) // error: we failed to obtain page size
		throw std::runtime_error(std::string("sysconf: ") + strerror(errno));

	m_fd = open(std::string(path).c_str(), O_RDWR);
	if (m_fd < 0) {
		if (!create) { // retry as read-only if we're not creating new file
			m_read_only = true;
			m_fd = open(std::string(path).c_str(), O_RDONLY);
		}
		if (m_fd < 0) // error: we failed to open file
			throw std::runtime_error(std::string("open: ") + strerror(errno));
	}
	if (!create) { // load existing file
		const u32 result = lseek(m_fd, 0, SEEK_END);
		if (result == -1) // error: we failed to seek to end of file
			throw std::runtime_error(std::string("lseek: ") + strerror(errno));

		m_buf_len = result;
	}
	else { // create new file
		m_buf_len = 62914560; // we default to 60MB; TODO: allow specifying tape image size, once MAME supports it
		const u32 write_len = 4096;
		u8 tmp_buf[write_len];
		memset(tmp_buf, 0xff, write_len); // we assume simh_marker::EOM == 0xffffffff
		for (u32 i = 0; i < m_buf_len / write_len; i++) {
			if (write(m_fd, tmp_buf, write_len) == -1) // error: we failed to write to file
				throw std::runtime_error(std::string("write ") + strerror(errno));
		}
		if (write(m_fd, tmp_buf, m_buf_len % write_len) == -1) // error: we failed to write to file
			throw std::runtime_error(std::string("write ") + strerror(errno));
	}
	if (lseek(m_fd, 0, SEEK_SET) == -1) // error: we failed to seek to beginning of file
		throw std::runtime_error(std::string("lseek: ") + strerror(errno));

	const int prot = m_read_only ? PROT_READ : (PROT_READ | PROT_WRITE);
	m_buf = (u8 *)mmap(nullptr, m_buf_len, prot, MAP_FILE | MAP_SHARED, m_fd, 0);
	if (!m_buf) // error: we failed to map file to memory
		throw std::runtime_error(std::string("mmap: ") + strerror(errno));

	if (madvise(m_buf, m_buf_len, MADV_SEQUENTIAL)) // error: we failed to advise memory will be accessed sequentially
		throw std::runtime_error(std::string("madvise: ") + strerror(errno));
}

simh_tape_file::~simh_tape_file()
{
	if (m_buf)
		munmap(m_buf, m_buf_len);

	if (m_fd >= 0)
		close(m_fd);
}

//////////////////////////////////////////////////////////////////////////////

// internal operations

void simh_tape_file::sync(const u32 len) const
{
	assert(!m_read_only);
	const u32 page_pos = (m_pos / m_page_size) * m_page_size; // round down to a multiple of page size, because msync requires address to be aligned to page boundary
	const u32 sync_len = (m_pos - page_pos) + len; // length to sync
	if (msync(&m_buf[page_pos], sync_len, MS_SYNC)) // error: we failed to sync memory to file
		throw std::runtime_error(std::string("msync: ") + strerror(errno));
}

//////////////////////////////////////////////////////////////////////////////

// position-preserving operations

const std::pair<const tape_status, const u32> simh_tape_file::read_position() const
{
	assert(m_pos <= m_buf_len);
	assert(m_pos % 2 == 0);
	if (m_pos == 0) // we're at BOM and next block is 0
		return std::pair(tape_status::BOM, 0);

	if (m_pos == m_buf_len) // we're at physical EOM and there is no next block
		return std::pair(tape_status::EOM, 0);

	// we need to count how many blocks are between BOM and us
	u32 blocks_num = 0;
	u32 tmp_pos = 0;
	while (tmp_pos < m_pos) {
		if (tmp_pos + 4 > m_buf_len) // error: truncated marker
			throw std::runtime_error("truncated marker");

		const simh_marker marker = (const simh_marker)get_u32le(&m_buf[tmp_pos]);
		if (marker == simh_marker::TAPE_MARK) { // we skip filemarks
			tmp_pos += 4;
			continue;
		}
		if (is_simh_marker_eod_forward(marker)) // error: we reached EOD
			return std::pair(is_ew() ? tape_status::UNKNOWN_EW : tape_status::UNKNOWN, 0);

		const simh_marker_class marker_class = get_simh_marker_class(marker);
		const u32 block_len = get_simh_marker_value(marker);
		const u32 pad_len = block_len % 2; // pad odd-length blocks with 1 byte
		const u32 read_len = 4 + block_len + pad_len + 4;
		switch (marker_class) {
			case simh_marker_class::PRIVATE_MARKER: // we skip other markers
			case simh_marker_class::RESERVED_MARKER:
				tmp_pos += 4;
				break;
			case simh_marker_class::GOOD_DATA_RECORD: // we try to count data blocks
				if (tmp_pos + read_len > m_buf_len) // error: truncated block
					throw std::runtime_error("truncated block");

				// we counted another block
				tmp_pos += read_len;
				blocks_num++;
				break;
			default: // we try to skip other blocks
				if (tmp_pos + read_len > m_buf_len) // error: truncated block
					throw std::runtime_error("truncated_block");

				tmp_pos += read_len;
		}
	}
	if (m_pos < tmp_pos) // we're inside some block
		return std::pair(is_ew() ? tape_status::UNKNOWN_EW : tape_status::UNKNOWN, 0);

	// we're right after some block (at EOD, or at filemark, or at another block)
	assert(tmp_pos == m_pos);
	return std::pair(is_ew() ? tape_status::EW : tape_status::OK, blocks_num);
}

//////////////////////////////////////////////////////////////////////////////

// non-destructive operations

void simh_tape_file::rewind(const bool eom)
{
	assert(m_pos <= m_buf_len);
	assert(m_pos % 2 == 0);
	m_pos = eom ? m_buf_len : 0;
}

const tape_status simh_tape_file::locate_block(const u32 req_block_addr)
{
	assert(m_pos <= m_buf_len);
	assert(m_pos % 2 == 0);
	u32 blocks_num = 0;
	m_pos = 0;
	while (m_pos < m_buf_len) {
		if (m_pos + 4 > m_buf_len) // error: truncated marker
			throw std::runtime_error("truncated marker");

		const simh_marker marker = (const simh_marker)get_u32le(&m_buf[m_pos]);
		if (marker == simh_marker::TAPE_MARK) { // we skip filemarks
			m_pos += 4;
			continue;
		}
		if (is_simh_marker_eod_forward(marker)) // error: we reached EOD
			return is_ew() ? tape_status::EOD_EW : tape_status::EOD;

		const simh_marker_class marker_class = get_simh_marker_class(marker);
		const u32 block_len = get_simh_marker_value(marker);
		const u32 pad_len = block_len % 2; // pad odd-length blocks with 1 byte
		const u32 read_len = 4 + block_len + pad_len + 4;
		switch (marker_class) {
			case simh_marker_class::PRIVATE_MARKER: // we skip other markers
			case simh_marker_class::RESERVED_MARKER:
				m_pos += 4;
				break;
			case simh_marker_class::GOOD_DATA_RECORD: // we try to count data blocks
				if (m_pos + read_len > m_buf_len) // error: truncated block
					throw std::runtime_error("truncated block");

				if (blocks_num == req_block_addr) // success: we located requested block
					return tape_status::OK;

				m_pos += read_len;
				blocks_num++;
				break;
			default: // we try to skip other blocks
				if (m_pos + read_len > m_buf_len) // error: truncated block
					throw std::runtime_error("truncated block");

				m_pos += read_len;
		}
	}
	// error: we reached physical EOM
	assert(m_pos == m_buf_len);
	return tape_status::EOM;
}

const tape_status simh_tape_file::space_eod()
{
	assert(m_pos <= m_buf_len);
	assert(m_pos % 2 == 0);
	while (m_pos < m_buf_len) {
		if (m_pos + 4 > m_buf_len) // error: truncated marker
			throw std::runtime_error("truncated marker");

		const simh_marker marker = (const simh_marker)get_u32le(&m_buf[m_pos]);
		if (marker == simh_marker::TAPE_MARK) { // we skip filemarks
			m_pos += 4;
			continue;
		}
		if (is_simh_marker_eod_forward(marker)) // success: we reached EOD
			return tape_status::OK;

		const simh_marker_class marker_class = get_simh_marker_class(marker);
		const u32 block_len = get_simh_marker_value(marker);
		const u32 pad_len = block_len % 2; // pad odd-length blocks with 1 byte
		const u32 read_len = 4 + block_len + pad_len + 4;
		switch (marker_class) {
			case simh_marker_class::PRIVATE_MARKER: // we skip other markers
			case simh_marker_class::RESERVED_MARKER:
				m_pos += 4;
				break;
			case simh_marker_class::GOOD_DATA_RECORD: // we try to skip all blocks
			default:
				if (m_pos + read_len > m_buf_len) // error: truncated block
					throw std::runtime_error("truncated block");

				m_pos += read_len;
		}
	}
	// error: we reached physical EOM
	assert(m_pos == m_buf_len);
	return tape_status::EOM;
}

const std::pair<const tape_status, const u32> simh_tape_file::space_blocks(const u32 req_blocks_num)
{
	assert(m_pos <= m_buf_len);
	assert(m_pos % 2 == 0);
	assert(req_blocks_num > 0);
	u32 blocks_num = 0;
	while (m_pos < m_buf_len) {
		if (m_pos + 4 > m_buf_len) // error: truncated marker
			throw std::runtime_error("truncated marker");

		const simh_marker marker = (const simh_marker)get_u32le(&m_buf[m_pos]);
		if (marker == simh_marker::TAPE_MARK) { // error: we reached filemark
			m_pos += 4;
			return std::pair(is_ew() ? tape_status::FILEMARK_EW : tape_status::FILEMARK, blocks_num);
		}
		if (is_simh_marker_eod_forward(marker)) // error: we reached EOD
			return std::pair(is_ew() ? tape_status::EOD_EW : tape_status::EOD, blocks_num);

		const simh_marker_class marker_class = get_simh_marker_class(marker);
		const u32 block_len = get_simh_marker_value(marker);
		const u32 pad_len = block_len % 2; // pad odd-length blocks with 1 byte
		const u32 read_len = 4 + block_len + pad_len + 4;
		switch (marker_class) {
			case simh_marker_class::PRIVATE_MARKER: // we skip other markers
			case simh_marker_class::RESERVED_MARKER:
				m_pos += 4;
				break;
			case simh_marker_class::GOOD_DATA_RECORD: // we try to count data blocks
				if (m_pos + read_len > m_buf_len) // error: truncated block
					throw std::runtime_error("truncated block");

				m_pos += read_len;
				blocks_num++;
				if (blocks_num == req_blocks_num) // success: we're done
					return std::pair(tape_status::OK, blocks_num);

				break;
			default: // we try to skip other blocks
				if (m_pos + read_len > m_buf_len) // error: truncated block
					throw std::runtime_error("truncated block");

				m_pos += read_len;
		}
	}
	// error: we reached physical EOM
	assert(m_pos == m_buf_len);
	return std::pair(tape_status::EOM, blocks_num);
}

const std::pair<const tape_status, const u32> simh_tape_file::space_filemarks(const u32 req_filemarks_num, const bool setmarks, const bool sequential)
{
	assert(m_pos <= m_buf_len);
	assert(m_pos % 2 == 0);
	assert(req_filemarks_num > 0);
	assert(!setmarks); // TODO: SIMH doesn't define setmarks
	assert(!sequential); // TODO: support spacing over sequential filemarks, once we have good way to test it
	u32 filemarks_num = 0;
	while (m_pos < m_buf_len) {
		if (m_pos + 4 > m_buf_len) // error: truncated marker
			throw std::runtime_error("truncated marker");

		const simh_marker marker = (const simh_marker)get_u32le(&m_buf[m_pos]);
		if (marker == simh_marker::TAPE_MARK) { // we count filemarks
			m_pos += 4;
			filemarks_num++;
			if (filemarks_num == req_filemarks_num) // success: we're done
				return std::pair(tape_status::OK, filemarks_num);

			continue;
		}
		if (is_simh_marker_eod_forward(marker)) // error: we reached EOD
			return std::pair(is_ew() ? tape_status::EOD_EW : tape_status::EOD, filemarks_num);

		const simh_marker_class marker_class = get_simh_marker_class(marker);
		const u32 block_len = get_simh_marker_value(marker);
		const u32 pad_len = block_len % 2; // pad odd-length blocks with 1 byte
		const u32 read_len = 4 + block_len + pad_len + 4;
		switch (marker_class) {
			case simh_marker_class::PRIVATE_MARKER: // we skip other markers
			case simh_marker_class::RESERVED_MARKER:
				m_pos += 4;
				break;
			case simh_marker_class::GOOD_DATA_RECORD: // we try to skip all blocks
			default:
				if (m_pos + read_len > m_buf_len) // error: truncated block
					throw std::runtime_error("truncated block");

				m_pos += read_len;
		}
	}
	// error: we reached physical EOM
	assert(m_pos == m_buf_len);
	return std::pair(tape_status::EOM, filemarks_num);
}

const std::pair<const tape_status, const u32> simh_tape_file::space_blocks_reverse(const u32 req_blocks_num)
{
	assert(m_pos <= m_buf_len);
	assert(m_pos % 2 == 0);
	assert(req_blocks_num > 0);
	u32 blocks_num = 0;
	while (m_pos > 0) {
		if (m_pos - 4 < 0) // error: truncated marker
			throw std::runtime_error("truncated marker");

		const simh_marker marker = (const simh_marker)get_u32le(&m_buf[m_pos - 4]);
		if (marker == simh_marker::TAPE_MARK) { // error: we reached filemark
			m_pos -= 4;
			return std::pair(is_ew() ? tape_status::FILEMARK_EW : tape_status::FILEMARK, blocks_num);
		}
		if (is_simh_marker_eod_reverse(marker)) // error: we reached EOD
			return std::pair(is_ew() ? tape_status::EOD_EW : tape_status::EOD, blocks_num);

		const simh_marker_class marker_class = get_simh_marker_class(marker);
		const u32 block_len = get_simh_marker_value(marker);
		const u32 pad_len = block_len % 2; // pad odd-length blocks with 1 byte
		const u32 read_len = 4 + block_len + pad_len + 4;
		switch (marker_class) {
			case simh_marker_class::PRIVATE_MARKER: // we skip other markers
			case simh_marker_class::RESERVED_MARKER:
				m_pos -= 4;
				break;
			case simh_marker_class::GOOD_DATA_RECORD: // we try to count data blocks
				if (m_pos - read_len < 0) // error: truncated block
					throw std::runtime_error("truncated block");

				m_pos -= read_len;
				blocks_num++;
				if (blocks_num == req_blocks_num) // success: we're done
					return std::pair(tape_status::OK, blocks_num);

				break;
			default: // we try to skip other blocks
				if (m_pos - read_len < 0) // error: truncated block
					throw std::runtime_error("truncated block");

				m_pos -= read_len;
		}
	}
	// error: we reached BOM
	assert(m_pos == 0);
	return std::pair(tape_status::BOM, blocks_num);
}

const std::pair<const tape_status, const u32> simh_tape_file::space_filemarks_reverse(const u32 req_filemarks_num, const bool setmarks, const bool sequential)
{
	assert(m_pos <= m_buf_len);
	assert(m_pos % 2 == 0);
	assert(req_filemarks_num > 0);
	assert(!setmarks); // TODO: SIMH doesn't define setmarks
	assert(!sequential); // TODO: support spacing over sequential filemarks, once we have good way to test it
	u32 filemarks_num = 0;
	while (m_pos > 0) {
		if (m_pos - 4 < 0) // error: truncated marker
			throw std::runtime_error("truncated marker");

		const simh_marker marker = (const simh_marker)get_u32le(&m_buf[m_pos - 4]);
		if (marker == simh_marker::TAPE_MARK) { // we count filemarks
			m_pos -= 4;
			filemarks_num++;
			if (filemarks_num == req_filemarks_num) // success: we're done
				return std::pair(tape_status::OK, filemarks_num);

			continue;
		}
		if (is_simh_marker_eod_reverse(marker)) // error: we reached EOD
			return std::pair(is_ew() ? tape_status::EOD_EW : tape_status::EOD, filemarks_num);

		const simh_marker_class marker_class = get_simh_marker_class(marker);
		const u32 block_len = get_simh_marker_value(marker);
		const u32 pad_len = block_len % 2; // pad odd-length blocks with 1 byte
		const u32 read_len = 4 + block_len + pad_len + 4;
		switch (marker_class) {
			case simh_marker_class::PRIVATE_MARKER: // we skip other markers
			case simh_marker_class::RESERVED_MARKER:
				m_pos -= 4;
				break;
			case simh_marker_class::GOOD_DATA_RECORD: // we try to skip all blocks
			default:
				if (m_pos - read_len < 0) // error: truncated block
					throw std::runtime_error("truncated block");

				m_pos -= read_len;
		}
	}
	// error: we reached BOM
	assert(m_pos == 0);
	return std::pair(tape_status::BOM, filemarks_num);
}

const std::pair<const tape_status, const u32> simh_tape_file::read_block(u8 *dst_buf, const u32 dst_buf_size)
{
	assert(m_pos <= m_buf_len);
	assert(m_pos % 2 == 0);
	while (m_pos < m_buf_len) {
		if (m_pos + 4 > m_buf_len) // error: truncated marker
			throw std::runtime_error("truncated marker");

		const simh_marker marker = (const simh_marker)get_u32le(&m_buf[m_pos]);
		if (marker == simh_marker::TAPE_MARK) { // error: we reached filemark
			m_pos += 4;
			return std::pair(is_ew() ? tape_status::FILEMARK_EW : tape_status::FILEMARK, 0);
		}
		if (is_simh_marker_eod_forward(marker)) // error: we reached EOD
			return std::pair(is_ew() ? tape_status::EOD_EW : tape_status::EOD, 0);

		const simh_marker_class marker_class = get_simh_marker_class(marker);
		const u32 block_len = get_simh_marker_value(marker);
		const u32 pad_len = block_len % 2; // pad odd-length blocks with 1 byte
		const u32 read_len = 4 + block_len + pad_len + 4;
		switch (marker_class) {
			case simh_marker_class::PRIVATE_MARKER: // we skip other markers
			case simh_marker_class::RESERVED_MARKER:
				m_pos += 4;
				break;
			case simh_marker_class::GOOD_DATA_RECORD: // we try to read data blocks
				if (m_pos + read_len > m_buf_len) // error: truncated block
					throw std::runtime_error("truncated block");

				if (block_len > dst_buf_size) // error: block length too big
					throw std::runtime_error("block length too big");

				// success: we read another block
				memcpy(dst_buf, &m_buf[m_pos + 4], block_len);
				m_pos += read_len;
				return std::pair(tape_status::OK, block_len);

			default: // we try to skip other blocks
				if (m_pos + read_len > m_buf_len) // error: truncated block
					throw std::runtime_error("truncated block");

				m_pos += read_len;
		}
	}
	// error: we reached physical EOM
	assert(m_pos == m_buf_len);
	return std::pair(tape_status::EOM, 0);
}

//////////////////////////////////////////////////////////////////////////////

// destructive operations

void simh_tape_file::erase(const bool eom)
{
	assert(!m_read_only);
	assert(m_pos <= m_buf_len);
	assert(m_pos % 2 == 0);
	const u32 write_len = m_buf_len - m_pos; // we always erase entire remainder of tape
	memset(&m_buf[m_pos], 0xff, write_len); // we assume simh_marker::EOM == 0xffffffff
	sync(write_len); // may throw
	m_pos += write_len;
}

const tape_status simh_tape_file::write_block(const u8 *const src_buf, const u32 req_block_len)
{
	assert(!m_read_only);
	assert(m_pos <= m_buf_len);
	assert(m_pos % 2 == 0);
	const u32 pad_len = req_block_len % 2; // pad odd-length blocks with 1 byte
	const u32 write_len = 4 + req_block_len + pad_len + 4;
	if (m_pos + write_len >= m_buf_len) // error: we reached physical EOM
		return tape_status::EOM;

	put_u32le(&m_buf[m_pos], req_block_len);
	memcpy(&m_buf[m_pos + 4], src_buf, req_block_len);
	memset(&m_buf[m_pos + 4 + req_block_len], 0, pad_len);
	put_u32le(&m_buf[m_pos + 4 + req_block_len + pad_len], req_block_len);
	sync(write_len); // may throw
	m_pos += write_len;
	return is_ew() ? tape_status::EW : tape_status::OK; // success: we wrote another block
}

const tape_status simh_tape_file::write_filemarks(const u32 req_filemarks_num, const bool setmarks)
{
	assert(!m_read_only);
	assert(m_pos <= m_buf_len);
	assert(m_pos % 2 == 0);
	assert(!setmarks); // TODO: SIMH doesn't define setmarks
	const u32 write_len = req_filemarks_num * 4;
	if (m_pos + write_len >= m_buf_len) // error: we reached physical EOM
		return tape_status::EOM;

	for (u32 i = 0; i < write_len; i += 4)
		put_u32le(&m_buf[m_pos + i], (const u32)simh_marker::TAPE_MARK);
	sync(write_len); // may throw
	m_pos += write_len;
	return is_ew() ? tape_status::EW : tape_status::OK; // success: we wrote all filemarks
}

//////////////////////////////////////////////////////////////////////////////
