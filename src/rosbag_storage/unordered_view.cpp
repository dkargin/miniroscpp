// Copyright (c) 2009, Willow Garage, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Willow Garage, Inc. nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "minibag/unordered_view.h"

#include <cstring>
#include <limits>
#include <sstream>
#include <utility>

#include "minibag/bag.h"
#include "minibag/constants.h"
#include "minibag/exceptions.h"

using miniros::Time;

namespace {

uint32_t readU32(uint8_t const* data) {
    uint32_t value;
    std::memcpy(&value, data, sizeof(value));
    return value;
}

//! A record header is uint32 length followed by that many header bytes.
//! Returns false when the buffer does not hold a complete header.
bool parseRecordHeader(uint8_t* data, uint32_t size, uint32_t offset,
                       miniros::Header& header, uint32_t& consumed) {
    if (size < offset || size - offset < 4)
        return false;
    uint32_t const header_len = readU32(data + offset);
    if (header_len > size - offset - 4)
        return false;
    std::string error;
    if (!header.parse(data + offset + 4, header_len, error))
        return false;
    consumed = 4 + header_len;
    return true;
}

} // namespace

namespace minibag {

UnorderedView::iterator::iterator() : view_(nullptr), end_(true) { }

UnorderedView::iterator::iterator(UnorderedView* view, bool end) : view_(view), end_(end) { }

UnorderedView::iterator::iterator(iterator const& other) : view_(other.view_), end_(other.end_) { }

UnorderedView::iterator& UnorderedView::iterator::operator=(iterator const& other) {
    view_ = other.view_;
    end_ = other.end_;
    return *this;
}

void UnorderedView::iterator::increment() {
    if (end_ || view_ == nullptr)
        return;
    if (!view_->advance())
        end_ = true;
}

bool UnorderedView::iterator::equal(iterator const& other) const {
    return end_ == other.end_ && view_ == other.view_;
}

MessageInstance& UnorderedView::iterator::dereference() const {
    if (end_ || view_ == nullptr || !view_->current_)
        throw BagException("Dereferenced the end of an UnorderedView");
    return *view_->current_;
}

UnorderedView::UnorderedView(std::string const& filename, Time const& start_time, Time const& end_time)
    : UnorderedView(filename, std::function<bool(ConnectionInfo const*)>(), start_time, end_time) { }

UnorderedView::UnorderedView(std::string const& filename,
                             std::function<bool(ConnectionInfo const*)> query,
                             Time const& start_time, Time const& end_time)
    : bag_(nullptr), query_(std::move(query)), start_(start_time), end_(end_time),
      finished_(false), chunk_pos_(0), chunk_size_(0), offset_(0) {
    owned_.reset(new Bag());
    owned_->open(filename, bagmode::Read);
    bag_ = owned_.get();
}

UnorderedView::UnorderedView(Bag& bag, Time const& start_time, Time const& end_time)
    : UnorderedView(bag, std::function<bool(ConnectionInfo const*)>(), start_time, end_time) { }

UnorderedView::UnorderedView(Bag& bag, std::function<bool(ConnectionInfo const*)> query,
                             Time const& start_time, Time const& end_time)
    : bag_(&bag), query_(std::move(query)), start_(start_time), end_(end_time),
      finished_(false), chunk_pos_(0), chunk_size_(0), offset_(0) {
    if (!bag.isOpen() || (bag.getMode() & bagmode::Read) == 0)
        throw BagException("Bag not opened for reading");
    if (bag.getMajorVersion() != 2)
        throw BagException("UnorderedView supports bag version 2.0 only");
}

UnorderedView::~UnorderedView() = default;

UnorderedView::iterator UnorderedView::begin() {
    reset();
    if (!advance())
        return end();
    return iterator(this, false);
}

UnorderedView::iterator UnorderedView::end() {
    return iterator(this, true);
}

void UnorderedView::reset() {
    finished_ = false;
    chunk_pos_ = 0;
    chunk_size_ = 0;
    offset_ = 0;
    current_.reset();
    // A leftover cache hit would serve the previous chunk's bytes.
    bag_->decompressed_chunk_ = std::numeric_limits<uint64_t>::max();
    bag_->seek(bag_->chunks_begin_pos_);
}

bool UnorderedView::advance() {
    if (finished_)
        return false;
    while (true) {
        if (offset_ >= chunk_size_) {
            if (!loadNextChunk()) {
                finished_ = true;
                current_.reset();
                return false;
            }
        }
        if (parseOne())
            return true;
    }
}

bool UnorderedView::loadNextChunk() {
    while (bag_->file_.getOffset() < bag_->index_data_pos_) {
        uint64_t const pos = bag_->file_.getOffset();
        miniros::Header header;
        uint32_t data_size = 0;
        if (!bag_->readHeader(header) || !bag_->readDataLength(data_size))
            throw BagFormatException("Error reading record while scanning bag");

        uint8_t op = 0;
        bag_->readField(*header.getValues(), OP_FIELD_NAME, true, &op);
        if (op == OP_INDEX_DATA) {
            uint64_t const at = bag_->file_.getOffset();
            if (bag_->index_data_pos_ < at || bag_->index_data_pos_ - at < data_size)
                throw BagFormatException("Index record extends past the chunk region");
            bag_->seek(data_size, std::ios_base::cur);
            continue;
        }
        if (op != OP_CHUNK) {
            std::stringstream ss;
            ss << "Unexpected record op " << static_cast<int>(op) << " while scanning bag";
            throw BagFormatException(ss.str());
        }

        ChunkHeader chunk_header;
        chunk_header.compressed_size = data_size;
        bag_->readField(*header.getValues(), COMPRESSION_FIELD_NAME, true, chunk_header.compression);
        bag_->readField(*header.getValues(), SIZE_FIELD_NAME, true, &chunk_header.uncompressed_size);

        if (chunk_header.compression == COMPRESSION_NONE)
            bag_->decompressRawChunk(chunk_header);
        else if (chunk_header.compression == COMPRESSION_BZ2)
            bag_->decompressBz2Chunk(chunk_header);
        else if (chunk_header.compression == COMPRESSION_LZ4)
            bag_->decompressLz4Chunk(chunk_header);
        else
            throw BagFormatException("Unknown compression: " + chunk_header.compression);

        if (bag_->decompress_buffer_.getSize() < chunk_header.uncompressed_size)
            throw BagFormatException("Decompressed chunk is shorter than its header claims");

        // decompressChunk() serves outgoing_chunk_buffer_ when this matches,
        // which is empty while reading. Keep the cache on decompress_buffer_.
        if (bag_->curr_chunk_info_.pos == pos)
            bag_->curr_chunk_info_.pos = std::numeric_limits<uint64_t>::max();
        bag_->decompressed_chunk_ = pos;
        bag_->current_buffer_ = &bag_->decompress_buffer_;

        chunk_pos_ = pos;
        chunk_size_ = chunk_header.uncompressed_size;
        offset_ = 0;
        return true;
    }
    return false;
}

bool UnorderedView::parseOne() {
    uint8_t* data = bag_->decompress_buffer_.getData();
    uint32_t const size = chunk_size_;
    uint32_t const offset = offset_;

    miniros::Header header;
    uint32_t header_bytes = 0;
    if (!parseRecordHeader(data, size, offset, header, header_bytes))
        throw BagFormatException("Truncated record in chunk");

    uint8_t op = 0;
    bag_->readField(*header.getValues(), OP_FIELD_NAME, true, &op);

    if (op == OP_CONNECTION) {
        uint32_t id = 0;
        std::string topic;
        bag_->readField(*header.getValues(), CONNECTION_FIELD_NAME, true, &id);
        bag_->readField(*header.getValues(), TOPIC_FIELD_NAME, true, topic);

        miniros::Header connection_header;
        uint32_t connection_bytes = 0;
        if (!parseRecordHeader(data, size, offset + header_bytes, connection_header, connection_bytes))
            throw BagFormatException("Truncated connection header in chunk");
        offset_ = offset + header_bytes + connection_bytes;
        rememberConnection(id, topic, connection_header);
        return false;
    }

    if (op != OP_MSG_DATA) {
        std::stringstream ss;
        ss << "Unexpected record op " << static_cast<int>(op) << " inside chunk";
        throw BagFormatException(ss.str());
    }

    if (size - offset < header_bytes + 4)
        throw BagFormatException("Truncated message in chunk");
    uint32_t const data_size = readU32(data + offset + header_bytes);
    if (data_size > size - offset - header_bytes - 4)
        throw BagFormatException("Message extends past the end of the chunk");
    offset_ = offset + header_bytes + 4 + data_size;

    uint32_t connection_id = 0;
    Time time;
    bag_->readField(*header.getValues(), CONNECTION_FIELD_NAME, true, &connection_id);
    bag_->readField(*header.getValues(), TIME_FIELD_NAME, true, time);

    std::map<uint32_t, ConnectionInfo*>::const_iterator found = bag_->connections_.find(connection_id);
    if (found == bag_->connections_.end()) {
        std::stringstream ss;
        ss << "Unknown connection ID: " << connection_id;
        throw BagFormatException(ss.str());
    }
    if (!accepts(found->second))
        return false;
    if (time < start_ || end_ < time)
        return false;

    IndexEntry entry;
    entry.time = time;
    entry.chunk_pos = chunk_pos_;
    entry.offset = offset;
    current_.reset(new MessageInstance(found->second, entry, *bag_));
    return true;
}

void UnorderedView::rememberConnection(uint32_t id, std::string const& topic,
                                       miniros::Header& connection_header) {
    std::map<uint32_t, ConnectionInfo*>::iterator found = bag_->connections_.find(id);
    if (found != bag_->connections_.end())
        return;

    ConnectionInfo* info = new ConnectionInfo();
    info->id = id;
    info->topic = topic;
    info->header = std::make_shared<miniros::M_string>();
    for (miniros::M_string::const_iterator it = connection_header.getValues()->begin();
         it != connection_header.getValues()->end(); ++it)
        (*info->header)[it->first] = it->second;
    info->msg_def = (*info->header)["message_definition"];
    info->datatype = (*info->header)["type"];
    info->md5sum = (*info->header)["md5sum"];
    bag_->connections_[id] = info;
}

bool UnorderedView::accepts(ConnectionInfo const* info) {
    std::map<uint32_t, bool>::iterator it = accept_.find(info->id);
    if (it != accept_.end())
        return it->second;
    bool const ok = !query_ || query_(info);
    accept_[info->id] = ok;
    return ok;
}

} // namespace minibag
