/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>

#include <miniros/header.h>

#include "minibag/message_instance.h"
#include "minibag/structures.h"

namespace minibag {

class Bag;

//! Forward scan of a bag 2.0 file.
/*!
 * Messages are yielded in the order they are stored, as soon as the chunk
 * that contains them has been read. Records that fail the topic query or
 * fall outside the time window are dropped. There is no timestamp sort and
 * no per-message index, so size(), seek, and getBeginTime() are not available.
 *
 * The file is read from the first chunk to the trailer. One scan is in
 * progress per view; copying an iterator does not fork that scan.
 *
 * Open the bag with Bag::open, then construct the view on that bag. This
 * does not call Bag::buildIndex(). A View on the same bag does. Callers
 * that must build with an older miniros test MINIBAG_HAS_UNORDERED_VIEW
 * and fall back to View. Include this header only in that branch.
 */
class ROSBAG_STORAGE_DECL UnorderedView
{
public:
    using Time = miniros::Time;

    class ROSBAG_STORAGE_DECL iterator
    {
    public:
        iterator();
        iterator(iterator const& other);
        iterator& operator=(iterator const& other);

        iterator& operator++()
        {
            increment();
            return *this;
        }

        MessageInstance& operator*() { return dereference(); }
        MessageInstance const& operator*() const { return dereference(); }
        MessageInstance* operator->() { return &dereference(); }
        MessageInstance const* operator->() const { return &dereference(); }

        bool operator==(iterator const& other) const { return equal(other); }
        bool operator!=(iterator const& other) const { return !equal(other); }

    private:
        friend class UnorderedView;

        iterator(UnorderedView* view, bool end);

        void increment();
        bool equal(iterator const& other) const;
        MessageInstance& dereference() const;

        UnorderedView* view_;
        bool end_;
    };

    typedef iterator const_iterator;

    UnorderedView(std::string const& filename,
                  Time const& start_time = miniros::TIME_MIN,
                  Time const& end_time = miniros::TIME_MAX);

    UnorderedView(std::string const& filename,
                  std::function<bool(ConnectionInfo const*)> query,
                  Time const& start_time = miniros::TIME_MIN,
                  Time const& end_time = miniros::TIME_MAX);

    UnorderedView(Bag& bag,
                  Time const& start_time = miniros::TIME_MIN,
                  Time const& end_time = miniros::TIME_MAX);

    UnorderedView(Bag& bag,
                  std::function<bool(ConnectionInfo const*)> query,
                  Time const& start_time = miniros::TIME_MIN,
                  Time const& end_time = miniros::TIME_MAX);

    ~UnorderedView();

    UnorderedView(UnorderedView const&) = delete;
    UnorderedView& operator=(UnorderedView const&) = delete;

    iterator begin();
    iterator end();

private:
    friend class iterator;

    void reset();
    //! Move to the next matching message. Returns false at the end of the scan.
    bool advance();
    bool loadNextChunk();
    //! Consume one record in the current chunk. Returns true when it produced a match.
    bool parseOne();
    void rememberConnection(uint32_t id, std::string const& topic, miniros::Header& connection_header);
    bool accepts(ConnectionInfo const* info);

    std::unique_ptr<Bag> owned_;
    Bag* bag_;
    std::function<bool(ConnectionInfo const*)> query_;
    Time start_;
    Time end_;

    bool finished_;
    uint64_t chunk_pos_;
    uint32_t chunk_size_;
    uint32_t offset_;
    std::unique_ptr<MessageInstance> current_;
    std::map<uint32_t, bool> accept_;
};

} // namespace minibag
