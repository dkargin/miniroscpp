/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2017, Open Source Robotics Foundation
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

#include "minibag/buffer.h"
#include "minibag/chunked_file.h"
#include "minibag/structures.h"

#include "miniros/header.h"

#include <stdint.h>
#include <string>
#include <functional>


namespace minibag {

class Bag;
class ChunkedFile;

class EncryptorBase
{
protected:
    EncryptorBase() { }

public:
    virtual ~EncryptorBase() { }

    //! Initialize encryptor
    /*!
     * \param bag The Bag instance
     * \param plugin_param The string parameter used while initializing the encryptor
     *
     * This method is called by setEncryptorPlugin, which loads an encryptor plugin.
     */
    virtual void initialize(Bag const& bag, std::string const& plugin_param) = 0;

    //! Encrypt chunk
    /*!
     * \return The byte size of the encrypted chunk
     * \param chunk_size The byte size of the original chunk
     * \param chunk_data_pos The start position of the chunk data in bag file stream
     * \param file The bag file stream
     *
     * This method reads the original chunk from [chunk_data_pos, chunk_data_pos+chunk_size), encrypts it, and
     * writes back to the file stream starting at chunk_data_pos.
     */
    virtual uint32_t encryptChunk(const uint32_t chunk_size, const uint64_t chunk_data_pos, ChunkedFile& file) = 0;

    //! Decrypt chunk
    /*!
     * \param chunk_header The header of the encrypted chunk
     * \param decrypted_chunk The buffer where decrypted chunk is written to
     * \param file The bag file stream from which the encrypted chunk is read
     *
     * This method reads the encrypted chunk from file stream, decrypts, and writes it to decrypted_chunk.
     */
    virtual void decryptChunk(ChunkHeader const& chunk_header, Buffer& decrypted_chunk, ChunkedFile& file) const = 0;

    //! Add encryptor information to bag file header
    /*!
     * \param header_fields The header fields of the bag
     *
     * Called for a bag being written, this method adds encryptor-specific fields to the bag file header. Those fields
     * are used when decrypting the bag. ENCRYPTOR_FIELD_NAME must be specified in the header except for NoEncryptor.
     */
    virtual void addFieldsToFileHeader(miniros::M_string& header_fields) const = 0;

    //! Read encryptor information from bag file header
    /*!
     * \param header_fields The header fields of the bag
     *
     * Called for a bag being read, this method reads encryptor-specific fields from the bag file header.
     */
    virtual void readFieldsFromFileHeader(miniros::M_string const& header_fields) = 0;

    //! Write encrypted header to bag file
    /*!
     * \param write_header The functor writing unencrypted header
     * \param header_fields The header fields to be written
     * \param file The bag file stream
     *
     * This method encrypts given header fields, and writes them to the bag file.
     */
    virtual void writeEncryptedHeader(std::function<void(miniros::M_string const&)> write_header, miniros::M_string const& header_fields, ChunkedFile& file) = 0;

    //! Read encrypted header from bag file
    /*!
     * \param read_header The functor reading unencrypted header
     * \param header The header object read
     * \param header_buffer The header buffer read
     * \param file The bag file stream
     *
     * This method reads and decrypts encrypted header to output header object (header) and buffer (header_buffer).
     */
    virtual bool readEncryptedHeader(std::function<bool(miniros::Header&)> read_header, miniros::Header& header, Buffer& header_buffer, ChunkedFile& file) = 0;
};

} // namespace minibag

