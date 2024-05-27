#pragma once

#include "minibag/stream.h"

#ifdef MINIBAG_HAS_BZIP2
#include <bzlib.h>
#endif

namespace minibag {

#ifdef MINIBAG_HAS_BZIP2
/*!
 * BZ2Stream uses libbzip2 (http://www.bzip.org) for reading/writing compressed data in the BZ2 format.
 */
class ROSBAG_STORAGE_DECL BZ2Stream : public Stream
{
public:
    BZ2Stream(ChunkedFile* file);

    CompressionType getCompressionType() const;

    void startWrite();
    void write(void* ptr, size_t size);
    void stopWrite();

    void startRead();
    void read(void* ptr, size_t size);
    void stopRead();

    void decompress(uint8_t* dest, unsigned int dest_len, uint8_t* source, unsigned int source_len);

private:
    int     verbosity_;        //!< level of debugging output (0-4; 0 default). 0 is silent, 4 is max verbose debugging output
    int     block_size_100k_;  //!< compression block size (1-9; 9 default). 9 is best compression, most memory
    int     work_factor_;      //!< compression behavior for worst case, highly repetitive data (0-250; 30 default)

    BZFILE* bzfile_;           //!< bzlib compressed file stream
    int     bzerror_;          //!< last error from bzlib
};
#endif

} // namespace minibag
