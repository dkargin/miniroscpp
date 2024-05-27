#pragma once

#include "minibag/stream.h"

#ifdef MINIBAG_HAS_LZ4
#include <miniros/lz4s.h>
#endif

namespace minibag {

#ifdef MINIBAG_HAS_LZ4
// LZ4Stream reads/writes compressed datat in the LZ4 format
// https://code.google.com/p/lz4/
class ROSBAG_STORAGE_DECL LZ4Stream : public Stream
{
public:
    LZ4Stream(ChunkedFile* file);
    ~LZ4Stream();

    CompressionType getCompressionType() const;

    void startWrite();
    void write(void* ptr, size_t size);
    void stopWrite();

    void startRead();
    void read(void* ptr, size_t size);
    void stopRead();

    void decompress(uint8_t* dest, unsigned int dest_len, uint8_t* source, unsigned int source_len);

private:
    LZ4Stream(const LZ4Stream&);
    LZ4Stream operator=(const LZ4Stream&);
    void writeStream(int action);

    char *buff_;
    int buff_size_;
    int block_size_id_;
    roslz4_stream lz4s_;
};
#endif

} // namespace minibag