// :mode=c++:
/*
decode.h - c++ wrapper for a base64 decoding algorithm

This is part of the libb64 project, and has been placed in the public domain.
For details, see http://sourceforge.net/projects/libb64
*/
#ifndef BASE64_DECODE_H
#define BASE64_DECODE_H

#include "xmlrpcpp/XmlRpcDecl.h"

#include <iostream>

namespace base64 {

typedef enum { step_a, step_b, step_c, step_d } base64_decodestep;

typedef struct {
  base64_decodestep step;
  char plainchar;
} base64_decodestate;

XMLRPCPP_DECL void base64_init_decodestate(base64_decodestate* state_in);

XMLRPCPP_DECL int base64_decode_value(signed char value_in);

XMLRPCPP_DECL int base64_decode_block(const char* code_in, const int length_in, char* plaintext_out, base64_decodestate* state_in);

struct XMLRPCPP_DECL Decoder {
  base64_decodestate _state;
  int _buffersize;

  Decoder(int buffersize_in = 512);

  int decode(char value_in);

  int decode(const char* code_in, const int length_in, char* plaintext_out);

  void decode(std::istream& istream_in, std::ostream& ostream_in);
};

} // namespace base64

#endif // BASE64_DECODE_H
