// :mode=c++:
/*
encode.h - c++ wrapper for a base64 encoding algorithm

This is part of the libb64 project, and has been placed in the public domain.
For details, see http://sourceforge.net/projects/libb64
*/
#ifndef BASE64_ENCODE_H
#define BASE64_ENCODE_H

#include <iostream>

#include "xmlrpcpp/XmlRpcDecl.h"

namespace base64 {

typedef enum { step_A, step_B, step_C } base64_encodestep;

typedef struct {
  base64_encodestep step;
  char result;
  int stepcount;
} base64_encodestate;

XMLRPCPP_DECL void base64_init_encodestate(base64_encodestate* state_in);

XMLRPCPP_DECL char base64_encode_value(char value_in);

XMLRPCPP_DECL int base64_encode_block(const char* plaintext_in, int length_in, char* code_out, base64_encodestate* state_in);

XMLRPCPP_DECL int base64_encode_blockend(char* code_out, base64_encodestate* state_in);

struct XMLRPCPP_DECL Encoder {
  base64_encodestate _state;
  int _buffersize;

  Encoder(int buffersize_in = 512);

  int encode(char value_in);

  int encode(const char* code_in, const int length_in, char* plaintext_out);

  int encode_end(char* plaintext_out);

  void encode(std::istream& istream_in, std::ostream& ostream_in);
};

} // namespace base64

#endif // BASE64_ENCODE_H
