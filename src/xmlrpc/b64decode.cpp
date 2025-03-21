/*
cdecoder.c - c source to a base64 decoding algorithm implementation

This is part of the libb64 project, and has been placed in the public domain.
For details, see http://sourceforge.net/projects/libb64
*/

#include "b64/decode.h"

namespace base64 {
int base64_decode_value(signed char value_in)
{
  static const signed char decoding[] = {62, -1, -1, -1, 63, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, -1, -1, -1, -2, -1, -1,
    -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, -1, -1, -1, -1,
    -1, -1, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51};
  static const int decoding_size = sizeof(decoding);
  int index = int(value_in) - 43;
  if (index < 0 || index >= decoding_size)
    return -1;
  return decoding[index];
}

void base64_init_decodestate(base64_decodestate* state_in)
{
  state_in->step = step_a;
  state_in->plainchar = 0;
}

int base64_decode_block(const char* code_in, const int length_in, char* plaintext_out, base64_decodestate* state_in)
{
  const char* codechar = code_in;
  char* plainchar = plaintext_out;
  signed char fragment;

  if (length_in == 0) {
    return 0;
  }

  *plainchar = state_in->plainchar;

  switch (state_in->step) {
    while (1) {
      case step_a:
        do {
          if (codechar == code_in + length_in) {
            state_in->step = step_a;
            state_in->plainchar = 0; // no state to save; use default value
            return plainchar - plaintext_out;
          }
          fragment = (char)base64_decode_value(*codechar++);
        } while (fragment < 0);
      *plainchar = (fragment & 0x03f) << 2;
      [[fallthrough]];
      case step_b:
        do {
          if (codechar == code_in + length_in) {
            state_in->step = step_b;
            state_in->plainchar = *plainchar;
            return plainchar - plaintext_out;
          }
          fragment = (char)base64_decode_value(*codechar++);
        } while (fragment < 0);
      *plainchar++ |= (fragment & 0x030) >> 4;
      *plainchar = (fragment & 0x00f) << 4;
      [[fallthrough]];
      case step_c:
        do {
          if (codechar == code_in + length_in) {
            state_in->step = step_c;
            state_in->plainchar = *plainchar;
            return plainchar - plaintext_out;
          }
          fragment = (char)base64_decode_value(*codechar++);
        } while (fragment < 0);
      *plainchar++ |= (fragment & 0x03c) >> 2;
      *plainchar = (fragment & 0x003) << 6;
      [[fallthrough]];
      case step_d:
        do {
          if (codechar == code_in + length_in) {
            state_in->step = step_d;
            state_in->plainchar = *plainchar;
            return plainchar - plaintext_out;
          }
          fragment = (char)base64_decode_value(*codechar++);
        } while (fragment < 0);
      *plainchar++ |= (fragment & 0x03f);
    }
  }
  /* control should not reach here */
  return plainchar - plaintext_out;
}

Decoder::Decoder(int buffersize_in) : _buffersize(buffersize_in)
{
  base64_init_decodestate(&_state);
}

int Decoder::decode(char value_in)
{
  return base64_decode_value(value_in);
}

int Decoder::decode(const char* code_in, const int length_in, char* plaintext_out)
{
  return base64_decode_block(code_in, length_in, plaintext_out, &_state);
}

void Decoder::decode(std::istream& istream_in, std::ostream& ostream_in)
{
  base64_init_decodestate(&_state);
  //
  const int N = _buffersize;
  char* code = new char[N];
  char* plaintext = new char[N];
  int codelength;
  int plainlength;

  do {
    istream_in.read((char*)code, N);
    codelength = istream_in.gcount();
    plainlength = decode(code, codelength, plaintext);
    ostream_in.write((const char*)plaintext, plainlength);
  } while (istream_in.good() && codelength > 0);
  //
  base64_init_decodestate(&_state);

  delete[] code;
  delete[] plaintext;
}

} // namespace base64