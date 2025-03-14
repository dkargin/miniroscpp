//
// Created by dkargin on 3/11/25.
//

#ifndef MINIROS_JSON_TOOLS_H
#define MINIROS_JSON_TOOLS_H

namespace miniros {

struct JsonState {
  // Current offset.
  int offset = 0;
  bool sameline = false;
};

struct JsonSettings {
  int tabs = 2;
};

} // namespace miniros

#endif //MINIROS_JSON_TOOLS_H
