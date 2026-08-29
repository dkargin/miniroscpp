//
// YAML → RpcValue. Nested maps and sequences of scalars; no anchors.
//

#include "miniros/utility/yaml.h"

#include <cctype>
#include <climits>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>

namespace miniros {
namespace {

struct Parser {
  const std::string& text;
  size_t pos = 0;
  Error error = Error::Ok;

  explicit Parser(const std::string& t) : text(t) {}

  bool eof() const { return pos >= text.size(); }
  char peek() const { return eof() ? '\0' : text[pos]; }

  void fail()
  {
    if (error == Error::Ok)
      error = Error::InvalidValue;
  }

  bool startsWith(const char* s) const
  {
    const size_t n = std::char_traits<char>::length(s);
    return pos + n <= text.size() && text.compare(pos, n, s) == 0;
  }

  void skipBom()
  {
    if (text.size() >= 3 && static_cast<unsigned char>(text[0]) == 0xef &&
        static_cast<unsigned char>(text[1]) == 0xbb &&
        static_cast<unsigned char>(text[2]) == 0xbf)
      pos = 3;
  }

  void skipSpaces()
  {
    while (!eof() && (peek() == ' ' || peek() == '\t' || peek() == '\r'))
      ++pos;
  }

  void skipComment()
  {
    if (peek() == '#') {
      while (!eof() && peek() != '\n')
        ++pos;
    }
  }

  void skipToEol()
  {
    skipSpaces();
    skipComment();
  }

  void skipNewlines()
  {
    while (peek() == '\n')
      ++pos;
  }

  /// Skip blank lines, comments, and trailing spaces. Stops at content or EOF.
  void skipBlanks()
  {
    while (!eof()) {
      skipToEol();
      if (peek() == '\n') {
        ++pos;
        continue;
      }
      break;
    }
  }

  int currentIndent() const
  {
    size_t s = pos;
    while (s > 0 && text[s - 1] != '\n')
      --s;
    int ind = 0;
    while (s < text.size() && text[s] == ' ') {
      ++ind;
      ++s;
    }
    return ind;
  }

  /// Column of the current character (spaces from start of line).
  int currentColumn() const
  {
    size_t s = pos;
    while (s > 0 && text[s - 1] != '\n')
      --s;
    return static_cast<int>(pos - s);
  }

  void skipPreamble()
  {
    skipBom();
    while (!eof()) {
      skipBlanks();
      if (currentIndent() != 0)
        break;
      if (startsWith("%YAML") || startsWith("---") || startsWith("...")) {
        while (!eof() && peek() != '\n')
          ++pos;
        continue;
      }
      break;
    }
  }

  void skipTag()
  {
    if (peek() != '!')
      return;
    ++pos;
    if (peek() == '!')
      ++pos;
    while (!eof()) {
      const unsigned char c = static_cast<unsigned char>(peek());
      if (std::isalnum(c) || c == '-' || c == '_' || c == '.')
        ++pos;
      else
        break;
    }
    skipSpaces();
  }

  bool looksLikeKey() const
  {
    size_t i = pos;
    if (i >= text.size())
      return false;
    const unsigned char first = static_cast<unsigned char>(text[i]);
    if (!std::isalnum(first) && first != '_' && first != '.')
      return false;
    while (i < text.size()) {
      const unsigned char c = static_cast<unsigned char>(text[i]);
      if (std::isalnum(c) || c == '_' || c == '-' || c == '.' || c == '/')
        ++i;
      else
        break;
    }
    while (i < text.size() && (text[i] == ' ' || text[i] == '\t'))
      ++i;
    return i < text.size() && text[i] == ':';
  }

  bool looksLikeDash() const
  {
    if (peek() != '-')
      return false;
    if (pos + 1 >= text.size())
      return true;
    const char n = text[pos + 1];
    return n == ' ' || n == '\t' || n == '\n' || n == '\r' || n == '#';
  }

  std::string parseKey()
  {
    const size_t start = pos;
    const unsigned char first = static_cast<unsigned char>(peek());
    if (!std::isalnum(first) && first != '_' && first != '.') {
      fail();
      return {};
    }
    ++pos;
    while (!eof()) {
      const unsigned char c = static_cast<unsigned char>(peek());
      if (std::isalnum(c) || c == '_' || c == '-' || c == '.' || c == '/')
        ++pos;
      else
        break;
    }
    std::string key = text.substr(start, pos - start);
    skipSpaces();
    if (peek() != ':') {
      fail();
      return {};
    }
    ++pos;
    return key;
  }

  RpcValue makeScalar(std::string raw)
  {
    while (!raw.empty() && (raw.back() == ' ' || raw.back() == '\t' || raw.back() == '\r'))
      raw.pop_back();
    size_t b = 0;
    while (b < raw.size() && (raw[b] == ' ' || raw[b] == '\t'))
      ++b;
    raw = raw.substr(b);

    if (raw.empty())
      return RpcValue(std::string());

    if (raw == "~" || raw == "null" || raw == "Null" || raw == "NULL")
      return RpcValue();

    auto lowerEq = [&](const char* word) {
      const size_t n = std::char_traits<char>::length(word);
      if (raw.size() != n)
        return false;
      for (size_t i = 0; i < n; ++i) {
        if (std::tolower(static_cast<unsigned char>(raw[i])) !=
            static_cast<unsigned char>(word[i]))
          return false;
      }
      return true;
    };
    if (lowerEq("true"))
      return RpcValue(true);
    if (lowerEq("false"))
      return RpcValue(false);

    char* end = nullptr;
    const char* cstr = raw.c_str();
    const bool isNum = (raw[0] == '+' || raw[0] == '-' || std::isdigit(static_cast<unsigned char>(raw[0])));
    if (isNum) {
      const bool isFloat = raw.find('.') != std::string::npos ||
                           raw.find('e') != std::string::npos ||
                           raw.find('E') != std::string::npos;
      if (isFloat) {
        double d = std::strtod(cstr, &end);
        if (end != cstr && *end == '\0')
          return RpcValue(d);
      } else {
        long v = std::strtol(cstr, &end, 10);
        if (end != cstr && *end == '\0') {
          if (v >= static_cast<long>(INT_MIN) && v <= static_cast<long>(INT_MAX))
            return RpcValue(static_cast<int>(v));
          return RpcValue(static_cast<double>(v));
        }
      }
    }
    return RpcValue(raw);
  }

  RpcValue parseQuoted()
  {
    const char q = peek();
    ++pos;
    std::string out;
    while (!eof() && peek() != q) {
      if (q == '"' && peek() == '\\') {
        ++pos;
        if (eof())
          break;
        const char e = peek();
        ++pos;
        if (e == 'n')
          out.push_back('\n');
        else if (e == 't')
          out.push_back('\t');
        else
          out.push_back(e);
        continue;
      }
      if (q == '\'' && peek() == '\'' && pos + 1 < text.size() && text[pos + 1] == '\'') {
        out.push_back('\'');
        pos += 2;
        continue;
      }
      out.push_back(peek());
      ++pos;
    }
    if (peek() != q)
      fail();
    else
      ++pos;
    return RpcValue(out);
  }

  RpcValue parseFlowScalar()
  {
    skipFlowWs();
    if (peek() == '"' || peek() == '\'')
      return parseQuoted();
    const size_t start = pos;
    while (!eof()) {
      const char c = peek();
      if (c == ',' || c == ']' || c == '}' || c == '\n')
        break;
      if (c == '#' && (pos == start || text[pos - 1] == ' ' || text[pos - 1] == '\t'))
        break;
      ++pos;
    }
    return makeScalar(text.substr(start, pos - start));
  }

  void skipFlowWs()
  {
    while (!eof()) {
      skipSpaces();
      if (peek() == '#') {
        skipComment();
        continue;
      }
      if (peek() == '\n') {
        ++pos;
        continue;
      }
      break;
    }
  }

  RpcValue parseFlowSeq()
  {
    ++pos; // [
    RpcValue arr = RpcValue::Array(0);
    skipFlowWs();
    if (peek() == ']') {
      ++pos;
      return arr;
    }
    int n = 0;
    while (!eof() && error == Error::Ok) {
      RpcValue item = parseFlowNode();
      arr.setSize(static_cast<size_t>(n + 1));
      arr[n] = item;
      ++n;
      skipFlowWs();
      if (peek() == ',') {
        ++pos;
        skipFlowWs();
        if (peek() == ']') {
          ++pos;
          break;
        }
        continue;
      }
      if (peek() == ']') {
        ++pos;
        break;
      }
      fail();
      break;
    }
    return arr;
  }

  RpcValue parseFlowMap()
  {
    ++pos; // {
    RpcValue obj = RpcValue::Dict();
    skipFlowWs();
    if (peek() == '}') {
      ++pos;
      return obj;
    }
    while (!eof() && error == Error::Ok) {
      skipFlowWs();
      std::string key;
      if (peek() == '"' || peek() == '\'') {
        RpcValue k = parseQuoted();
        key = static_cast<const std::string&>(k);
      } else {
        const size_t start = pos;
        while (!eof() && peek() != ':' && peek() != ',' && peek() != '}' && peek() != '\n')
          ++pos;
        key = text.substr(start, pos - start);
        while (!key.empty() && (key.back() == ' ' || key.back() == '\t'))
          key.pop_back();
      }
      skipFlowWs();
      if (peek() != ':') {
        fail();
        break;
      }
      ++pos;
      RpcValue val = parseFlowNode();
      obj[key] = val;
      skipFlowWs();
      if (peek() == ',') {
        ++pos;
        skipFlowWs();
        if (peek() == '}') {
          ++pos;
          break;
        }
        continue;
      }
      if (peek() == '}') {
        ++pos;
        break;
      }
      fail();
      break;
    }
    return obj;
  }

  RpcValue parseFlowNode()
  {
    skipFlowWs();
    skipTag();
    skipFlowWs();
    if (peek() == '[')
      return parseFlowSeq();
    if (peek() == '{')
      return parseFlowMap();
    return parseFlowScalar();
  }

  RpcValue parseBlockScalar()
  {
    if (peek() == '"' || peek() == '\'') {
      RpcValue v = parseQuoted();
      skipToEol();
      return v;
    }
    const size_t start = pos;
    while (!eof() && peek() != '\n') {
      if (peek() == '#' && (pos == start || text[pos - 1] == ' ' || text[pos - 1] == '\t'))
        break;
      ++pos;
    }
    RpcValue v = makeScalar(text.substr(start, pos - start));
    skipToEol();
    return v;
  }

  RpcValue parseBlockMap(int keyIndent)
  {
    RpcValue obj = RpcValue::Dict();
    while (!eof() && error == Error::Ok) {
      skipBlanks();
      if (eof())
        break;
      if (currentColumn() != keyIndent)
        break;
      if (!looksLikeKey())
        break;
      const std::string key = parseKey();
      if (error != Error::Ok)
        break;
      obj[key] = parseValue(keyIndent);
    }
    return obj;
  }

  RpcValue parseBlockSeq(int dashIndent)
  {
    RpcValue arr = RpcValue::Array(0);
    int n = 0;
    while (!eof() && error == Error::Ok) {
      skipBlanks();
      if (eof())
        break;
      if (currentIndent() != dashIndent || !looksLikeDash())
        break;
      ++pos; // '-'
      skipSpaces();
      RpcValue item;
      skipTag();
      skipSpaces();
      if (peek() == '\n' || peek() == '#' || eof()) {
        skipToEol();
        if (peek() == '\n')
          ++pos;
        skipBlanks();
        if (eof() || currentIndent() <= dashIndent)
          item = RpcValue();
        else if (looksLikeDash())
          item = parseBlockSeq(currentIndent());
        else
          item = parseBlockMap(currentIndent());
      } else if (peek() == '[') {
        item = parseFlowSeq();
        skipToEol();
      } else if (peek() == '{') {
        item = parseFlowMap();
        skipToEol();
      } else if (looksLikeKey()) {
        const int mapIndent = currentColumn();
        item = parseBlockMap(mapIndent);
      } else {
        item = parseBlockScalar();
      }
      arr.setSize(static_cast<size_t>(n + 1));
      arr[n] = item;
      ++n;
    }
    return arr;
  }

  RpcValue parseValue(int parentIndent)
  {
    skipSpaces();
    skipTag();
    skipSpaces();
    if (peek() == '[') {
      RpcValue v = parseFlowSeq();
      skipToEol();
      return v;
    }
    if (peek() == '{') {
      RpcValue v = parseFlowMap();
      skipToEol();
      return v;
    }
    if (!eof() && peek() != '\n' && peek() != '#')
      return parseBlockScalar();

    skipToEol();
    if (peek() == '\n')
      ++pos;
    skipBlanks();
    if (eof() || currentIndent() <= parentIndent)
      return RpcValue();
    const int ind = currentIndent();
    if (looksLikeDash())
      return parseBlockSeq(ind);
    return parseBlockMap(ind);
  }

  RpcValue parseDocument()
  {
    skipPreamble();
    skipBlanks();
    if (eof())
      return RpcValue::Dict();
    const int ind = currentIndent();
    if (looksLikeDash())
      return parseBlockSeq(ind);
    if (peek() == '[')
      return parseFlowSeq();
    if (peek() == '{')
      return parseFlowMap();
    if (looksLikeKey())
      return parseBlockMap(ind);
    return parseBlockScalar();
  }
};

} // namespace

std::string rpcValueToString(const RpcValue& value)
{
  switch (value.getType()) {
  case RpcValue::TypeBoolean:
    return static_cast<bool>(value) ? "1" : "0";
  case RpcValue::TypeInt:
    return std::to_string(static_cast<int>(value));
  case RpcValue::TypeDouble: {
    std::ostringstream os;
    os << static_cast<double>(value);
    return os.str();
  }
  case RpcValue::TypeString:
    return static_cast<const std::string&>(value);
  default:
    return {};
  }
}

Error parseYaml(const std::string& text, RpcValue& out)
{
  Parser p(text);
  out = p.parseDocument();
  p.skipBlanks();
  if (p.error != Error::Ok)
    return p.error;
  return Error::Ok;
}

Error loadYamlFile(const std::string& path, RpcValue& out)
{
  if (path.empty())
    return Error::FileNotFound;
  std::ifstream in(path);
  if (!in)
    return Error::FileNotFound;
  std::ostringstream ss;
  ss << in.rdbuf();
  if (!in && !in.eof())
    return Error::SystemError;
  return parseYaml(ss.str(), out);
}

} // namespace miniros
