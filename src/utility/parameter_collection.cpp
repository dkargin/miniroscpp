//
// ParameterCollection implementation.
//

#include "miniros/utility/parameter_collection.h"
#include "miniros/utility/yaml.h"

#include <cassert>
#include <cctype>
#include <cstdlib>
#include <sstream>
#include <stdexcept>

namespace miniros {

namespace {

bool enumHasCode(const std::vector<EnumOption>& options, const std::string& code)
{
  for (const EnumOption& opt : options) {
    if (opt.code == code)
      return true;
  }
  return false;
}

std::string yamlOneLine(const std::string& text)
{
  std::string out;
  out.reserve(text.size());
  bool space = false;
  for (char c : text) {
    if (c == '\n' || c == '\r' || c == '\t') {
      space = true;
      continue;
    }
    if (c == ' ' && (out.empty() || space)) {
      space = true;
      continue;
    }
    if (space && !out.empty())
      out.push_back(' ');
    space = false;
    out.push_back(c);
  }
  return out;
}

bool yamlPlainScalar(const std::string& s)
{
  if (s.empty())
    return false;
  auto lowerEq = [&](const char* word) {
    if (s.size() != std::char_traits<char>::length(word))
      return false;
    for (size_t i = 0; i < s.size(); ++i) {
      if (std::tolower(static_cast<unsigned char>(s[i])) !=
          static_cast<unsigned char>(word[i]))
        return false;
    }
    return true;
  };
  if (lowerEq("true") || lowerEq("false") || lowerEq("null") || lowerEq("yes") ||
      lowerEq("no") || lowerEq("on") || lowerEq("off") || lowerEq("~"))
    return false;

  const unsigned char first = static_cast<unsigned char>(s[0]);
  if (!std::isalnum(first) && first != '_' && first != '.')
    return false;
  for (char ch : s) {
    const unsigned char c = static_cast<unsigned char>(ch);
    if (std::isalnum(c) || c == '_' || c == '-' || c == '.' || c == '/')
      continue;
    return false;
  }
  return true;
}

std::string yamlQuoteScalar(const std::string& s)
{
  if (yamlPlainScalar(s))
    return s;
  std::string out = "\"";
  out.reserve(s.size() + 2);
  for (char c : s) {
    if (c == '\\' || c == '"')
      out.push_back('\\');
    out.push_back(c);
  }
  out.push_back('"');
  return out;
}

} // namespace

std::string ParamSpec::valueAsString() const
{
  if (type == ParamType::Bool)
    return bool_value ? "1" : "0";
  if (type == ParamType::Int)
    return std::to_string(int_value);
  if (type == ParamType::Double) {
    std::ostringstream oss;
    oss << double_value;
    return oss.str();
  }
  if (type == ParamType::Enum)
    return string_value;
  return {};
}

ParameterCollection::ParameterCollection() = default;

ParameterCollection::ParameterCollection(const ParameterCollection& other)
{
  Lock lock{other.mutex_};
  params_ = other.params_;
}

ParameterCollection& ParameterCollection::operator=(const ParameterCollection& other)
{
  if (this == &other)
    return *this;
  std::lock(mutex_, other.mutex_);
  Lock selfLock{mutex_, std::adopt_lock};
  Lock otherLock{other.mutex_, std::adopt_lock};
  params_ = other.params_;
  return *this;
}

ParamSpecRef::ParamSpecRef(ParameterCollection* collection, size_t index)
  : collection_(collection), index_(index)
{}

ParamSpecRef& ParamSpecRef::label(std::string text)
{
  ParameterCollection::Lock lock{collection_->mutex_};
  collection_->params_[index_].label = std::move(text);
  return *this;
}

ParamSpecRef& ParamSpecRef::description(std::string text)
{
  ParameterCollection::Lock lock{collection_->mutex_};
  collection_->params_[index_].description = std::move(text);
  return *this;
}

ParamSpecRef& ParamSpecRef::min(int value)
{
  ParameterCollection::Lock lock{collection_->mutex_};
  ParamSpec& spec = collection_->params_[index_];
  if (spec.type == ParamType::Int) {
    spec.int_min = value;
    spec.has_min = true;
  } else if (spec.type == ParamType::Double) {
    spec.double_min = static_cast<double>(value);
    spec.has_min = true;
  }
  return *this;
}

ParamSpecRef& ParamSpecRef::max(int value)
{
  ParameterCollection::Lock lock{collection_->mutex_};
  ParamSpec& spec = collection_->params_[index_];
  if (spec.type == ParamType::Int) {
    spec.int_max = value;
    spec.has_max = true;
  } else if (spec.type == ParamType::Double) {
    spec.double_max = static_cast<double>(value);
    spec.has_max = true;
  }
  return *this;
}

ParamSpecRef& ParamSpecRef::min(double value)
{
  ParameterCollection::Lock lock{collection_->mutex_};
  ParamSpec& spec = collection_->params_[index_];
  if (spec.type == ParamType::Double) {
    spec.double_min = value;
    spec.has_min = true;
  } else if (spec.type == ParamType::Int) {
    spec.int_min = static_cast<int>(value);
    spec.has_min = true;
  }
  return *this;
}

ParamSpecRef& ParamSpecRef::max(double value)
{
  ParameterCollection::Lock lock{collection_->mutex_};
  ParamSpec& spec = collection_->params_[index_];
  if (spec.type == ParamType::Double) {
    spec.double_max = value;
    spec.has_max = true;
  } else if (spec.type == ParamType::Int) {
    spec.int_max = static_cast<int>(value);
    spec.has_max = true;
  }
  return *this;
}

ParamSpecRef ParameterCollection::addBool(const std::string& name, bool value)
{
  Lock lock{mutex_};
  ParamSpec spec;
  spec.name = name;
  spec.type = ParamType::Bool;
  spec.bool_value = value;
  params_.push_back(std::move(spec));
  return ParamSpecRef(this, params_.size() - 1);
}

ParamSpecRef ParameterCollection::addInt(const std::string& name, int value)
{
  Lock lock{mutex_};
  ParamSpec spec;
  spec.name = name;
  spec.type = ParamType::Int;
  spec.int_value = value;
  params_.push_back(std::move(spec));
  return ParamSpecRef(this, params_.size() - 1);
}

ParamSpecRef ParameterCollection::addDouble(const std::string& name, double value)
{
  Lock lock{mutex_};
  ParamSpec spec;
  spec.name = name;
  spec.type = ParamType::Double;
  spec.double_value = value;
  params_.push_back(std::move(spec));
  return ParamSpecRef(this, params_.size() - 1);
}

ParamSpecRef ParameterCollection::addEnum(const std::string& name, std::vector<EnumOption> options,
                                          const std::string& selected)
{
  Lock lock{mutex_};
  ParamSpec spec;
  spec.name = name;
  spec.type = ParamType::Enum;
  spec.enum_options = std::move(options);
  if (selected.empty() || !enumHasCode(spec.enum_options, selected)) {
    if (!spec.enum_options.empty())
      spec.string_value = spec.enum_options.front().code;
  } else {
    spec.string_value = selected;
  }
  params_.push_back(std::move(spec));
  return ParamSpecRef(this, params_.size() - 1);
}

ParamSpec* ParameterCollection::findLocked(Lock& lock, const std::string& name)
{
  assert(lock.owns_lock());
  assert(lock.mutex() == &mutex_);
  for (ParamSpec& p : params_) {
    if (p.name == name)
      return &p;
  }
  return nullptr;
}

const ParamSpec* ParameterCollection::findLocked(Lock& lock, const std::string& name) const
{
  assert(lock.owns_lock());
  assert(lock.mutex() == &mutex_);
  for (const ParamSpec& p : params_) {
    if (p.name == name)
      return &p;
  }
  return nullptr;
}

bool ParameterCollection::has(const std::string& name) const
{
  Lock lock{mutex_};
  return findLocked(lock, name) != nullptr;
}

bool ParameterCollection::getBool(const std::string& name) const
{
  Lock lock{mutex_};
  const ParamSpec* p = findLocked(lock, name);
  if (!p || p->type != ParamType::Bool)
    throw std::runtime_error("ParameterCollection::getBool: bad param " + name);
  return p->bool_value;
}

int ParameterCollection::getInt(const std::string& name) const
{
  Lock lock{mutex_};
  const ParamSpec* p = findLocked(lock, name);
  if (!p)
    throw std::runtime_error("ParameterCollection::getInt: missing " + name);
  if (p->type == ParamType::Int)
    return p->int_value;
  if (p->type == ParamType::Enum) {
    char* end = nullptr;
    long v = std::strtol(p->string_value.c_str(), &end, 10);
    if (end != p->string_value.c_str() && *end == '\0')
      return static_cast<int>(v);
  }
  throw std::runtime_error("ParameterCollection::getInt: not an int param " + name);
}

double ParameterCollection::getDouble(const std::string& name) const
{
  Lock lock{mutex_};
  const ParamSpec* p = findLocked(lock, name);
  if (!p || p->type != ParamType::Double)
    throw std::runtime_error("ParameterCollection::getDouble: bad param " + name);
  return p->double_value;
}

std::string ParameterCollection::getString(const std::string& name) const
{
  Lock lock{mutex_};
  const ParamSpec* p = findLocked(lock, name);
  if (!p)
    throw std::runtime_error("ParameterCollection::getString: missing " + name);
  return p->valueAsString();
}

Error ParameterCollection::setBool(const std::string& name, bool value)
{
  Lock lock{mutex_};
  ParamSpec* p = findLocked(lock, name);
  if (!p || p->type != ParamType::Bool)
    return Error::ParameterNotFound;
  p->bool_value = value;
  return Error::Ok;
}

Error ParameterCollection::setInt(const std::string& name, int value)
{
  Lock lock{mutex_};
  ParamSpec* p = findLocked(lock, name);
  if (!p)
    return Error::ParameterNotFound;
  if (p->type == ParamType::Int) {
    if (p->has_min && value < p->int_min)
      return Error::InvalidValue;
    if (p->has_max && value > p->int_max)
      return Error::InvalidValue;
    p->int_value = value;
    return Error::Ok;
  }
  if (p->type == ParamType::Enum) {
    const std::string code = std::to_string(value);
    if (!enumHasCode(p->enum_options, code))
      return Error::InvalidValue;
    p->string_value = code;
    return Error::Ok;
  }
  return Error::InvalidValue;
}

Error ParameterCollection::setDouble(const std::string& name, double value)
{
  Lock lock{mutex_};
  ParamSpec* p = findLocked(lock, name);
  if (!p || p->type != ParamType::Double)
    return Error::ParameterNotFound;
  if (p->has_min && value < p->double_min)
    return Error::InvalidValue;
  if (p->has_max && value > p->double_max)
    return Error::InvalidValue;
  p->double_value = value;
  return Error::Ok;
}

Error ParameterCollection::setString(const std::string& name, const std::string& value)
{
  Lock lock{mutex_};
  ParamSpec* p = findLocked(lock, name);
  if (!p || p->type != ParamType::Enum)
    return Error::ParameterNotFound;
  if (!enumHasCode(p->enum_options, value))
    return Error::InvalidValue;
  p->string_value = value;
  return Error::Ok;
}

Error ParameterCollection::validateAndAssign(ParamSpec& spec, const std::string& raw)
{
  if (spec.type == ParamType::Bool) {
    spec.bool_value = (raw == "1" || raw == "true" || raw == "on" || raw == "yes");
    return Error::Ok;
  }
  if (spec.type == ParamType::Int) {
    char* end = nullptr;
    long v = std::strtol(raw.c_str(), &end, 10);
    if (end == raw.c_str() || *end != '\0')
      return Error::InvalidValue;
    const int iv = static_cast<int>(v);
    if (spec.has_min && iv < spec.int_min)
      return Error::InvalidValue;
    if (spec.has_max && iv > spec.int_max)
      return Error::InvalidValue;
    spec.int_value = iv;
    return Error::Ok;
  }
  if (spec.type == ParamType::Double) {
    char* end = nullptr;
    double v = std::strtod(raw.c_str(), &end);
    if (end == raw.c_str() || *end != '\0')
      return Error::InvalidValue;
    if (spec.has_min && v < spec.double_min)
      return Error::InvalidValue;
    if (spec.has_max && v > spec.double_max)
      return Error::InvalidValue;
    spec.double_value = v;
    return Error::Ok;
  }
  if (spec.type == ParamType::Enum) {
    if (!enumHasCode(spec.enum_options, raw))
      return Error::InvalidValue;
    spec.string_value = raw;
    return Error::Ok;
  }
  return Error::InvalidValue;
}

Error ParameterCollection::assignFromString(const std::string& name, const std::string& raw)
{
  Lock lock{mutex_};
  ParamSpec* p = findLocked(lock, name);
  if (!p)
    return Error::ParameterNotFound;
  return validateAndAssign(*p, raw);
}

std::vector<ParamSpec> ParameterCollection::specs() const
{
  Lock lock{mutex_};
  return params_;
}

void ParameterCollection::copyValuesFrom(const ParameterCollection& other)
{
  if (this == &other)
    return;
  std::lock(mutex_, other.mutex_);
  Lock selfLock{mutex_, std::adopt_lock};
  Lock otherLock{other.mutex_, std::adopt_lock};
  for (ParamSpec& dst : params_) {
    const ParamSpec* src = other.findLocked(otherLock, dst.name);
    if (!src || src->type != dst.type)
      continue;
    dst.bool_value = src->bool_value;
    dst.int_value = src->int_value;
    dst.double_value = src->double_value;
    dst.string_value = src->string_value;
  }
}

std::string ParameterCollection::toYaml(const std::string& fileComment) const
{
  Lock lock{mutex_};
  std::ostringstream os;
  os << "%YAML:1.0\n---\n";
  const std::string header = yamlOneLine(fileComment);
  if (!header.empty())
    os << "# " << header << "\n";
  for (const ParamSpec& spec : params_) {
    std::string comment = spec.description.empty() ? spec.displayLabel() : spec.description;
    comment = yamlOneLine(comment);
    if (!comment.empty())
      os << "# " << comment << "\n";
    os << spec.name << ": ";
    switch (spec.type) {
    case ParamType::Bool:
      os << (spec.bool_value ? 1 : 0);
      break;
    case ParamType::Int:
      os << spec.int_value;
      break;
    case ParamType::Double:
      os << spec.double_value;
      break;
    case ParamType::Enum:
      os << yamlQuoteScalar(spec.string_value);
      break;
    }
    os << "\n";
  }
  return os.str();
}

Error ParameterCollection::loadYaml(const RpcValue& root)
{
  if (root.getType() == RpcValue::TypeInvalid)
    return Error::Ok;
  if (root.getType() != RpcValue::TypeStruct)
    return Error::InvalidValue;

  Lock lock{mutex_};
  for (auto it = root.begin(); it != root.end(); ++it) {
    ParamSpec* p = findLocked(lock, it->first);
    if (!p)
      continue;
    const RpcValue::Type t = it->second.getType();
    if (t != RpcValue::TypeBoolean && t != RpcValue::TypeInt &&
        t != RpcValue::TypeDouble && t != RpcValue::TypeString)
      continue;
    validateAndAssign(*p, rpcValueToString(it->second));
  }
  return Error::Ok;
}

Error ParameterCollection::loadYaml(const std::string& text)
{
  RpcValue root;
  const Error err = parseYaml(text, root);
  if (!err)
    return err;
  return loadYaml(root);
}

Error ParameterCollection::loadYamlFile(const std::string& path)
{
  RpcValue root;
  const Error err = miniros::loadYamlFile(path, root);
  if (!err)
    return err;
  return loadYaml(root);
}

} // namespace miniros
