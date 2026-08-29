//
// Typed parameter storage independent of HTTP / HTML.
//

#ifndef MINIROS_UTILITY_PARAMETER_COLLECTION_H
#define MINIROS_UTILITY_PARAMETER_COLLECTION_H

#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "miniros/errors.h"
#include "miniros/macros.h"

namespace miniros {
namespace XmlRpc {
class XmlRpcValue;
}

/// One choice in an enum parameter.
struct MINIROS_DECL EnumOption {
  std::string code;         ///< Stored / submitted value (e.g. "30", "2x2").
  std::string description;  ///< Optional UI label; empty → use code.
};

enum class ParamType {
  Bool,
  Int,
  Double,
  Enum,
};

/// Descriptor + current value for one parameter.
struct MINIROS_DECL ParamSpec {
  std::string name;          ///< Stable id (form field name, getters).
  std::string label;         ///< Human-readable UI text; empty → use name. i18n later.
  std::string description;   ///< Longer help (e.g. tooltip).
  ParamType type = ParamType::Bool;

  bool bool_value = false;
  int int_value = 0;
  double double_value = 0.0;
  std::string string_value;  ///< Enum code.

  bool has_min = false;
  bool has_max = false;
  int int_min = 0;
  int int_max = 0;
  double double_min = 0.0;
  double double_max = 0.0;
  std::vector<EnumOption> enum_options;

  /// Label shown in UI (falls back to name).
  const std::string& displayLabel() const
  {
    return label.empty() ? name : label;
  }

  /// Canonical string form used by HTML forms / comparison.
  std::string valueAsString() const;
};

/// Result of attempting to apply a proposed parameter set.
struct MINIROS_DECL ApplyReport {
  Error error = Error::Ok;
  /// Generic human-readable failure (e.g. "Failed to initialize camera").
  std::string message;
  /// Per-parameter messages (param name → text).
  std::map<std::string, std::string> field_errors;

  bool ok() const
  {
    return error == Error::Ok && message.empty() && field_errors.empty();
  }

  void reject(std::string msg, Error err = Error::InvalidValue)
  {
    message = std::move(msg);
    if (error == Error::Ok)
      error = err;
  }

  void rejectField(const std::string& name, std::string msg, Error err = Error::InvalidValue)
  {
    field_errors[name] = std::move(msg);
    if (error == Error::Ok)
      error = err;
  }
};

class ParameterCollection;

/// Fluent handle for a newly added parameter. Keeps an index into the collection
/// (safe under append-only registration; do not store across erasures).
class MINIROS_DECL ParamSpecRef {
public:
  ParamSpecRef& label(std::string text);
  ParamSpecRef& description(std::string text);
  ParamSpecRef& min(int value);
  ParamSpecRef& max(int value);
  ParamSpecRef& min(double value);
  ParamSpecRef& max(double value);

private:
  friend class ParameterCollection;
  ParamSpecRef(ParameterCollection* collection, size_t index);

  ParameterCollection* collection_ = nullptr;
  size_t index_ = 0;
};

/// Thread-safe collection of named typed parameters.
class MINIROS_DECL ParameterCollection {
public:
  ParameterCollection();
  ParameterCollection(const ParameterCollection& other);
  ParameterCollection& operator=(const ParameterCollection& other);

  /// Register with name + default value; chain .label() / .description() / .min() / .max().
  ParamSpecRef addBool(const std::string& name, bool value);
  ParamSpecRef addInt(const std::string& name, int value);
  ParamSpecRef addDouble(const std::string& name, double value);
  /// @param selected - enum code; empty → first option.
  ParamSpecRef addEnum(const std::string& name, std::vector<EnumOption> options,
                       const std::string& selected = {});

  bool has(const std::string& name) const;
  bool getBool(const std::string& name) const;
  int getInt(const std::string& name) const;
  double getDouble(const std::string& name) const;
  std::string getString(const std::string& name) const;

  Error setBool(const std::string& name, bool value);
  Error setInt(const std::string& name, int value);
  Error setDouble(const std::string& name, double value);
  Error setString(const std::string& name, const std::string& value);

  /// Assign from a raw form string; updates this spec in-place.
  Error assignFromString(const std::string& name, const std::string& raw);

  std::vector<ParamSpec> specs() const;

  /// Replace all values from another collection (same parameter set expected).
  void copyValuesFrom(const ParameterCollection& other);

  /// YAML text of current values. Each parameter is preceded by a comment from
  /// its description (or display label).
  /// @param fileComment - optional comment written at the top of the document.
  std::string toYaml(const std::string& fileComment = {}) const;

  /// Overlay matching keys from a YAML document. Unknown keys are ignored.
  Error loadYaml(const std::string& text);
  /// Same as loadYaml() after reading the file. Missing file → FileNotFound.
  Error loadYamlFile(const std::string& path);
  /// Overlay matching keys from a parsed YAML struct.
  Error loadYaml(const XmlRpc::XmlRpcValue& root);

private:
  friend class ParamSpecRef;

  using Lock = std::unique_lock<std::mutex>;

  /// Caller must already own mutex_ (pass the lock as proof).
  ParamSpec* findLocked(Lock& lock, const std::string& name);
  const ParamSpec* findLocked(Lock& lock, const std::string& name) const;
  static Error validateAndAssign(ParamSpec& spec, const std::string& raw);

  mutable std::mutex mutex_;
  std::vector<ParamSpec> params_;
};

} // namespace miniros

#endif // MINIROS_UTILITY_PARAMETER_COLLECTION_H
