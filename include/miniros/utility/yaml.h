//
// Small YAML reader for nested maps / sequences of scalars.
// No anchors, aliases, or merge keys.
//

#ifndef MINIROS_UTILITY_YAML_H
#define MINIROS_UTILITY_YAML_H

#include <string>

#include "miniros/errors.h"
#include "miniros/macros.h"
#include "miniros/xmlrpcpp/XmlRpcValue.h"

namespace miniros {

using RpcValue = XmlRpc::XmlRpcValue;

/// Parse a YAML document into RpcValue (struct, array, or scalar).
/// Understands nested maps, block and flow sequences, flow maps, comments,
/// `%YAML` / `---` headers, and `!!tags` (the tag is ignored).
MINIROS_DECL Error parseYaml(const std::string& text, RpcValue& out);

/// Read a file and parseYaml it. Missing file → FileNotFound.
MINIROS_DECL Error loadYamlFile(const std::string& path, RpcValue& out);

/// Canonical string for a scalar RpcValue (bool as "1"/"0"). Empty if not a scalar.
MINIROS_DECL std::string rpcValueToString(const RpcValue& value);

} // namespace miniros

#endif // MINIROS_UTILITY_YAML_H
