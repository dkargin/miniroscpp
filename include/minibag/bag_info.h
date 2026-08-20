#pragma once

#include "minibag/macros.h"

#include <string>

namespace minibag {

class Bag;

struct BagInfoOptions
{
    bool yaml = false;
    bool freq = false;
    std::string key;
};

//! Format bag metadata in the same style as `rosbag info`.
ROSBAG_STORAGE_DECL std::string formatBagInfo(Bag const& bag, BagInfoOptions const& opts = BagInfoOptions());

} // namespace minibag
