#pragma once

// =============================================================================
// MtlLoader.h — Parse Wavefront .mtl material libraries
// =============================================================================

#include "engine/mesh/Material.h"

#include <string>
#include <vector>

namespace ge {

/// Loads all newmtl blocks from a .mtl file. Throws if the file cannot be read.
[[nodiscard]] std::vector<Material> loadMtlFile(const std::string& path);

} // namespace ge
