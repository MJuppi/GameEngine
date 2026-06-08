#pragma once

#include "engine/asset/TextureData.h"

#include <string>

namespace ge {

/// Loads a texture image from disk into CPU memory.
[[nodiscard]] TextureData loadTextureFile(const std::string& path);

} // namespace ge
