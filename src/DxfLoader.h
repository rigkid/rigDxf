#pragma once

#include <string>
#include "ParsedPathDoc.h"

namespace rigkit {
namespace dxf {

/**
 * @brief Load ASCII DXF LINE and LWPOLYLINE entities into ParsedPathDoc.
 */
bool loadFile(const std::string& path, plot::ParsedPathDoc& out);

} // namespace dxf
} // namespace rigkit
