#pragma once

#include <string>
#include <vector>

#include "DxfDocument.h"
#include "ParsedPathDoc.h"

namespace rigkit {
namespace dxf {

/** @brief Load ASCII DXF into a typed Document (dxflib). */
bool loadFile(const std::string& path, Document& out);

/** @brief Load from memory (ASCII DXF bytes). */
bool loadBuffer(const std::vector<char>& bytes, Document& out);

/**
 * @brief Convenience: loadFile then flattenToParsed for PlotDoc.
 * @details Keeps the historical ParsedPathDoc entry point.
 */
bool loadFile(const std::string& path, plot::ParsedPathDoc& out);

/** @brief Bake typed entities to path bags for the plot pipeline. */
void flattenToParsed(const Document& doc, plot::ParsedPathDoc& out, float tolMm = 0.25f);

/** @brief Reflect about horizontal centre of bounds (DXF Y-up ↔ screen). */
void flipY(Document& doc);

/** @brief Replace each partial circular arc with its complement (idempotent). */
void complementArcs(Document& doc);

/** @brief Optional post-load curve recovery — never run automatically on load. */
CurvePromotionResult promoteCurves(Document& doc, const CurvePromotionSettings& settings = {});

/** @brief Write R2000 ASCII DXF (flat geometry; no INSERT structure). */
bool save(const Document& doc, const std::string& path);

/** @brief ACI index → RGBA (0–1). */
glm::vec4 aciToColor(int aci);

} // namespace dxf
} // namespace rigkit

