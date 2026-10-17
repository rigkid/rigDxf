#pragma once

#include <string>
#include <vector>

#include "DxfDocument.h"
#include "ParsedPathDoc.h"

namespace rigkit {
namespace dxf {

/**
 * @brief Load ASCII DXF into a typed Document (dxflib).
 * @details 2D geometry subset - not the Autodesk DXF spec. Coverage:
 *          `docs/dxf-coverage.md`. `save` writes the same kinds this keeps.
 *          Degenerate / unresolved items land in `Document::errors`.
 */
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

/** @brief Stitch exploded LINE / open polylines that share endpoints. */
void joinConnectedPaths(Document& doc, float tolMm = 0.02f);

/** @brief Shift every entity by @p delta (then recompute bounds). */
void translate(Document& doc, const glm::vec2& delta);

/** @brief Move bbox min to the origin so page size matches the drawing. */
void originToPage(Document& doc);

/** @brief Reflect about horizontal centre of bounds (DXF Y-up  <->  screen). */
void flipY(Document& doc);

/** @brief Replace each partial circular arc with its complement (idempotent). */
void complementArcs(Document& doc);

/** @brief Optional post-load curve recovery - never run automatically on load. */
CurvePromotionResult promoteCurves(Document& doc, const CurvePromotionSettings& settings = {});

/** @brief Keep / drop text and dimensions after load (import options). */
struct ImportFilter {
	bool text = true;
	bool dimensions = true;
	bool dimensionsOwnLayer = false;
};

/** @brief Apply @p filter in place (empty layers dropped, bounds recomputed). */
void applyImportFilter(Document& doc, const ImportFilter& filter);

/**
 * @brief Rebuild a Document from plot IR so save matches flatten.
 * @details Paths / texts / images only - typed Arc/Ellipse/Spline stay flattened.
 */
Document documentFromParsed(const plot::ParsedPathDoc& parsed);

/** @brief Write every kept entity kind (same set loadFile can produce). */
bool save(const Document& doc, const std::string& path);

/** @brief ACI index to RGBA (0-1). */
glm::vec4 aciToColor(int aci);

} // namespace dxf
} // namespace rigkit
