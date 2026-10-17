#pragma once

#include <glm/glm.hpp>
#include <cstdint>
#include <string>
#include <vector>

#include "CArc.h"
#include "CEllipse.h"
#include "CImages.h"
#include "CPath.h"
#include "CSpline.h"
#include "CTexts.h"
#include "ParsedPathDoc.h"

namespace rigkit {
namespace dxf {

/** @brief One DXF entity as typed POD (no dual path+metadata store). */
struct Entity {
	enum class Kind : uint8_t { Path, Arc, Ellipse, Spline, Point, Text, Image, XLine, Ray };

	Kind kind = Kind::Path;
	std::string layer = "0";
	glm::vec4 color{0.f, 0.f, 0.f, 1.f};
	/// True when the entity came from a DIMENSION *D block (or dim fallback text).
	bool fromDimension = false;
	/// Solid hatch / SOLID / TRACE / 3DFACE - flatten as a filled path.
	bool filled = false;
	/// Stroke in mm (0 = default hairline). Polyline geometric width, else lineweight.
	float strokeWidthMm = 0.f;

	ecs::CPath path;
	ecs::CArc arc;
	ecs::CEllipse ellipse;
	/// Degrees - major-axis angle for Ellipse / elliptical Arc (transform-equivalent).
	float rotationDegrees = 0.f;
	ecs::CSpline spline;
	ecs::TextItem text;
	ecs::ImageItem image;
	glm::vec2 xlineOrigin{0.f, 0.f};
	glm::vec2 xlineDir{1.f, 0.f};
};

struct Layer {
	std::string name;
	glm::vec4 color{0.f, 0.f, 0.f, 1.f};
	bool visible = true;
	std::vector<Entity> entities;
};

/**
 * @brief Loaded DXF as layers of typed entities.
 * @details Plot interchange uses flattenToParsed to ParsedPathDoc / CPaths.
 */
struct Document {
	std::vector<Layer> layers;
	float minX = 0.f;
	float minY = 0.f;
	float contentW = 0.f;
	float contentH = 0.f;
	std::vector<std::string> errors;

	bool empty() const { return layers.empty(); }
};

struct CurvePromotionSettings {
	bool promoteBulgeArcCircles = true;
	bool promoteBulgeArcs = true;
	bool promoteFacetedCircles = false;
	bool promoteFacetedArcs = false;
	int facetedCircleMinVertices = 16;
	int facetedArcMinVertices = 8;
	float facetedCircleMaxRelativeError = 0.01f;
	float facetedCircleMaxEdgeVariance = 0.05f;
};

struct CurvePromotionResult {
	int circlesPromoted = 0;
	int arcsPromoted = 0;
	int entitiesExamined = 0;
};

} // namespace dxf
} // namespace rigkit
