#pragma once

#include <cstdint>
#include <glm/glm.hpp>
#include <string>
#include <vector>

#include "CArc.h"
#include "CEllipse.h"
#include "CPath.h"
#include "CSpline.h"
#include "ParsedPathDoc.h"

namespace rigkit {
namespace dxf {

/** @brief One DXF entity as typed POD (no dual path+metadata store). */
struct Entity {
	enum class Kind : uint8_t { Path, Arc, Ellipse, Spline, Point };

	Kind kind = Kind::Path;
	std::string layer = "0";
	glm::vec4 color{0.f, 0.f, 0.f, 1.f};

	ecs::CPath path;
	ecs::CArc arc;
	ecs::CEllipse ellipse;
	/// Degrees - major-axis angle for Ellipse / elliptical Arc (transform-equivalent).
	float rotationDegrees = 0.f;
	ecs::CSpline spline;
};

struct Layer {
	std::string name;
	glm::vec4 color{0.f, 0.f, 0.f, 1.f};
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

