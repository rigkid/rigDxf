#include "DxfLoader.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <numeric>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "dl_attributes.h"
#include "dl_codes.h"
#include "dl_creationadapter.h"
#include "dl_dxf.h"
#include "dl_entities.h"
#include "dl_writer_ascii.h"

#include "PathFlatten.h"
#include "PrimitiveVertices.h"

namespace rigkit {
namespace dxf {
namespace {

using Cmd = ecs::CPath::Cmd;

constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 2.f * kPi;
constexpr float kDegToRad = kPi / 180.f;
constexpr float kRadToDeg = 180.f / kPi;
constexpr double kTwoPiD = 6.283185307179586476925;

/// Chord error used when a curve has to leave the typed lane (DXF write).
constexpr float kExportTolerance = 0.05f;
/// Nested INSERT ceiling — cyclic block references would otherwise never end.
constexpr int kMaxInsertDepth = 16;

// -------------------- Colour --------------------

glm::vec4 color24ToRgba(int color24) {
	if (color24 < 0) {
		return {0.f, 0.f, 0.f, 1.f};
	}
	return {static_cast<float>((color24 >> 16) & 0xFF) / 255.f,
			static_cast<float>((color24 >> 8) & 0xFF) / 255.f,
			static_cast<float>(color24 & 0xFF) / 255.f, 1.f};
}

/// Fallback hue for layers that say BYLAYER without ever declaring a colour.
glm::vec4 paletteColor(int index) {
	static const glm::vec4 kPalette[] = {
		{1.00f, 0.31f, 0.31f, 1.f}, {0.31f, 0.78f, 0.31f, 1.f}, {0.31f, 0.55f, 1.00f, 1.f},
		{1.00f, 0.78f, 0.24f, 1.f}, {0.78f, 0.31f, 1.00f, 1.f}, {0.31f, 0.86f, 0.86f, 1.f},
		{1.00f, 0.55f, 0.24f, 1.f}, {0.78f, 0.78f, 0.78f, 1.f}};
	const int count = static_cast<int>(sizeof(kPalette) / sizeof(kPalette[0]));
	return kPalette[((index % count) + count) % count];
}

glm::vec4 resolveLayerColor(const DL_Attributes& attrs, int paletteIndex) {
	if (attrs.getColor24() >= 0) {
		return color24ToRgba(attrs.getColor24());
	}
	const int aci = attrs.getColor();
	if (aci == 256 || aci == 0) {
		return paletteColor(paletteIndex);
	}
	return aciToColor(aci);
}

glm::vec4 resolveEntityColor(const DL_Attributes& attrs, const glm::vec4& layerColor) {
	if (attrs.getColor24() >= 0) {
		return color24ToRgba(attrs.getColor24());
	}
	const int aci = attrs.getColor();
	if (aci == 256) {
		return layerColor;
	}
	if (aci == 0) {
		return {0.f, 0.f, 0.f, 1.f};
	}
	return aciToColor(aci);
}

// -------------------- Geometry helpers --------------------

/// Segment count for a full turn that keeps the chord within @p tolMm.
int segmentsForTolerance(float radius, float tolMm) {
	if (radius <= 0.f) {
		return 12;
	}
	const float tol = tolMm > 0.f ? tolMm : 0.25f;
	const float ratio = std::clamp(1.f - tol / radius, -1.f, 1.f);
	const float step = 2.f * std::acos(ratio);
	if (step <= 1e-5f) {
		return 2048;
	}
	return std::clamp(static_cast<int>(std::ceil(kTwoPi / step)), 12, 2048);
}

glm::vec2 rotateAbout(const glm::vec2& p, const glm::vec2& center, float cosT, float sinT) {
	const glm::vec2 d = p - center;
	return {center.x + d.x * cosT - d.y * sinT, center.y + d.x * sinT + d.y * cosT};
}

/// CArc samples with the entity rotation folded in (CArc itself is axis aligned).
std::vector<glm::vec2> arcPolyline(const ecs::CArc& arc, float rotationDegrees, float tolMm) {
	std::vector<glm::vec2> pts =
		ecs::verticesOf(arc, segmentsForTolerance(arc.majorRadius(), tolMm));
	if (std::abs(rotationDegrees) > 1e-6f) {
		const float rad = rotationDegrees * kDegToRad;
		const float c = std::cos(rad);
		const float s = std::sin(rad);
		const glm::vec2 center{arc.cx, arc.cy};
		for (auto& p : pts) {
			p = rotateAbout(p, center, c, s);
		}
	}
	return pts;
}

ecs::CArc fullTurnOf(const ecs::CEllipse& ellipse) {
	ecs::CArc arc;
	arc.cx = ellipse.cx;
	arc.cy = ellipse.cy;
	arc.rx = ellipse.rx;
	arc.ry = ellipse.ry;
	arc.startAngleDegrees = 0.f;
	arc.endAngleDegrees = 360.f;
	return arc;
}

float pointRunLength(const std::vector<glm::vec2>& pts) {
	float sum = 0.f;
	for (std::size_t i = 1; i < pts.size(); ++i) {
		sum += glm::length(pts[i] - pts[i - 1]);
	}
	return sum;
}

std::vector<glm::vec2> splinePolyline(const ecs::CSpline& spline, float tolMm) {
	const auto& hint = spline.controlPoints.empty() ? spline.fitPoints : spline.controlPoints;
	// Sample density follows the requested chord tolerance through the length of
	// the defining point run — a NURBS never strays far from it.
	const float step = std::max(tolMm * 8.f, 0.1f);
	const int segments =
		std::clamp(static_cast<int>(std::ceil(pointRunLength(hint) / step)), 16, 4096);

	std::vector<glm::vec2> pts = ecs::verticesOf(spline, segments);
	if (!pts.empty()) {
		return pts;
	}
	// Fit-point-only or short knot vector: the defining run is the best we have.
	pts = spline.controlPoints.empty() ? spline.fitPoints : spline.controlPoints;
	if (spline.closed && pts.size() > 2) {
		pts.push_back(pts.front());
	}
	return pts;
}

bool sweepIsFullTurn(float startDegrees, float endDegrees) {
	return std::abs(endDegrees - startDegrees) >= 359.9f;
}

/// Contours in document space. Path keeps its curves; the rest are sampled.
std::vector<std::vector<glm::vec2>> contoursOfEntity(const Entity& entity, float tolMm) {
	switch (entity.kind) {
	case Entity::Kind::Path:
		return plot::flattenPathContours(entity.path, tolMm);
	case Entity::Kind::Arc: {
		auto pts = arcPolyline(entity.arc, entity.rotationDegrees, tolMm);
		if (pts.size() < 2) {
			return {};
		}
		return {std::move(pts)};
	}
	case Entity::Kind::Ellipse: {
		auto pts = arcPolyline(fullTurnOf(entity.ellipse), entity.rotationDegrees, tolMm);
		if (pts.size() < 2) {
			return {};
		}
		return {std::move(pts)};
	}
	case Entity::Kind::Spline: {
		auto pts = splinePolyline(entity.spline, tolMm);
		if (pts.size() < 2) {
			return {};
		}
		return {std::move(pts)};
	}
	case Entity::Kind::Point:
		return {};
	}
	return {};
}

/// Single point of a Point entity; false when the entity carries no anchor.
bool pointOfEntity(const Entity& entity, glm::vec2& out) {
	if (entity.kind != Entity::Kind::Point || entity.path.commands.empty()) {
		return false;
	}
	out = entity.path.commands.front().p;
	return true;
}

struct Bounds {
	float minX = 0.f;
	float minY = 0.f;
	float maxX = 0.f;
	float maxY = 0.f;
	bool empty = true;

	void add(const glm::vec2& p) {
		if (empty) {
			minX = maxX = p.x;
			minY = maxY = p.y;
			empty = false;
			return;
		}
		minX = std::min(minX, p.x);
		minY = std::min(minY, p.y);
		maxX = std::max(maxX, p.x);
		maxY = std::max(maxY, p.y);
	}
};

/// AutoCAD's construction-point layer never belongs in the artwork extents.
bool skipLayerForBounds(const std::string& name) {
	std::string lower = name;
	std::transform(lower.begin(), lower.end(), lower.begin(),
				   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
	return lower == "defpoints";
}

void recomputeBounds(Document& doc) {
	Bounds b;
	for (const auto& layer : doc.layers) {
		if (skipLayerForBounds(layer.name)) {
			continue;
		}
		for (const auto& entity : layer.entities) {
			glm::vec2 point{0.f, 0.f};
			if (pointOfEntity(entity, point)) {
				b.add(point);
				continue;
			}
			for (const auto& contour : contoursOfEntity(entity, 0.25f)) {
				for (const auto& p : contour) {
					b.add(p);
				}
			}
		}
	}
	doc.minX = b.empty ? 0.f : b.minX;
	doc.minY = b.empty ? 0.f : b.minY;
	doc.contentW = b.empty ? 0.f : (b.maxX - b.minX);
	doc.contentH = b.empty ? 0.f : (b.maxY - b.minY);
}

// -------------------- Entity construction --------------------

Entity makePolylineEntity(const std::vector<glm::vec2>& pts) {
	Entity e;
	e.kind = Entity::Kind::Path;
	if (pts.size() < 2) {
		return e;
	}
	std::size_t last = pts.size();
	bool close = false;
	if (pts.size() > 2 && glm::distance(pts.front(), pts.back()) < 1e-6f) {
		--last;
		close = true;
	}
	e.path.commands.push_back({Cmd::MoveTo, pts[0], {}, {}});
	for (std::size_t i = 1; i < last; ++i) {
		e.path.commands.push_back({Cmd::LineTo, pts[i], {}, {}});
	}
	if (close) {
		e.path.commands.push_back({Cmd::Close, {}, {}, {}});
	}
	return e;
}

Entity makeLineEntity(const glm::vec2& a, const glm::vec2& b) {
	Entity e;
	e.kind = Entity::Kind::Path;
	e.path.commands.push_back({Cmd::MoveTo, a, {}, {}});
	e.path.commands.push_back({Cmd::LineTo, b, {}, {}});
	return e;
}

Entity makeCircleEntity(const glm::vec2& center, float radius) {
	Entity e;
	e.kind = Entity::Kind::Ellipse;
	e.ellipse.cx = center.x;
	e.ellipse.cy = center.y;
	e.ellipse.rx = radius;
	e.ellipse.ry = radius;
	return e;
}

Entity makeArcEntity(const glm::vec2& center, float radius, float startDegrees,
					 float endDegrees) {
	Entity e;
	e.kind = Entity::Kind::Arc;
	e.arc.cx = center.x;
	e.arc.cy = center.y;
	e.arc.setRadius(radius);
	e.arc.startAngleDegrees = startDegrees;
	e.arc.endAngleDegrees = endDegrees;
	return e;
}

/**
 * @brief DXF ELLIPSE data as a typed entity.
 * @details A full turn is an Ellipse; anything shorter is an elliptical Arc.
 * Both carry the major-axis angle in `rotationDegrees`.
 */
Entity makeEllipseEntity(const glm::vec2& center, const glm::vec2& majorAxis, float ratio,
						 double startParam, double endParam) {
	Entity e;
	const float majorLen = glm::length(majorAxis);
	const float minorLen = majorLen * ratio;
	e.rotationDegrees = std::atan2(majorAxis.y, majorAxis.x) * kRadToDeg;

	// DXF ellipse parameters run counter-clockwise from angle1 to angle2.
	double start = startParam;
	double end = endParam;
	while (end < start) {
		end += kTwoPiD;
	}
	if (end - start >= kTwoPiD - 1e-6) {
		e.kind = Entity::Kind::Ellipse;
		e.ellipse.cx = center.x;
		e.ellipse.cy = center.y;
		e.ellipse.rx = majorLen;
		e.ellipse.ry = minorLen;
		return e;
	}
	e.kind = Entity::Kind::Arc;
	e.arc.cx = center.x;
	e.arc.cy = center.y;
	e.arc.rx = majorLen;
	e.arc.ry = minorLen;
	e.arc.startAngleDegrees = static_cast<float>(start) * kRadToDeg;
	e.arc.endAngleDegrees = static_cast<float>(end) * kRadToDeg;
	return e;
}

struct BulgeVertex {
	glm::vec2 pos{0.f, 0.f};
	double bulge = 0.0;
};

/// Put the start angle in [0, 360) without touching the sweep.
void normalizeArcAngles(ecs::CArc& arc) {
	const float sweep = arc.endAngleDegrees - arc.startAngleDegrees;
	float start = std::fmod(arc.startAngleDegrees, 360.f);
	if (start < 0.f) {
		start += 360.f;
	}
	arc.startAngleDegrees = start;
	arc.endAngleDegrees = start + sweep;
}

/**
 * @brief Circular arc described by a polyline bulge (tangent of a quarter sweep).
 * @details The bulge is signed: it carries the whole included angle, so the
 * centre offset along the chord normal and the sweep both come straight out of
 * it — no side-of-the-chord guesswork, and half-turns fall out at h = 0.
 * @return Empty when the bulge is flat or the chord has no length.
 */
std::optional<ecs::CArc> arcFromBulge(const glm::vec2& p1, const glm::vec2& p2, double bulge) {
	if (std::abs(bulge) < 1e-10) {
		return std::nullopt;
	}
	const double chordLen = static_cast<double>(glm::distance(p1, p2));
	const double theta = 4.0 * std::atan(bulge);
	const double halfSin = std::sin(theta * 0.5);
	const double halfTan = std::tan(theta * 0.5);
	if (chordLen < 1e-10 || std::abs(halfSin) < 1e-12) {
		return std::nullopt;
	}

	const double radius = std::abs(chordLen / (2.0 * halfSin));
	const glm::vec2 mid = (p1 + p2) * 0.5f;
	const glm::vec2 chord = p2 - p1;
	const glm::vec2 normal = glm::normalize(glm::vec2(-chord.y, chord.x));
	const float offset = std::abs(halfTan) < 1e-9
							 ? 0.f
							 : static_cast<float>((chordLen * 0.5) / halfTan);
	const glm::vec2 center = mid + normal * offset;

	ecs::CArc arc;
	arc.cx = center.x;
	arc.cy = center.y;
	arc.setRadius(static_cast<float>(radius));
	arc.startAngleDegrees = std::atan2(p1.y - center.y, p1.x - center.x) * kRadToDeg;
	arc.endAngleDegrees = arc.startAngleDegrees + static_cast<float>(theta) * kRadToDeg;
	normalizeArcAngles(arc);
	return arc;
}

/**
 * @brief Split a bulge polyline into straight Path runs and standalone Arc entities.
 * @details Arcs stay exact instead of riding along as chords on a CPath.
 */
void appendBulgePolyline(const std::vector<BulgeVertex>& verts, bool closed,
						 std::vector<Entity>& out) {
	const std::size_t count = verts.size();
	if (count < 2) {
		return;
	}
	const std::size_t segments = closed ? count : count - 1;

	std::vector<glm::vec2> run;
	auto flushRun = [&]() {
		if (run.size() >= 2) {
			out.push_back(makePolylineEntity(run));
		}
		run.clear();
	};

	for (std::size_t i = 0; i < segments; ++i) {
		const glm::vec2 a = verts[i].pos;
		const glm::vec2 b = verts[(i + 1) % count].pos;
		if (auto arc = arcFromBulge(a, b, verts[i].bulge)) {
			flushRun();
			Entity e;
			e.kind = Entity::Kind::Arc;
			e.arc = *arc;
			out.push_back(std::move(e));
			continue;
		}
		if (run.empty()) {
			run.push_back(a);
		}
		run.push_back(b);
	}
	flushRun();
}

/// HATCH boundary edge as geometry. Pattern fill is not imported.
bool appendHatchEdge(const DL_HatchEdgeData& edge, std::vector<Entity>& out) {
	switch (edge.type) {
	case 0: {
		if (edge.vertices.size() < 2) {
			return false;
		}
		std::vector<BulgeVertex> verts;
		verts.reserve(edge.vertices.size());
		for (const auto& v : edge.vertices) {
			if (v.size() < 2) {
				continue;
			}
			verts.push_back({glm::vec2(static_cast<float>(v[0]), static_cast<float>(v[1])),
							 v.size() >= 3 ? v[2] : 0.0});
		}
		if (verts.size() < 2) {
			return false;
		}
		appendBulgePolyline(verts, verts.size() >= 3, out);
		return true;
	}
	case 1: {
		const glm::vec2 a(static_cast<float>(edge.x1), static_cast<float>(edge.y1));
		const glm::vec2 b(static_cast<float>(edge.x2), static_cast<float>(edge.y2));
		if (glm::distance(a, b) < 1e-9f) {
			return false;
		}
		out.push_back(makeLineEntity(a, b));
		return true;
	}
	case 2: {
		const glm::vec2 center(static_cast<float>(edge.cx), static_cast<float>(edge.cy));
		const float radius = static_cast<float>(edge.radius);
		if (radius <= 0.f) {
			return false;
		}
		float start = static_cast<float>(edge.angle1) * kRadToDeg;
		float end = static_cast<float>(edge.angle2) * kRadToDeg;
		if (edge.ccw) {
			while (end < start) {
				end += 360.f;
			}
		} else {
			while (end > start) {
				end -= 360.f;
			}
		}
		if (sweepIsFullTurn(start, end)) {
			out.push_back(makeCircleEntity(center, radius));
		} else {
			out.push_back(makeArcEntity(center, radius, start, end));
		}
		return true;
	}
	case 3: {
		const glm::vec2 center(static_cast<float>(edge.cx), static_cast<float>(edge.cy));
		const glm::vec2 major(static_cast<float>(edge.mx), static_cast<float>(edge.my));
		if (glm::length(major) <= 0.f) {
			return false;
		}
		out.push_back(makeEllipseEntity(center, major, static_cast<float>(edge.ratio), edge.angle1,
										edge.angle2));
		return true;
	}
	case 4: {
		if (edge.controlPoints.empty() && edge.fitPoints.empty()) {
			return false;
		}
		Entity e;
		e.kind = Entity::Kind::Spline;
		e.spline.degree = static_cast<int>(edge.degree);
		e.spline.closed = edge.periodic;
		for (double k : edge.knots) {
			e.spline.knots.push_back(static_cast<float>(k));
		}
		for (const auto& cp : edge.controlPoints) {
			if (cp.size() >= 2) {
				e.spline.controlPoints.emplace_back(static_cast<float>(cp[0]),
													static_cast<float>(cp[1]));
			}
		}
		for (const auto& fp : edge.fitPoints) {
			if (fp.size() >= 2) {
				e.spline.fitPoints.emplace_back(static_cast<float>(fp[0]),
												static_cast<float>(fp[1]));
			}
		}
		if (e.spline.controlPoints.empty() && e.spline.fitPoints.empty()) {
			return false;
		}
		out.push_back(std::move(e));
		return true;
	}
	default:
		return false;
	}
}

// -------------------- INSERT expansion --------------------

struct InsertXform {
	glm::vec2 base{0.f, 0.f};
	glm::vec2 offset{0.f, 0.f};
	float sx = 1.f;
	float sy = 1.f;
	float angleDegrees = 0.f;
};

glm::vec2 xformVector(const InsertXform& t, glm::vec2 v) {
	v.x *= t.sx;
	v.y *= t.sy;
	if (std::abs(t.angleDegrees) > 1e-9f) {
		const float rad = t.angleDegrees * kDegToRad;
		const float c = std::cos(rad);
		const float s = std::sin(rad);
		v = {c * v.x - s * v.y, s * v.x + c * v.y};
	}
	return v;
}

glm::vec2 xformPoint(const InsertXform& t, const glm::vec2& p) {
	return xformVector(t, p - t.base) + t.offset;
}

/**
 * @brief Move an axis-aligned-plus-rotation arc through an INSERT transform.
 * @details `P(u) = c + major·cos u + minor·sin u` survives any affine map, so the
 * parameter is untouched and only the principal axes have to be recovered (Rytz).
 * That keeps non-uniform block scale exact instead of demoting the arc to chords.
 */
void xformArcAxes(const InsertXform& t, float& cx, float& cy, float& rx, float& ry,
				  float& rotationDegrees, float& startDegrees, float& endDegrees) {
	const float rot = rotationDegrees * kDegToRad;
	const float cr = std::cos(rot);
	const float sr = std::sin(rot);
	const glm::vec2 major = xformVector(t, {rx * cr, rx * sr});
	const glm::vec2 minor = xformVector(t, {-ry * sr, ry * cr});
	const glm::vec2 center = xformPoint(t, {cx, cy});

	cx = center.x;
	cy = center.y;

	const float mm = glm::dot(major, major);
	const float nn = glm::dot(minor, minor);
	const float mn = glm::dot(major, minor);
	float uStar = 0.f;
	if (std::abs(mn) > 1e-12f || std::abs(mm - nn) > 1e-12f) {
		uStar = 0.5f * std::atan2(2.f * mn, mm - nn);
	}
	const float cu = std::cos(uStar);
	const float su = std::sin(uStar);
	const glm::vec2 axisA = major * cu + minor * su;
	const glm::vec2 axisB = minor * cu - major * su;

	rx = glm::length(axisA);
	ry = glm::length(axisB);
	if (rx <= 1e-9f) {
		return;
	}
	rotationDegrees = std::atan2(axisA.y, axisA.x) * kRadToDeg;

	// A mirroring transform flips the parameter direction with the handedness.
	const float handed = (axisA.x * axisB.y - axisA.y * axisB.x) < 0.f ? -1.f : 1.f;
	const float shift = uStar * kRadToDeg;
	startDegrees = handed * (startDegrees - shift);
	endDegrees = handed * (endDegrees - shift);
}

Entity xformEntity(const Entity& src, const InsertXform& t) {
	Entity e = src;
	switch (e.kind) {
	case Entity::Kind::Path:
	case Entity::Kind::Point:
		for (auto& cmd : e.path.commands) {
			if (cmd.type == Cmd::CubicTo) {
				cmd.c1 = xformPoint(t, cmd.c1);
				cmd.c2 = xformPoint(t, cmd.c2);
			} else if (cmd.type == Cmd::QuadTo) {
				cmd.c1 = xformPoint(t, cmd.c1);
			}
			if (cmd.type != Cmd::Close) {
				cmd.p = xformPoint(t, cmd.p);
			}
		}
		break;
	case Entity::Kind::Arc:
		xformArcAxes(t, e.arc.cx, e.arc.cy, e.arc.rx, e.arc.ry, e.rotationDegrees,
					 e.arc.startAngleDegrees, e.arc.endAngleDegrees);
		break;
	case Entity::Kind::Ellipse: {
		float start = 0.f;
		float end = 360.f;
		xformArcAxes(t, e.ellipse.cx, e.ellipse.cy, e.ellipse.rx, e.ellipse.ry, e.rotationDegrees,
					 start, end);
		break;
	}
	case Entity::Kind::Spline:
		for (auto& p : e.spline.controlPoints) {
			p = xformPoint(t, p);
		}
		for (auto& p : e.spline.fitPoints) {
			p = xformPoint(t, p);
		}
		break;
	}
	return e;
}

// -------------------- dxflib adapter --------------------

struct Block {
	glm::vec2 base{0.f, 0.f};
	std::vector<Entity> entities;
};

/** @brief dxflib callbacks to typed POD entities, layer by layer. */
class RigDxfAdapter : public DL_CreationAdapter {
  public:
	void addLayer(const DL_LayerData& data) override {
		m_layerColors[data.name] =
			resolveLayerColor(getAttributes(), static_cast<int>(m_layers.size()));
		ensureLayer(data.name);
	}

	void addPoint(const DL_PointData& d) override {
		Entity e;
		e.kind = Entity::Kind::Point;
		e.path.commands.push_back(
			{Cmd::MoveTo, {static_cast<float>(d.x), static_cast<float>(d.y)}, {}, {}});
		pushEntity(std::move(e));
	}

	void addLine(const DL_LineData& d) override {
		pushEntity(makeLineEntity({static_cast<float>(d.x1), static_cast<float>(d.y1)},
								  {static_cast<float>(d.x2), static_cast<float>(d.y2)}));
	}

	void addArc(const DL_ArcData& d) override {
		const float start = static_cast<float>(d.angle1);
		float end = static_cast<float>(d.angle2);
		// DXF arcs always run counter-clockwise from angle1 to angle2.
		while (end < start) {
			end += 360.f;
		}
		pushEntity(makeArcEntity({static_cast<float>(d.cx), static_cast<float>(d.cy)},
								 static_cast<float>(d.radius), start, end));
	}

	void addCircle(const DL_CircleData& d) override {
		pushEntity(makeCircleEntity({static_cast<float>(d.cx), static_cast<float>(d.cy)},
									static_cast<float>(d.radius)));
	}

	void addEllipse(const DL_EllipseData& d) override {
		const glm::vec2 major(static_cast<float>(d.mx), static_cast<float>(d.my));
		if (glm::length(major) <= 0.f) {
			noteSkipped("ELLIPSE_DEGENERATE");
			return;
		}
		pushEntity(makeEllipseEntity({static_cast<float>(d.cx), static_cast<float>(d.cy)}, major,
									 static_cast<float>(d.ratio), d.angle1, d.angle2));
	}

	void addPolyline(const DL_PolylineData& d) override {
		m_polylineClosed = (d.flags & 1) != 0;
		m_pendingVertices.clear();
	}

	void addVertex(const DL_VertexData& d) override {
		m_pendingVertices.push_back(
			{glm::vec2(static_cast<float>(d.x), static_cast<float>(d.y)), d.bulge});
	}

	void addSpline(const DL_SplineData& d) override {
		flushSpline();
		m_spline = ecs::CSpline{};
		m_spline.degree = static_cast<int>(d.degree);
		m_spline.closed = (d.flags & 1) != 0;
		m_splinePending = true;
	}

	void addKnot(const DL_KnotData& d) override {
		m_spline.knots.push_back(static_cast<float>(d.k));
	}

	void addControlPoint(const DL_ControlPointData& d) override {
		m_spline.controlPoints.emplace_back(static_cast<float>(d.x), static_cast<float>(d.y));
		m_spline.weights.push_back(static_cast<float>(d.w));
	}

	void addFitPoint(const DL_FitPointData& d) override {
		m_spline.fitPoints.emplace_back(static_cast<float>(d.x), static_cast<float>(d.y));
	}

	void addTrace(const DL_TraceData& d) override {
		std::vector<glm::vec2> pts;
		pts.reserve(5);
		for (int i = 0; i < 4; ++i) {
			const glm::vec2 p(static_cast<float>(d.x[i]), static_cast<float>(d.y[i]));
			if (pts.empty() || glm::distance(p, pts.back()) > 1e-5f) {
				pts.push_back(p);
			}
		}
		if (pts.size() >= 3 && glm::distance(pts.front(), pts.back()) > 1e-5f) {
			pts.push_back(pts.front());
		}
		if (pts.size() < 2) {
			return;
		}
		pushEntity(makePolylineEntity(pts));
	}

	void addSolid(const DL_SolidData& d) override { addTrace(d); }

	void add3dFace(const DL_3dFaceData& d) override { addTrace(d); }

	void addHatch(const DL_HatchData&) override {
		m_hatchActive = true;
		m_hatchEntities.clear();
	}

	void addHatchLoop(const DL_HatchLoopData&) override {}

	void addHatchEdge(const DL_HatchEdgeData& d) override {
		if (!appendHatchEdge(d, m_hatchEntities)) {
			noteSkipped("HATCH_EDGE");
		}
	}

	void addBlock(const DL_BlockData& data) override {
		if (data.name.empty() || data.name[0] == '*') {
			m_collectingBlock = false;
			m_activeBlock.clear();
			return;
		}
		m_activeBlock = data.name;
		m_blocks[m_activeBlock].base =
			glm::vec2(static_cast<float>(data.bpx), static_cast<float>(data.bpy));
		m_collectingBlock = true;
	}

	void endBlock() override {
		m_collectingBlock = false;
		m_activeBlock.clear();
	}

	void addInsert(const DL_InsertData& d) override { expandInsert(d, 0); }

	void endEntity() override {
		if (m_hatchActive) {
			m_hatchActive = false;
			for (auto& e : m_hatchEntities) {
				pushEntity(std::move(e));
			}
			m_hatchEntities.clear();
			return;
		}
		if (!m_pendingVertices.empty()) {
			flushPolyline();
			return;
		}
		flushSpline();
	}

	void endSequence() override { flushSpline(); }

	void addXLine(const DL_XLineData&) override { noteSkipped("XLINE"); }
	void addRay(const DL_RayData&) override { noteSkipped("RAY"); }
	void addText(const DL_TextData&) override { noteSkipped("TEXT"); }
	void addMText(const DL_MTextData&) override { noteSkipped("MTEXT"); }
	void addArcAlignedText(const DL_ArcAlignedTextData&) override { noteSkipped("ARCALIGNEDTEXT"); }
	void addAttribute(const DL_AttributeData&) override { noteSkipped("ATTRIB"); }
	void addDimAlign(const DL_DimensionData&, const DL_DimAlignedData&) override {
		noteSkipped("DIMENSION");
	}
	void addDimLinear(const DL_DimensionData&, const DL_DimLinearData&) override {
		noteSkipped("DIMENSION");
	}
	void addDimRadial(const DL_DimensionData&, const DL_DimRadialData&) override {
		noteSkipped("DIMENSION");
	}
	void addDimDiametric(const DL_DimensionData&, const DL_DimDiametricData&) override {
		noteSkipped("DIMENSION");
	}
	void addDimAngular(const DL_DimensionData&, const DL_DimAngular2LData&) override {
		noteSkipped("DIMENSION");
	}
	void addDimAngular3P(const DL_DimensionData&, const DL_DimAngular3PData&) override {
		noteSkipped("DIMENSION");
	}
	void addDimOrdinate(const DL_DimensionData&, const DL_DimOrdinateData&) override {
		noteSkipped("DIMENSION");
	}
	void addLeader(const DL_LeaderData&) override { noteSkipped("LEADER"); }
	void addLeaderVertex(const DL_LeaderVertexData&) override {}
	void addImage(const DL_ImageData&) override { noteSkipped("IMAGE"); }
	void linkImage(const DL_ImageDefData&) override {}

	/** @brief Finish deferred work and hand over the layers. */
	std::vector<Layer> takeLayers() {
		flushSpline();
		flushPendingInserts();
		return std::move(m_layers);
	}

	/** @brief One line naming what the file carried but this loader dropped. */
	std::string summary() const {
		std::ostringstream ss;
		bool any = false;
		if (m_deferredInsertResolved > 0) {
			ss << "deferred INSERT resolved=" << m_deferredInsertResolved << " ";
			any = true;
		}
		if (m_deferredInsertFailed > 0) {
			ss << "INSERT unresolved=" << m_deferredInsertFailed << " ";
			any = true;
		}
		for (const auto& [key, count] : m_skipped) {
			if (count <= 0) {
				continue;
			}
			ss << key << "=" << count << " ";
			any = true;
		}
		return any ? ss.str() : std::string();
	}

  private:
	std::vector<Layer> m_layers;
	std::map<std::string, std::size_t> m_layerIndex;
	std::map<std::string, glm::vec4> m_layerColors;
	std::map<std::string, Block> m_blocks;

	bool m_collectingBlock = false;
	std::string m_activeBlock;

	std::vector<BulgeVertex> m_pendingVertices;
	bool m_polylineClosed = false;

	ecs::CSpline m_spline;
	bool m_splinePending = false;

	bool m_hatchActive = false;
	std::vector<Entity> m_hatchEntities;

	std::vector<DL_InsertData> m_pendingInserts;
	std::map<std::string, int> m_skipped;
	int m_deferredInsertResolved = 0;
	int m_deferredInsertFailed = 0;

	void noteSkipped(const char* key) { ++m_skipped[key]; }

	Layer& ensureLayer(const std::string& name) {
		const auto it = m_layerIndex.find(name);
		if (it != m_layerIndex.end()) {
			return m_layers[it->second];
		}
		m_layerIndex[name] = m_layers.size();
		Layer layer;
		layer.name = name;
		const auto colorIt = m_layerColors.find(name);
		layer.color = (colorIt != m_layerColors.end())
						  ? colorIt->second
						  : paletteColor(static_cast<int>(m_layers.size()));
		m_layers.push_back(std::move(layer));
		return m_layers.back();
	}

	glm::vec4 layerColorFor(const std::string& name) const {
		const auto it = m_layerColors.find(name);
		if (it != m_layerColors.end()) {
			return it->second;
		}
		const auto idxIt = m_layerIndex.find(name);
		return paletteColor(idxIt != m_layerIndex.end() ? static_cast<int>(idxIt->second) : 0);
	}

	void pushEntity(Entity&& e) {
		std::string layerName = getAttributes().getLayer();
		if (layerName.empty()) {
			layerName = "0";
		}
		e.layer = layerName;
		e.color = resolveEntityColor(getAttributes(), layerColorFor(layerName));
		pushToTarget(std::move(e));
	}

	void pushToTarget(Entity&& e) {
		if (m_collectingBlock && !m_activeBlock.empty()) {
			m_blocks[m_activeBlock].entities.push_back(std::move(e));
			return;
		}
		ensureLayer(e.layer).entities.push_back(std::move(e));
	}

	void flushPolyline() {
		std::vector<Entity> pieces;
		appendBulgePolyline(m_pendingVertices, m_polylineClosed, pieces);
		for (auto& e : pieces) {
			pushEntity(std::move(e));
		}
		m_pendingVertices.clear();
		m_polylineClosed = false;
	}

	void flushSpline() {
		if (!m_splinePending) {
			return;
		}
		m_splinePending = false;
		if (m_spline.controlPoints.empty() && m_spline.fitPoints.empty()) {
			m_spline = ecs::CSpline{};
			return;
		}
		// Empty weights mean "all ones" — do not carry a redundant array.
		const bool rational =
			std::any_of(m_spline.weights.begin(), m_spline.weights.end(),
						[](float w) { return std::abs(w - 1.f) > 1e-6f; });
		if (!rational) {
			m_spline.weights.clear();
		}

		Entity e;
		e.kind = Entity::Kind::Spline;
		e.spline = std::move(m_spline);
		m_spline = ecs::CSpline{};
		pushEntity(std::move(e));
	}

	void expandInsert(const DL_InsertData& d, int depth) {
		if (d.name.empty() || depth > kMaxInsertDepth) {
			return;
		}
		const auto it = m_blocks.find(d.name);
		if (it == m_blocks.end()) {
			// Blocks may be defined after the INSERT that uses them.
			m_pendingInserts.push_back(d);
			return;
		}

		const Block& block = it->second;
		const int cols = std::max(1, d.cols);
		const int rows = std::max(1, d.rows);
		for (int row = 0; row < rows; ++row) {
			for (int col = 0; col < cols; ++col) {
				InsertXform t;
				t.base = block.base;
				t.offset = {static_cast<float>(d.ipx + static_cast<double>(col) * d.colSp),
							static_cast<float>(d.ipy + static_cast<double>(row) * d.rowSp)};
				t.sx = static_cast<float>(d.sx);
				t.sy = static_cast<float>(d.sy);
				t.angleDegrees = static_cast<float>(d.angle);

				for (const auto& src : block.entities) {
					Entity expanded = xformEntity(src, t);
					if (expanded.layer.empty()) {
						expanded.layer = "0";
					}
					pushToTarget(std::move(expanded));
				}
			}
		}
	}

	void flushPendingInserts() {
		int passes = 0;
		bool progress = true;
		while (progress && passes < 32 && !m_pendingInserts.empty()) {
			progress = false;
			++passes;
			std::vector<DL_InsertData> queue;
			queue.swap(m_pendingInserts);
			for (auto& pending : queue) {
				if (m_blocks.count(pending.name) != 0) {
					expandInsert(pending, 0);
					++m_deferredInsertResolved;
					progress = true;
				} else {
					m_pendingInserts.push_back(std::move(pending));
				}
			}
		}
		m_deferredInsertFailed += static_cast<int>(m_pendingInserts.size());
		m_pendingInserts.clear();
	}
};

void buildDocument(RigDxfAdapter& adapter, Document& out) {
	out.layers = adapter.takeLayers();
	if (const std::string skipped = adapter.summary(); !skipped.empty()) {
		out.errors.push_back("Skipped/unresolved: " + skipped);
	}
	out.layers.erase(std::remove_if(out.layers.begin(), out.layers.end(),
									[](const Layer& l) { return l.entities.empty(); }),
					 out.layers.end());
	recomputeBounds(out);
}

// -------------------- Typed entity to CPath --------------------

ecs::CPath pathOfEntity(const Entity& entity, float tolMm) {
	if (entity.kind == Entity::Kind::Path) {
		return entity.path;
	}
	if (entity.kind == Entity::Kind::Point) {
		return {};
	}

	std::vector<glm::vec2> pts;
	bool close = false;
	if (entity.kind == Entity::Kind::Arc) {
		pts = arcPolyline(entity.arc, entity.rotationDegrees, tolMm);
		close = entity.arc.pie ||
				sweepIsFullTurn(entity.arc.startAngleDegrees, entity.arc.endAngleDegrees);
	} else if (entity.kind == Entity::Kind::Ellipse) {
		pts = arcPolyline(fullTurnOf(entity.ellipse), entity.rotationDegrees, tolMm);
		close = true;
	} else {
		pts = splinePolyline(entity.spline, tolMm);
		close = entity.spline.closed;
	}
	if (pts.size() < 2) {
		return {};
	}

	ecs::CPath path;
	path.commands.reserve(pts.size() + 1);
	path.commands.push_back({Cmd::MoveTo, pts.front(), {}, {}});
	std::size_t last = pts.size();
	if (close && glm::distance(pts.front(), pts.back()) < 1e-6f) {
		--last;
	}
	for (std::size_t i = 1; i < last; ++i) {
		path.commands.push_back({Cmd::LineTo, pts[i], {}, {}});
	}
	if (close) {
		path.commands.push_back({Cmd::Close, {}, {}, {}});
	}
	return path;
}

// -------------------- Curve promotion helpers --------------------

/// Anchors of a single-contour straight polyline; false when curves are present.
bool polylineAnchors(const ecs::CPath& path, std::vector<glm::vec2>& out, bool& closed) {
	out.clear();
	closed = false;
	int moveCount = 0;
	for (const auto& cmd : path.commands) {
		switch (cmd.type) {
		case Cmd::MoveTo:
			if (++moveCount > 1) {
				return false;
			}
			out.push_back(cmd.p);
			break;
		case Cmd::LineTo:
			out.push_back(cmd.p);
			break;
		case Cmd::Close:
			closed = true;
			break;
		default:
			return false;
		}
	}
	if (out.size() >= 2 && glm::distance(out.front(), out.back()) < 1e-5f) {
		out.pop_back();
		closed = true;
	}
	return out.size() >= 2;
}

bool fitCircle(const std::vector<glm::vec2>& pts, float maxRelativeError, glm::vec2& center,
			   float& radius) {
	glm::vec2 mean{0.f, 0.f};
	for (const auto& p : pts) {
		mean += p;
	}
	mean /= static_cast<float>(pts.size());

	float avgR = 0.f;
	for (const auto& p : pts) {
		avgR += glm::distance(p, mean);
	}
	avgR /= static_cast<float>(pts.size());
	if (avgR <= 1e-4f) {
		return false;
	}

	float maxErr = 0.f;
	for (const auto& p : pts) {
		maxErr = std::max(maxErr, std::abs(glm::distance(p, mean) - avgR));
	}
	if (maxErr / avgR > maxRelativeError && maxErr > 0.1f) {
		return false;
	}

	center = mean;
	radius = avgR;
	return true;
}

bool edgesAreEven(const std::vector<glm::vec2>& pts, float maxVariance) {
	std::vector<float> lengths;
	lengths.reserve(pts.size());
	for (std::size_t i = 0; i < pts.size(); ++i) {
		lengths.push_back(glm::distance(pts[i], pts[(i + 1) % pts.size()]));
	}
	const float mean =
		std::accumulate(lengths.begin(), lengths.end(), 0.f) / static_cast<float>(lengths.size());
	if (mean <= 1e-4f) {
		return false;
	}
	for (float len : lengths) {
		if (std::abs(len - mean) / mean > maxVariance) {
			return false;
		}
	}
	return true;
}

// -------------------- DXF write --------------------

void writeArcOrEllipse(DL_Dxf& dxf, DL_WriterA& dw, const DL_Attributes& attrib, float cx, float cy,
					   float rx, float ry, float rotationDegrees, float startDegrees,
					   float endDegrees) {
	if (rx <= 0.f || ry <= 0.f) {
		return;
	}
	// DXF only stores counter-clockwise sweeps; a clockwise arc is the same
	// geometry read the other way round.
	if (endDegrees < startDegrees) {
		std::swap(startDegrees, endDegrees);
	}
	if (endDegrees - startDegrees > 360.f) {
		endDegrees = startDegrees + 360.f;
	}

	if (std::abs(rx - ry) <= 1e-4f * std::max(rx, ry) && std::abs(rotationDegrees) < 1e-4f) {
		dxf.writeArc(dw, DL_ArcData(cx, cy, 0.0, rx, startDegrees, endDegrees), attrib);
		return;
	}

	float majorLen = rx;
	float minorLen = ry;
	float majorAngle = rotationDegrees;
	float startParam = startDegrees;
	float endParam = endDegrees;
	if (minorLen > majorLen) {
		// DXF wants ratio = minor / major <= 1, so step to the other axis.
		std::swap(majorLen, minorLen);
		majorAngle += 90.f;
		startParam -= 90.f;
		endParam -= 90.f;
	}
	const float rad = majorAngle * kDegToRad;
	dxf.writeEllipse(dw,
					 DL_EllipseData(cx, cy, 0.0, majorLen * std::cos(rad), majorLen * std::sin(rad),
									0.0, minorLen / majorLen, startParam * kDegToRad,
									endParam * kDegToRad),
					 attrib);
}

bool writeSplineEntity(DL_Dxf& dxf, DL_WriterA& dw, const ecs::CSpline& spline,
					   const DL_Attributes& attrib) {
	const int degree = spline.degree;
	const int nControl = static_cast<int>(spline.controlPoints.size());
	const int nKnots = static_cast<int>(spline.knots.size());
	const int nFit = static_cast<int>(spline.fitPoints.size());
	if (degree < 1 || nControl < degree + 1 || nKnots != nControl + degree + 1) {
		return false;
	}

	dxf.writeSpline(dw, DL_SplineData(degree, nKnots, nControl, nFit, spline.closed ? 1 : 0),
					attrib);
	for (float k : spline.knots) {
		dxf.writeKnot(dw, DL_KnotData(k));
	}
	for (std::size_t i = 0; i < spline.controlPoints.size(); ++i) {
		const double w = i < spline.weights.size() ? spline.weights[i] : 1.0;
		dxf.writeControlPoint(
			dw, DL_ControlPointData(spline.controlPoints[i].x, spline.controlPoints[i].y, 0.0, w));
	}
	for (const auto& p : spline.fitPoints) {
		dxf.writeFitPoint(dw, DL_FitPointData(p.x, p.y, 0.0));
	}
	return true;
}

void writeContours(DL_Dxf& dxf, DL_WriterA& dw, const Entity& entity,
				   const DL_Attributes& attrib) {
	auto contours = contoursOfEntity(entity, kExportTolerance);
	for (auto& contour : contours) {
		bool closed = false;
		if (contour.size() > 2 && glm::distance(contour.front(), contour.back()) < 1e-6f) {
			contour.pop_back();
			closed = true;
		}
		if (contour.size() < 2) {
			continue;
		}
		dxf.writePolyline(
			dw, DL_PolylineData(static_cast<int>(contour.size()), 0, 0, closed ? 1 : 0), attrib);
		for (const auto& v : contour) {
			dxf.writeVertex(dw, DL_VertexData(v.x, v.y, 0.0));
		}
		dxf.writePolylineEnd(dw);
	}
}

void writeEntity(DL_Dxf& dxf, DL_WriterA& dw, const Entity& entity, const DL_Attributes& attrib) {
	switch (entity.kind) {
	case Entity::Kind::Point: {
		glm::vec2 p{0.f, 0.f};
		if (pointOfEntity(entity, p)) {
			dxf.writePoint(dw, DL_PointData(p.x, p.y, 0.0), attrib);
		}
		return;
	}
	case Entity::Kind::Arc:
		writeArcOrEllipse(dxf, dw, attrib, entity.arc.cx, entity.arc.cy, entity.arc.rx,
						  entity.arc.ry, entity.rotationDegrees, entity.arc.startAngleDegrees,
						  entity.arc.endAngleDegrees);
		return;
	case Entity::Kind::Ellipse: {
		const auto& el = entity.ellipse;
		if (el.rx <= 0.f || el.ry <= 0.f) {
			return;
		}
		if (std::abs(el.rx - el.ry) <= 1e-4f * std::max(el.rx, el.ry)) {
			dxf.writeCircle(dw, DL_CircleData(el.cx, el.cy, 0.0, el.rx), attrib);
			return;
		}
		writeArcOrEllipse(dxf, dw, attrib, el.cx, el.cy, el.rx, el.ry, entity.rotationDegrees, 0.f,
						  360.f);
		return;
	}
	case Entity::Kind::Spline:
		if (writeSplineEntity(dxf, dw, entity.spline, attrib)) {
			return;
		}
		break;
	case Entity::Kind::Path: {
		const auto& cmds = entity.path.commands;
		if (cmds.size() == 2 && cmds[0].type == Cmd::MoveTo && cmds[1].type == Cmd::LineTo) {
			dxf.writeLine(dw, DL_LineData(cmds[0].p.x, cmds[0].p.y, 0.0, cmds[1].p.x, cmds[1].p.y,
										  0.0),
						  attrib);
			return;
		}
		break;
	}
	}
	writeContours(dxf, dw, entity, attrib);
}

} // namespace

// -------------------- Public API --------------------

glm::vec4 aciToColor(int aci) {
	const int index = aci < 0 ? -aci : aci;
	if (index <= 0 || index >= 256) {
		return {0.f, 0.f, 0.f, 1.f};
	}
	const double* rgb = dxfColors[index];
	return {static_cast<float>(rgb[0]), static_cast<float>(rgb[1]), static_cast<float>(rgb[2]),
			1.f};
}

bool loadFile(const std::string& path, Document& out) {
	out = {};
	RigDxfAdapter adapter;
	DL_Dxf reader;
	if (!reader.in(path, &adapter)) {
		out.errors.push_back("rigDxf: could not read '" + path + "'");
		return false;
	}
	buildDocument(adapter, out);
	return !out.empty();
}

bool loadBuffer(const std::vector<char>& bytes, Document& out) {
	out = {};
	std::istringstream stream(std::string(bytes.begin(), bytes.end()));
	RigDxfAdapter adapter;
	DL_Dxf reader;
	if (!reader.in(stream, &adapter)) {
		out.errors.push_back("rigDxf: could not parse DXF buffer");
		return false;
	}
	buildDocument(adapter, out);
	return !out.empty();
}

bool loadFile(const std::string& path, plot::ParsedPathDoc& out) {
	out = {};
	Document doc;
	if (!loadFile(path, doc)) {
		return false;
	}
	flattenToParsed(doc, out);
	return !out.empty();
}

void flattenToParsed(const Document& doc, plot::ParsedPathDoc& out, float tolMm) {
	out = {};
	Bounds bounds;
	for (const auto& layer : doc.layers) {
		plot::PathSourceLayer dst;
		dst.name = layer.name.empty() ? "0" : layer.name;
		dst.layerColor = layer.color;

		for (const auto& entity : layer.entities) {
			ecs::CPath path = pathOfEntity(entity, tolMm);
			if (path.commands.size() < 2) {
				continue;
			}
			for (const auto& contour : plot::flattenPathContours(path, tolMm)) {
				for (const auto& p : contour) {
					bounds.add(p);
				}
			}
			dst.paths.paths.push_back(std::move(path));
			dst.paths.pathColors.push_back(entity.color);
			dst.paths.pathFilled.push_back(0);
			dst.paths.pathStroked.push_back(1);
		}

		if (!dst.paths.paths.empty()) {
			out.layers.push_back(std::move(dst));
		}
	}

	out.minX = bounds.empty ? 0.f : bounds.minX;
	out.minY = bounds.empty ? 0.f : bounds.minY;
	out.contentW = bounds.empty ? 0.f : (bounds.maxX - bounds.minX);
	out.contentH = bounds.empty ? 0.f : (bounds.maxY - bounds.minY);
}

void flipY(Document& doc) {
	const float top = doc.minY + doc.contentH;
	for (auto& layer : doc.layers) {
		for (auto& entity : layer.entities) {
			switch (entity.kind) {
			case Entity::Kind::Path:
			case Entity::Kind::Point:
				for (auto& cmd : entity.path.commands) {
					if (cmd.type == Cmd::CubicTo) {
						cmd.c1.y = top - cmd.c1.y;
						cmd.c2.y = top - cmd.c2.y;
					} else if (cmd.type == Cmd::QuadTo) {
						cmd.c1.y = top - cmd.c1.y;
					}
					if (cmd.type != Cmd::Close) {
						cmd.p.y = top - cmd.p.y;
					}
				}
				break;
			case Entity::Kind::Arc:
				// Reflection maps the parameter and the axis angle to their
				// negatives, which also reverses the sweep direction.
				entity.arc.cy = top - entity.arc.cy;
				entity.arc.startAngleDegrees = -entity.arc.startAngleDegrees;
				entity.arc.endAngleDegrees = -entity.arc.endAngleDegrees;
				entity.rotationDegrees = -entity.rotationDegrees;
				normalizeArcAngles(entity.arc);
				break;
			case Entity::Kind::Ellipse:
				entity.ellipse.cy = top - entity.ellipse.cy;
				entity.rotationDegrees = -entity.rotationDegrees;
				break;
			case Entity::Kind::Spline:
				for (auto& p : entity.spline.controlPoints) {
					p.y = top - p.y;
				}
				for (auto& p : entity.spline.fitPoints) {
					p.y = top - p.y;
				}
				break;
			}
		}
	}
	recomputeBounds(doc);
}

void complementArcs(Document& doc) {
	for (auto& layer : doc.layers) {
		for (auto& entity : layer.entities) {
			if (entity.kind != Entity::Kind::Arc) {
				continue;
			}
			auto& arc = entity.arc;
			const float sweep = arc.endAngleDegrees - arc.startAngleDegrees;
			if (std::abs(sweep) >= 359.9f || std::abs(sweep) < 1e-4f) {
				continue;
			}
			// Same circle and same direction, the other way round the rim.
			const float turn = sweep > 0.f ? 360.f : -360.f;
			const float start = arc.endAngleDegrees;
			arc.endAngleDegrees = arc.startAngleDegrees + turn;
			arc.startAngleDegrees = start;
			normalizeArcAngles(arc);
		}
	}
	recomputeBounds(doc);
}

CurvePromotionResult promoteCurves(Document& doc, const CurvePromotionSettings& settings) {
	CurvePromotionResult result;

	// Bulge arcs already land as Arc entities during load, so the bulge settings
	// have nothing left to recover — only faceted polylines are candidates here.
	const bool wantFaceted = settings.promoteFacetedCircles || settings.promoteFacetedArcs;

	for (auto& layer : doc.layers) {
		for (auto& entity : layer.entities) {
			++result.entitiesExamined;
			if (!wantFaceted || entity.kind != Entity::Kind::Path) {
				continue;
			}

			std::vector<glm::vec2> pts;
			bool closed = false;
			if (!polylineAnchors(entity.path, pts, closed)) {
				continue;
			}

			glm::vec2 center{0.f, 0.f};
			float radius = 0.f;

			if (closed && settings.promoteFacetedCircles &&
				pts.size() >= static_cast<std::size_t>(settings.facetedCircleMinVertices) &&
				fitCircle(pts, settings.facetedCircleMaxRelativeError, center, radius) &&
				edgesAreEven(pts, settings.facetedCircleMaxEdgeVariance)) {
				const glm::vec4 color = entity.color;
				const std::string layerName = entity.layer;
				entity = makeCircleEntity(center, radius);
				entity.color = color;
				entity.layer = layerName;
				++result.circlesPromoted;
				continue;
			}

			if (!closed && settings.promoteFacetedArcs &&
				pts.size() >= static_cast<std::size_t>(settings.facetedArcMinVertices) &&
				fitCircle(pts, settings.facetedCircleMaxRelativeError, center, radius)) {
				// Unwrap the sample angles so the sweep keeps its direction.
				std::vector<float> angles;
				angles.reserve(pts.size());
				for (const auto& p : pts) {
					angles.push_back(std::atan2(p.y - center.y, p.x - center.x) * kRadToDeg);
				}
				for (std::size_t i = 1; i < angles.size(); ++i) {
					while (angles[i] - angles[i - 1] > 180.f) {
						angles[i] -= 360.f;
					}
					while (angles[i] - angles[i - 1] < -180.f) {
						angles[i] += 360.f;
					}
				}
				const float sweep = angles.back() - angles.front();
				if (std::abs(sweep) < 5.f || std::abs(sweep) > 359.f) {
					continue;
				}
				const glm::vec4 color = entity.color;
				const std::string layerName = entity.layer;
				entity = makeArcEntity(center, radius, angles.front(), angles.back());
				entity.color = color;
				entity.layer = layerName;
				++result.arcsPromoted;
			}
		}
	}

	if (result.circlesPromoted > 0 || result.arcsPromoted > 0) {
		recomputeBounds(doc);
	}
	return result;
}

bool save(const Document& doc, const std::string& path) {
	DL_Dxf dxf;
	DL_WriterA* dw = dxf.out(path.c_str(), DL_VERSION_2000);
	if (dw == nullptr) {
		return false;
	}

	dxf.writeHeader(*dw);
	dw->sectionEnd();

	dw->sectionTables();
	dxf.writeVPort(*dw);

	dw->table("LTYPE", 3);
	dxf.writeLinetype(*dw, DL_LinetypeData("BYBLOCK", "", 0, 0, 0.0));
	dxf.writeLinetype(*dw, DL_LinetypeData("BYLAYER", "", 0, 0, 0.0));
	dxf.writeLinetype(*dw, DL_LinetypeData("CONTINUOUS", "Solid line", 0, 0, 0.0));
	dw->tableEnd();

	int layerCount = 1;
	for (const auto& layer : doc.layers) {
		if (layer.name != "0") {
			++layerCount;
		}
	}
	dw->table("LAYER", layerCount);
	dxf.writeLayer(*dw, DL_LayerData("0", 0),
				   DL_Attributes("", DL_Codes::black, -1, "CONTINUOUS", 1.0));
	for (const auto& layer : doc.layers) {
		if (layer.name == "0") {
			continue;
		}
		// Layer table entries need a concrete ACI; BYLAYER only means something
		// on an entity.
		dxf.writeLayer(*dw, DL_LayerData(layer.name, 0),
					   DL_Attributes("", DL_Codes::white, -1, "CONTINUOUS", 1.0));
	}
	dw->tableEnd();

	dw->table("STYLE", 1);
	dxf.writeStyle(*dw, DL_StyleData("Standard", 0, 2.5, 1.0, 0.0, 0, 2.5, "", ""));
	dw->tableEnd();

	dxf.writeView(*dw);
	dxf.writeUcs(*dw);

	dw->table("APPID", 1);
	dxf.writeAppid(*dw, "ACAD");
	dw->tableEnd();

	dxf.writeDimStyle(*dw, 2.5, 1.25, 0.625, 0.625, 2.5);
	dxf.writeBlockRecord(*dw);
	dw->tableEnd();
	dw->sectionEnd();

	dw->sectionBlocks();
	dxf.writeBlock(*dw, DL_BlockData("*Model_Space", 0, 0.0, 0.0, 0.0));
	dxf.writeEndBlock(*dw, "*Model_Space");
	dxf.writeBlock(*dw, DL_BlockData("*Paper_Space", 0, 0.0, 0.0, 0.0));
	dxf.writeEndBlock(*dw, "*Paper_Space");
	dw->sectionEnd();

	dw->sectionEntities();
	for (const auto& layer : doc.layers) {
		const DL_Attributes attrib(layer.name.empty() ? "0" : layer.name, DL_Codes::bylayer, -1,
								   "BYLAYER", 1.0);
		for (const auto& entity : layer.entities) {
			writeEntity(dxf, *dw, entity, attrib);
		}
	}
	dw->sectionEnd();

	dxf.writeObjects(*dw);
	dxf.writeObjectsEnd(*dw);
	dw->dxfEOF();
	dw->close();
	delete dw;
	return true;
}

} // namespace dxf
} // namespace rigkit
