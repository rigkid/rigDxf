#include "DxfLoader.h"

#include <glm/glm.hpp>

#include "PathFlatten.h"
#include "PrimitiveVertices.h"
#include "dl_attributes.h"
#include "dl_codes.h"
#include "dl_creationadapter.h"
#include "dl_dxf.h"
#include "dl_entities.h"
#include "dl_writer_ascii.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <map>
#include <numeric>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

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
/// Nested INSERT ceiling - cyclic block references would otherwise never end.
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
	static const glm::vec4 kPalette[] = {{1.00f, 0.31f, 0.31f, 1.f}, {0.31f, 0.78f, 0.31f, 1.f},
										 {0.31f, 0.55f, 1.00f, 1.f}, {1.00f, 0.78f, 0.24f, 1.f},
										 {0.78f, 0.31f, 1.00f, 1.f}, {0.31f, 0.86f, 0.86f, 1.f},
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

float chordError(const glm::vec2& mid, const glm::vec2& a, const glm::vec2& b) {
	const glm::vec2 ab = b - a;
	const float len2 = glm::dot(ab, ab);
	if (len2 < 1e-12f) {
		return glm::length(mid - a);
	}
	const float t = glm::dot(mid - a, ab) / len2;
	return glm::length(mid - (a + t * ab));
}

std::vector<glm::vec2> splinePolyline(const ecs::CSpline& spline, float tolMm) {
	if (ecs::verticesOf(spline, 2).empty()) {
		std::vector<glm::vec2> pts =
			spline.controlPoints.empty() ? spline.fitPoints : spline.controlPoints;
		if (spline.closed && pts.size() > 2) {
			pts.push_back(pts.front());
		}
		return pts;
	}

	const float tol = tolMm > 0.f ? tolMm : 0.25f;
	constexpr int kSeeds = 16;
	constexpr int kMaxDepth = 12;
	constexpr int kMaxPoints = 4096;

	std::vector<glm::vec2> pts;
	pts.reserve(static_cast<std::size_t>(kSeeds) + 1);
	int remaining = kMaxPoints;
	pts.push_back(ecs::pointOn(spline, 0.f));

	const auto subdiv = [&](auto& self, const glm::vec2& a, const glm::vec2& b, float t0, float t1,
							int depth) -> void {
		if (remaining <= 0) {
			pts.push_back(b);
			return;
		}
		const float tm = 0.5f * (t0 + t1);
		const glm::vec2 mid = ecs::pointOn(spline, tm);
		if (chordError(mid, a, b) <= tol || depth >= kMaxDepth) {
			pts.push_back(b);
			--remaining;
			return;
		}
		self(self, a, mid, t0, tm, depth + 1);
		self(self, mid, b, tm, t1, depth + 1);
	};

	for (int i = 0; i < kSeeds; ++i) {
		const float t0 = static_cast<float>(i) / static_cast<float>(kSeeds);
		const float t1 = static_cast<float>(i + 1) / static_cast<float>(kSeeds);
		subdiv(subdiv, ecs::pointOn(spline, t0), ecs::pointOn(spline, t1), t0, t1, 0);
	}
	if (spline.closed && pts.size() > 2 && glm::distance(pts.front(), pts.back()) > 1e-6f) {
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
	case Entity::Kind::Text:
	case Entity::Kind::Image:
	case Entity::Kind::XLine:
	case Entity::Kind::Ray:
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
			if (entity.kind == Entity::Kind::Text) {
				b.add(entity.text.baseline);
				continue;
			}
			if (entity.kind == Entity::Kind::Image) {
				b.add(entity.image.pos);
				b.add(entity.image.pos + entity.image.size);
				continue;
			}
			if (entity.kind == Entity::Kind::XLine || entity.kind == Entity::Kind::Ray) {
				b.add(entity.xlineOrigin);
				continue;
			}
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

Entity makeArcEntity(const glm::vec2& center, float radius, float startDegrees, float endDegrees) {
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
 * it - no side-of-the-chord guesswork, and half-turns fall out at h = 0.
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
	const float offset =
		std::abs(halfTan) < 1e-9 ? 0.f : static_cast<float>((chordLen * 0.5) / halfTan);
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
	case Entity::Kind::Text:
		e.text.baseline = xformPoint(t, e.text.baseline);
		e.text.rotationDegrees += t.angleDegrees;
		e.text.size *= std::max(std::abs(t.sx), std::abs(t.sy));
		break;
	case Entity::Kind::Image:
		e.image.pos = xformPoint(t, e.image.pos);
		e.image.size.x *= std::abs(t.sx);
		e.image.size.y *= std::abs(t.sy);
		break;
	case Entity::Kind::XLine:
	case Entity::Kind::Ray:
		e.xlineOrigin = xformPoint(t, e.xlineOrigin);
		e.xlineDir = xformVector(t, e.xlineDir);
		break;
	}
	return e;
}

bool isDimBlockName(const std::string& name) {
	return name.size() >= 2 && name[0] == '*' && (name[1] == 'D' || name[1] == 'd');
}

std::string cleanDxfText(std::string s) {
	std::string out;
	out.reserve(s.size());
	for (std::size_t i = 0; i < s.size(); ++i) {
		if (i + 2 < s.size() && s[i] == '%' && s[i + 1] == '%') {
			const char code = static_cast<char>(std::tolower(static_cast<unsigned char>(s[i + 2])));
			if (code == 'd') {
				out += " deg";
				i += 2;
				continue;
			}
			if (code == 'c') {
				out += "D";
				i += 2;
				continue;
			}
			if (code == 'p') {
				out += "+/-";
				i += 2;
				continue;
			}
		}
		if (s[i] == '\\' && i + 1 < s.size()) {
			const char n = s[i + 1];
			if (n == 'P' || n == 'p' || n == '~') {
				out.push_back(' ');
				++i;
				continue;
			}
			if (n == '\\' || n == '{' || n == '}') {
				out.push_back(n);
				++i;
				continue;
			}
			// Skip a formatting code until ';' or end of token.
			std::size_t j = i + 1;
			while (j < s.size() && s[j] != ';' && s[j] != '\\' && s[j] != '{') {
				++j;
			}
			if (j < s.size() && s[j] == ';') {
				i = j;
				continue;
			}
		}
		if (s[i] == '{' || s[i] == '}') {
			continue;
		}
		out.push_back(s[i]);
	}
	return out;
}

Entity makeTextEntity(const std::string& utf8, const glm::vec2& baseline, float height,
					  float rotationDegrees) {
	Entity e;
	e.kind = Entity::Kind::Text;
	e.text.text = cleanDxfText(utf8);
	e.text.size = height > 0.f ? height : 2.5f;
	e.text.baseline = baseline;
	e.text.rotationDegrees = rotationDegrees;
	return e;
}

Entity makeXLineEntity(Entity::Kind kind, const glm::vec2& origin, const glm::vec2& dir) {
	Entity e;
	e.kind = kind;
	e.xlineOrigin = origin;
	const float len = glm::length(dir);
	e.xlineDir = len > 1e-12f ? dir / len : glm::vec2{1.f, 0.f};
	return e;
}

// Clip infinite / half line to an AABB (Liang-Barsky).
bool clipAxisLine(const glm::vec2& origin, const glm::vec2& dir, float minX, float minY, float maxX,
				  float maxY, bool ray, glm::vec2& a, glm::vec2& b) {
	if (maxX <= minX || maxY <= minY) {
		return false;
	}
	float t0 = ray ? 0.f : -1e6f;
	float t1 = 1e6f;
	const float p[4] = {-dir.x, dir.x, -dir.y, dir.y};
	const float q[4] = {origin.x - minX, maxX - origin.x, origin.y - minY, maxY - origin.y};
	for (int i = 0; i < 4; ++i) {
		if (std::abs(p[i]) < 1e-12f) {
			if (q[i] < 0.f) {
				return false;
			}
			continue;
		}
		const float t = q[i] / p[i];
		if (p[i] < 0.f) {
			t0 = std::max(t0, t);
		} else {
			t1 = std::min(t1, t);
		}
	}
	if (t0 > t1) {
		return false;
	}
	a = origin + dir * t0;
	b = origin + dir * t1;
	return glm::length(b - a) > 1e-6f;
}

ecs::CPath pathOfEntity(const Entity& entity, float tolMm);

// -------------------- dxflib adapter --------------------

struct Block {
	glm::vec2 base{0.f, 0.f};
	std::vector<Entity> entities;
};

/** @brief Constant / vertex geometric width on the polyline being read (mm). */
float polylineGeomWidthMm(DL_Dxf* reader) {
	if (!reader) {
		return 0.f;
	}
	// LWPOLYLINE 43 = constant width. Else 40/41 start/end (last vertex wins).
	double w = reader->getRealValue(43, 0.0);
	if (w <= 0.0) {
		w = reader->getRealValue(40, 0.0);
	}
	if (w <= 0.0) {
		w = reader->getRealValue(41, 0.0);
	}
	return static_cast<float>(w);
}

/** @brief dxflib callbacks to typed POD entities, layer by layer. */
class RigDxfAdapter : public DL_CreationAdapter {
  public:
	void bindReader(DL_Dxf* reader) { m_reader = reader; }

	void addLayer(const DL_LayerData& data) override {
		m_layerColors[data.name] =
			resolveLayerColor(getAttributes(), static_cast<int>(m_layers.size()));
		Layer& layer = ensureLayer(data.name);
		layer.visible = !data.off && (data.flags & 1) == 0;
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
		m_polylineWidthMm = polylineGeomWidthMm(m_reader);
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
		Entity e = makePolylineEntity(pts);
		e.filled = true;
		pushEntity(std::move(e));
	}

	void addSolid(const DL_SolidData& d) override { addTrace(d); }

	void add3dFace(const DL_3dFaceData& d) override { addTrace(d); }

	void addHatch(const DL_HatchData& d) override {
		m_hatchActive = true;
		m_hatchSolid = d.solid;
		m_hatchEntities.clear();
	}

	void addHatchLoop(const DL_HatchLoopData&) override {}

	void addHatchEdge(const DL_HatchEdgeData& d) override {
		if (!appendHatchEdge(d, m_hatchEntities)) {
			noteSkipped("HATCH_EDGE");
		}
	}

	void addBlock(const DL_BlockData& data) override {
		m_dimBlock = false;
		if (data.name.empty()) {
			m_collectingBlock = false;
			m_activeBlock.clear();
			return;
		}
		const bool dim = isDimBlockName(data.name);
		if (data.name[0] == '*' && !dim) {
			m_collectingBlock = false;
			m_activeBlock.clear();
			return;
		}
		m_activeBlock = data.name;
		m_blocks[m_activeBlock].base =
			glm::vec2(static_cast<float>(data.bpx), static_cast<float>(data.bpy));
		m_collectingBlock = true;
		m_dimBlock = dim;
	}

	void endBlock() override {
		m_collectingBlock = false;
		m_activeBlock.clear();
	}

	void addInsert(const DL_InsertData& d) override { expandInsert(d, 0); }

	void endEntity() override {
		if (m_hatchActive) {
			m_hatchActive = false;
			if (m_hatchSolid) {
				ecs::CPath combined;
				for (const auto& e : m_hatchEntities) {
					const ecs::CPath piece = pathOfEntity(e, 0.25f);
					combined.commands.insert(combined.commands.end(), piece.commands.begin(),
											 piece.commands.end());
				}
				if (combined.commands.size() >= 2) {
					Entity e;
					e.kind = Entity::Kind::Path;
					e.path = std::move(combined);
					e.filled = true;
					pushEntity(std::move(e));
				}
			} else {
				for (auto& e : m_hatchEntities) {
					pushEntity(std::move(e));
				}
			}
			m_hatchEntities.clear();
			m_hatchSolid = false;
			return;
		}
		if (m_leaderActive) {
			m_leaderActive = false;
			if (m_leaderPts.size() >= 2) {
				pushEntity(makePolylineEntity(m_leaderPts));
			}
			m_leaderPts.clear();
			return;
		}
		if (m_mtextPending) {
			flushMText();
			return;
		}
		if (!m_pendingVertices.empty()) {
			flushPolyline();
			return;
		}
		flushSpline();
	}

	void endSequence() override { flushSpline(); }

	void addXLine(const DL_XLineData& d) override {
		pushEntity(makeXLineEntity(Entity::Kind::XLine,
								   {static_cast<float>(d.bx), static_cast<float>(d.by)},
								   {static_cast<float>(d.dx), static_cast<float>(d.dy)}));
	}
	void addRay(const DL_RayData& d) override {
		pushEntity(makeXLineEntity(Entity::Kind::Ray,
								   {static_cast<float>(d.bx), static_cast<float>(d.by)},
								   {static_cast<float>(d.dx), static_cast<float>(d.dy)}));
	}
	void addText(const DL_TextData& d) override { pushText(d); }
	void addMText(const DL_MTextData& d) override {
		flushMText();
		m_mtext = d;
		m_mtextPending = true;
	}
	void addMTextChunk(const std::string& chunk) override {
		if (m_mtextPending) {
			m_mtext.text += chunk;
		}
	}
	void addArcAlignedText(const DL_ArcAlignedTextData& d) override {
		const float mid = 0.5f * static_cast<float>(d.startAngle + d.endAngle);
		const float radius = static_cast<float>(d.radius + d.offset);
		pushEntity(makeTextEntity(d.text,
								  {static_cast<float>(d.cx) + radius * std::cos(mid),
								   static_cast<float>(d.cy) + radius * std::sin(mid)},
								  static_cast<float>(d.height), mid * kRadToDeg));
	}
	void addAttribute(const DL_AttributeData& d) override { pushText(d); }
	void addDimAlign(const DL_DimensionData& d, const DL_DimAlignedData&) override {
		rememberDimText(d);
	}
	void addDimLinear(const DL_DimensionData& d, const DL_DimLinearData&) override {
		rememberDimText(d);
	}
	void addDimRadial(const DL_DimensionData& d, const DL_DimRadialData&) override {
		rememberDimText(d);
	}
	void addDimDiametric(const DL_DimensionData& d, const DL_DimDiametricData&) override {
		rememberDimText(d);
	}
	void addDimAngular(const DL_DimensionData& d, const DL_DimAngular2LData&) override {
		rememberDimText(d);
	}
	void addDimAngular3P(const DL_DimensionData& d, const DL_DimAngular3PData&) override {
		rememberDimText(d);
	}
	void addDimOrdinate(const DL_DimensionData& d, const DL_DimOrdinateData&) override {
		rememberDimText(d);
	}
	void addLeader(const DL_LeaderData&) override {
		m_leaderActive = true;
		m_leaderPts.clear();
	}
	void addLeaderVertex(const DL_LeaderVertexData& d) override {
		m_leaderPts.emplace_back(static_cast<float>(d.x), static_cast<float>(d.y));
	}
	void addImage(const DL_ImageData& d) override {
		Entity e;
		e.kind = Entity::Kind::Image;
		e.image.pos = {static_cast<float>(d.ipx), static_cast<float>(d.ipy)};
		const float px = static_cast<float>(std::max(1, d.width));
		const float py = static_cast<float>(std::max(1, d.height));
		e.image.size = {static_cast<float>(std::hypot(d.ux, d.uy)) * px,
						static_cast<float>(std::hypot(d.vx, d.vy)) * py};
		const auto it = m_imageFiles.find(d.ref);
		e.image.href = (it != m_imageFiles.end()) ? it->second : d.ref;
		e.image.opacity = 1.f - static_cast<float>(d.fade) / 100.f;
		pushEntity(std::move(e));
	}
	void linkImage(const DL_ImageDefData& d) override {
		if (d.ref.empty()) {
			return;
		}
		m_imageFiles[d.ref] = d.file;
		auto patch = [&](Entity& e) {
			if (e.kind == Entity::Kind::Image && e.image.href == d.ref) {
				e.image.href = d.file;
			}
		};
		for (auto& layer : m_layers) {
			for (auto& e : layer.entities) {
				patch(e);
			}
		}
		for (auto& [name, block] : m_blocks) {
			(void)name;
			for (auto& e : block.entities) {
				patch(e);
			}
		}
	}

	/** @brief Finish deferred work and hand over the layers. */
	std::vector<Layer> takeLayers() {
		flushSpline();
		flushMText();
		flushPendingInserts();
		m_collectingBlock = false;
		m_dimBlock = false;
		m_activeBlock.clear();
		flushDimensionBlocks();
		resolveImageHrefs();
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
	bool m_dimBlock = false;
	std::string m_activeBlock;
	DL_MTextData m_mtext{0, 0, 0, 1, 0, 0, 2.5, 0, 7, 1, 1, 1.0, "", "Standard", 0};
	bool m_mtextPending = false;
	std::vector<Entity> m_dimFallbackTexts;
	bool m_flushedDimGeometry = false;

	DL_Dxf* m_reader = nullptr;
	std::vector<BulgeVertex> m_pendingVertices;
	bool m_polylineClosed = false;
	float m_polylineWidthMm = 0.f;

	ecs::CSpline m_spline;
	bool m_splinePending = false;

	bool m_hatchActive = false;
	bool m_hatchSolid = false;
	std::vector<Entity> m_hatchEntities;

	bool m_leaderActive = false;
	std::vector<glm::vec2> m_leaderPts;
	std::map<std::string, std::string> m_imageFiles;

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
		if (e.strokeWidthMm <= 0.f) {
			const int width = getAttributes().getWidth();
			if (width > 0) {
				e.strokeWidthMm = static_cast<float>(width) * 0.01f;
			}
		}
		if (m_dimBlock) {
			e.fromDimension = true;
		}
		if (e.kind == Entity::Kind::Text) {
			e.text.rgba = e.color;
		}
		pushToTarget(std::move(e));
	}

	void resolveImageHrefs() {
		auto patch = [&](Entity& e) {
			if (e.kind != Entity::Kind::Image) {
				return;
			}
			const auto it = m_imageFiles.find(e.image.href);
			if (it != m_imageFiles.end()) {
				e.image.href = it->second;
			}
		};
		for (auto& layer : m_layers) {
			for (auto& e : layer.entities) {
				patch(e);
			}
		}
		for (auto& [name, block] : m_blocks) {
			(void)name;
			for (auto& e : block.entities) {
				patch(e);
			}
		}
	}

	void pushText(const DL_TextData& d) {
		const bool useAlign = d.hJustification != 0 || d.vJustification != 0;
		const glm::vec2 origin =
			useAlign ? glm::vec2(static_cast<float>(d.apx), static_cast<float>(d.apy))
					 : glm::vec2(static_cast<float>(d.ipx), static_cast<float>(d.ipy));
		pushEntity(makeTextEntity(d.text, origin, static_cast<float>(d.height),
								  static_cast<float>(d.angle)));
	}

	void flushMText() {
		if (!m_mtextPending) {
			return;
		}
		m_mtextPending = false;
		float angle = static_cast<float>(m_mtext.angle);
		if (std::abs(m_mtext.dirx) + std::abs(m_mtext.diry) > 1e-8) {
			angle = std::atan2(static_cast<float>(m_mtext.diry), static_cast<float>(m_mtext.dirx)) *
					kRadToDeg;
		}
		pushEntity(makeTextEntity(
			m_mtext.text, {static_cast<float>(m_mtext.ipx), static_cast<float>(m_mtext.ipy)},
			static_cast<float>(m_mtext.height), angle));
		m_mtext = DL_MTextData(0, 0, 0, 1, 0, 0, 2.5, 0, 7, 1, 1, 1.0, "", "Standard", 0);
	}

	void rememberDimText(const DL_DimensionData& d) {
		if (d.text.empty() || d.text == "<>" || d.text == " ") {
			return;
		}
		const float height = d.dimScale > 0.0 ? static_cast<float>(d.dimScale * 2.5) : 2.5f;
		Entity e = makeTextEntity(d.text, {static_cast<float>(d.mpx), static_cast<float>(d.mpy)},
								  height, static_cast<float>(d.angle));
		e.fromDimension = true;
		m_dimFallbackTexts.push_back(std::move(e));
	}

	void flushDimensionBlocks() {
		for (const auto& [name, block] : m_blocks) {
			if (!isDimBlockName(name)) {
				continue;
			}
			for (const auto& src : block.entities) {
				Entity e = src;
				e.fromDimension = true;
				if (e.kind == Entity::Kind::Text) {
					e.text.rgba = e.color;
				}
				pushToTarget(std::move(e));
				m_flushedDimGeometry = true;
			}
		}
		if (!m_flushedDimGeometry) {
			for (auto& e : m_dimFallbackTexts) {
				pushToTarget(std::move(e));
			}
		}
		m_dimFallbackTexts.clear();
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
			if (m_polylineWidthMm > 0.f) {
				e.strokeWidthMm = m_polylineWidthMm;
			}
			pushEntity(std::move(e));
		}
		m_pendingVertices.clear();
		m_polylineClosed = false;
		m_polylineWidthMm = 0.f;
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
		// Empty weights mean "all ones" - do not carry a redundant array.
		const bool rational = std::any_of(m_spline.weights.begin(), m_spline.weights.end(),
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
	if (entity.kind == Entity::Kind::Point || entity.kind == Entity::Kind::Text ||
		entity.kind == Entity::Kind::Image || entity.kind == Entity::Kind::XLine ||
		entity.kind == Entity::Kind::Ray) {
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

void writeContours(DL_Dxf& dxf, DL_WriterA& dw, const Entity& entity, const DL_Attributes& attrib) {
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

bool writeFilledHatch(DL_Dxf& dxf, DL_WriterA& dw, const Entity& entity,
					  const DL_Attributes& attrib) {
	auto contours = contoursOfEntity(entity, kExportTolerance);
	int loops = 0;
	for (auto& contour : contours) {
		if (contour.size() > 2 && glm::distance(contour.front(), contour.back()) < 1e-6f) {
			contour.pop_back();
		}
		if (contour.size() >= 3) {
			++loops;
		}
	}
	if (loops <= 0) {
		return false;
	}
	const DL_HatchData hatch(loops, true, 1.0, 0.0, "SOLID");
	dxf.writeHatch1(dw, hatch, attrib);
	for (const auto& contour : contours) {
		if (contour.size() < 3) {
			continue;
		}
		const int n = static_cast<int>(contour.size());
		dxf.writeHatchLoop1(dw, DL_HatchLoopData(n));
		for (int i = 0; i < n; ++i) {
			const auto& a = contour[static_cast<std::size_t>(i)];
			const auto& b = contour[static_cast<std::size_t>((i + 1) % n)];
			dxf.writeHatchEdge(dw, DL_HatchEdgeData(a.x, a.y, b.x, b.y));
		}
		dxf.writeHatchLoop2(dw, DL_HatchLoopData(n));
	}
	dxf.writeHatch2(dw, hatch, attrib);
	return true;
}

struct PendingImageWrite {
	unsigned long handle = 0;
	DL_ImageData data{"", 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 50, 50, 0};
};

void writeEntity(DL_Dxf& dxf, DL_WriterA& dw, const Entity& entity, const DL_Attributes& attrib,
				 std::vector<PendingImageWrite>& images) {
	if (entity.filled && entity.kind != Entity::Kind::Text && entity.kind != Entity::Kind::Image) {
		if (writeFilledHatch(dxf, dw, entity, attrib)) {
			return;
		}
	}
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
	case Entity::Kind::Text: {
		const auto& t = entity.text;
		dxf.writeText(dw,
					  DL_TextData(t.baseline.x, t.baseline.y, 0.0, t.baseline.x, t.baseline.y, 0.0,
								  t.size, 1.0, 0, 0, 0, t.text, "Standard", t.rotationDegrees),
					  attrib);
		return;
	}
	case Entity::Kind::Image: {
		const auto& img = entity.image;
		const double sx = std::max(1e-6, static_cast<double>(img.size.x));
		const double sy = std::max(1e-6, static_cast<double>(img.size.y));
		PendingImageWrite pending;
		pending.data =
			DL_ImageData(img.href, img.pos.x, img.pos.y, 0.0, sx, 0.0, 0.0, 0.0, sy, 0.0, 1, 1, 50,
						 50, static_cast<int>(std::lround((1.f - img.opacity) * 100.f)));
		pending.handle = dxf.writeImage(dw, pending.data, attrib);
		images.push_back(std::move(pending));
		return;
	}
	case Entity::Kind::XLine:
		dxf.writeXLine(dw,
					   DL_XLineData(entity.xlineOrigin.x, entity.xlineOrigin.y, 0.0,
									entity.xlineDir.x, entity.xlineDir.y, 0.0),
					   attrib);
		return;
	case Entity::Kind::Ray:
		dxf.writeRay(dw,
					 DL_RayData(entity.xlineOrigin.x, entity.xlineOrigin.y, 0.0, entity.xlineDir.x,
								entity.xlineDir.y, 0.0),
					 attrib);
		return;
	case Entity::Kind::Path: {
		const auto& cmds = entity.path.commands;
		if (cmds.size() == 2 && cmds[0].type == Cmd::MoveTo && cmds[1].type == Cmd::LineTo) {
			dxf.writeLine(dw,
						  DL_LineData(cmds[0].p.x, cmds[0].p.y, 0.0, cmds[1].p.x, cmds[1].p.y, 0.0),
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
	adapter.bindReader(&reader);
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
	adapter.bindReader(&reader);
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
	struct PendingClip {
		std::size_t layerIndex = 0;
		Entity entity;
	};
	std::vector<PendingClip> clips;

	auto pushPath = [&](plot::PathSourceLayer& dst, const Entity& entity, ecs::CPath path) {
		if (path.commands.size() < 2) {
			return;
		}
		for (const auto& contour : plot::flattenPathContours(path, tolMm)) {
			for (const auto& p : contour) {
				bounds.add(p);
			}
		}
		dst.paths.paths.push_back(std::move(path));
		dst.paths.pathColors.push_back(entity.color);
		dst.paths.pathFilled.push_back(entity.filled ? 1 : 0);
		dst.paths.pathStroked.push_back(entity.filled ? 0 : 1);
		// Always store a slot so later fallbacks do not become 1 mm.
		dst.paths.pathStrokeWidths.resize(dst.paths.paths.size(), 0.f);
		if (entity.strokeWidthMm > 0.f) {
			dst.paths.pathStrokeWidths.back() = entity.strokeWidthMm;
		}
	};

	for (const auto& layer : doc.layers) {
		plot::PathSourceLayer dst;
		dst.name = layer.name.empty() ? "0" : layer.name;
		dst.layerColor = layer.color;
		dst.visible = layer.visible;

		for (const auto& entity : layer.entities) {
			if (entity.kind == Entity::Kind::Text) {
				if (!entity.text.text.empty()) {
					ecs::TextItem item = entity.text;
					item.rgba = entity.color;
					dst.texts.items.push_back(std::move(item));
					bounds.add(entity.text.baseline);
				}
				continue;
			}
			if (entity.kind == Entity::Kind::Image) {
				dst.images.push_back(entity.image);
				bounds.add(entity.image.pos);
				bounds.add(entity.image.pos + entity.image.size);
				continue;
			}
			if (entity.kind == Entity::Kind::XLine || entity.kind == Entity::Kind::Ray) {
				PendingClip pending;
				pending.layerIndex = out.layers.size();
				pending.entity = entity;
				clips.push_back(std::move(pending));
				continue;
			}
			pushPath(dst, entity, pathOfEntity(entity, tolMm));
		}

		out.layers.push_back(std::move(dst));
	}

	float minX = bounds.empty ? -100.f : bounds.minX;
	float minY = bounds.empty ? -100.f : bounds.minY;
	float maxX = bounds.empty ? 100.f : bounds.maxX;
	float maxY = bounds.empty ? 100.f : bounds.maxY;
	const float padX = std::max(10.f, (maxX - minX) * 0.1f);
	const float padY = std::max(10.f, (maxY - minY) * 0.1f);
	minX -= padX;
	minY -= padY;
	maxX += padX;
	maxY += padY;

	for (const auto& clip : clips) {
		if (clip.layerIndex >= out.layers.size()) {
			continue;
		}
		glm::vec2 a{0.f, 0.f};
		glm::vec2 b{0.f, 0.f};
		if (!clipAxisLine(clip.entity.xlineOrigin, clip.entity.xlineDir, minX, minY, maxX, maxY,
						  clip.entity.kind == Entity::Kind::Ray, a, b)) {
			continue;
		}
		pushPath(out.layers[clip.layerIndex], clip.entity, makeLineEntity(a, b).path);
	}

	out.layers.erase(std::remove_if(out.layers.begin(), out.layers.end(),
									[](const plot::PathSourceLayer& l) {
										return l.paths.paths.empty() && l.texts.items.empty() &&
											   l.images.empty();
									}),
					 out.layers.end());

	out.minX = bounds.empty ? 0.f : bounds.minX;
	out.minY = bounds.empty ? 0.f : bounds.minY;
	out.contentW = bounds.empty ? 0.f : (bounds.maxX - bounds.minX);
	out.contentH = bounds.empty ? 0.f : (bounds.maxY - bounds.minY);
}

void joinConnectedPaths(Document& doc, float tolMm) {
	const float tol = std::max(tolMm, 1e-6f);
	bool any = false;

	for (auto& layer : doc.layers) {
		struct Open {
			std::vector<glm::vec2> pts;
			glm::vec4 color{0.f, 0.f, 0.f, 1.f};
			std::string name;
		};
		std::vector<Open> open;
		std::vector<Entity> keep;
		keep.reserve(layer.entities.size());

		for (auto& entity : layer.entities) {
			if (entity.kind != Entity::Kind::Path) {
				keep.push_back(std::move(entity));
				continue;
			}
			std::vector<glm::vec2> pts;
			bool closed = false;
			if (!polylineAnchors(entity.path, pts, closed) || closed) {
				keep.push_back(std::move(entity));
				continue;
			}
			Open o;
			o.pts = std::move(pts);
			o.color = entity.color;
			o.name = entity.layer;
			open.push_back(std::move(o));
		}

		std::vector<char> used(open.size(), 0);
		std::vector<Open> joined;
		joined.reserve(open.size());
		for (std::size_t i = 0; i < open.size(); ++i) {
			if (used[i]) {
				continue;
			}
			used[i] = 1;
			Open cur = std::move(open[i]);
			bool grew = true;
			while (grew) {
				grew = false;
				for (std::size_t j = 0; j < open.size(); ++j) {
					if (used[j]) {
						continue;
					}
					auto& cand = open[j];
					if (glm::distance(cur.pts.back(), cand.pts.front()) <= tol) {
						cur.pts.insert(cur.pts.end(), cand.pts.begin() + 1, cand.pts.end());
					} else if (glm::distance(cur.pts.back(), cand.pts.back()) <= tol) {
						cur.pts.insert(cur.pts.end(), cand.pts.rbegin() + 1, cand.pts.rend());
					} else if (glm::distance(cur.pts.front(), cand.pts.back()) <= tol) {
						cur.pts.insert(cur.pts.begin(), cand.pts.begin(), cand.pts.end() - 1);
					} else if (glm::distance(cur.pts.front(), cand.pts.front()) <= tol) {
						cur.pts.insert(cur.pts.begin(), cand.pts.rbegin(), cand.pts.rend() - 1);
					} else {
						continue;
					}
					used[j] = 1;
					grew = true;
				}
			}
			if (cur.pts.size() > 2 && glm::distance(cur.pts.front(), cur.pts.back()) <= tol) {
				cur.pts.back() = cur.pts.front();
			}
			joined.push_back(std::move(cur));
		}

		if (joined.size() < open.size()) {
			any = true;
		}
		for (auto& piece : joined) {
			Entity e = makePolylineEntity(piece.pts);
			e.color = piece.color;
			e.layer = piece.name;
			keep.push_back(std::move(e));
		}
		layer.entities = std::move(keep);
	}

	if (any) {
		recomputeBounds(doc);
	}
}

void translate(Document& doc, const glm::vec2& delta) {
	if (std::fabs(delta.x) < 1e-8f && std::fabs(delta.y) < 1e-8f) {
		return;
	}
	for (auto& layer : doc.layers) {
		for (auto& entity : layer.entities) {
			switch (entity.kind) {
			case Entity::Kind::Path:
			case Entity::Kind::Point:
				for (auto& cmd : entity.path.commands) {
					if (cmd.type == Cmd::CubicTo) {
						cmd.c1 += delta;
						cmd.c2 += delta;
					} else if (cmd.type == Cmd::QuadTo) {
						cmd.c1 += delta;
					}
					if (cmd.type != Cmd::Close) {
						cmd.p += delta;
					}
				}
				break;
			case Entity::Kind::Arc:
				entity.arc.cx += delta.x;
				entity.arc.cy += delta.y;
				break;
			case Entity::Kind::Ellipse:
				entity.ellipse.cx += delta.x;
				entity.ellipse.cy += delta.y;
				break;
			case Entity::Kind::Spline:
				for (auto& p : entity.spline.controlPoints) {
					p += delta;
				}
				for (auto& p : entity.spline.fitPoints) {
					p += delta;
				}
				break;
			case Entity::Kind::Text:
				entity.text.baseline += delta;
				break;
			case Entity::Kind::Image:
				entity.image.pos += delta;
				break;
			case Entity::Kind::XLine:
			case Entity::Kind::Ray:
				entity.xlineOrigin += delta;
				break;
			}
		}
	}
	recomputeBounds(doc);
}

void originToPage(Document& doc) {
	recomputeBounds(doc);
	translate(doc, {-doc.minX, -doc.minY});
}

void applyImportFilter(Document& doc, const ImportFilter& filter) {
	std::vector<Entity> dimMoved;
	for (auto& layer : doc.layers) {
		std::vector<Entity> keep;
		keep.reserve(layer.entities.size());
		for (auto& e : layer.entities) {
			if (e.kind == Entity::Kind::Text && !e.fromDimension && !filter.text) {
				continue;
			}
			if (e.fromDimension && !filter.dimensions) {
				continue;
			}
			if (e.fromDimension && filter.dimensionsOwnLayer) {
				e.layer = "Dimensions";
				dimMoved.push_back(std::move(e));
				continue;
			}
			keep.push_back(std::move(e));
		}
		layer.entities = std::move(keep);
	}

	if (!dimMoved.empty()) {
		Layer* dest = nullptr;
		for (auto& layer : doc.layers) {
			if (layer.name == "Dimensions") {
				dest = &layer;
				break;
			}
		}
		if (dest == nullptr) {
			Layer layer;
			layer.name = "Dimensions";
			doc.layers.push_back(std::move(layer));
			dest = &doc.layers.back();
		}
		for (auto& e : dimMoved) {
			dest->entities.push_back(std::move(e));
		}
	}

	doc.layers.erase(std::remove_if(doc.layers.begin(), doc.layers.end(),
									[](const Layer& l) { return l.entities.empty(); }),
					 doc.layers.end());
	recomputeBounds(doc);
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
			case Entity::Kind::Text:
				entity.text.baseline.y = top - entity.text.baseline.y;
				entity.text.rotationDegrees = -entity.text.rotationDegrees;
				break;
			case Entity::Kind::Image:
				entity.image.pos.y = top - (entity.image.pos.y + entity.image.size.y);
				break;
			case Entity::Kind::XLine:
			case Entity::Kind::Ray:
				entity.xlineOrigin.y = top - entity.xlineOrigin.y;
				entity.xlineDir.y = -entity.xlineDir.y;
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
	// have nothing left to recover - only faceted polylines are candidates here.
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
				const bool filled = entity.filled;
				const float width = entity.strokeWidthMm;
				const bool dim = entity.fromDimension;
				entity = makeCircleEntity(center, radius);
				entity.color = color;
				entity.layer = layerName;
				entity.filled = filled;
				entity.strokeWidthMm = width;
				entity.fromDimension = dim;
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
				const bool filled = entity.filled;
				const float width = entity.strokeWidthMm;
				const bool dim = entity.fromDimension;
				entity = makeArcEntity(center, radius, angles.front(), angles.back());
				entity.color = color;
				entity.layer = layerName;
				entity.filled = filled;
				entity.strokeWidthMm = width;
				entity.fromDimension = dim;
				++result.arcsPromoted;
			}
		}
	}

	if (result.circlesPromoted > 0 || result.arcsPromoted > 0) {
		recomputeBounds(doc);
	}
	return result;
}

Document documentFromParsed(const plot::ParsedPathDoc& parsed) {
	Document doc;
	for (const auto& src : parsed.layers) {
		Layer layer;
		layer.name = src.name.empty() ? "0" : src.name;
		layer.color = src.layerColor;
		layer.visible = src.visible;
		for (std::size_t i = 0; i < src.paths.paths.size(); ++i) {
			Entity e;
			e.kind = Entity::Kind::Path;
			e.path = src.paths.paths[i];
			e.layer = layer.name;
			e.color = (i < src.paths.pathColors.size()) ? src.paths.pathColors[i] : layer.color;
			e.filled = src.paths.isFilled(i);
			e.strokeWidthMm = src.paths.strokeWidth(i, 0.f);
			layer.entities.push_back(std::move(e));
		}
		for (const auto& text : src.texts.items) {
			Entity e;
			e.kind = Entity::Kind::Text;
			e.text = text;
			e.layer = layer.name;
			e.color = text.rgba;
			layer.entities.push_back(std::move(e));
		}
		for (const auto& image : src.images) {
			Entity e;
			e.kind = Entity::Kind::Image;
			e.image = image;
			e.layer = layer.name;
			layer.entities.push_back(std::move(e));
		}
		if (!layer.entities.empty()) {
			doc.layers.push_back(std::move(layer));
		}
	}
	recomputeBounds(doc);
	return doc;
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
	bool wroteZero = false;
	for (const auto& layer : doc.layers) {
		if (layer.name != "0") {
			continue;
		}
		dxf.writeLayer(*dw, DL_LayerData("0", layer.visible ? 0 : 1, !layer.visible),
					   DL_Attributes("", DL_Codes::black, -1, "CONTINUOUS", 1.0));
		wroteZero = true;
		break;
	}
	if (!wroteZero) {
		dxf.writeLayer(*dw, DL_LayerData("0", 0),
					   DL_Attributes("", DL_Codes::black, -1, "CONTINUOUS", 1.0));
	}
	for (const auto& layer : doc.layers) {
		if (layer.name == "0") {
			continue;
		}
		// Layer table entries need a concrete ACI; BYLAYER only means something
		// on an entity.
		dxf.writeLayer(*dw, DL_LayerData(layer.name, layer.visible ? 0 : 1, !layer.visible),
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
	std::vector<PendingImageWrite> images;
	for (const auto& layer : doc.layers) {
		for (const auto& entity : layer.entities) {
			int width = -1;
			if (entity.strokeWidthMm > 0.f) {
				width = std::max(1, static_cast<int>(std::lround(entity.strokeWidthMm * 100.f)));
			}
			const DL_Attributes attrib(layer.name.empty() ? "0" : layer.name, DL_Codes::bylayer,
									   width, "BYLAYER", 1.0);
			writeEntity(dxf, *dw, entity, attrib, images);
		}
	}
	dw->sectionEnd();

	dxf.writeObjects(*dw);
	for (const auto& image : images) {
		dxf.writeImageDef(*dw, static_cast<int>(image.handle), image.data);
	}
	dxf.writeObjectsEnd(*dw);
	dw->dxfEOF();
	dw->close();
	delete dw;
	return true;
}

} // namespace dxf
} // namespace rigkit
