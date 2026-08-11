#include "DxfLoader.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <string>
#include <vector>

namespace rigkit {
namespace dxf {
namespace {

bool readPair(std::istream& in, int& code, std::string& value) {
	std::string codeLine;
	if (!std::getline(in, codeLine)) {
		return false;
	}
	if (!std::getline(in, value)) {
		return false;
	}
	if (!value.empty() && value.back() == '\r') {
		value.pop_back();
	}
	if (!codeLine.empty() && codeLine.back() == '\r') {
		codeLine.pop_back();
	}
	try {
		code = std::stoi(codeLine);
	} catch (...) {
		return false;
	}
	return true;
}

void recomputeBounds(plot::ParsedPathDoc& doc) {
	float minX = 0, minY = 0, maxX = 0, maxY = 0;
	bool any = false;
	for (const auto& layer : doc.layers) {
		for (const auto& p : layer.paths.paths) {
			for (const auto& c : p.commands) {
				if (c.type == ecs::CPath::Cmd::Close) {
					continue;
				}
				if (!any) {
					minX = maxX = c.p.x;
					minY = maxY = c.p.y;
					any = true;
				} else {
					minX = std::min(minX, c.p.x);
					minY = std::min(minY, c.p.y);
					maxX = std::max(maxX, c.p.x);
					maxY = std::max(maxY, c.p.y);
				}
			}
		}
	}
	doc.minX = minX;
	doc.minY = minY;
	doc.contentW = any ? (maxX - minX) : 0.f;
	doc.contentH = any ? (maxY - minY) : 0.f;
}

plot::PathSourceLayer& layerByName(std::map<std::string, size_t>& index,
								   plot::ParsedPathDoc& doc, const std::string& name) {
	const std::string key = name.empty() ? "0" : name;
	auto it = index.find(key);
	if (it != index.end()) {
		return doc.layers[it->second];
	}
	plot::PathSourceLayer layer;
	layer.name = key;
	doc.layers.push_back(std::move(layer));
	index[key] = doc.layers.size() - 1;
	return doc.layers.back();
}

ecs::CPath makeArcPath(float cx, float cy, float r, float startDeg, float endDeg, bool closed) {
	ecs::CPath path;
	if (r <= 0.f) {
		return path;
	}
	// DXF angles: degrees CCW from +X.
	float a0 = startDeg;
	float a1 = endDeg;
	while (a1 < a0) {
		a1 += 360.f;
	}
	const float sweep = a1 - a0;
	const int segments = std::max(8, static_cast<int>(std::ceil(sweep / 6.f)));
	using Cmd = ecs::CPath::Cmd;
	for (int i = 0; i <= segments; ++i) {
		const float t = a0 + sweep * (static_cast<float>(i) / static_cast<float>(segments));
		const float rad = t * 0.01745329252f;
		const glm::vec2 p{cx + r * std::cos(rad), cy + r * std::sin(rad)};
		path.commands.push_back({i == 0 ? Cmd::MoveTo : Cmd::LineTo, p, {}, {}});
	}
	if (closed) {
		path.commands.push_back({Cmd::Close, {}, {}, {}});
	}
	return path;
}

} // namespace

bool loadFile(const std::string& path, plot::ParsedPathDoc& out) {
	out = {};
	std::ifstream in(path);
	if (!in) {
		return false;
	}

	std::map<std::string, size_t> layerIndex;
	std::string currentLayer = "0";

	int code = 0;
	std::string value;

	enum class Mode { None, Line, LwPoly, Circle, Arc };
	Mode mode = Mode::None;

	float x1 = 0, y1 = 0, x2 = 0, y2 = 0;
	bool haveX1 = false, haveY1 = false, haveX2 = false, haveY2 = false;

	std::vector<glm::vec2> polyPts;
	bool polyClosed = false;
	float polyX = 0.f;
	bool havePolyX = false;

	float cx = 0, cy = 0, radius = 0;
	float startAngle = 0, endAngle = 360;
	bool haveCx = false, haveCy = false, haveR = false;

	auto target = [&]() -> plot::PathSourceLayer& {
		return layerByName(layerIndex, out, currentLayer);
	};

	auto flushLine = [&]() {
		if (mode == Mode::Line && haveX1 && haveY1 && haveX2 && haveY2) {
			ecs::CPath path;
			using Cmd = ecs::CPath::Cmd;
			path.commands.push_back({Cmd::MoveTo, {x1, y1}, {}, {}});
			path.commands.push_back({Cmd::LineTo, {x2, y2}, {}, {}});
			target().paths.paths.push_back(std::move(path));
		}
		haveX1 = haveY1 = haveX2 = haveY2 = false;
	};

	auto flushPoly = [&]() {
		if (mode == Mode::LwPoly && polyPts.size() >= 2) {
			ecs::CPath path;
			using Cmd = ecs::CPath::Cmd;
			path.commands.push_back({Cmd::MoveTo, polyPts.front(), {}, {}});
			for (size_t i = 1; i < polyPts.size(); ++i) {
				path.commands.push_back({Cmd::LineTo, polyPts[i], {}, {}});
			}
			if (polyClosed) {
				path.commands.push_back({Cmd::Close, {}, {}, {}});
			}
			target().paths.paths.push_back(std::move(path));
		}
		polyPts.clear();
		polyClosed = false;
		havePolyX = false;
	};

	auto flushCircle = [&]() {
		if (mode == Mode::Circle && haveCx && haveCy && haveR) {
			target().paths.paths.push_back(makeArcPath(cx, cy, radius, 0.f, 360.f, true));
		}
		haveCx = haveCy = haveR = false;
		startAngle = 0.f;
		endAngle = 360.f;
	};

	auto flushArc = [&]() {
		if (mode == Mode::Arc && haveCx && haveCy && haveR) {
			target().paths.paths.push_back(makeArcPath(cx, cy, radius, startAngle, endAngle, false));
		}
		haveCx = haveCy = haveR = false;
		startAngle = 0.f;
		endAngle = 360.f;
	};

	auto flush = [&]() {
		flushLine();
		flushPoly();
		flushCircle();
		flushArc();
		mode = Mode::None;
	};

	while (readPair(in, code, value)) {
		if (code == 8) {
			currentLayer = value.empty() ? "0" : value;
		}
		if (code == 0) {
			flush();
			if (value == "LINE") {
				mode = Mode::Line;
			} else if (value == "LWPOLYLINE") {
				mode = Mode::LwPoly;
			} else if (value == "CIRCLE") {
				mode = Mode::Circle;
			} else if (value == "ARC") {
				mode = Mode::Arc;
			}
			continue;
		}

		try {
			if (mode == Mode::Line) {
				if (code == 10) {
					x1 = std::stof(value);
					haveX1 = true;
				} else if (code == 20) {
					y1 = std::stof(value);
					haveY1 = true;
				} else if (code == 11) {
					x2 = std::stof(value);
					haveX2 = true;
				} else if (code == 21) {
					y2 = std::stof(value);
					haveY2 = true;
				}
			} else if (mode == Mode::LwPoly) {
				if (code == 70) {
					polyClosed = (std::stoi(value) & 1) != 0;
				} else if (code == 10) {
					polyX = std::stof(value);
					havePolyX = true;
				} else if (code == 20 && havePolyX) {
					polyPts.push_back({polyX, std::stof(value)});
					havePolyX = false;
				}
			} else if (mode == Mode::Circle || mode == Mode::Arc) {
				if (code == 10) {
					cx = std::stof(value);
					haveCx = true;
				} else if (code == 20) {
					cy = std::stof(value);
					haveCy = true;
				} else if (code == 40) {
					radius = std::stof(value);
					haveR = true;
				} else if (mode == Mode::Arc && code == 50) {
					startAngle = std::stof(value);
				} else if (mode == Mode::Arc && code == 51) {
					endAngle = std::stof(value);
				}
			}
		} catch (...) {
		}
	}
	flush();

	out.layers.erase(std::remove_if(out.layers.begin(), out.layers.end(),
									[](const plot::PathSourceLayer& l) { return l.paths.empty(); }),
					 out.layers.end());
	if (out.layers.empty()) {
		return false;
	}
	recomputeBounds(out);
	return true;
}

} // namespace dxf
} // namespace rigkit
