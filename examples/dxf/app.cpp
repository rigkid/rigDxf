#include "app.h"

#include "core/pack/MPack.h"
#include "core/RigKitEngine.h"
#include "core/util/AppPaths.h"
#include "packs/rigComponent/src/PrimitiveVertices.h"
#include "packs/rigComponent/src/rig.h"
#include "packs/rigComponent/src/rigComponent.h"
#include "packs/rigDxf/src/DxfLoader.h"
#include "packs/rigDxf/src/rigDxf.h"
#include "packs/rigImGui/src/IWindow.h"
#include "packs/rigImGui/src/MWindow.h"
#include "packs/rigImGui/src/rigImGui.h"
#include "packs/rigPlotComponent/src/PathFlatten.h"
#include "packs/rigPlotComponent/src/rigPlotComponent.h"
#include "packs/rigSystems/src/rigSystems.h"
#include "imgui.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <spdlog/spdlog.h>
#include <string>
#include <vector>

namespace {

ImU32 toCol32(const glm::vec4& c, float alpha = 1.f) {
	return IM_COL32(int(c.r * 255.f), int(c.g * 255.f), int(c.b * 255.f), int(c.a * alpha * 255.f));
}

void appendPolyline(ImDrawList* dl, rigkit::View2D& view, const std::vector<glm::vec2>& pts,
					ImU32 col, float thickness, bool closed) {
	if (pts.size() < 2) {
		return;
	}
	for (size_t i = 1; i < pts.size(); ++i) {
		const ImVec2 a = {view.toScreen(pts[i - 1].x, pts[i - 1].y).x,
						  view.toScreen(pts[i - 1].x, pts[i - 1].y).y};
		const ImVec2 b = {view.toScreen(pts[i].x, pts[i].y).x, view.toScreen(pts[i].x, pts[i].y).y};
		dl->AddLine(a, b, col, thickness);
	}
	if (closed && pts.size() > 2) {
		const ImVec2 a = {view.toScreen(pts.back().x, pts.back().y).x,
						  view.toScreen(pts.back().x, pts.back().y).y};
		const ImVec2 b = {view.toScreen(pts.front().x, pts.front().y).x,
						  view.toScreen(pts.front().x, pts.front().y).y};
		dl->AddLine(a, b, col, thickness);
	}
}

std::vector<glm::vec2> entityPolyline(const rigkit::dxf::Entity& e) {
	using Kind = rigkit::dxf::Entity::Kind;
	switch (e.kind) {
	case Kind::Path:
	case Kind::Point:
		return rigkit::plot::flattenPath(e.path, 0.25f);
	case Kind::Arc: {
		auto pts = rigkit::ecs::verticesOf(e.arc);
		if (std::fabs(e.rotationDegrees) > 1e-4f) {
			const float rad = e.rotationDegrees * 0.01745329252f;
			const float c = std::cos(rad);
			const float s = std::sin(rad);
			for (auto& p : pts) {
				const float x = p.x - e.arc.cx;
				const float y = p.y - e.arc.cy;
				p.x = e.arc.cx + c * x - s * y;
				p.y = e.arc.cy + s * x + c * y;
			}
		}
		return pts;
	}
	case Kind::Ellipse: {
		rigkit::ecs::CArc full;
		full.cx = e.ellipse.cx;
		full.cy = e.ellipse.cy;
		full.rx = e.ellipse.rx;
		full.ry = e.ellipse.ry;
		full.startAngleDegrees = 0.f;
		full.endAngleDegrees = 360.f;
		auto pts = rigkit::ecs::verticesOf(full);
		if (std::fabs(e.rotationDegrees) > 1e-4f) {
			const float rad = e.rotationDegrees * 0.01745329252f;
			const float c = std::cos(rad);
			const float s = std::sin(rad);
			for (auto& p : pts) {
				const float x = p.x - e.ellipse.cx;
				const float y = p.y - e.ellipse.cy;
				p.x = e.ellipse.cx + c * x - s * y;
				p.y = e.ellipse.cy + s * x + c * y;
			}
		}
		return pts;
	}
	case Kind::Spline:
		return rigkit::ecs::verticesOf(e.spline);
	}
	return {};
}

class DxfControlWindow : public rigkit::IWindow {
  public:
	explicit DxfControlWindow(DxfHeroApp* app)
		: IWindow("DXF", ImGuiWindowFlags_NoCollapse), m_app(app) {}

  protected:
	void renderContents() override {
		if (!m_app) {
			return;
		}
		auto& doc = m_app->doc();
		ImGui::TextWrapped("%s", m_app->filename().c_str());
		if (!doc.empty()) {
			ImGui::TextDisabled("%.1f x %.1f   %d layers", doc.contentW, doc.contentH,
								(int)doc.layers.size());
			int entities = 0;
			for (const auto& layer : doc.layers) {
				entities += (int)layer.entities.size();
			}
			ImGui::TextDisabled("%d entities", entities);
		}
		for (const auto& err : doc.errors) {
			ImGui::TextColored(ImVec4(1.f, 0.6f, 0.3f, 1.f), "%s", err.c_str());
		}

		ImGui::Separator();
		if (ImGui::Button("Fit View", ImVec2(-1, 0))) {
			m_app->fitView();
		}
		if (ImGui::Button(m_app->flipped() ? "Mirror Y [on]" : "Mirror Y [off]", ImVec2(-1, 0))) {
			m_app->toggleFlipY();
		}
		if (ImGui::Button(m_app->arcsComplemented() ? "Complement Arcs [on]" : "Complement Arcs [off]",
						  ImVec2(-1, 0))) {
			m_app->toggleComplementArcs();
		}
		if (ImGui::Button(m_app->curvesPromoted() ? "Promote Curves [on]" : "Promote Curves [off]",
						  ImVec2(-1, 0))) {
			m_app->togglePromoteCurves();
		}
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip(
				"Opt-in faceted circle/arc recovery (off by default, same as ofxDxf).\n"
				"Bulge arcs already become Arc entities on load.\n"
				"Try the Faceted layer 24-gon.");
		}
		ImGui::Checkbox("Show grid", &m_app->showGrid());

		ImGui::Separator();
		ImGui::TextUnformatted("Layers");
		if (ImGui::SmallButton("All")) {
			for (auto& [n, v] : m_app->layerVisible()) {
				(void)n;
				v = true;
			}
		}
		ImGui::SameLine();
		if (ImGui::SmallButton("None")) {
			for (auto& [n, v] : m_app->layerVisible()) {
				(void)n;
				v = false;
			}
		}
		for (const auto& layer : doc.layers) {
			bool& vis = m_app->layerVisible()[layer.name];
			ImGui::PushID(layer.name.c_str());
			ImGui::Checkbox("##vis", &vis);
			ImGui::SameLine();
			ImGui::ColorButton("##c", ImVec4(layer.color.r, layer.color.g, layer.color.b, 1.f),
							   ImGuiColorEditFlags_NoTooltip | ImGuiColorEditFlags_NoDragDrop,
							   ImVec2(14, 14));
			ImGui::SameLine();
			ImGui::Text("%s (%d)", layer.name.c_str(), (int)layer.entities.size());
			ImGui::PopID();
		}

		ImGui::Separator();
		ImGui::TextDisabled("Scroll=zoom  LMB=pan  F=fit");
		ImGui::TextDisabled("M=mirror  C=complement  P=promote");
	}

	DxfHeroApp* m_app = nullptr;
};

class DxfViewportWindow : public rigkit::IWindow {
  public:
	explicit DxfViewportWindow(DxfHeroApp* app)
		: IWindow("Preview", ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar |
								 ImGuiWindowFlags_NoScrollWithMouse),
		  m_app(app) {}

  protected:
	void renderContents() override {
		if (!m_app) {
			return;
		}
		auto& view = m_app->view();
		const ImVec2 canvasPos = ImGui::GetCursorScreenPos();
		const ImVec2 avail = ImGui::GetContentRegionAvail();
		const float w = std::max(80.f, avail.x);
		const float h = std::max(80.f, avail.y);

		ImGui::InvisibleButton("dxf_canvas", ImVec2(w, h),
							   ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonMiddle);
		const bool hovered = ImGui::IsItemHovered();
		ImGuiIO& io = ImGui::GetIO();

		view.canvasOrigin = {canvasPos.x, canvasPos.y};
		view.canvasW = w;
		view.canvasH = h;
		view.contentSize = {std::max(1.f, m_app->doc().contentW),
							std::max(1.f, m_app->doc().contentH)};
		view.hovered = hovered;
		view.updateDerived();

		if (hovered && io.MouseWheel != 0.f) {
			view.applyScrollZoom(io.MouseWheel, io.MousePos.x, io.MousePos.y);
		}
		if (ImGui::IsItemActive() &&
			(ImGui::IsMouseDragging(ImGuiMouseButton_Left) ||
			 ImGui::IsMouseDragging(ImGuiMouseButton_Middle))) {
			view.applyPanDelta(io.MouseDelta.x, io.MouseDelta.y);
		}
		if (hovered && !io.WantCaptureKeyboard) {
			if (ImGui::IsKeyPressed(ImGuiKey_F)) {
				m_app->fitView();
			}
			if (ImGui::IsKeyPressed(ImGuiKey_M)) {
				m_app->toggleFlipY();
			}
			if (ImGui::IsKeyPressed(ImGuiKey_C)) {
				m_app->toggleComplementArcs();
			}
			if (ImGui::IsKeyPressed(ImGuiKey_P)) {
				m_app->togglePromoteCurves();
			}
			if (ImGui::IsKeyPressed(ImGuiKey_G)) {
				m_app->showGrid() = !m_app->showGrid();
			}
		}

		ImDrawList* dl = ImGui::GetWindowDrawList();
		dl->AddRectFilled(canvasPos, ImVec2(canvasPos.x + w, canvasPos.y + h),
						  IM_COL32(28, 28, 35, 255));
		dl->PushClipRect(canvasPos, ImVec2(canvasPos.x + w, canvasPos.y + h), true);

		if (m_app->showGrid()) {
			drawGrid(dl, view);
		}

		const float ox = m_app->doc().minX;
		const float oy = m_app->doc().minY;
		for (const auto& layer : m_app->doc().layers) {
			auto it = m_app->layerVisible().find(layer.name);
			if (it != m_app->layerVisible().end() && !it->second) {
				continue;
			}
			const ImU32 col = toCol32(layer.color);
			for (const auto& e : layer.entities) {
				auto pts = entityPolyline(e);
				if (pts.empty()) {
					continue;
				}
				for (auto& p : pts) {
					p.x -= ox;
					p.y -= oy;
				}
				if (e.kind == rigkit::dxf::Entity::Kind::Point && !pts.empty()) {
					const ImVec2 s = {view.toScreen(pts[0].x, pts[0].y).x,
									  view.toScreen(pts[0].x, pts[0].y).y};
					dl->AddCircleFilled(s, 3.f, col);
					continue;
				}
				const bool closed = e.kind == rigkit::dxf::Entity::Kind::Ellipse ||
									(e.kind == rigkit::dxf::Entity::Kind::Arc && e.arc.pie);
				appendPolyline(dl, view, pts, col, 1.5f, closed);
			}
		}

		dl->PopClipRect();
	}

  private:
	void drawGrid(ImDrawList* dl, rigkit::View2D& view) {
		float step = 10.f;
		while (step * view.zoomAbs < 12.f && step < 1e6f) {
			step *= 10.f;
		}
		while (step * view.zoomAbs > 120.f && step > 1e-3f) {
			step /= 10.f;
		}
		const float left = view.toContent(view.canvasOrigin.x, view.canvasOrigin.y).x;
		const float top = view.toContent(view.canvasOrigin.x, view.canvasOrigin.y).y;
		const float right =
			view.toContent(view.canvasOrigin.x + view.canvasW, view.canvasOrigin.y + view.canvasH).x;
		const float bottom =
			view.toContent(view.canvasOrigin.x + view.canvasW, view.canvasOrigin.y + view.canvasH).y;
		const float x0 = std::floor(left / step) * step;
		const float y0 = std::floor(top / step) * step;
		int count = 0;
		for (float x = x0; x <= right && count < 250; x += step, ++count) {
			const bool main = std::fmod(std::fabs(x), step * 10.f) < step * 0.01f;
			const ImU32 col = main ? IM_COL32(50, 50, 65, 255) : IM_COL32(38, 38, 50, 255);
			dl->AddLine(ImVec2(view.toScreen(x, top).x, view.toScreen(x, top).y),
						ImVec2(view.toScreen(x, bottom).x, view.toScreen(x, bottom).y), col);
		}
		count = 0;
		for (float y = y0; y <= bottom && count < 250; y += step, ++count) {
			const bool main = std::fmod(std::fabs(y), step * 10.f) < step * 0.01f;
			const ImU32 col = main ? IM_COL32(50, 50, 65, 255) : IM_COL32(38, 38, 50, 255);
			dl->AddLine(ImVec2(view.toScreen(left, y).x, view.toScreen(left, y).y),
						ImVec2(view.toScreen(right, y).x, view.toScreen(right, y).y), col);
		}
	}

	DxfHeroApp* m_app = nullptr;
};

} // namespace

void DxfHeroApp::fitView() {
	m_view.contentSize = {std::max(1.f, m_doc.contentW), std::max(1.f, m_doc.contentH)};
	m_view.fitToCanvas();
}

void DxfHeroApp::reloadPresent() {
	m_layerVisible.clear();
	for (const auto& layer : m_doc.layers) {
		std::string lower = layer.name;
		std::transform(lower.begin(), lower.end(), lower.begin(),
					   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
		m_layerVisible[layer.name] = (lower != "defpoints");
	}
	fitView();
}

void DxfHeroApp::applyFlags() {
	m_doc = m_source;
	if (m_flipped) {
		rigkit::dxf::flipY(m_doc);
	}
	if (m_arcsComplemented) {
		rigkit::dxf::complementArcs(m_doc);
	}
	if (m_curvesPromoted) {
		rigkit::dxf::CurvePromotionSettings settings;
		settings.promoteFacetedCircles = true;
		settings.promoteFacetedArcs = true;
		const auto result = rigkit::dxf::promoteCurves(m_doc, settings);
		spdlog::info("dxf - promoteCurves circles={} arcs={} examined={}", result.circlesPromoted,
					 result.arcsPromoted, result.entitiesExamined);
	}
	reloadPresent();
}

void DxfHeroApp::toggleFlipY() {
	if (m_source.empty()) {
		return;
	}
	m_flipped = !m_flipped;
	applyFlags();
}

void DxfHeroApp::toggleComplementArcs() {
	if (m_source.empty()) {
		return;
	}
	m_arcsComplemented = !m_arcsComplemented;
	applyFlags();
}

void DxfHeroApp::togglePromoteCurves() {
	if (m_source.empty()) {
		return;
	}
	m_curvesPromoted = !m_curvesPromoted;
	applyFlags();
}

void DxfHeroApp::openDxf(const std::string& path) {
	rigkit::dxf::Document next;
	if (!rigkit::dxf::loadFile(path, next) || next.empty()) {
		spdlog::error("dxf - failed to load {}", path);
		return;
	}
	m_source = std::move(next);
	m_sourcePath = path;
	m_filename = path;
	try {
		m_filename = std::filesystem::path(path).filename().string();
	} catch (...) {
	}
	m_flipped = true;
	m_arcsComplemented = false;
	m_curvesPromoted = false;
	applyFlags();

	int entities = 0;
	for (const auto& layer : m_doc.layers) {
		entities += static_cast<int>(layer.entities.size());
		spdlog::info("dxf - layer '{}' entities={}", layer.name, layer.entities.size());
	}
	spdlog::info("dxf - loaded {} layers={} entities={}", m_filename, m_doc.layers.size(),
				 entities);
	for (const auto& err : m_doc.errors) {
		spdlog::warn("dxf - {}", err);
	}
	if (auto* ui = m_engine ? m_engine->getUiManager() : nullptr) {
		ui->noteRecentFile(path);
	}
}

void DxfHeroApp::setup() {
	spdlog::info("dxf - typed Document preview");
	m_engine->setClearColor(0.11f, 0.12f, 0.15f, 1.f);
	auto* packs = m_engine->getPackManager();
	if (!packs) {
		return;
	}
	packs->registerPack(std::make_shared<rigkit::rigComponent>());
	packs->registerPack(std::make_shared<rigkit::rigSystems>());
	packs->registerPack(std::make_shared<rigkit::rigPlotComponent>());
	packs->registerPack(std::make_shared<rigkit::rigDxf>());
	packs->registerPack(std::make_shared<rigkit::rigImGui>());
	packs->initAll();
	packs->setupAll();

	if (auto* ui = m_engine->getUiManager()) {
		ui->registerFileAction(
			"Open...",
			[this, ui]() {
				ui->openFileDialog("Open DXF", {".dxf"},
								   [this](const std::string& path) { openDxf(path); });
			},
			"Ctrl+O");
		ui->setRecentFileOpenHandler([this](const std::string& path) { openDxf(path); });
		if (auto* wm = ui->getWindowManager()) {
			wm->createWindow<DxfControlWindow>(this);
			wm->createWindow<DxfViewportWindow>(this);
			wm->showWindow("DXF");
			wm->showWindow("Preview");
		}
	}

	const std::string path = AppPaths::joinPath(AppPaths::getDataDir(), "demo.dxf");
	openDxf(path);
}

