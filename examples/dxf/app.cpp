#include "app.h"

#include "core/RigKitEngine.h"
#include "core/pack/MPack.h"
#include "core/util/AppPaths.h"
#include "packs/rigComponent/src/rig.h"
#include "packs/rigComponent/src/rigComponent.h"
#include "packs/rigDxf/src/DxfLoader.h"
#include "packs/rigDxf/src/rigDxf.h"
#include "packs/rigPlotComponent/src/CPaths.h"
#include "packs/rigPlotComponent/src/ParsedPathDoc.h"
#include "packs/rigPlotComponent/src/PathFlatten.h"
#include "packs/rigPlotComponent/src/rigPlotComponent.h"
#include "packs/rigSystems/src/rigSystems.h"

#include <fstream>

namespace {
void presentBag(rigkit::MEcs& ecs, const rigkit::ecs::CPaths& bag, const char* name,
				 float scale = 2.5f, float ox = 40.f, float oy = 40.f, float r = 0.25f,
				 float g = 0.75f, float b = 0.95f) {
	std::vector<glm::vec3> positions;
	std::vector<uint32_t> indices;
	for (const auto& path : bag.paths) {
		const auto poly = rigkit::plot::flattenPath(path, 0.5f);
		if (poly.size() < 2) {
			continue;
		}
		const uint32_t base = static_cast<uint32_t>(positions.size());
		for (size_t i = 0; i < poly.size(); ++i) {
			positions.emplace_back(poly[i].x * scale + ox, poly[i].y * scale + oy, 0.f);
			if (i + 1 < poly.size()) {
				indices.push_back(base + static_cast<uint32_t>(i));
				indices.push_back(base + static_cast<uint32_t>(i + 1));
			}
		}
	}
	if (positions.size() < 2) {
		return;
	}
	if (auto e = ecs.findEntity(name); e != entt::null) {
		ecs.destroyEntity(e);
	}
	rig::makeMesh(ecs, std::move(positions), std::move(indices), rigkit::ecs::CMesh::Mode::Lines,
				  rig::fill(r, g, b), name);
}
void bagFromParsed(const rigkit::plot::ParsedPathDoc& doc, rigkit::ecs::CPaths& bag) {
	for (const auto& layer : doc.layers) {
		for (const auto& p : layer.paths.paths) {
			bag.paths.push_back(p);
		}
	}
}
} // namespace

void DxfHeroApp::setup() {
	spdlog::info("dxf - rigDxf LINE load -> mesh");
	m_engine->setClearColor(0.11f, 0.12f, 0.15f, 1.f);
	auto* packs = m_engine->getPackManager();
	if (!packs) {
		return;
	}
	packs->registerPack(std::make_shared<rigkit::rigComponent>());
	packs->registerPack(std::make_shared<rigkit::rigSystems>());
	packs->registerPack(std::make_shared<rigkit::rigPlotComponent>());
	packs->registerPack(std::make_shared<rigkit::rigDxf>());
	packs->initAll();
	packs->setupAll();
	auto* ecs = m_engine->getECSManager();
	if (!ecs) {
		return;
	}
	const std::string path = AppPaths::joinPath(AppPaths::getDataDir(), "demo.dxf");
	{
		std::ofstream out(path, std::ios::binary);
		out << "0\nSECTION\n2\nENTITIES\n"
			   "0\nLINE\n8\nDemo\n10\n10.0\n20\n10.0\n11\n80.0\n21\n10.0\n"
			   "0\nLINE\n8\nDemo\n10\n80.0\n20\n10.0\n11\n80.0\n21\n60.0\n"
			   "0\nLINE\n8\nDemo\n10\n80.0\n20\n60.0\n11\n10.0\n21\n60.0\n"
			   "0\nLINE\n8\nDemo\n10\n10.0\n20\n60.0\n11\n10.0\n21\n10.0\n"
			   "0\nENDSEC\n0\nEOF\n";
	}
	rigkit::plot::ParsedPathDoc doc;
	if (!rigkit::dxf::loadFile(path, doc) || doc.empty()) {
		spdlog::error("dxf - load failed");
		return;
	}
	rigkit::ecs::CPaths bag;
	bagFromParsed(doc, bag);
	presentBag(*ecs, bag, "dxf-preview", 6.f, 100.f, 100.f, 0.4f, 0.9f, 0.5f);
}
