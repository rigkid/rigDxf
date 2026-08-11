#include "DxfLoader.h"

#include <cstdio>
#include <fstream>
#include <string>

int main() {
	const char* path = "dxf_smoke_tmp.dxf";
	{
		std::ofstream out(path, std::ios::binary);
		out << "0\nSECTION\n2\nENTITIES\n"
			   "0\nLINE\n8\nOutline\n10\n0.0\n20\n0.0\n11\n10.0\n21\n0.0\n"
			   "0\nCIRCLE\n8\nOutline\n10\n5.0\n20\n5.0\n40\n2.0\n"
			   "0\nARC\n8\nOutline\n10\n5.0\n20\n5.0\n40\n3.0\n50\n0.0\n51\n90.0\n"
			   "0\nLWPOLYLINE\n8\nCurves\n90\n2\n70\n0\n10\n0.0\n20\n0.0\n42\n0.5\n10\n4.0\n20\n0.0\n"
			   "0\nENDSEC\n0\nEOF\n";
	}

	rigkit::dxf::Document doc;
	const bool ok = rigkit::dxf::loadFile(path, doc);
	std::remove(path);
	if (!ok || doc.empty()) {
		std::printf("dxf_smoke FAIL parse\n");
		return 1;
	}

	size_t entities = 0;
	size_t arcs = 0;
	size_t ellipses = 0;
	size_t paths = 0;
	for (const auto& layer : doc.layers) {
		for (const auto& e : layer.entities) {
			++entities;
			using Kind = rigkit::dxf::Entity::Kind;
			if (e.kind == Kind::Arc) {
				++arcs;
			} else if (e.kind == Kind::Ellipse) {
				++ellipses;
			} else if (e.kind == Kind::Path) {
				++paths;
			}
		}
	}

	rigkit::plot::ParsedPathDoc parsed;
	rigkit::dxf::flattenToParsed(doc, parsed);
	size_t flatPaths = 0;
	for (const auto& layer : parsed.layers) {
		flatPaths += layer.paths.paths.size();
	}

	std::printf("dxf_smoke layers=%zu entities=%zu arcs=%zu ellipses=%zu paths=%zu flat=%zu\n",
				doc.layers.size(), entities, arcs, ellipses, paths, flatPaths);
	if (entities < 3 || arcs < 1 || ellipses < 1 || flatPaths < 1) {
		std::printf("dxf_smoke FAIL counts\n");
		return 1;
	}
	return 0;
}

