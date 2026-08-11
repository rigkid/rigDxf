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
			   "0\nENDSEC\n0\nEOF\n";
	}
	rigkit::plot::ParsedPathDoc doc;
	const bool ok = rigkit::dxf::loadFile(path, doc);
	std::remove(path);
	if (!ok || doc.empty()) {
		std::printf("dxf_smoke FAIL parse\n");
		return 1;
	}
	size_t paths = 0;
	for (const auto& layer : doc.layers) {
		paths += layer.paths.paths.size();
	}
	std::printf("dxf_smoke layers=%zu paths=%zu\n", doc.layers.size(), paths);
	return paths > 0 ? 0 : 1;
}
