#pragma once
#include "core/U_core.h"
#include "core/util/View2D.h"
#include "packs/rigDxf/src/DxfDocument.h"

#include <map>
#include <string>
#include <vector>

class DxfHeroApp : public rigkit::IApp {
  public:
	DxfHeroApp() {
		window().width = 1100;
		window().height = 700;
		window().title = "rigDxf - DXF preview";
	}
	void setup() override;
	void update(float) override {}
	void draw() override {}

	rigkit::dxf::Document& doc() { return m_doc; }
	const rigkit::dxf::Document& doc() const { return m_doc; }
	rigkit::View2D& view() { return m_view; }
	std::map<std::string, bool>& layerVisible() { return m_layerVisible; }
	bool& showGrid() { return m_showGrid; }
	bool flipped() const { return m_flipped; }
	bool arcsComplemented() const { return m_arcsComplemented; }
	bool curvesPromoted() const { return m_curvesPromoted; }
	const std::string& filename() const { return m_filename; }

	void reloadPresent();
	void fitView();
	void toggleFlipY();
	void toggleComplementArcs();
	void togglePromoteCurves();
	/** @brief Load a DXF path, apply view flags, refit layers/view. */
	void openDxf(const std::string& path);

  private:
	/** @brief Rebuild m_doc from m_source with flip / complement / promote flags. */
	void applyFlags();

	rigkit::dxf::Document m_source;
	rigkit::dxf::Document m_doc;
	rigkit::View2D m_view;
	std::map<std::string, bool> m_layerVisible;
	std::string m_filename;
	std::string m_sourcePath;
	bool m_showGrid = true;
	bool m_flipped = true;
	bool m_arcsComplemented = false;
	bool m_curvesPromoted = false;
};

