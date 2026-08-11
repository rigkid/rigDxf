#pragma once
#include "core/U_core.h"

class DxfHeroApp : public rigkit::IApp {
  public:
	DxfHeroApp() {
		window().width = 800;
		window().height = 600;
		window().title = "rigDxf - dxf";
	}
	void setup() override;
	void update(float) override {}
	void draw() override {}

  private:
};
