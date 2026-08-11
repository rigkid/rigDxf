#pragma once

#include "core/pack/IPack.h"

namespace rigkit {

/**
 * @brief DXF import pack — fills ParsedPathDoc / CPaths.
 * @details Depends on rigPlotComponent.
 */
class rigDxf : public IPack {
  public:
	rigDxf();
	bool init() override;
	void setup() override;
};

} // namespace rigkit
