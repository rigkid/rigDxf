#include "rigDxf.h"
#include <spdlog/spdlog.h>
#include "core/pack/PackRegistry.h"

namespace rigkit {

rigDxf::rigDxf() : IPack("rigDxf") {
	

}

bool rigDxf::init() {
	spdlog::info("[rigDxf] init");
	return true;
}

void rigDxf::setup() {
	spdlog::info("[rigDxf] setup (LINE / LWPOLYLINE loader ready)");
}

} // namespace rigkit

namespace {
struct rigDxfRegistrar {
	rigDxfRegistrar() {
		rigkit::PackRegistry::instance().addFactory("rigDxf", []() {
			return std::shared_ptr<rigkit::IPack>(std::make_shared<rigkit::rigDxf>());
		});
	}
};
static rigDxfRegistrar rigDxf_auto_reg;
} // namespace
