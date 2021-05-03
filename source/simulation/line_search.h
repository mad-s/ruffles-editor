#pragma once

#include "common/common.h"

#include "simulation/simulator.h"

#include <LBFGSB.h>

namespace ruffles::simulation {

class LineSearch : public Simulator {
public:
	LineSearch(const SimulationMesh &mesh);

	virtual void reset(const SimulationMesh &mesh);

	virtual bool step(SimulationMesh &mesh) override;

	virtual void menu_callback() override;

	real step_size = 0.1;
};

}
