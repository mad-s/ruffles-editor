#pragma once

#include "simulation/simulation_mesh.h"

namespace ruffles::simulation {

class Simulator {
public:
	virtual void reset(const SimulationMesh &) {
	}
	virtual bool step(SimulationMesh &) = 0;

	virtual void menu_callback() {}
};

}
