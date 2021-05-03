#pragma once

#include "common/common.h"
#include "simulation/simulator.h"

#include "simulation/lbfgs.h"
#include "simulation/verlet.h"

namespace ruffles::simulation {

class Combination : public Simulator {
public:
	LBFGS lbfgs;
	bool lbfgs_converged = false;
	Verlet verlet;

	Combination(const SimulationMesh &mesh);

	virtual void reset(const SimulationMesh &mesh);

	virtual bool step(SimulationMesh &mesh) override;

	virtual void menu_callback() override;
};

}
