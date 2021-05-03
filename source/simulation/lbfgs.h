#pragma once

#include "common/common.h"

#include "simulation/simulator.h"

#include <LBFGSB.h>

namespace ruffles::simulation {

class LBFGS : public Simulator {
public:
	LBFGS(const SimulationMesh &mesh, LBFGSpp::LBFGSBParam<real> param = LBFGSpp::LBFGSBParam<real>());

	virtual void reset(const SimulationMesh &mesh);

	virtual bool step(SimulationMesh &mesh) override;

	virtual void menu_callback() override;

	LBFGSpp::LBFGSBSolver<real> solver;
	real energy;
};

}
