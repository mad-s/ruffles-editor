#pragma once

#include "simulation/simulator.h"
#include "common/common.h"

namespace ruffles::simulation {

class Verlet : public Simulator {
public:
	Verlet(const SimulationMesh &mesh);

	virtual void reset(const SimulationMesh &mesh);

	virtual bool step(SimulationMesh &mesh) override;

	virtual void menu_callback() override;

	VectorX vel;

	real energy = std::numeric_limits<real>::infinity();
	real dt = 0.000001;
	real damp = 0.99;
	real gamma = 0.5;
	real epsilon = 1e-3;
	VectorX grad;
};

}
