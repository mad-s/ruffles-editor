#include "simulation/combination.h"

namespace ruffles::simulation {

Combination::Combination(const SimulationMesh &mesh)
	: lbfgs(mesh), verlet(mesh) {
}

void Combination::reset(const SimulationMesh &mesh) {
	lbfgs_converged = false;
	lbfgs.reset(mesh);
	verlet.reset(mesh);
}

bool Combination::step(SimulationMesh &mesh) {
	if (lbfgs_converged) {
		return verlet.step(mesh);
	} else {
		lbfgs_converged = lbfgs.step(mesh);
		if (lbfgs_converged) {
			cout << "LBFGS converged!" << endl;
		}
		return false;
	}
}

void Combination::menu_callback() {
}

}
