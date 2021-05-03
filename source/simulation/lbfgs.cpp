#include "simulation/lbfgs.h"

namespace ruffles::simulation {

LBFGS::LBFGS(const SimulationMesh &mesh, LBFGSpp::LBFGSBParam<real> param) :
	solver((*new LBFGSpp::LBFGSBParam<real>(param)))
{
	reset(mesh);
}

void LBFGS::reset(const SimulationMesh &) {
}

bool LBFGS::step(SimulationMesh &mesh) {
	VectorX lb = mesh.lb.replicate(mesh.dof()/2, 1);
	VectorX ub = mesh.ub.replicate(mesh.dof()/2, 1);

	auto f = [&] (const VectorX &x, VectorX &grad) -> real {
		grad.setZero();
		real e = mesh.energy(x, &grad);
		return e;
	};

	bool lbfgs_converged = false;
	try {
		int nsteps = solver.minimize(f, mesh.x, energy, lb, ub);
		(void)nsteps;
	} catch(std::runtime_error &e) {
		dbg(e.what());
		lbfgs_converged = true;
	}
	if (mesh.relax_air_mesh()) {
		// air mesh changed, run again
		return false;
	} else {
		return lbfgs_converged;
	}
}

void LBFGS::menu_callback() {}

}
