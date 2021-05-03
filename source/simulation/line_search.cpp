#include "simulation/line_search.h"

namespace ruffles::simulation {

LineSearch::LineSearch(const SimulationMesh &mesh)
{
	reset(mesh);
}

void LineSearch::reset(const SimulationMesh &) {
	step_size = 0.1;
}

bool LineSearch::step(SimulationMesh &mesh) {
	auto f = [&] (const VectorX &x, VectorX &grad) -> real {
		grad.setZero();
		real e = mesh.energy(x, &grad);
		return e;
	};

	VectorX dir(mesh.x.size());

	real f0 = f(mesh.x, dir);

	dir *= -1;
	dir.normalize();

	int iterations = 0;
	while (iterations < 100) {
		VectorX x = mesh.x + step_size * dir;
		real f = mesh.energy(x, nullptr);
		dbg(f);
		dbg(f0);
		if (f < f0) {
			mesh.x = x;
			break;
		}
		step_size *= 0.5;
		dbg(step_size);
		iterations++;
	}
	if (iterations == 0) {
		step_size *= 1.1;
		dbg(step_size);
	}
	dbg(iterations);

	if (mesh.relax_air_mesh()) {
		// air mesh changed, run again
		return false;
	} else {
		return dir.norm() < 1e-5;
	}
}

void LineSearch::menu_callback() {}

}
