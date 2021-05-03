#include "simulation/verlet.h"

#include "common/imgui.h"

namespace ruffles::simulation {

Verlet::Verlet(const SimulationMesh &mesh) {
	reset(mesh);
}

void Verlet::reset(const SimulationMesh &mesh) {
	vel  = VectorX::Zero(mesh.dof());
	grad = VectorX::Zero(mesh.dof());
}

bool Verlet::step(SimulationMesh &mesh) {
	grad.setZero();
	VectorX &pos = mesh.x;
	energy = mesh.energy(pos, &grad);

	
	//VectorX acc = -grad.array() / mesh.m.array();
	VectorX acc = -grad;
	

	// modified verlet scheme using a single evaluation
	vel += dt * (gamma) * acc;
	pos += dt * vel + dt*dt * 0.5 * acc;
	vel += dt * (1-gamma) * acc;


	for (int i = 0; i < pos.size(); i += 2) {
		if (pos(i) < mesh.lb(0)) {
			pos(i) = mesh.lb(0);
			vel(i) = 0.;
		}
		if (pos(i) > mesh.ub(0)) {
			pos(i) = mesh.ub(0);
			vel(i) = 0.;
		}
		if (pos(i+1) < mesh.lb(1)) {
			pos(i+1) = mesh.lb(1);
			vel(i+1) = 0.;
		}
		if (pos(i+1) > mesh.ub(1)) {
			pos(i+1) = mesh.ub(1);
			vel(i+1) = 0.;
		}
	}

	vel *= damp;

	bool converged = acc.squaredNorm() + vel.squaredNorm() < pos.size() * epsilon;

	if (mesh.relax_air_mesh()) {
		return false;
	} else {
		return converged;
	}
}

void Verlet::menu_callback() {
	if (ImGui::InputReal("dt", &dt, 0.01, 0.1)) {
		dt = max(0., dt);
	}
	if (ImGui::InputReal("damp", &damp, 0.1, 1.)) {
		damp = max(0., min(1., damp));
	}
}

}
