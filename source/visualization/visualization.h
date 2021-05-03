#pragma once

#include "common/common.h"
#include "simulation/simulation_mesh.h"

namespace ruffles::visualization {

class Visualization {
public:
	Matrix<real, -1, 3> V;
	int nv = 0;
	Matrix<int, -1, 3>  F;
	Matrix<real, -1, 3> C;
	int nf = 0;

	Visualization() {
		clear();
	}

	int push_vertex(real x, real y, real z);
	int push_vertex(Vector3 pos);
	int push_face(int a, int b, int c, Vector3 color=Vector3(0.8,0.8,0.8));

	void draw_simulation_mesh(ruffles::simulation::SimulationMesh &mesh);

	void clear();

	void compress();

};

}

