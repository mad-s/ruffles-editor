#include "visualization/visualization.h"


namespace ruffles::visualization {

using ruffles::simulation::SimulationMesh;

int Visualization::push_vertex(real x, real y, real z) {
	return push_vertex(Vector3(x,y,z));
}

int Visualization::push_vertex(Vector3 pos) {
	if (V.rows() == nv) {
		int new_cap = nv == 0 ? 1 : 2*nv;
		V.conservativeResizeLike(decltype(V)::Zero(new_cap, 3));
	}

	V.row(nv) << pos.transpose();
	nv++;
	return nv - 1;
}

int Visualization::push_face(int a, int b, int c, Vector3 color) {
	if (F.rows() == nf) {
		int new_cap = nf == 0 ? 1 : 2*nf;
		F.conservativeResizeLike(decltype(F)::Zero(new_cap, 3));
		C.conservativeResizeLike(decltype(C)::Zero(new_cap, 3));
	}

	F.row(nf) << a, b, c;
	C.row(nf) = color.transpose();
	nf++;
	return nf - 1;
}

void Visualization::draw_simulation_mesh(SimulationMesh &mesh) {
}

void Visualization::clear() {
	V.resize(0,3);
	C.resize(0,3);
	F.resize(0,3);
	nv = 0;
	nf = 0;
}

void Visualization::compress() {
	V.conservativeResize(nv, 3);
	F.conservativeResize(nf, 3);
	C.conservativeResize(nf, 3);
}

}
