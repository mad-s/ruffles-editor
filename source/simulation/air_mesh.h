#pragma once

#include "common/common.h"
#include "common/cgal_util.h"

#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>

namespace ruffles::simulation {

class SimulationMesh;

typedef CGAL::Triangulation_data_structure_2<
		CGAL::Triangulation_vertex_base_with_info_2<int, K>,
		CGAL::Constrained_triangulation_face_base_2<K>
> TriangulationDataStructure;

typedef CGAL::Triangulation_2<K, TriangulationDataStructure> Triangulation;
typedef CGAL::Constrained_Delaunay_triangulation_2<K,TriangulationDataStructure> CDT;

class AirMesh {
public:
	CDT cdt;
	vector<std::variant<Vector2, int>> vertices;

	AirMesh();
	AirMesh(SimulationMesh &mesh);


	void clear();
	bool empty() const;

	bool relax(const VectorX &x);

	real penalty(real k, const VectorX &x, VectorX *grad) const;
	real barrier(real k, const VectorX &x, VectorX *grad) const;

	void project(VectorX &x) const;
};

}
