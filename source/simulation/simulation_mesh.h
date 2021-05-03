#pragma once

#include "common/common.h"
#include <variant>
#include "common/clone_helper.h"
#include "simulation/air_mesh.h"

#include <igl/serialize.h>

namespace ruffles::simulation {

using std::holds_alternative;
using std::get_if;

class SimulationMesh {
public:
	// vertices are fixed(Vector2) or movable(int index into x)
	struct Vertex : std::variant<Vector2, int> {
		real width = 1.0;
		real mass = 0.;
		vector<real> z = {0., 1.};

		Vertex() = delete;
		Vertex(int ix) : std::variant<Vector2, int>(ix) {}
		Vertex(Vector2 pos) : std::variant<Vector2, int>(pos) {}
		bool fixed() {
			return get_if<Vector2>(this) != nullptr;
		}
	};

	struct Segment {
		listref<Vertex> start;
		listref<Vertex> end;
		real length;
		Segment(listref<Vertex> start, listref<Vertex> end, real length)
			: start(start), end(end), length(length)
		{}
	};


	int dof() const;
	VectorX x;
	VectorX m;

	list<Vertex> vertices;
	list<Segment> segments;
	vector<array<listref<Segment>, 2>> connection_bends;

	vector<pair<listref<Vertex>, real>> extra_mass;
	vector<pair<listref<Vertex>, Vector2>> external_forces;

	AirMesh air_mesh = AirMesh();

	real k_global = 10000.0; // global energy factor
	real k_bend = 14500.0; // 80g
	real density = 0.080; // 80 g
	real lambda_membrane = 5e5; // ???
	real lambda_air_mesh = 1e4; // ???
	Vector2 gravity = Vector2(0.,-981.);

	Vector2 lb = Vector2(-infinity, 0.);
	Vector2 ub = Vector2(infinity, infinity);

	static SimulationMesh generate_horizontal_strip(real length, real h);

	real energy(const VectorX &x, VectorX *grad) const;
	Vector2 get_vertex_position(Vertex &v) const;

	listref<Vertex> push_vertex(Vector2 position, bool fixed = false);
	listref<Segment> push_segment(listref<Vertex> a, listref<Vertex> b, real length);
	listref<Segment> insert_segment(listref<Vertex> a, listref<Vertex> b, real length, listref<Segment> position);
	array<listref<Segment>,2> split_segment(listref<Segment> seg);

	void generate_air_mesh();
	bool relax_air_mesh();

	bool consistent_lengths() const;
	void perturb(real epsilon);

	void interpolate_missing_z();
	

	/// Remove vertices that are not referenced by any segment
	/// Also remove unneeded degrees of freedom from unused entries of x
	void cleanup();
	void update_vertex_mass();

	void verify();

	real total_mass() const;

	template<typename Tr>
	SimulationMesh clone(Tr &tr) { // const
		SimulationMesh res;
		res.x = x;
		res.k_bend = k_bend;
		res.lambda_membrane = lambda_membrane;
		res.lambda_air_mesh = lambda_air_mesh;
		res.gravity = gravity;
		res.lb = lb;
		res.ub = ub;

		tr.transform(vertices.begin(), vertices.end(), res.vertices, [&](Vertex x){return x;});
		tr.transform(segments.begin(), segments.end(), res.segments, [&](Segment seg){
			return Segment(tr(seg.start), tr(seg.end), seg.length);
		});
		std::transform(connection_bends.begin(), connection_bends.end(), std::back_inserter(res.connection_bends), [&](array<listref<Segment>, 2> x) {
			return array<listref<Segment>, 2>({tr(x[0]), tr(x[1])});
		});
		return res;
	}
};
std::ostream &operator<<(std::ostream &, const SimulationMesh::Vertex &);
std::ostream &operator<<(std::ostream &, const SimulationMesh::Segment &);

}
