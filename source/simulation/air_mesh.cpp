#include "simulation/air_mesh.h"
#include "simulation/simulation_mesh.h"
#include <unordered_map>

// for listref_hash<T>
#include "common/clone_helper.h"

namespace ruffles::simulation {

AirMesh::AirMesh() {
}

AirMesh::AirMesh(SimulationMesh &mesh) {
	vector<CDT::Vertex_handle> cdt_vertices;
	std::unordered_map<listref<SimulationMesh::Vertex>, int, listref_hash<SimulationMesh::Vertex>> indices;

	int i = 0;
	for (auto it = mesh.vertices.begin(); it != mesh.vertices.end(); ++it) {
		Vector2 x = mesh.get_vertex_position(*it);
		CDT::Vertex_handle vh = cdt.insert(Point(x(0), x(1)));
		vh->info() = i;
		cdt_vertices.push_back(vh);
		vertices.push_back(static_cast<std::variant<Vector2,int>>(*it));

		indices.insert({it, i});

		i++;
	}
	for (auto seg : mesh.segments) {
		CDT::Vertex_handle a = cdt_vertices[indices[seg.start]];
		CDT::Vertex_handle b = cdt_vertices[indices[seg.end]];
		cdt.insert_constraint(a, b);
	}

	relax(mesh.x);
}


void AirMesh::clear() {
	cdt.clear();
	vertices.clear();
}

bool AirMesh::empty() const {
	return cdt.number_of_faces() == 0;
}


bool AirMesh::relax(const VectorX &x) {
	auto get_vertex_position = [&](int ix) -> Vector2 {
		Vector2 res;
		if (auto fixed = get_if<Vector2>(&vertices[ix])) {
			res = *fixed;
		}
		if (auto index = get_if<int>(&vertices[ix])) {
			res = x.segment<2>(2**index);
		}
		return res;
	};

	auto quality = [&](CDT::Vertex_handle va, CDT::Vertex_handle vb, CDT::Vertex_handle vc) -> real {
		if (cdt.is_infinite(va) ||
		    cdt.is_infinite(vb) ||
		    cdt.is_infinite(vc)) {
			// triangles with h ⪅ 0.1b should flip to infinite tringles
			//return 0.;
			return 0.05;
		}
		Vector2 a = get_vertex_position(va->info());
		Vector2 b = get_vertex_position(vb->info());
		Vector2 c = get_vertex_position(vc->info());
		auto ab = b-a;
		auto bc = c-b;
		auto ac = c-a;
		real area = 0.5*(ab.x() * ac.y() - ab.y() * ac.x());
		if (area < 0) {
			// don't flip inverted triangles, preserve topology
			return -std::numeric_limits<real>::infinity();
		}
		return area / (ab.squaredNorm() + ac.squaredNorm() + bc.squaredNorm());
	};

	bool any_flips = false;
	bool has_flips;
	do {
		has_flips = false;
		for (auto edge = cdt.edges_begin(); edge != cdt.edges_end(); ++edge) {
			if (cdt.is_constrained(*edge)) {
				// don't flip constrained edges
				continue;
			}
			auto f1 = edge->first;
			auto f2 = f1->neighbor(edge->second);

			bool flip = false;
			auto a = f1->vertex(edge->second);
			auto b = f1->vertex((edge->second+1)%3);
			auto c = f1->vertex((edge->second+2)%3);
			auto d = f2->vertex(f2->index(f1));

			real old_quality = min(quality(a,b,c), quality(b,d,c));
			real new_quality = min(quality(a,b,d), quality(a,d,c));

			flip = std::isfinite(old_quality) ? new_quality > old_quality : false;
			if (flip) {
				cdt.flip(f1, edge->second);
				has_flips = true;
				any_flips = true;
			}
		}
	} while (has_flips);

	return any_flips;
}

real AirMesh::penalty(real k, const VectorX &x, VectorX *grad) const {
	auto get_vertex_position = [&](int ix) -> Vector2 {
		Vector2 res;
		if (auto fixed = get_if<Vector2>(&vertices[ix])) {
			res = *fixed;
		}
		if (auto index = get_if<int>(&vertices[ix])) {
			res = x.segment<2>(2**index);
		}
		return res;
	};
	real res = 0;
	for (auto face = cdt.finite_faces_begin(); face != cdt.finite_faces_end(); ++face) {
		Vector2 a = get_vertex_position(face->vertex(0)->info());
		Vector2 b = get_vertex_position(face->vertex(1)->info());
		Vector2 c = get_vertex_position(face->vertex(2)->info());

		auto ab = b-a;
		auto ac = c-a;
		auto bc = c-b;
		real area = ab.x() * ac.y() - ab.y() * ac.x();

		//real circumference = ab.norm() + ac.norm() + bc.norm();

		real penalty;
		if (area >= 0) {
			penalty = 0;
		} else {
			// TODO: divide by circumference for multiresolution?
			penalty = -area ; // /circumference;
		}

		res += k * penalty;

		if (grad) {
			Vector2 darea_da(-bc.y(),  bc.x());
			Vector2 darea_db( ac.y(), -ac.x());
			Vector2 darea_dc(-ab.y(),  ab.x());

			Vector2 dpenalty_da = -darea_da;
			Vector2 dpenalty_db = -darea_db;
			Vector2 dpenalty_dc = -darea_dc;
			real fac; // dW/dpenalty
			fac = k;
			if (area >= 0) {
				fac = 0;
			} else {
			}
			if (const int *ix = get_if<int>(&vertices[face->vertex(0)->info()])) {
				grad->segment<2>(2**ix) += fac * dpenalty_da;
			}
			if (const int *ix = get_if<int>(&vertices[face->vertex(1)->info()])) {
				grad->segment<2>(2**ix) += fac * dpenalty_db;
			}
			if (const int *ix = get_if<int>(&vertices[face->vertex(2)->info()])) {
				grad->segment<2>(2**ix) += fac * dpenalty_dc;
			}
		}
	}
	return res;
}
real AirMesh::barrier(real k, const VectorX &x, VectorX *grad) const {
	(void)k;
	(void)x;
	(void)grad;
	// TODO
	return 0.;
}
void AirMesh::project(VectorX &x) const {
	if (empty()) {
		return;
	}
	auto get_vertex_position = [&](int ix) -> Vector2 {
		Vector2 res;
		if (auto fixed = get_if<Vector2>(&vertices[ix])) {
			res = *fixed;
		}
		if (auto index = get_if<int>(&vertices[ix])) {
			res = x.segment<2>(2**index);
		}
		return res;
	};

	bool changed;
	do {
		changed = false;

		for (auto face = cdt.finite_faces_begin(); face != cdt.finite_faces_end(); ++face) {
			Vector2 a = get_vertex_position(face->vertex(0)->info());
			Vector2 b = get_vertex_position(face->vertex(1)->info());
			Vector2 c = get_vertex_position(face->vertex(2)->info());

			auto ab = b-a;
			auto ac = c-a;
			auto bc = c-b;
			real area = ab.x() * ac.y() - ab.y() * ac.x();

			real constraint = area - 1e-3;
			if (constraint < 0) {
				changed = true;

				// Newton-raphson constraint solver
				Vector2 darea_da(-bc.y(),  bc.x());
				Vector2 darea_db( ac.y(), -ac.x());
				Vector2 darea_dc(-ab.y(),  ab.x());

				real sqnorm = 0.;
				if (const int *ix = get_if<int>(&vertices[face->vertex(0)->info()])) {
					sqnorm += darea_da.squaredNorm();
				}
				if (const int *ix = get_if<int>(&vertices[face->vertex(1)->info()])) {
					sqnorm += darea_db.squaredNorm();
				}
				if (const int *ix = get_if<int>(&vertices[face->vertex(2)->info()])) {				
					sqnorm += darea_dc.squaredNorm();
				}

				real fac = constraint / sqnorm;

				if (const int *ix = get_if<int>(&vertices[face->vertex(0)->info()])) {
					x.segment<2>(2**ix) += fac * darea_da;
				}
				if (const int *ix = get_if<int>(&vertices[face->vertex(1)->info()])) {
					x.segment<2>(2**ix) += fac * darea_db;
				}
				if (const int *ix = get_if<int>(&vertices[face->vertex(2)->info()])) {
					x.segment<2>(2**ix) += fac * darea_dc;
				}
			}
		}
	} while(changed);
}

}
