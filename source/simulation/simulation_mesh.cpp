#include "simulation/simulation_mesh.h"
#include <numeric>


namespace ruffles::simulation {

using Segment = SimulationMesh::Segment;
using Vertex = SimulationMesh::Vertex;

listref<Vertex> SimulationMesh::push_vertex(Vector2 position, bool fixed) {
	air_mesh.clear();
	if (fixed) {
		return vertices.insert(vertices.end(), Vertex(position));
	} else {
		// TODO: don't reallocate every single time
		x.conservativeResize(x.size()+2);
		x.tail<2>() = position;
		m.conservativeResize(dof());
		m.tail<2>() = Vector2(1.,1.); // TODO

		int index = dof()/2-1;
		return vertices.insert(vertices.end(), Vertex(index));
	}
}

listref<Segment> SimulationMesh::push_segment(listref<Vertex> a, listref<Vertex> b, real length) {
	return insert_segment(a,b,length,segments.end());
}
listref<Segment> SimulationMesh::insert_segment(listref<Vertex> a, listref<Vertex> b, real length, listref<Segment> position) {
	return segments.insert(position, Segment(a,b,length));
}

array<listref<Segment>, 2> SimulationMesh::split_segment(listref<Segment> seg) {
	Vector2 center_pos = 0.5 * (get_vertex_position(*seg->start) + get_vertex_position(*seg->end));
	listref<Vertex> center = push_vertex(center_pos);
	center->width = 0.5*(seg->start->width + seg->end->width);
	if (seg->start->z.size() && seg->end->z.size()) {
		center->z = {
			0.5*(seg->start->z.front()+seg->end->z.front()),
			0.5*(seg->start->z.back() +seg->end->z.back())
		};
	}
	center->width = 0.5*(seg->start->width + seg->end->width);
	listref<Segment> a = insert_segment(seg->start, center, 0.5*seg->length, seg);
	listref<Segment> b = insert_segment(center, seg->end, 0.5*seg->length, seg);

	auto mark_constrained = [this](auto f, int i, bool constrained) {
		f->set_constraint(i, constrained);
		f->neighbor(i)->set_constraint(air_mesh.cdt.mirror_index(f,i), constrained);
	};

	if (!air_mesh.empty()) {
		// TODO: update air mesh
		int center_ix = air_mesh.vertices.size();
		air_mesh.vertices.push_back(static_cast<std::variant<Vector2,int>>(*center));
		int start_ix = std::distance(vertices.begin(), seg->start);
		int end_ix   = std::distance(vertices.begin(), seg->end);

		// TODO: find the edge faster
		for (auto it = air_mesh.cdt.finite_edges_begin(); it != air_mesh.cdt.finite_edges_end(); ++it) {
			if (it->first->vertex((it->second+1)%3)->info() == start_ix &&
			    it->first->vertex((it->second+2)%3)->info() == end_ix) {
				mark_constrained(it->first, it->second, false);
				auto center_vx = air_mesh.cdt.tds().insert_in_edge(it->first, it->second);
				center_vx->info() = center_ix;
				auto c = center_vx->incident_edges();
				bool found_first = false;
				bool found_second = false;
				do {
					if ((c->first->vertex((c->second+1)%3)->info() == start_ix)
					 || (c->first->vertex((c->second+1)%3)->info() == end_ix)
					 || (c->first->vertex((c->second+2)%3)->info() == start_ix)
					 || (c->first->vertex((c->second+2)%3)->info() == end_ix)) {
						mark_constrained(c->first, c->second, true);
					}
				} while (++c != center_vx->incident_edges());
				
				cout << "Found!!!!" << endl;
				break;
			}
		}
	}
	segments.erase(seg);

	return {a,b};
	
}

void SimulationMesh::cleanup() {
	int dof_old = dof()/2;
	vector<bool> used(dof_old, false);

	for (auto v : vertices) {
		if (int *ix = get_if<int>(&v)) {
			used[*ix] = true;
		}
	}
	
	vector<int> newindex(dof_old, 0);
	int dof_new = 0;
	for (int i = 0; i < dof_old; i++) {
		newindex[i] = dof_new;
		dof_new += used[i];
	}

	for (auto &v : vertices) {
		if (int *ix = get_if<int>(&v)) {
			*ix = newindex[*ix];
		}
	}

	VectorX new_x = VectorX::Constant(2*dof_new, std::nan(""));
	for (int i = 0; i < dof_old; i++) {
		if (used[i]) {
			new_x.segment<2>(2*newindex[i]) = x.segment<2>(2*i);
		}
	}
	assert(!new_x.hasNaN());
	x = new_x;
}

void SimulationMesh::update_vertex_mass() {
	for (auto &vert : vertices) {
		vert.mass = 0.;
	}
	for (auto &seg : segments) {
		real half_length = 0.5*seg.length;
		real center_width = 0.5*(seg.start->width+seg.end->width);
		seg.start->mass += 0.5*(seg.start->width+center_width)*half_length * density;
		seg.end->mass   += 0.5*(seg.end->width  +center_width)*half_length * density;
	}
	for (auto &vert : vertices) {
		if (int *ix = get_if<int>(&vert)) {
			m.segment<2>(2**ix) = Vector2(vert.mass, vert.mass);
		}
	}
}

Vector2 SimulationMesh::get_vertex_position(Vertex &vx) const {
	Vector2 res;
	if (auto fixed = get_if<Vector2>(&vx)) {
		res = *fixed;
	}
	if (auto index = get_if<int>(&vx)) {
		res = x.segment<2>(2**index);
	}
	return res;
}

real SimulationMesh::energy(const VectorX &x, VectorX *grad) const {
	auto get_vertex = [&](Vertex &vx) -> Vector2 {
		return get_vertex_position(vx);
	};

	if (grad) {
		grad->setZero();
		assert(x.size() == grad->size());
	}


	real res = 0.;

	// bending energy
	Vector6 grad_theta = Vector6::Zero();
	auto add_bending_energy = [&](listref<Vertex> a, listref<Vertex> b, listref<Vertex> c, real avg_length) {
		Vector6 corner;
		corner <<
			get_vertex(*a),
			get_vertex(*b),
			get_vertex(*c);

		real theta = angle(corner, grad ? &grad_theta : nullptr);
		real theta_tilde = M_PI;

		real energy_local = (theta-theta_tilde)*(theta-theta_tilde);
		res += k_global*k_bend*b->width/avg_length * energy_local;
		if (grad) {
			real fac = k_global*k_bend*b->width/avg_length*2*(theta-theta_tilde);
			if (int *ax = get_if<int>(&*a))
				grad->segment<2>(2**ax) += fac * grad_theta.segment<2>(0);
			if (int *bx = get_if<int>(&*b))
				grad->segment<2>(2**bx) += fac * grad_theta.segment<2>(2);
			if (int *cx = get_if<int>(&*c))
				grad->segment<2>(2**cx) += fac * grad_theta.segment<2>(4);
		}
	};

	for (auto it = segments.begin(); std::next(it) != segments.end(); ++it) {
		assert(it->end == std::next(it)->start);
		add_bending_energy(it->start, it->end, std::next(it)->end, 0.5*(it->length + std::next(it)->length));
	}
	for (auto &[a,b] : connection_bends) {
		array<listref<Vertex>, 4> points {
			a->start,
			a->end,
			b->start,
			b->end,
		};

		listref<Vertex> x,y,z;
		for (int i = 0; i < 4; i++) {
			for (int j = i+1; j < 4; j++) {
				if (points[i] == points[j]) {
					y = points[i];
				}
			}
		}
		bool has_x = false;
		for (int i = 0; i < 4; i++) {
			if(points[i] == y)
				continue;
			if (has_x) {
				z = points[i];
			} else {
				x = points[i];
				has_x = true;
			}
		}

		add_bending_energy(x,y,z, 0.5*(a->length + b->length));
	}

	// membrane energy / constraint
	for (auto seg : segments) {
		real h_tilde = seg.length;
		Vector2 a = get_vertex(*seg.start);
		Vector2 b = get_vertex(*seg.end);
		Vector2 d = b-a;
		
		real h = d.norm();

		res += k_global * lambda_membrane * (h-h_tilde)*(h-h_tilde);

		if (grad) {
			Vector2 dhda = 1/(2*h) * -d;
			Vector2 dhdb = 1/(2*h) *  d;
			if (int *ix = get_if<int>(&*seg.start))
				grad->segment<2>(2**ix) += k_global*lambda_membrane * 2*(h-h_tilde)*dhda;
			if (int *ix = get_if<int>(&*seg.end))
				grad->segment<2>(2**ix) += k_global*lambda_membrane * 2*(h-h_tilde)*dhdb;
		}
	}
	
	// gravity

	// intrinsic mass
	for (auto vert : vertices) {
		res -= k_global * vert.mass * get_vertex_position(vert).dot(gravity);
		if (grad) {
			if (int *ix = get_if<int>(&vert)) {
				grad->segment<2>(2**ix) -= k_global * vert.mass * gravity;
			}
		}
	}

	// extrinsic mass
	for (auto &[v, m] : extra_mass) {
		if (int *ix = get_if<int>(&*v)) {
			grad->segment<2>(2**ix) -= k_global * m * gravity;
		}
	}

	// external forces
	for (auto &[v, f] : external_forces) {
		res += get_vertex_position(*v).dot(f);
		if (int *ix = get_if<int>(&*v)) {
			grad->segment<2>(2**ix) -= k_global * f;
		}
	}
	
	res += air_mesh.penalty(k_global * lambda_air_mesh, x, grad);

	return res;
}

void SimulationMesh::verify() {

	for (auto it = vertices.begin(); it != vertices.end(); ++it) {
		if (int *ix = get_if<int>(&*it)) {
			assert(*ix >= 0);
			assert(2**ix < dof());
		}
	}
	for (auto it = segments.begin(); it != segments.end(); ++it) {
		assert(it->length > 0.);
		if (it != segments.begin()) {
			assert(std::prev(it)->end == it->start);
		}
	}

	assert(k_bend >= 0.);
	assert(lambda_membrane >= 0.);
	assert(lambda_air_mesh >= 0.);
}

void SimulationMesh::generate_air_mesh() {
	air_mesh = AirMesh(*this);
	dbg(air_mesh.cdt.number_of_faces());
}

bool SimulationMesh::relax_air_mesh() {
	if (air_mesh.empty()) {
		return false;
	}

	assert(vertices.size() == air_mesh.vertices.size());

	return air_mesh.relax(x);
}

SimulationMesh SimulationMesh::generate_horizontal_strip(real length, real h) {
	SimulationMesh res;
	auto last = res.push_vertex(Vector2(-h, 0.), true);
	for (real x = 0.; x < length; x += h) {
		auto next = res.push_vertex(Vector2(x, 0.), x == 0);
		res.push_segment(last, next, h);
		last = next;
	}
	res.lb = Vector2(-infinity, -infinity);
	return res;
}

int SimulationMesh::dof() const {
	return x.size();
}

bool SimulationMesh::consistent_lengths() const {
	for (auto &seg : segments) {
		Vector2 a = get_vertex_position(*seg.start);
		Vector2 b = get_vertex_position(*seg.end);
		real f = (b-a).norm() / seg.length;
		if (f < 0.9 || f > 1.1) {
			return false;
		}
	}
	return true;
}

void SimulationMesh::perturb(real epsilon) {
	for (auto &v : vertices) {
		if (int *ix = get_if<int>(&v)) {
			x.segment<2>(2**ix) += epsilon * Vector2::Random();
		}
	}
}

real SimulationMesh::total_mass() const {
	real res = 0.;
	for (auto &x : vertices) {
		res += x.mass;
	}
	for (auto &[v, m] : extra_mass) {
		res += m;
	}
	return res;
}

void SimulationMesh::interpolate_missing_z() {
	// structured interpolation on connectivity graph

	std::unordered_map<listref<Vertex>, int, listref_hash<Vertex>> vx_to_ix;
	vector<listref<Vertex>> ix_to_vx;

	vector<vector<pair<int,real>>> edges(vertices.size());
	{int i = 0;
	for (auto it = vertices.begin(); it != vertices.end(); ++it, i++) {
		vx_to_ix.emplace(it, i);
		ix_to_vx.push_back(it);
	}}
	for (auto &seg : segments) {
		edges[vx_to_ix[seg.start]].emplace_back(vx_to_ix[seg.end], seg.length);
		edges[vx_to_ix[seg.end]].emplace_back(vx_to_ix[seg.start], seg.length);
	}


	for (int i = 0; i < vertices.size(); i++) {
		if (ix_to_vx[i]->z.empty()) {
			vector<pair<int, real>> stack;
			stack.emplace_back(i, 0.);
			vector<bool> vis(vertices.size(), false);
			Vector2 total = Vector2::Zero();
			real sum_weights = 0.;
			while (!stack.empty()) {
				auto [ix, dist] = stack.back();
				stack.pop_back();
				if (vis[ix]) continue;
				vis[ix] = true;
				auto vx = ix_to_vx[ix];
				if (vx->z.empty()) {
					for (auto &[n,l] : edges[ix]) {
						stack.emplace_back(n, dist+l);
					}
				} else {
					Vector2 z = Vector2(vx->z.front(), vx->z.back());
					real w = 1. / dist;
					total += z*w;
					sum_weights += w;
				}
			}
			total /= sum_weights;
			ix_to_vx[i]->z = {total(0), total(1)};
		}
	}
}


std::ostream &operator<<(std::ostream &os, const SimulationMesh::Vertex &vx) {
	if (const int *ix = get_if<int>(&vx)) {
		os << "Movable(" << *ix << ")";
	}
	if (const Vector2 *pos = get_if<Vector2>(&vx)) {
		os << "Fixed(" << *pos << ")";
	}
	return os;
}

std::ostream &operator<<(std::ostream &os, const SimulationMesh::Segment &seg) {
	return os << "Segment {"
		<< "start: " << *seg.start << ", "
		<< "end: " << *seg.end << ", "
		<< "length: " << seg.length << "}";
}

}
