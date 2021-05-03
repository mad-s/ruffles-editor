#include "ruffle/ruffle.h"

#include <unordered_set>

#include <chrono>

namespace ruffles {

using simulation::SimulationMesh;

using Segment = SimulationMesh::Segment;
using Vertex = SimulationMesh::Vertex;
using ConnectionPoint = Ruffle::ConnectionPoint;
using Section = Ruffle::Section;

Ruffle::Ruffle() {}

void Ruffle::update_simulation_mesh() {
	// update the length of each segment
	for (auto &section : sections) {
		real segment_length = section.length / section.mesh_segments.size();

		for (auto segment : section.mesh_segments) {
			segment->length = segment_length;
		}
		
		if (segment_length > 2*h) {
			vector<listref<SimulationMesh::Segment>> new_mesh_segments;
			
			listref<SimulationMesh::Segment> old_first = section.mesh_segments.front();
			listref<SimulationMesh::Segment> old_last  = section.mesh_segments.back();

			for (auto it = section.mesh_segments.begin(); it != section.mesh_segments.end(); ++it) {
				auto [a,b] = simulation_mesh.split_segment(*it);
				new_mesh_segments.push_back(a);
				new_mesh_segments.push_back(b);
			}
			section.mesh_segments = new_mesh_segments;

			listref<SimulationMesh::Segment> new_first = section.mesh_segments.front();
			listref<SimulationMesh::Segment> new_last  = section.mesh_segments.back();

			for (auto &vx : {section.start, section.end}) {
				for (int side = 0; side < 2; ++side) {
					for (auto &it : vx->connecting_segments[side]) {
						if (it == old_first) {
							it = new_first;
						}
						if (it == old_last) {
							it = new_last;
						}
					}
				}
			}

			simulation_mesh.relax_air_mesh();
			create_connection_bends();
		}
		
	}

	simulation_mesh.update_vertex_mass();
}



listref<ConnectionPoint> Ruffle::push_connection_point(Vector2 position, bool fixed) {
	listref<Vertex> mesh_vertex = simulation_mesh.push_vertex(position, fixed);
	return connection_points.insert(connection_points.end(), ConnectionPoint(position, mesh_vertex));
}

void Ruffle::dissolve_connection_point(listref<ConnectionPoint> point) {
	assert(point->connecting_segments[0].size() == 1);
	assert(point->connecting_segments[1].size() == 1);


	for (auto it = sections.begin(); it != sections.end(); ++it) {
		// join the two adjacent sections
		if (it->start == point) {
			// move segments and length to the previous section
			auto prev = std::prev(it);
			prev->end = it->end;
			prev->length += it->length;
			if (it->type == Section::Type::Outline) {
				prev->type = Section::Type::Outline;
			}
			std::copy(
				it->mesh_segments.begin(),
				it->mesh_segments.end(),
				std::back_inserter(prev->mesh_segments)
			);

			// remove deleted section from outline
			listref<OutlineSection> in_outline = std::find(outline_sections.begin(), outline_sections.end(), it);
			if (in_outline != outline_sections.end()) {
				outline_sections.erase(in_outline);
			}
			// delete section
			it = sections.erase(it);
			it--; // because we will increment it again
		}
	}

	// remove the point
	connection_points.erase(point);
}

Section Ruffle::create_section(listref<ConnectionPoint> a, listref<ConnectionPoint> b, real length, std::function<Vector2(real)> shape) {
	return create_section(a,b,length,shape,simulation_mesh.segments.end());
}
Section Ruffle::create_section(listref<ConnectionPoint> a, listref<ConnectionPoint> b, real length, std::function<Vector2(real)> shape, listref<Segment> segment_pos) {
	/*
	if (last_connection_point) {
		assert(a == last_connection_point);
	}
	last_connection_point = b;
	*/

	assert(shape(0)     .isApprox(a->position));
	assert(shape(length).isApprox(b->position));

	int num_segments = (int)max(3,round(length/h));
	real segment_length = length / num_segments;

	Section section(a, b, length);

	listref<Vertex> prev_vertex = a->mesh_vertex;

	for (int i = 0; i < num_segments; i++) {
		listref<Vertex> next_vertex;
		if (i == num_segments-1) {
			next_vertex = b->mesh_vertex;
		} else {
			Vector2 next_pos = shape((i+1)*segment_length);
			next_vertex = simulation_mesh.push_vertex(next_pos);
		}

		auto segment = simulation_mesh.insert_segment(prev_vertex, next_vertex, segment_length, segment_pos);
		section.mesh_segments.push_back(segment);

		prev_vertex = next_vertex;
	}


	a->connecting_segments[a->last_direction].push_back(section.mesh_segments.front());
	b->connecting_segments[b->last_direction].push_back(section.mesh_segments.back());
	b->last_direction = !b->last_direction;

	return section;
}
Section Ruffle::create_bezier_section(listref<ConnectionPoint> a, listref<ConnectionPoint> b, listref<Segment> segment_pos, Vector2 ta, Vector2 tb) {
	Vector2 xa = simulation_mesh.get_vertex_position(*a->mesh_vertex);
	Vector2 xb = simulation_mesh.get_vertex_position(*b->mesh_vertex);

	// cubic bezier curve
	auto eval = [&](real t) {
		return (1.-t)*(1.-t)*(1.-t)*xa +
		     3*(1.-t)*(1.-t)*t     *(xa+ta) +
		     3*(1.-t)*t*t          *(xb+tb) +
		       t*t*t               *xb;
	};

	// how to determine #samples?
	int samples = 100;
	vector<real> cumsum(samples);
	real length = 0.;
	real dt = 1./(samples-1);
	{
		cumsum[0] = 0.;
		Vector2 last = eval(0.);
		for (int i = 1; i < samples; i++) {
			Vector2 pos = eval(i*dt);
			length += (pos-last).norm();
			cumsum[i] = length;
			last = pos;
		}
	}

	return create_section(a, b, length, [&](real pos) {
		auto it = std::upper_bound(cumsum.begin(), cumsum.end(), pos);
		if (it == cumsum.begin()) {
			return eval(0.);
		}
		if (it == cumsum.end()) {
			return eval(1.);
		}
		int index= it-cumsum.begin();
		real t = dt * (real(index-1) + (pos-cumsum[index-1])/(cumsum[index]-cumsum[index-1]));
		return eval(t);
	}, segment_pos);
}

void Ruffle::create_connection_bends() {
	simulation_mesh.connection_bends.clear();
	for (auto conn : connection_points) {
		int i = 0;
		for (auto a : conn.connecting_segments[0]) {
			int j = 0;
		for (auto b : conn.connecting_segments[1]) {
			//if (i == 0 || j == 0) {
			if (std::next(a) != b && std::prev(a) != b) {
				simulation_mesh.connection_bends.push_back(array<listref<Segment>, 2>{a, b});
			}
			//}
			j++;
		}
		i++;
		}
	}
}

listref<Section> Ruffle::subdivide(listref<Section> section) {
	assert(section->mesh_segments.size() >= 2);

	// number of segments in the first half
	unsigned split = section->mesh_segments.size() / 2;
	real first_length = 0;
	for (unsigned i = 0; i < split; i++) {
		first_length += section->mesh_segments[i]->length;
	}
	real second_length = section->length - first_length;

	listref<Vertex> center_vertex = section->mesh_segments[split]->start;
	listref<ConnectionPoint> center = connection_points.insert(connection_points.end(), ConnectionPoint(simulation_mesh.get_vertex_position(*center_vertex), center_vertex));

	Section first_half(section->start, center, first_length);
	Section second_half(center, section->end, second_length);
	//first_half.type = section->type;
	//second_half.type = section->type;

	for (unsigned i = 0; i < split; i++) {
		first_half.mesh_segments.push_back(section->mesh_segments[i]);
	}
	for (unsigned i = split; i < section->mesh_segments.size(); i++) {
		second_half.mesh_segments.push_back(section->mesh_segments[i]);
	}
	center->connecting_segments[0].push_back(first_half.mesh_segments.back());
	center->connecting_segments[1].push_back(second_half.mesh_segments.front());
	center->last_direction = 1;

	listref<Section> first  = sections.insert(section, first_half);
	listref<Section> second = sections.insert(section, second_half);

	listref<OutlineSection> in_outline = std::find(outline_sections.begin(), outline_sections.end(), section);
	if (in_outline != outline_sections.end()) {
		if (in_outline->reversed) {
			outline_sections.insert(in_outline, OutlineSection(second, in_outline->reversed));
			outline_sections.insert(in_outline, OutlineSection(first, in_outline->reversed));
		} else {
			outline_sections.insert(in_outline, OutlineSection(first, in_outline->reversed));
			outline_sections.insert(in_outline, OutlineSection(second, in_outline->reversed));
		}
		outline_sections.erase(in_outline);
	}
	sections.erase(section);

	return first;
}

listref<Section> Ruffle::densify(listref<Section> section) {
	simulation_mesh.air_mesh.clear();

	assert(section != sections.begin());
	assert(section != std::prev(sections.end()));

	assert(std::prev(section)->start == std::next(section)->end);

	listref<Segment> insertion_point = section->mesh_segments[0];

	// these point inside
	Vector2 t0 = get_tangent(*section->start);
	Vector2 t1 = get_tangent(*section->end);

	Vector2 x0 = 
		simulation_mesh.get_vertex_position(*section->start->mesh_vertex);
	Vector2 x1 = 
		simulation_mesh.get_vertex_position(*section->end->mesh_vertex);

	dbg(t0);
	dbg(t1);

	Vector2 xc = 0.5*(x0+x1);
	Vector2 tc = (t0+t1).normalized();


	listref<Vertex> center_vertex = simulation_mesh.push_vertex(xc);
	listref<ConnectionPoint> c = connection_points.insert(connection_points.end(), ConnectionPoint(xc, center_vertex));

	listref<ConnectionPoint> a = subdivide(std::prev(section))->end;
	listref<ConnectionPoint> b = subdivide(std::next(section))->end;

	Vector2 xa = simulation_mesh.get_vertex_position(*a->mesh_vertex);
	Vector2 xb = simulation_mesh.get_vertex_position(*b->mesh_vertex);
	Vector2 ta = -get_tangent(*a); // first section runs to outside, so negate
	Vector2 tb =  get_tangent(*b); // second section runs to inside


	real outer_tangent_length = 0.4 * (x1-x0).norm();
	real middle_tangent_length = 0.4 * 0.5*((xa-xc).norm() + (xb-xc).norm());
	real inner_tangent_length = 0.4 * (xa-xb).norm();
	listref<Section> s0 = sections.insert(section, create_bezier_section(section->start, c, insertion_point, -outer_tangent_length * t0, -outer_tangent_length*tc));
	listref<Section> s1 = sections.insert(section, create_bezier_section(c, a, insertion_point, middle_tangent_length * tc, -middle_tangent_length*ta));
	listref<Section> s2 = sections.insert(section, create_bezier_section(a, b, insertion_point, inner_tangent_length * ta, inner_tangent_length*tb));
	listref<Section> s3 = sections.insert(section, create_bezier_section(b, c, insertion_point, -middle_tangent_length * tb, middle_tangent_length*tc));
	listref<Section> s4 = sections.insert(section, create_bezier_section(c, section->end, insertion_point, -outer_tangent_length * tc, -outer_tangent_length*t1));


	s0->type = section->type;
	s1->type = Section::Type::DensifiedStraight;
	s2->type = Section::Type::DensifiedCurved;
	s3->type = Section::Type::DensifiedStraight;
	s4->type = section->type;



	// insert connecting segments at a and b
	a->connecting_segments[1].push_back(s1->mesh_segments.back());
	a->connecting_segments[0].push_back(s2->mesh_segments.front());
	b->connecting_segments[1].push_back(s2->mesh_segments.back());
	b->connecting_segments[0].push_back(s3->mesh_segments.front());
	// TODO: {a,b}->last_direction = ???

	// change connecting segments at start and end
	for (auto point : {section->start, section->end}) {
		for (int side = 0; side < 2; side++) {
			for (auto &it : point->connecting_segments[side]) {
				if (it == section->mesh_segments.front()) {
					it = s0->mesh_segments.front();
				}
				if (it == section->mesh_segments.back()) {
					it = s4->mesh_segments.back();
				}
			}
		}
	}

	// update bends for the simulation mesh
	create_connection_bends();

	// update simulation mesh
	for (auto it = section->mesh_segments.begin(); it != section->mesh_segments.end(); ++it) {
		if (it != section->mesh_segments.begin()) {
			simulation_mesh.vertices.erase((*it)->start);
		}
		simulation_mesh.segments.erase(*it);
	}
	simulation_mesh.cleanup();

	// update outline
	listref<OutlineSection> in_outline = std::find(outline_sections.begin(), outline_sections.end(), section);
	if (in_outline != outline_sections.end()) {
		if (in_outline->reversed) {
			outline_sections.insert(in_outline, OutlineSection(s4, in_outline->reversed));
			outline_sections.insert(in_outline, OutlineSection(s0, in_outline->reversed));
		} else {
			outline_sections.insert(in_outline, OutlineSection(s0, in_outline->reversed));
			outline_sections.insert(in_outline, OutlineSection(s4, in_outline->reversed));
		}
		outline_sections.erase(in_outline);
	}

	// delete section
	sections.erase(section);

	verify();
	

	return s0;
}

void Ruffle::delete_section(listref<Section> section) {
	// remove connecting segments
	for (int side = 0; side < 2; ++side) {
		std::remove(
			section->start->connecting_segments[side].begin(), 
			section->start->connecting_segments[side].end(), 
			section->mesh_segments.front());
		std::remove(
			section->end->connecting_segments[side].begin(), 
			section->end->connecting_segments[side].end(), 
			section->mesh_segments.back());
	}

	for (auto it = section->mesh_segments.begin(); it != section->mesh_segments.end(); ++it) {
		if (it != section->mesh_segments.begin()) {
			simulation_mesh.vertices.erase((*it)->start);
		}
		simulation_mesh.segments.erase(*it);
	}
	simulation_mesh.cleanup();

	sections.erase(section);
}

void Ruffle::undensify(listref<Section> section) {
	assert(section->end == std::next(section, 4)->start);

	for (int i = 0; i < 3; i++) {
		delete_section(std::next(section));
	}

	assert(section->end->connecting_segments[0].size() == 2);
	assert(section->end->connecting_segments[1].size() == 0);

	// move segments back to opposite halves
	section->end->connecting_segments[1].push_back(section->end->connecting_segments[0].back());
	section->end->connecting_segments[0].pop_back();
	create_connection_bends();
}


Vector2 Ruffle::get_tangent(ConnectionPoint &p) {
	int count = 0;
	Vector2 tangent(0.,0.);
	for (int side = 0; side < 2; side++) {
		for (auto it : p.connecting_segments[side]) {
			Vector2 dir = (simulation_mesh.get_vertex_position(*it->end)-
				      simulation_mesh.get_vertex_position(*it->start)).normalized();
			if ((it->end == p.mesh_vertex) ^ (side == 0)) {
				dir *= -1;
			}
			tangent += dir;
			count++;
		}
	}
	if (count == 0) {
		return Vector2(0.,0.);
	}
	return (tangent/count).normalized();
}

listref<Section> Ruffle::densify2(listref<Section> section) {
	simulation_mesh.air_mesh.clear();
	
	section = subdivide(section);
	auto a = section->start;
	auto b = section->end;
	auto c = std::next(section)->end;

	Vector2 xa = simulation_mesh.get_vertex_position(*a->mesh_vertex);
	Vector2 xb = simulation_mesh.get_vertex_position(*b->mesh_vertex);
	Vector2 xc = simulation_mesh.get_vertex_position(*c->mesh_vertex);

	bool side;
	if (xb.x() < xa.x() && xb.x() < xc.x()) {
		side = true;
	} else if (xb.x() > xa.x() && xb.x() > xc.x()) {
		side = false;
	} else {
		throw std::runtime_error("Can't densify horizontal sections!");
	}

	// clear connecting segment so new section attaches properly
	for (int side = 0; side < 2; side++) {
		b->connecting_segments[side].clear();
	}
	b->last_direction = 1;

	real scale = 1.0;
	real length = scale*5.137; // numerical length
	auto middle = create_section(b, b, length, [&](real t) -> Vector2 {
		t *= 2*M_PI / length;
		// TODO: this assumes curve is going up
		return xb + scale * Vector2(side?(1-cos(t)):(cos(t)-1), -sin(t)*sin(t/2)*sin(t/2));
	}, std::next(section)->mesh_segments.front());
	sections.insert(std::next(section), middle);

	// attach existing sections properly
	b->connecting_segments[0].push_back(section->mesh_segments.back());
	b->connecting_segments[0].push_back(std::next(section)->mesh_segments.front());

	return section;
	
}

listref<Section> Ruffle::densify3(listref<Section> section) {

	if (section == sections.begin()) return section;
	if (section == std::prev(sections.end())) return section;

	simulation_mesh.air_mesh.clear();


	std::unordered_set<listref<ConnectionPoint>, listref_hash<ConnectionPoint>> visited_vertices;
	for (auto it = std::next(section); it != sections.end(); ++it) {
		visited_vertices.emplace(it->end);
	}
	listref<ConnectionPoint> apex;
	int apex_count = 0;
	for (auto it = section; it != sections.begin(); ) {
		--it;
		if (visited_vertices.count(it->start)) {
			apex_count++;
			apex = it->start;
		}
	}

	if (apex_count != 1) {
		return section;
		panic("at the disco");
	}

	real distance_a = 0;
	real distance_b = 0;


	// connection point a (before section)
	listref<ConnectionPoint> a;
	if (apex == std::prev(section)->start) { // single-segment: subdivide
		a = subdivide(std::prev(section))->end;
	} else { // multi-segment: find closest connection point to center
		real distance = 0;
		auto it = section;
		do {
			--it;
			distance += it->length;
		} while (it->start != apex);

		real best_delta = infinity;
		real position = 0.;
		it = section;
		do {
			--it;
			position += it->length;
			real delta = abs(position-0.5*distance);
			if (delta < best_delta) {
				best_delta = delta;
				a = it->start;
			}
		} while (it->start != apex);
	}
	assert(a != apex);

	// connection point b (before section)
	listref<ConnectionPoint> b;
	if (apex == std::next(section)->end) { // single-segment: subdivide
		b = subdivide(std::next(section))->end;
	} else { // multi-segment: find closest connection point to center
		real distance = 0;
		auto it = section;
		do {
			++it;
			distance += it->length;
		} while (it->end != apex);

		real best_delta = infinity;
		real position = 0.;
		it = section;
		do {
			++it;
			position += it->length;
			real delta = abs(position-0.5*distance);
			if (delta < best_delta) {
				best_delta = delta;
				b = it->end;
			}
		} while (it->end != apex);
	}
	assert(b != apex);


	listref<Segment> insertion_point = section->mesh_segments[0];

	// these point inside
	Vector2 t0 = get_tangent(*section->start);
	Vector2 t1 = get_tangent(*section->end);

	dbg(t0);
	dbg(t1);

	Vector2 x0 = 
		simulation_mesh.get_vertex_position(*section->start->mesh_vertex);
	Vector2 x1 = 
		simulation_mesh.get_vertex_position(*section->end->mesh_vertex);

	Vector2 xc = 0.5*(x0+x1);
	Vector2 tc = (t0+t1).normalized();
	dbg(tc);


	listref<Vertex> center_vertex = simulation_mesh.push_vertex(xc);
	listref<ConnectionPoint> c = connection_points.insert(connection_points.end(), ConnectionPoint(xc, center_vertex));

	Vector2 xa = simulation_mesh.get_vertex_position(*a->mesh_vertex);
	Vector2 xb = simulation_mesh.get_vertex_position(*b->mesh_vertex);
	Vector2 ta = -get_tangent(*a); // first section runs to outside, so negate
	Vector2 tb =  get_tangent(*b); // second section runs to inside
	dbg(ta);
	dbg(tb);


	real outer_tangent_length = 0.4 * (x1-x0).norm();
	real middle_tangent_length = 0.4 * 0.5*((xa-xc).norm() + (xb-xc).norm());
	real inner_tangent_length = 0.4 * (xa-xb).norm();
	listref<Section> s0 = sections.insert(section, create_bezier_section(section->start, c, insertion_point, -outer_tangent_length * t0, -outer_tangent_length*tc));
	listref<Section> s1 = sections.insert(section, create_bezier_section(c, a, insertion_point, middle_tangent_length * tc, -middle_tangent_length*ta));
	listref<Section> s2 = sections.insert(section, create_bezier_section(a, b, insertion_point, inner_tangent_length * ta, inner_tangent_length*tb));
	listref<Section> s3 = sections.insert(section, create_bezier_section(b, c, insertion_point, -middle_tangent_length * tb, middle_tangent_length*tc));
	listref<Section> s4 = sections.insert(section, create_bezier_section(c, section->end, insertion_point, -outer_tangent_length * tc, -outer_tangent_length*t1));

	if (section->type == Section::Type::Outline) {
		dbg("was outline!!!");
		s0->type = Section::Type::Outline;
		s4->type = Section::Type::Outline;
	}

	// insert connecting segments at a and b
	a->connecting_segments[1].push_back(s1->mesh_segments.back());
	a->connecting_segments[0].push_back(s2->mesh_segments.front());
	b->connecting_segments[1].push_back(s2->mesh_segments.back());
	b->connecting_segments[0].push_back(s3->mesh_segments.front());
	// TODO: {a,b}->last_direction = ???

	// change connecting segments at start and end
	for (auto point : {section->start, section->end}) {
		for (int side = 0; side < 2; side++) {
			for (auto &it : point->connecting_segments[side]) {
				if (it == section->mesh_segments.front()) {
					it = s0->mesh_segments.front();
				}
				if (it == section->mesh_segments.back()) {
					it = s4->mesh_segments.back();
				}
			}
		}
	}

	// update bends for the simulation mesh
	create_connection_bends();

	// update simulation mesh
	for (auto it = section->mesh_segments.begin(); it != section->mesh_segments.end(); ++it) {
		if (it != section->mesh_segments.begin()) {
			simulation_mesh.vertices.erase((*it)->start);
		}
		simulation_mesh.segments.erase(*it);
	}
	simulation_mesh.cleanup();

	// update outline
	listref<OutlineSection> in_outline = std::find(outline_sections.begin(), outline_sections.end(), section);
	if (in_outline != outline_sections.end()) {
		if (in_outline->reversed) {
			outline_sections.insert(in_outline, OutlineSection(s4, in_outline->reversed));
			outline_sections.insert(in_outline, OutlineSection(s0, in_outline->reversed));
		} else {
			outline_sections.insert(in_outline, OutlineSection(s0, in_outline->reversed));
			outline_sections.insert(in_outline, OutlineSection(s4, in_outline->reversed));
		}
		outline_sections.erase(in_outline);
	}

	// delete section
	sections.erase(section);

	verify();

	return s0;
}


Ruffle Ruffle::create_ruffle_stack(int steps, real height, real width, real h) {
	dbg(steps);
	dbg(width);
	dbg(height);
	Ruffle res;
	res.h = h;

	real diagonal = hypot(width, height);
	real semicircle = 0.5*M_PI*height;

	auto bl = res.push_connection_point(Vector2(0.,0.), true);
	bl->last_direction=1;
	auto br = res.push_connection_point(Vector2(width,0.), false);
	br->last_direction=1; // fix densify
	auto original_br = br;


	list<OutlineSection> outline_right;
	list<OutlineSection> outline_left;

	auto sec_0 = res.create_section(bl, br, width,
	[&](real pos) {
		return Vector2(pos, 0.);
	});
	outline_right.emplace_back(res.sections.insert(res.sections.end(), sec_0), false);

	for (int i = 0; i < steps; i++) {
		auto tl = res.push_connection_point(Vector2(0.,   (i+1)*height));
		auto tr = res.push_connection_point(Vector2(width,(i+1)*height));


		auto sec_1 = res.create_section(br, tr, semicircle,
		[&](real pos) -> Vector2 {
			real phi = M_PI*pos/semicircle;
			real r = height*0.5;
			return Vector2(width+r*sin(phi), i*height + r*(1. - cos(phi)));
		});
		sec_1.type = Section::Type::Outline;
		outline_right.emplace_back(res.sections.insert(res.sections.end(), sec_1), false);

		auto sec_2 = res.create_section(tr, bl, diagonal,
		[&](real pos) -> Vector2 {
			real a = 1. - pos / diagonal;
			return Vector2(a*width, (i+a)*height);
		});
		sec_2.type = Section::Type::Interior;
		res.sections.push_back(sec_2);

		auto sec_3 = res.create_section(bl, tl, semicircle,
		[&](real pos) -> Vector2 {
			real phi = M_PI*pos/semicircle;
			real r = height*0.5;
			return Vector2(-r*sin(phi), i*height + r*(1. - cos(phi)));
		});
		sec_3.type = Section::Type::Outline;
		outline_left.emplace_back(res.sections.insert(res.sections.end(), sec_3), false);

		auto sec_4 = res.create_section(tl, tr, width,
		[&](real pos) {
			return Vector2(pos, (i+1)*height);
		});
		sec_4.type = Section::Type::Interior;
		if (i == steps-1) {
			sec_4.type = Section::Type::Outline;
		}
		res.sections.push_back(sec_4);

		bl = tl;
		br = tr;
	}
	outline_left.emplace_back(std::prev(res.sections.end()), false);
	for (auto it = outline_right.begin(); it != outline_right.end(); ++it) {
		res.outline_sections.push_back(*it);
	}
	for (auto it = outline_left.rbegin(); it != outline_left.rend(); ++it) {
		res.outline_sections.push_back(OutlineSection(it->section, !it->reversed));
	}

	dbg(res.outline_sections);

	//res.dissolve_connection_point(bl); // actually top left vertex
	//res.dissolve_connection_point(original_br);

	res.create_connection_bends();
	res.update_simulation_mesh();

	return res;
}


Ruffle Ruffle::create_horizontal_stack(int steps, real height, real width, real h) {
	Ruffle res;
	res.h = h;

	real diagonal = hypot(width, height);
	real semicircle = 0.5*M_PI*width;

	auto bl = res.push_connection_point(Vector2(0.,0.5*height), false);
	bl->last_direction=1;
	auto br = res.push_connection_point(Vector2(0.,1.5*height), false);
	auto original_br = br;


	list<OutlineSection> outline_right;
	list<OutlineSection> outline_left;

	outline_right.emplace_back(res.sections.insert(res.sections.end(), res.create_section(bl, br, width,
	[&](real pos) {
		return Vector2(pos, 0.);
	})), false);

	for (int i = 0; i < steps; i++) {
		auto tl = res.push_connection_point(Vector2((i+1)*width, 0.5*height));
		auto tr = res.push_connection_point(Vector2((i+1)*width, 1.5*height));

		outline_right.emplace_back(res.sections.insert(res.sections.end(), res.create_section(br, tr, semicircle,
		[&](real pos) -> Vector2 {
			real phi = M_PI*pos/semicircle;
			real r = width*0.5;
			return Vector2(i*width + r*(1. - cos(phi)), 1.5*height+r*sin(phi));
		})), false);

		res.sections.push_back(res.create_section(tr, bl, diagonal,
		[&](real pos) -> Vector2 {
			real a = 1. - pos / diagonal;
			return Vector2((i+a)*width, (0.5+a)*height);
		}));

		outline_left.emplace_back(res.sections.insert(res.sections.end(), res.create_section(bl, tl, semicircle,
		[&](real pos) -> Vector2 {
			real phi = M_PI*pos/semicircle;
			real r = width*0.5;
			return Vector2(i*width + r*(1. - cos(phi)), width*0.5-r*sin(phi));
		})), false);

		res.sections.push_back(res.create_section(tl, tr, width,
		[&](real pos) {
			return Vector2((i+1)*width, 0.5*height + pos);
		}));

		bl = tl;
		br = tr;
	}
	outline_left.emplace_back(std::prev(res.sections.end()), false);
	for (auto it = outline_right.begin(); it != outline_right.end(); ++it) {
		res.outline_sections.push_back(*it);
	}
	for (auto it = outline_left.rbegin(); it != outline_left.rend(); ++it) {
		res.outline_sections.push_back(OutlineSection(it->section, !it->reversed));
	}

	dbg(res.outline_sections);

	res.dissolve_connection_point(bl); // actually top left vertex
	res.dissolve_connection_point(original_br);

	res.create_connection_bends();
	res.update_simulation_mesh();

	return res;
}

Ruffle Ruffle::create_stack_along_curve(MatrixX points, optimization::TargetShape &target, real h) {
	Ruffle res;
	res.h = h;
	int n = points.rows(); 
	bool closed = points.bottomRows<1>() == points.topRows<1>();
	constexpr real height_width_ratio = 0.5;
	constexpr real inner_width_ratio = 0.6;
	constexpr real outer_control_length_ratio = 0.8;
	constexpr real inner_control_length_ratio = 0.4;

	real curve_length = 0.;
	for (int i = 0; i < n-1; i++) {
		curve_length += (points.row(i+1)-points.row(i)).norm();
	}
	dbg(curve_length);

	vector<real> samples;
	{
	real position = 0.;
	int segment = 0;
	real segment_start = 0.;
	while (segment < n-1) {
		real segment_length = (points.row(segment+1)-points.row(segment)).norm();
		samples.push_back(position);
		real alpha = (position - segment_start) / segment_length;
		Vector2 xy = (points.row(segment+1)*alpha + points.row(segment)*(1-alpha)).transpose();
		dbg(xy);
		Vector2 tangent = (points.row(segment+1)-points.row(segment)).transpose() / segment_length;
		Vector2 normal(tangent.y(), -tangent.x()); // pointing to the right
		dbg(normal);
		// TODO: raycast at angle?
		real right = target.raycast(xy, normal);
		real left = target.raycast(xy, -normal);
		real width = max(1., left + right);
		dbg(width);
		if (!std::isfinite(width)) {
			samples.pop_back(); // we went over the target shape
			break;
		}
		// don't add points yet, we still need to resample later

		position += height_width_ratio * width;
		while (position - segment_start > segment_length && segment < n-1) {
			segment_start += segment_length;
			segment++;
			if (segment < n-1) {
				segment_length = (points.row(segment+1)-points.row(segment)).norm();
			} else {
				break;
			}
		}
	}
	}

	// TODO: resample to fit whole line
	dbg_list(samples);
	real scale_factor = curve_length / samples.back();
	for (auto &t : samples) {
		t *= scale_factor;
	}

	int segment = 0;
	real segment_start = 0.;

	listref<ConnectionPoint> left, right;
	Vector2 prev_normal;
	for (int i = 0; i < samples.size(); i++) {
		real t = samples[i];
		real segment_length = (points.row(segment+1)-points.row(segment)).norm();
		while (t - segment_start > segment_length) {
			segment++;
			if (segment >= n-1) {
				assert(i == samples.size() - 1);
				dbg(t-segment_start);
				dbg(segment_length);
				segment--;
				break;
			}
			segment_start += segment_length;
			segment_length = (points.row(segment+1)-points.row(segment)).norm();
		}

		real alpha = (t - segment_start) / segment_length;
		Vector2 xy = (points.row(segment+1)*alpha + points.row(segment)*(1-alpha)).transpose();
		dbg(xy);
		Vector2 tangent = (points.row(segment+1)-points.row(segment)).transpose() / segment_length;
		Vector2 normal(tangent.y(), -tangent.x()); // pointing to the right
		// TODO: raycast at angle?
		real right_dist = target.raycast(xy, normal);
		real left_dist = target.raycast(xy, -normal);

		bool fixed = i == 0 || i == samples.size()-1;
		auto nleft  = res.push_connection_point(xy - inner_width_ratio*left_dist*normal, fixed);
		auto nright = res.push_connection_point(xy + inner_width_ratio*right_dist*normal, fixed);
		if (i > 0) {
			real right_dist = (nright->position - right->position).norm();
			real left_dist  = (nleft->position  - left->position).norm();
			res.sections.push_back(res.create_bezier_section(
				right, nright, res.simulation_mesh.segments.end(),
				right_dist * outer_control_length_ratio * prev_normal,
				right_dist * outer_control_length_ratio * normal
			));
			res.sections.push_back(res.create_bezier_section(
				nright, left, res.simulation_mesh.segments.end(),
				-right_dist * inner_control_length_ratio * normal,
				  left_dist * inner_control_length_ratio * prev_normal
			));
			res.sections.push_back(res.create_bezier_section(
				left, nleft, res.simulation_mesh.segments.end(),
				-left_dist * outer_control_length_ratio * prev_normal,
				-left_dist * outer_control_length_ratio * normal
			));
		} else {
			nleft->last_direction = 1;
			nright->last_direction = 1;
		}
		res.sections.push_back(res.create_bezier_section(
				nleft, nright, res.simulation_mesh.segments.end(),
				Vector2::Zero(),
				Vector2::Zero()
		));

		left  = nleft;
		right = nright;
		prev_normal = normal;
	}

	res.create_connection_bends();
	res.update_simulation_mesh();

	dbg(res.simulation_mesh.total_mass());

	return res;
}



void Ruffle::verify() {
	simulation_mesh.verify();

	for (auto it = connection_points.begin(); it != connection_points.end(); ++it) {
		for (int side = 0; side < 2; side++) {
			for (auto seg : it->connecting_segments[side]) {
				assert(seg->start == it->mesh_vertex
				    || seg->end   == it->mesh_vertex);
			}
		}
	}

	for (auto it = sections.begin(); it != sections.end(); ++it) {
		assert(it->length > 0.);
		if (it != sections.begin()) {
			assert(std::prev(it)->end == it->start);
		}
		assert(it->start->mesh_vertex == it->mesh_segments.front()->start);
		assert(it->end->mesh_vertex == it->mesh_segments.back()->end);
	}

	// outline must be a closed loop
	listref<ConnectionPoint> prev = outline_sections.back().end();
	for (auto it = outline_sections.begin(); it != outline_sections.end(); ++it) {
		assert(it->start() == prev);
		prev = it->end();
	}
}


void Ruffle::physics_solve() {
	if (simulator == nullptr) {
		cerr << "No simulator set!" << endl;
		return;
	}

	auto start = std::chrono::steady_clock::now();

	simulator->reset(simulation_mesh);
	simulation_mesh.relax_air_mesh();

	int steps = 1;
	for(; steps < 1000 && !simulator->step(simulation_mesh); steps++) {
		//simulation_mesh.relax_air_mesh();
	}

	cerr << "Converged? in " << steps << " steps" << endl;

	auto end = std::chrono::steady_clock::now();
	real elapsed = std::chrono::duration_cast<std::chrono::duration<real>>(end-start).count();

	last_physics_solve_time = elapsed;
	physics_solve_total_time += elapsed;
	physics_solve_count += 1;
}


void Ruffle::Serialize(std::vector<char> &buffer) const {

}

void Ruffle::Deserialize(const std::vector<char> &buffer) {

}

std::ostream &operator<<(std::ostream &os, const Ruffle::ConnectionPoint &point) {
	return os << "ConnectionPoint {"
		<< "position: " << point.position.transpose() << ", "
		<< "mesh_vertex: " << *point.mesh_vertex << ", "
		//<< "connecting_segments: " << point.connecting_segments << ", "
		//<< "last_direction: " << point.last_direction 
		<< "}";
}

std::ostream &operator<<(std::ostream &os, const Ruffle::Section &sec) {
	return os << "Section {"
		<< "start: " << *sec.start << ", "
		<< "end: " << *sec.end << ", "
		<< "length: " << sec.length << ","
		//<< "mesh_segments: " << sec.mesh_segments << ","
		<< "}";
}


std::ostream &operator<<(std::ostream &os, const Ruffle::OutlineSection &sec) {
	return os << "OutlineSection {"
		<< "section: " << *sec.section << ", "
		<< "reversed: " << sec.reversed << ", "
		<< "}";
}

}
