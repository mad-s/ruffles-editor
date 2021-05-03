#pragma once

#include "common/common.h"

#include "simulation/simulation_mesh.h"
#include "simulation/simulator.h"

namespace ruffles {
class Ruffle;
}
#include "optimization/target_shape.h"

#include "common/clone_helper.h"

namespace ruffles {

class Ruffle : igl::SerializableBase {
public:
	struct ConnectionPoint {
		Vector2 position;

		listref<simulation::SimulationMesh::Vertex> mesh_vertex;
		array<vector<listref<simulation::SimulationMesh::Segment>>, 2> connecting_segments;
		int last_direction;

		ConnectionPoint(Vector2 _position, listref<simulation::SimulationMesh::Vertex> _mesh_vertex)
			: position(_position), mesh_vertex(_mesh_vertex), connecting_segments(), last_direction(0)
		{};

	};

	struct Section {
		enum class Type {
			Regular, // unknown type, don't optimize
			Outline, // sections on the outline
			Interior, // interior sections generated during initialization
			DensifiedStraight, // the two relatively straight sections c-a, c-b from densify
			DensifiedCurved, // the single curved section a-b from densify
		};

		listref<ConnectionPoint> start;
		listref<ConnectionPoint> end;
		real length;
		Type type = Type::Regular;


		vector<listref<simulation::SimulationMesh::Segment>> mesh_segments;

		Section(listref<ConnectionPoint> _start, listref<ConnectionPoint> _end, real _length)
			: start(_start), end(_end), length(_length)
		{};
	};

	struct OutlineSection {
		listref<Section> section;
		bool reversed;

		OutlineSection(listref<Section> section, bool reversed)
			: section(section), reversed(reversed)
		{}

		bool operator==(const listref<Section> &sec) const {
			return section == sec;
		}
		listref<ConnectionPoint> start() {
			return reversed ?
				section->end:
				section->start;
		}
		listref<ConnectionPoint> end() {
			return reversed ?
				section->start:
				section->end;
		}
	};

	simulation::SimulationMesh simulation_mesh;
	std::unique_ptr<simulation::Simulator> simulator;

	list<ConnectionPoint> connection_points;
	list<Section> sections;

	list<OutlineSection> outline_sections;

	real h;


	Ruffle();
	// named constructors
	static Ruffle create_ruffle_stack(int steps, real height, real width, real h);
	static Ruffle create_horizontal_stack(int steps, real height, real width, real h);
	static Ruffle create_stack_along_curve(MatrixX points, ruffles::optimization::TargetShape &target, real h);

	listref<ConnectionPoint> push_connection_point(Vector2 position, bool fixed = false);
	Section create_section(listref<ConnectionPoint> a, listref<ConnectionPoint> b, real length, std::function<Vector2(real)> shape);
	Section create_section(listref<ConnectionPoint> a, listref<ConnectionPoint> b, real length, std::function<Vector2(real)> shape, listref<simulation::SimulationMesh::Segment> segment_pos);
	Section create_bezier_section(listref<ConnectionPoint> a, listref<ConnectionPoint> b, listref<simulation::SimulationMesh::Segment> segment_pos, Vector2 ta=Vector2(0.,0.), Vector2 tb=Vector2(0.,0.));
	void create_connection_bends();

	void dissolve_connection_point(listref<ConnectionPoint> i);
	listref<Section> subdivide(listref<Section> section);
	void delete_section(listref<Section> section);
	listref<Section> densify(listref<Section> section);
	void undensify(listref<Section> section);

	listref<Section> densify2(listref<Section> section);
	listref<Section> densify3(listref<Section> section);

	Vector2 get_tangent(ConnectionPoint &p);

	void update_simulation_mesh();
	void physics_solve();

	real last_physics_solve_time = 0.;
	real physics_solve_total_time = 0.;
	int physics_solve_count = 0;

	void verify();

	Ruffle clone() { // const
		struct DefaultTr: 
			public Translate<simulation::SimulationMesh::Vertex>,
			public Translate<simulation::SimulationMesh::Segment>,
			public Translate<ConnectionPoint>,
			public Translate<Section>
		{
			using Translate<simulation::SimulationMesh::Vertex>::operator();
			using Translate<simulation::SimulationMesh::Segment>::operator();
			using Translate<ConnectionPoint>::operator();
			using Translate<Section>::operator();
			using Translate<simulation::SimulationMesh::Vertex>::transform;
			using Translate<simulation::SimulationMesh::Segment>::transform;
			using Translate<ConnectionPoint>::transform;
			using Translate<Section>::transform;
		} tr;
		return clone(tr);
	}
	template<typename Tr>
	Ruffle clone(Tr &tr = Tr()) { // const
		Ruffle res;
		res.simulation_mesh = simulation_mesh.clone(tr);
		//res.simulator = /// ?;
		tr.transform(connection_points.begin(), connection_points.end(), res.connection_points, [&](ConnectionPoint x) {
			ConnectionPoint new_point(x.position, tr(x.mesh_vertex));
			new_point.last_direction = x.last_direction;
			for (int side = 0; side < 2; side++) {
				std::transform(x.connecting_segments[side].begin(), x.connecting_segments[side].end(), std::back_inserter(new_point.connecting_segments[side]), tr);
			}
			return new_point;
		});
		std::transform(sections.begin(), sections.end(), std::back_inserter(res.sections), [&](Section x) {
			Section new_section(tr(x.start), tr(x.end), x.length);
			std::transform(x.mesh_segments.begin(), x.mesh_segments.end(), std::back_inserter(new_section.mesh_segments), tr);
			return new_section;
		});
		return res;
	}

	virtual void Serialize(std::vector<char> &buffer) const override;
	virtual void Deserialize(const std::vector<char> &buffer) override;
};

std::ostream &operator<<(std::ostream &, const Ruffle::ConnectionPoint &);
std::ostream &operator<<(std::ostream &, const Ruffle::Section &);
std::ostream &operator<<(std::ostream &, const Ruffle::OutlineSection &);

}

/*
namespace igl::serialization {
void serialize(const ruffles::Ruffle &ruffle, std::vector<char> &buf);
void deserialize(ruffles::Ruffle &ruffle, const std::vector<char> &buf);
}
*/