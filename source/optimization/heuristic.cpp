#include "optimization/heuristic.h"

#include "common/cgal_util.h"


namespace ruffles::optimization {

Heuristic::Heuristic() {
	
}
Heuristic::Heuristic(TargetShape target)
 : target(std::move(target)) {
}

void Heuristic::step(Ruffle &ruffle) {
	for (auto section = ruffle.sections.begin(); section != ruffle.sections.end(); ++section) {
		if (section->type != Ruffle::Section::Type::Outline)
			continue;
		real closest_distance = infinity;
		for (auto &seg : section->mesh_segments) {
			auto vertex = seg->start;
			Vector2 pos = ruffle.simulation_mesh.get_vertex_position(*vertex);
			
			//real x = target.intersect_horizontal(pos(1))[(i & 2) ? 0 : 1];
			closest_distance = min(closest_distance, target.signed_distance(pos));
			/*if (std::isfinite(x)) {
				closest_distance = min(closest_distance,
					(i & 2) ? (pos(0) - x) : (x - pos(0)));
			}*/
		}
		dbg(closest_distance);

		if (std::isfinite(closest_distance)) {
			section->length += eta_outer * closest_distance;
		}
	}

	for (auto section = ruffle.sections.begin(); section != ruffle.sections.end(); ++section) {
		if (section->type != Ruffle::Section::Type::Interior)
			continue;
		real dist_a = target.signed_distance(ruffle.simulation_mesh.get_vertex_position(*section->start->mesh_vertex));
		real dist_b = target.signed_distance(ruffle.simulation_mesh.get_vertex_position(*section->end->mesh_vertex));

		// we have:          l ~= alpha * (l + dist_a + dist_b)
		//          (1-alpha)l ~= dist_a + dist_b
		// approach 1: iteration
		section->length = inner_ratio * (section->length + dist_a + dist_b);
		// approach 2: direct "solution" for l
		// TODO
	}

	for (auto section = ruffle.sections.begin(); section != ruffle.sections.end(); ++section) {
		section->length = max(section->length, min_length);
	}

	ruffle.update_simulation_mesh();
	ruffle.physics_solve();
}


void Heuristic::step_inner(Ruffle &ruffle) {
	for (auto section = ruffle.sections.begin(); section != ruffle.sections.end(); ++section) {
		if (section->type != Ruffle::Section::Type::Interior)
			continue;
		real dist_a = target.signed_distance(ruffle.simulation_mesh.get_vertex_position(*section->start->mesh_vertex));
		real dist_b = target.signed_distance(ruffle.simulation_mesh.get_vertex_position(*section->end->mesh_vertex));

		// we have:          l ~= alpha * (l + dist_a + dist_b)
		//          (1-alpha)l ~= dist_a + dist_b
		// approach 1: iteration
		section->length = inner_ratio * (section->length + dist_a + dist_b);
		// approach 2: direct "solution" for l
		// TODO
	}

	for (auto section = ruffle.sections.begin(); section != ruffle.sections.end(); ++section) {
		section->length = max(section->length, min_length);
	}

	ruffle.update_simulation_mesh();
	ruffle.physics_solve();
}

void Heuristic::step_outer(Ruffle &ruffle) {
	for (auto section = ruffle.sections.begin(); section != ruffle.sections.end(); ++section) {
		if (section->type != Ruffle::Section::Type::Outline)
			continue;
		real closest_distance = infinity;
		for (auto &seg : section->mesh_segments) {
			auto vertex = seg->start;
			Vector2 pos = ruffle.simulation_mesh.get_vertex_position(*vertex);
			
			//real x = target.intersect_horizontal(pos(1))[(i & 2) ? 0 : 1];
			closest_distance = min(closest_distance, target.signed_distance(pos));
			/*if (std::isfinite(x)) {
				closest_distance = min(closest_distance,
					(i & 2) ? (pos(0) - x) : (x - pos(0)));
			}*/
		}
		dbg(closest_distance);

		if (std::isfinite(closest_distance)) {
			section->length += eta_outer * closest_distance;
		}
	}


	for (auto section = ruffle.sections.begin(); section != ruffle.sections.end(); ++section) {
		section->length = max(section->length, min_length);
	}

	ruffle.update_simulation_mesh();
	ruffle.physics_solve();
}

}
