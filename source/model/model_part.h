#pragma once

#include "model/mesh_model.h"
#include "model/plane.h"

#include "ruffle/ruffle.h"
#include "optimization/target_shape.h"
#include "optimization/heuristic.h"

#include <optional>

namespace ruffles::model {

	class ModelPart
	{
	public:
		friend class Serializer;

		ModelPart(Mesh& segment);
		// shut up c++
    	ModelPart(ModelPart&&) = default;
		virtual ~ModelPart() { };

		Mesh& segment(); //TODO rename to submesh, to not confuse with ruffle segments
		void segment(Mesh& value); //recut outline with reference plane (if exists)

		Eigen::MatrixXd& cutline();
		void cutline(Eigen::MatrixXd& value); //re-init ruffle (?)

		optimization::TargetShape &target(); //TODO rename, confusing with data_model.target()
		Ruffle &ruffle();
		void ruffle(Ruffle &&x);

		optimization::Heuristic heuristic;

		std::optional<Vector3> apex = {}; // for conical ruffles

		Plane& plane();
		Plane& ground_plane();

		real h = 0.5;
		real u0_ratio = 1./6.;
		int stack_count;
		real step_width;
		real step_height;
		bool horizontal;

		void reinit_ruffle();

		void clear();

		void add_child(ModelPart *child);
		void clear_children();

		void update_extra_masses();


		//TODO add groundplane for simulation
		void intersect_ruffle();

	private:
		Mesh _segment;

		vector<ModelPart*> _children;

		//polyline from plane cut, store only longest polyline
		optimization::TargetShape target_shape;
		Ruffle _ruffle;

		Plane _plane;
		Plane _ground_plane;

		void update();

		vector<real> intersect_at(Vector2 uv);
	};
}
