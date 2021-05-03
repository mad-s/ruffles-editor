#pragma once

#include "common/common.h"
#include "common/cgal_util.h"

namespace ruffles::optimization {
	class TargetShape;
}

#include "ruffle/ruffle.h"

namespace ruffles::optimization {

class TargetShape {
public:
	Polygon target;

	Matrix<real, -1, -1> V;
	Matrix<int, -1, -1> F;

	Vector3 origin = Vector3(0.,0.,0.);
	Vector3 u_dir  = Vector3(1.,0.,0.);
	Vector3 v_dir  = Vector3(0.,1.,0.);

	real k = 1.0;
	real lambda = 1e3;

	TargetShape();
	TargetShape(Polygon);
	TargetShape(Eigen::MatrixXd &cut_shape, Vector3 origin, Vector3 u_dir, Vector3 v_dir);

	real height();
	real avg_width();

	real energy(Ruffle &ruffle);
	array<real,2> intersect_horizontal(real height);
	real signed_distance(Vector2 pos);
	real raycast(Vector2 ro, Vector2 rd);
};

}
