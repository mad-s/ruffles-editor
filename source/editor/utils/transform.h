#pragma once

#include <Eigen/Core>

namespace ruffles::utils {

	Eigen::Matrix3d get_tangent_space(const Eigen::Vector3d& N);

	Eigen::VectorXd get_dimensions(const Eigen::MatrixXd& V);
	Eigen::Vector3d get_centroid(const Eigen::MatrixXd& V);
	void center_mesh(Eigen::MatrixXd& V);

}