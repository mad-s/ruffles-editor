#pragma once

#include <vector>
#include <Eigen/Core>

namespace ruffles::editor
{

	struct Intersector
	{
		Eigen::MatrixXd get_cross_section(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const Eigen::Vector3d& plane_origin, const Eigen::Vector3d& plane_normal);
		std::vector<Eigen::MatrixXd> get_cross_sections(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const Eigen::Vector3d& plane_origin, const Eigen::Vector3d& plane_normal);
	};

}