#pragma once

#include <vector>
#include <Eigen/Core>

namespace ruffles::utils {

	void concatenate_meshes(const std::vector<Eigen::MatrixXd>& list_V, const std::vector<Eigen::MatrixXi>& list_F, Eigen::MatrixXd& out_concatenated_V, Eigen::MatrixXi& out_concatenated_F);

}