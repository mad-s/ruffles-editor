#include "editor/utils/transform.h"
#include <Eigen/Dense>

#include "editor/utils/logger.h"

namespace ruffles::utils {

	Eigen::Matrix3d get_tangent_space(const Eigen::Vector3d& N)
	{
		Eigen::Vector3d T = abs(N(0)) > .9 ? Eigen::Vector3d(0, 1, 0) : Eigen::Vector3d(1, 0, 0);
		T -= T.dot(N) * N;
		T.normalize();

		Eigen::Vector3d B = N.cross(T);
		write_log(4) << "  T = " << T.transpose() << ", N = " << N.transpose() << ", B = " << B.transpose() << std::endl;


		Eigen::Matrix3d frame;
		frame.col(0) = T;
		frame.col(1) = N;
		frame.col(2) = B;

		return frame;
	}

	Eigen::VectorXd get_dimensions(const Eigen::MatrixXd& V)
	{
		const Eigen::RowVectorXd min_point = V.colwise().minCoeff();
		const Eigen::RowVectorXd max_point = V.colwise().maxCoeff();
		return (max_point - min_point).eval();
	}

	Eigen::Vector3d get_centroid(const Eigen::MatrixXd& V)
	{
		const auto min_point = V.colwise().minCoeff();
		const auto max_point = V.colwise().maxCoeff();
		auto centroid = (0.5 * (min_point + max_point)).eval();

		return centroid;
	}

	void center_mesh(Eigen::MatrixXd& V)
	{
		auto centroid = get_centroid(V);
		Eigen::Vector3d shift;
		shift.setConstant(0);
		shift.head(centroid.size()) = -centroid.cast<double>();

		V = V.rowwise() + shift.transpose();
		write_log(4) << "translate = " << to_mathematica(shift.transpose()) << std::endl;
	}

}