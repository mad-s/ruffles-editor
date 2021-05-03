#include "model/plane.h"

#include "editor/utils/transform.h"
#include "editor/tools/intersector_adapter.h"

#include "editor/utils/logger.h"

#include <Eigen/SVD>

namespace ruffles::model {

Plane::Plane()
{
	initialize();
}

void Plane::initialize()
{
	Eigen::Matrix3d tangent = ruffles::utils::get_tangent_space(normal()); //TNB

	V.resize(4, 3);
	F.resize(2, 3);
	V << Eigen::RowVector3d::Zero(), tangent.col(0).transpose(), (tangent.col(0) + tangent.col(2)).transpose(), tangent.col(2).transpose();
	F << Eigen::RowVector3i(0, 1, 2), Eigen::RowVector3i(0, 2, 3);

	V0 = V;
}

void Plane::reset()
{
	V = V0;
	t = Eigen::RowVector3d::Zero();
	R = Eigen::Matrix3d::Identity();

	translate_N = 0.0;
	rotate_X = 0.0;
	rotate_Y = 0.0;
	rotate_Z = 0.0;
}

Eigen::Vector3d Plane::origin()
{
	if (V.rows() < 1)
		return Eigen::Vector3d(0, 0, 0);

	Eigen::Vector3d e0 = (V.row(1) - V.row(0)).eval() * 0.5;
	Eigen::Vector3d e1 = (V.row(3) - V.row(0)).eval() * 0.5;
	return V.row(0).transpose() + (e0+e1);
}

Eigen::Vector3d Plane::normal()
{
	if (V.rows() < 1)
		return Eigen::Vector3d(0, 0, 1);

	Eigen::Vector3d e0 = V.row(1) - V.row(0);
	Eigen::Vector3d e1 = V.row(3) - V.row(0);
	return e0.cross(e1).normalized();
}

void Plane::align(const Eigen::MatrixXd& V_target)
{
	auto target_centroid = utils::get_centroid(V_target);
	
	//scale to fit size
	auto dim = utils::get_dimensions(V_target);
	dim *= 1.2;
	V.col(0) *= dim.row(0);
	V.col(1) *= dim.row(1);
	V.col(2) *= dim.row(2);
	
	//move to match centroid
	auto translate = (target_centroid - utils::get_centroid(V)).eval();
	V = V.rowwise() + translate.transpose();
}

void Plane::align_pca(const Eigen::MatrixXd& V_target)
{
	auto mu = V_target.colwise().mean();
	MatrixX centered = V_target.rowwise() - mu;
	auto svd = centered.transpose().jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
	auto U = 5*svd.matrixU();

	V.row(0) = mu.transpose() - U.col(0) - U.col(1);
	V.row(1) = mu.transpose() + U.col(0) - U.col(1);
	V.row(2) = mu.transpose() + U.col(0) + U.col(1);
	V.row(3) = mu.transpose() - U.col(0) + U.col(1);
}

void Plane::flip()
{
	V.row(1).swap(V.row(3));
}

Eigen::MatrixXd Plane::cut(Mesh& mesh)
{
	ruffles::editor::Intersector intersector;
	auto cutline = intersector.get_cross_section(mesh.V(), mesh.F(), V.row(0), normal());
	return cutline;
}

void Plane::update_translation()
{
	V = V.rowwise() + (normal().transpose() * translate_N);
	translate_N = 0.0;
}

void Plane::update_rotation()
{
	Eigen::AngleAxisd rollAngle(rotate_Z, Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd yawAngle(rotate_Y, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd pitchAngle(rotate_X, Eigen::Vector3d::UnitX());

	Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
	R = q.matrix();

	Eigen::RowVector3d current_t = origin().transpose();
	V = V.rowwise() - current_t;
	V = V * R;
	V = V.rowwise() + current_t;

	rotate_X = 0.0;
	rotate_Y = 0.0;
	rotate_Z = 0.0;
}

void Plane::clear()
{
	V0.resize(0, Eigen::NoChange);
	V.resize(0, Eigen::NoChange);
	F.resize(0, Eigen::NoChange);

	reset();
}

void Plane::InitSerialization()
{
	this->Add(V0, "V0");
	this->Add(V, "V");
	this->Add(F, "F");
}

}