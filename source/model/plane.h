#pragma once

#include <Eigen/Dense>
#include <igl/serialize.h>

#include <model/mesh_model.h>

namespace ruffles::model {

struct Plane : public igl::Serializable
{
public: 
	Plane();

	Eigen::MatrixXd V;
	Eigen::MatrixXi F;

	double translate_N = 0.0;
	double rotate_X = 0.0;
	double rotate_Y = 0.0;
	double rotate_Z = 0.0;

	Eigen::Vector3d origin(); 
	Eigen::Vector3d normal();

	void reset();
	void align(const Eigen::MatrixXd& V);
	void align_pca(const Eigen::MatrixXd &V);
	void flip();
	Eigen::MatrixXd cut(Mesh& value);

	void update_translation();
	void update_rotation();

	void clear();

private:
	Eigen::MatrixXd V0;

	//transform
	Eigen::RowVector3d t;
	Eigen::Matrix3d R;

	void initialize();

	// Inherited via Serializable
	virtual void InitSerialization() override;
};

}
