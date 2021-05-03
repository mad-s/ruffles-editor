#pragma once
#include "common/common.h"
#include <igl/serialize.h>

namespace ruffles::model {

class Mesh : public igl::Serializable
{
public:

	Mesh() { };
	Mesh(Eigen::MatrixXd& V, Eigen::MatrixXi& F) : _V(V), _F(F) { };
	Mesh(Eigen::MatrixXd& V, Eigen::MatrixXi& F, Eigen::MatrixXd& NV) : _V(V), _F(F), _NV(NV)
	{
		_keep_NV = NV.rows() > 1;
	};

	virtual ~Mesh() {}


	Eigen::MatrixXd& V();
	void V(Eigen::MatrixXd& value);

	Eigen::MatrixXi& F();
	void F(Eigen::MatrixXi& value);

	Eigen::MatrixXd& NV();
	Eigen::MatrixXd& NF();

	std::vector<std::vector<int>>& adjacency_VV();
	std::vector<std::vector<int>>& adjacency_VF();
	Eigen::MatrixXi& adjacency_FF();

	bool is_valid();
	bool is_vertex_valid(int vertex_index);

	void clear();


private:

	Eigen::MatrixXd _V;
	Eigen::MatrixXi _F;

	Eigen::MatrixXd _NV;
	Eigen::MatrixXd _NF;

	std::vector<std::vector<int>> _adjacency_VV;
	std::vector<std::vector<int>> _adjacency_VF;
	Eigen::MatrixXi _adjacency_FF;

	bool _keep_NV;
	void update();

	// Inherited via Serializable
	virtual void InitSerialization() override;
};
}