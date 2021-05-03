#include "model/mesh_model.h"

#include <igl/per_vertex_normals.h>
#include <igl/per_face_normals.h>

#include <igl/adjacency_list.h>
#include <igl/vertex_triangle_adjacency.h>
#include <igl/triangle_triangle_adjacency.h>


namespace ruffles::model {

Eigen::MatrixXd& Mesh::V()
{
	return _V;
}

void Mesh::V(Eigen::MatrixXd& value)
{
	_V = value;
	update();
}

Eigen::MatrixXi& Mesh::F()
{
	return _F;
}
void Mesh::F(Eigen::MatrixXi& value)
{
	_F = value;
	update();
}

Eigen::MatrixXd& Mesh::NV()
{
	if (_NV.rows() < 1)
		igl::per_vertex_normals(_V, _F, igl::PER_VERTEX_NORMALS_WEIGHTING_TYPE_AREA, _NV);

	return _NV;
}

Eigen::MatrixXd& Mesh::NF()
{
	if (_NF.rows() < 1)
		igl::per_face_normals(_V, _F, _NF);

	return _NF;
}

std::vector<std::vector<int>>& Mesh::adjacency_VV()
{
	if (_adjacency_VV.size() < 1)
		igl::adjacency_list(_F, _adjacency_VV);

	return _adjacency_VV;
}

std::vector<std::vector<int>>& Mesh::adjacency_VF()
{
	if (_adjacency_VF.size() < 1)
	{
		std::vector<std::vector<int>> VFi_unused;
		igl::vertex_triangle_adjacency(_V, _F, _adjacency_VF, VFi_unused);
	}

	return _adjacency_VF;
}
Eigen::MatrixXi& Mesh::adjacency_FF()
{
	if (_adjacency_FF.size() < 1)
		igl::triangle_triangle_adjacency(_F, _adjacency_FF);

	return _adjacency_FF;
}

bool Mesh::is_valid()
{
	return _V.rows() > 0 && _F.rows() > 0;
}

bool Mesh::is_vertex_valid(int vertex_index)
{
	if (!is_valid())
		return false;

	return vertex_index >= 0 && vertex_index < V().rows();
}

void Mesh::clear()
{
	_V.resize(0, Eigen::NoChange);
	_F.resize(0, Eigen::NoChange);

	update();
	_NV.resize(0, Eigen::NoChange);
}

void Mesh::update()
{ 
	if(!_keep_NV)
		_NV.resize(0, Eigen::NoChange);
	_NF.resize(0, Eigen::NoChange);

	_adjacency_VV.clear();
	_adjacency_VF.clear();
	_adjacency_FF.resize(0, Eigen::NoChange);
}

void Mesh::InitSerialization()
{
	this->Add(_V, "V");
	this->Add(_F, "F");
}

}
