#include "editor/utils/concatenate_meshes.h"

namespace ruffles::utils {


//TODO add type template
void append_matrix(const Eigen::MatrixXd& to_append, Eigen::MatrixXd& out_longer_matrix)
{
	if (!out_longer_matrix.rows())
	{
		out_longer_matrix = to_append;
		return;
	}

	const auto temp = out_longer_matrix;
	out_longer_matrix.resize(temp.rows() + to_append.rows(), 3);
	out_longer_matrix << temp, to_append;
}

void append_matrix(const Eigen::MatrixXi& to_append, Eigen::MatrixXi& out_longer_matrix)
{
	if (!out_longer_matrix.rows())
	{
		out_longer_matrix = to_append;
		return;
	}

	const auto temp = out_longer_matrix;
	out_longer_matrix.resize(temp.rows() + to_append.rows(), 3);
	out_longer_matrix << temp, to_append;
}


void concatenate_meshes(const std::vector<Eigen::MatrixXd>& list_V, const std::vector<Eigen::MatrixXi>& list_F, Eigen::MatrixXd& out_concatenated_V, Eigen::MatrixXi& out_concatenated_F)
{
	out_concatenated_V.resize(0,0);
	out_concatenated_F.resize(0,0);

	if (list_V.size() != list_F.size())
		return;

	for (int i = 0; i < list_V.size(); i++)
	{
		auto V = list_V[i];
		auto F = list_F[i];

		Eigen::RowVector3i offset(out_concatenated_V.rows(), out_concatenated_V.rows(), out_concatenated_V.rows());
		Eigen::MatrixXi offset_faces = F.rowwise() + offset;

		//out_concatenated_F << offset_faces;
		//out_concatenated_V << V;
		append_matrix(offset_faces, out_concatenated_F);
		append_matrix(V, out_concatenated_V);
	}
}


//Mesh meshhelper::concatenate_meshes(const std::vector<Eigen::MatrixXd>& list_V, const std::vector<Eigen::MatrixXi>& list_F)
//{
//	if (list_V.size() != list_F.size())
//		exit(EXIT_FAILURE);
//
//	Mesh concatenated;
//
//	for (int i = 0; i < list_V.size(); i++)
//	{
//		auto V = list_V[i];
//		auto F = list_F[i];
//
//		Eigen::RowVector3i offset(concatenated.V.rows(), concatenated.V.rows(), concatenated.V.rows());
//		Eigen::MatrixXi offset_faces = F.rowwise() + offset;
//
//		append_matrix(offset_faces, concatenated.F);
//		append_matrix(V, concatenated.V);
//	}
//
//	return concatenated;
//}
//
//Mesh meshhelper::concatenate_meshes(const std::vector<Mesh>& meshes)
//{
//	Mesh concatenated;
//	vector<int> cumulative_offset_vertices;
//	vector<int> cumulative_offset_faces;
//	concatenate_meshes(meshes, concatenated, cumulative_offset_vertices, cumulative_offset_faces);
//
//	return concatenated;
//}
//
//void meshhelper::concatenate_meshes(const std::vector<Mesh>& meshes, Mesh& out_concatenated, std::vector<int>& out_cumulative_offset_vertices, std::vector<int>& out_cumulative_offset_faces)
//{
//	out_concatenated.V.resize(0, 3);
//	out_concatenated.F.resize(0, 3);
//	out_cumulative_offset_vertices.clear();
//	out_cumulative_offset_faces.clear();
//
//	for (Mesh mesh : meshes)
//	{
//		int number_vertices = out_concatenated.V.rows();
//		out_cumulative_offset_vertices.push_back(number_vertices);
//		out_cumulative_offset_faces.push_back(out_concatenated.F.rows());
//
//		Eigen::RowVector3i offset(number_vertices, number_vertices, number_vertices);
//		Eigen::MatrixXi offset_faces = mesh.F.rowwise() + offset;
//
//		append_matrix(offset_faces, out_concatenated.F);
//		append_matrix(mesh.V, out_concatenated.V);
//	}
//}

}