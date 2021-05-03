#include "editor/tools/intersector_adapter.h"
#include "editor/utils/logger.h"

#include "MeshPlaneIntersect.hpp"
typedef MeshPlaneIntersect<double, int> ExternIntersector;

namespace ruffles::editor
{
std::vector<ExternIntersector::Face>  convert(const Eigen::MatrixXi& matrix);
std::vector<ExternIntersector::Vec3D> convert(const Eigen::MatrixXd& matrix);
Eigen::MatrixXd convert(const std::vector<ExternIntersector::Vec3D>& extern_matrix);

ExternIntersector::Vec3D convert_to_extern(const Eigen::RowVector3d& vector);

double get_length(Eigen::MatrixXd line);


Eigen::MatrixXd Intersector::get_cross_section(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const Eigen::Vector3d& plane_origin, const Eigen::Vector3d& plane_normal)
{
	std::vector<Eigen::MatrixXd> result = get_cross_sections(V, F, plane_origin, plane_normal);
	if (result.size() == 1)
		return result[0];

	double max_length = get_length(result[0]);
	int i_max = 0;

	for (int i = 1; i < result.size(); i++)
	{
		double l = get_length(result[i]);
		if (l <= max_length)
			continue;

		max_length = l;
		i_max = i;
	}

	return result[i_max];
}

std::vector<Eigen::MatrixXd> Intersector::get_cross_sections(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const Eigen::Vector3d& plane_origin, const Eigen::Vector3d& plane_normal)
{
	std::vector<ExternIntersector::Vec3D> externV = convert(V);
	std::vector<ExternIntersector::Face>  externF = convert(F);
	ExternIntersector::Mesh intersect_mesh(externV, externF);

	ExternIntersector::Plane intersect_plane;
	intersect_plane.origin = convert_to_extern(plane_origin);
	intersect_plane.normal = convert_to_extern(plane_normal);

	std::vector<ExternIntersector::Path3D> result = intersect_mesh.Intersect(intersect_plane);
	write_log(4) << "Cross section:: found " << result.size() << " intersecting parts." << std::endl;

	std::vector<Eigen::MatrixXd> outlines;
	for (int i = 0; i < result.size(); i++)
		outlines.push_back(convert(result[i].points));

	return outlines;
}

std::vector<ExternIntersector::Face> convert(const Eigen::MatrixXi& matrix)
{
	std::vector<ExternIntersector::Face> extern_matrix(matrix.rows());
	for (int r = 0; r < matrix.rows(); r++)
		for (int c = 0; c < 3; c++)
			extern_matrix[r][c] = matrix(r,c);

	return extern_matrix;
}

std::vector<ExternIntersector::Vec3D> convert(const Eigen::MatrixXd& matrix)
{
	std::vector<ExternIntersector::Vec3D> extern_matrix(matrix.rows());
	for (int r = 0; r < matrix.rows(); r++)
		for (int c = 0; c < 3; c++)
			extern_matrix[r][c] = matrix(r, c);

	return extern_matrix;
}

Eigen::MatrixXd convert(const std::vector<ExternIntersector::Vec3D>& extern_matrix)
{
	Eigen::MatrixXd matrix(extern_matrix.size(), 3);
	for (int r = 0; r < matrix.rows(); r++)
		for (int c = 0; c < 3; c++)
			matrix(r, c) = extern_matrix[r][c];

	return matrix;
}

ExternIntersector::Vec3D convert_to_extern(const Eigen::RowVector3d& vector)
{
	std::array<double, 3> a;
	for (int i = 0; i < 3; i++)
		a[i] = vector[i];

	return a;
}

double get_length(Eigen::MatrixXd line)
{
	double l = 0.0;
	for (int i = 1; i < line.rows(); i++)
		l += (line.row(i - 1) - line.row(i)).norm();

	return l;
}

}
