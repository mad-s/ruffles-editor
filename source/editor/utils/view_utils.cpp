#include "editor/utils/view_utils.h"

#include <igl/unproject_in_mesh.h>
#include "editor/style/colors.h"


namespace ruffles::utils {

int get_vertex_from_screen(igl::opengl::glfw::Viewer& viewer, int mouse_x, int mouse_y, const Eigen::MatrixXd& V, const Eigen::MatrixXi& F)
{
	// Cast a ray in the view direction starting from the mouse position
	double x = mouse_x;
	double y = viewer.core().viewport(3) - mouse_y;

	Eigen::RowVector3d pt;
	std::vector<igl::Hit> hits;

	igl::unproject_in_mesh(Eigen::Vector2f(x, y), viewer.core().view, viewer.core().proj, viewer.core().viewport, V, F, pt, hits);

	int vi = -1;
	if (hits.size() <= 0)
		return vi;

	int fi = hits[0].id;
	Eigen::RowVector3d bc;
	bc << 1.0 - hits[0].u - hits[0].v, hits[0].u, hits[0].v;

	auto coeff = bc.maxCoeff(&vi);
	//write_log(0) << endl << "get_vertex_from_screen: hits.size() = " << hits.size() << ", max coeff = " << coeff << endl;

	vi = F(fi, vi);
	return vi;
}

void get_curve_edges(const Eigen::MatrixXd& curve, Eigen::MatrixXd& out_start, Eigen::MatrixXd& out_end)
{
	if (!curve.rows())
		return;

	out_start.resize(curve.rows(), curve.cols());
	out_end.resize(curve.rows(), curve.cols());

	for (int j = 0; j < curve.rows() - 1; j++)
	{
		out_start.row(j) = curve.row(j);
		out_end.row(j) = curve.row(j + 1);
	}
	out_start.bottomRows<1>() = curve.bottomRows<1>();
	out_end.bottomRows<1>() = curve.topRows<1>();
}

void show_curve(igl::opengl::glfw::Viewer& viewer, const Eigen::MatrixXd& curve, const Eigen::RowVector3d& color, bool show_points)
{
	Eigen::MatrixXd edges_start, edges_end;
	get_curve_edges(curve, edges_start, edges_end);

	viewer.data().add_edges(edges_start, edges_end, color);

	if (show_points)
		viewer.data().add_points(curve, color);
}

void render_highlighted_curves(igl::opengl::glfw::Viewer& viewer, const std::vector<Eigen::MatrixXd>& curves, const int selected_index, const Eigen::RowVector3d& color, const bool show_points)
{
	if (curves.size() < 1)
		return;

	const auto color_unselected = Colors::darker(Colors::darker(color));

	//render the selected curve
	if (selected_index >= 0 && selected_index < curves.size())
		show_curve(viewer, curves[selected_index], color, show_points);

	//render all other curves
	for (int i = 0; i < curves.size(); i++)
	{
		if (i != selected_index)
			show_curve(viewer, curves[i], color_unselected, show_points);
	}
}

}