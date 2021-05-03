#pragma once

#include <igl/opengl/glfw/Viewer.h>

namespace ruffles::utils {

	int get_vertex_from_screen(igl::opengl::glfw::Viewer& viewer, int mouse_x, int mouse_y, const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);

	void get_curve_edges(const Eigen::MatrixXd& curve, Eigen::MatrixXd& out_start, Eigen::MatrixXd& out_end);
	void show_curve(igl::opengl::glfw::Viewer& viewer, const Eigen::MatrixXd& curve, const Eigen::RowVector3d& color, bool show_points = false);
	void render_highlighted_curves(igl::opengl::glfw::Viewer& viewer, const std::vector<Eigen::MatrixXd>& curves, const int selected_index, const Eigen::RowVector3d& color, const bool show_points = false);

}