#pragma once

#include "editor/style/colors.h"

namespace ruffles::style
{
	// fill colors
	const Eigen::Vector3d fill_mesh = Colors::GRAY_ULTRALIGHT;
	const Eigen::Vector3d fill_ruffle = Colors::ACCENT_LIGHT;
	const Eigen::Vector3d fill_cuttingplane = Colors::GRAY_ULTRALIGHT;
	const Eigen::Vector3d fill_groundplane = Colors::GRAY_MID;
	const double saturation_selected = 1.0;
	const double saturation_unselected = 0.6;

	// line colors
	const Eigen::Vector3d wire_color = Colors::GRAY_MID;
	const Eigen::Vector3d line_color_plane = Colors::GRAY_LIGHT;
	const Eigen::Vector3d line_color_cutline = Colors::ACCENT;

	// line thicknesses
	const double wire_thickness = 0.5;
	const double line_thickness = 1.0;
	const double line_selected_thickness = line_thickness * 4.0;

	// point sizes
	const double points_small = 5.0;
	const double points_large = points_small * 2.0;

	// point colors
	const Eigen::Vector3d point_color = Colors::GRAY_DARK;
	const Eigen::Vector3d point_color_selected = Colors::ACCENT;
}