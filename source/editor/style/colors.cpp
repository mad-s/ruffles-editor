#include "editor/style/colors.h"

#include <igl/rgb_to_hsv.h>
#include <igl/hsv_to_rgb.h>

Eigen::Vector3d Colors::darker(Eigen::Vector3d color)
{
	return value(0.8, color);
}

Eigen::Vector3d Colors::ligher(Eigen::Vector3d color)
{
	return saturation(0.8, color);
}

Eigen::Vector3d Colors::value(double value, Eigen::Vector3d color)
{
	Eigen::Vector3d hsv;
	igl::rgb_to_hsv(color, hsv);
	hsv(2) = value;

	Eigen::Vector3d rgb;
	igl::hsv_to_rgb(hsv, rgb);
	return rgb;
}

Eigen::Vector3d Colors::saturation(double saturation, Eigen::Vector3d color)
{
	Eigen::Vector3d hsv;
	igl::rgb_to_hsv(color, hsv);
	hsv(1) = saturation;

	Eigen::Vector3d rgb;
	igl::hsv_to_rgb(hsv, rgb);
	return rgb;
}

Eigen::Vector4f Colors::to_4f(Eigen::Vector3d color)
{
	return Eigen::Vector4f(color(0), color(1), color(2), 1.0f);
}
