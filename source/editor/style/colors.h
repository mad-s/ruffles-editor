#pragma once

#include <Eigen/Core>


namespace Colors
{
	const Eigen::Vector3d BLACK				= Eigen::Vector3d(0.001, 0.001, 0.001);
	const Eigen::Vector3d WHITE				= Eigen::Vector3d(0.999, 0.999, 0.999);

	const Eigen::Vector3d GRAY_DARK			= Eigen::Vector3d(0.3, 0.3, 0.3);
	const Eigen::Vector3d GRAY_MID			= Eigen::Vector3d(0.5, 0.5, 0.5);
	const Eigen::Vector3d GRAY_LIGHT		= Eigen::Vector3d(0.7, 0.7, 0.7);
	const Eigen::Vector3d GRAY_ULTRALIGHT	= Eigen::Vector3d(0.85, 0.85, 0.85);

	const Eigen::Vector3d RED				= Eigen::Vector3d(0.999, 0.001, 0.001);
	const Eigen::Vector3d GREEN				= Eigen::Vector3d(0.001, 0.999, 0.001);
	const Eigen::Vector3d BLUE				= Eigen::Vector3d(0.001, 0.001, 0.999);
	
	const Eigen::Vector3d CYAN				= Eigen::Vector3d(0.001, 0.999, 0.999);
	const Eigen::Vector3d YELLOW			= Eigen::Vector3d(0.999, 0.999, 0.001);
	const Eigen::Vector3d MAGENTA			= Eigen::Vector3d(0.999, 0.001, 0.999);


	//#E6E2DF
	const Eigen::Vector3d TAUPE_LIGHT		= Eigen::Vector3d(230.0/255.0, 226.0/255.0, 223.0/255.0);

	//the office blue: #00B0F0
	const Eigen::Vector3d ACCENT			= Eigen::Vector3d(000.0/255.0, 176.0/255.0, 240.0/255.0);
	const Eigen::Vector3d ACCENT_LIGHT		= Eigen::Vector3d(207.0/255.0, 233.0/255.0, 243.0/255.0);
	const Eigen::Vector3d ACCENT_DARK		= Eigen::Vector3d(000.0/255.0, 094.0/255.0, 128.0/255.0);
	const Eigen::Vector3d ACCENT_GRAY		= Eigen::Vector3d(170.0/255.0, 221.0/255.0, 243.0/255.0);


	Eigen::Vector3d darker(Eigen::Vector3d color);
	Eigen::Vector3d ligher(Eigen::Vector3d color);

	Eigen::Vector3d value(double value, Eigen::Vector3d color);
	Eigen::Vector3d saturation(double value, Eigen::Vector3d color);

	Eigen::Vector4f to_4f(Eigen::Vector3d color);
}
