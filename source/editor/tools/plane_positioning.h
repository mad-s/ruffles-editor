#pragma once

#include "editor/tools/abstract_tool.h"
#include "model/model_part.h"
#include "model/plane.h"

using namespace ruffles::model;
namespace ruffles::editor {

enum PlaneType
{
	Cutting,
	Ground
};

class PlanePositioning : public AbstractTool
{

public:
	PlanePositioning(ViewModel& view_model, DataModel& data_model);

	bool is_visible = true;

	virtual void update_view(igl::opengl::glfw::Viewer& viewer) override;
	virtual void update_menu(Menu& menu) override;

	virtual bool callback_key_up(igl::opengl::glfw::Viewer& viewer, unsigned int key, int modifiers) override;


private:
	ModelPart* selected_part = NULL;
	Plane* selected_plane = NULL;
	PlaneType selected_plane_type = PlaneType::Cutting;

	std::vector<int> cut_view_indices;
	std::vector<int> ground_view_indices;

	bool is_groundplane_visible = false;
	bool is_cutplane_visible = true;


	void update_part_view(igl::opengl::glfw::Viewer& viewer, int part_index);
	void update_plane(igl::opengl::glfw::Viewer& viewer, Plane& plane);
	void update_selected_data();
	bool have_parts_changed();
	void perform_cut();

	void add_plane_wireframe(Eigen::MatrixXd V, Eigen::Vector3d color);
};

}