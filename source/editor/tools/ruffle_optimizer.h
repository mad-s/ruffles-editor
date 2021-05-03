#pragma once

#include "editor/tools/abstract_tool.h"
#include "model/model_part.h"
#include "model/plane.h"

#include "optimization/heuristic.h"

using namespace ruffles::model;
namespace ruffles::editor {

class RuffleOptimizer : public AbstractTool
{

public:
	RuffleOptimizer(ViewModel& view_model, DataModel& data_model);

	bool is_visible = true;

	virtual void update_view(igl::opengl::glfw::Viewer& viewer) override;
	virtual void update_menu(Menu& menu) override;

	virtual bool callback_key_up(igl::opengl::glfw::Viewer& viewer, unsigned int key, int modifiers) override;


private: 
	int prev_selected_part = -1;
	ModelPart* part = NULL;
	std::vector<int> view_indices;
	int ruffle_mesh_view_index = -1;


	void update_part_view(igl::opengl::glfw::Viewer& viewer, int part_index);
	bool have_parts_changed();
};

}