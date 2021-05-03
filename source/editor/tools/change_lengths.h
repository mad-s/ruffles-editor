#pragma once

#include "editor/tools/abstract_tool.h"
#include "model/data_model.h"

using namespace ruffles::model;
namespace ruffles::editor {


// maybe i'm to stupid to type, but c++ doesn't like it when i name this tool ChangeLengths for some reason
// i've tried for like 20 minutes
class SomeTool : public AbstractTool
{
public:
	SomeTool(ViewModel& view_model, DataModel& data_model);

	virtual void update_view(igl::opengl::glfw::Viewer& viewer) override;
	virtual void update_menu(Menu& menu) override;

	virtual bool callback_key_up(igl::opengl::glfw::Viewer& viewer, unsigned int key, int modifiers) override;
	virtual bool callback_mouse_move(igl::opengl::glfw::Viewer& viewer, int mouse_x, int mouse_y) override;
	virtual bool callback_mouse_up(igl::opengl::glfw::Viewer& viewer, int button, int modifier) override;

private:
	int view_index = -1;

	int pre_selected_vertex = -1;
};

}