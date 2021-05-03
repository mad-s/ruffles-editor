#pragma once

#include "editor/elements/abstract_element.h"
#include "editor/view_model.h"
#include "model/data_model.h"

using namespace ruffles::model;
namespace ruffles::editor {

class AbstractTool : public AbstractElement
{
public:
	//AbstractTool(ViewModel& view_model) : view_model(view_model) {};
	AbstractTool(ViewModel& view_model, DataModel& data_model) : AbstractElement(view_model.viewer, view_model.menu), view_model(view_model), data_model(data_model) {};

	virtual void update_view(igl::opengl::glfw::Viewer& viewer) = 0;
	virtual void update_menu(Menu& menu) = 0;

	virtual bool is_enabled()
	{
		return view_model.active_tool == tool_type;
	}


protected:
	ViewModel& view_model;
	DataModel& data_model;
	RuffleTool tool_type;

	ImGuiTreeNodeFlags_ auto_header_open()
	{
		if (is_enabled())
			return ImGuiTreeNodeFlags_DefaultOpen;

		return ImGuiTreeNodeFlags_None;
	}
};

}