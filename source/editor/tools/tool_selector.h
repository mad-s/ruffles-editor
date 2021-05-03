#pragma once

#include "editor/tools/abstract_tool.h"
#include "model/model_part.h"

using namespace ruffles::model;
namespace ruffles::editor {

class ToolSelector : public AbstractTool
{

public:
	ToolSelector(ViewModel& view_model, DataModel& data_model) : AbstractTool(view_model, data_model) {};

	virtual void update_view(igl::opengl::glfw::Viewer& viewer) override;
	virtual void update_menu(Menu& menu) override;
};
}