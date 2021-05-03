#pragma once

#include "editor/tools/abstract_tool.h"


using namespace ruffles::model;
namespace ruffles::editor {

class MeshLoader : public AbstractTool
{

public:
	MeshLoader(ViewModel& view_model, DataModel& data_model);

	virtual void update_menu(Menu& menu) override;
	virtual void update_view(igl::opengl::glfw::Viewer& viewer) override;
	bool load_mesh(std::string file);
	bool write_mesh(std::string file);

private:

};

}