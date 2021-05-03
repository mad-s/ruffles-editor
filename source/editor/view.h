#pragma once

#include <igl/opengl/glfw/Viewer.h>

#include "model/data_model.h"
#include "editor/view_model.h"

using namespace ruffles::model;
namespace ruffles::editor {

class View
{
public:
	View(DataModel& data_model) : data_model(data_model) {};
	
	bool launch();
	void load_scene(const std::string scene_file);


private:
	DataModel& data_model;
	ViewModel  view_model;

	bool is_initialized = false;
	bool is_launched = false;

	bool initialize();
	void initialize_view_elements();
	void register_callbacks();

	bool callback_update_view(igl::opengl::glfw::Viewer& viewer);
	void callback_update_menu();

	bool callback_key_down(igl::opengl::glfw::Viewer& viewer, unsigned int key, int modifiers);
	bool callback_key_up(igl::opengl::glfw::Viewer& viewer, unsigned int key, int modifiers);

	bool callback_mouse_down(igl::opengl::glfw::Viewer& viewer, int button, int modifier);
	bool callback_mouse_move(igl::opengl::glfw::Viewer& viewer, int mouse_x, int mouse_y);
	bool callback_mouse_up(igl::opengl::glfw::Viewer& viewer, int button, int modifier);

	void try_load_model();

};

}