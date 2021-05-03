#pragma once

#include <igl/opengl/glfw/Viewer.h>
#include "editor/menu.h"

namespace ruffles::editor {

class AbstractElement
{

public:
	AbstractElement(igl::opengl::glfw::Viewer& viewer, Menu& menu) : viewer(viewer), menu(menu) {};

	//int get_ID() { return ID; };

	virtual void update_view(igl::opengl::glfw::Viewer& viewer) = 0;
	virtual void update_menu(Menu& menu) = 0;

	virtual bool callback_key_down(igl::opengl::glfw::Viewer& viewer, unsigned int key, int modifiers) { return false; };
	virtual bool callback_key_up(igl::opengl::glfw::Viewer& viewer, unsigned int key, int modifiers) { return false; };

	virtual bool callback_mouse_down(igl::opengl::glfw::Viewer& viewer, int button, int modifier) { return false; };
	virtual bool callback_mouse_move(igl::opengl::glfw::Viewer& viewer, int mouse_x, int mouse_y) { return false; };
	virtual bool callback_mouse_up(igl::opengl::glfw::Viewer& viewer, int button, int modifier) { return false; };

	void mark_changed() {has_changed = true;}

protected:
	igl::opengl::glfw::Viewer& viewer;
	Menu& menu;

	//int ID = -1;
	bool has_changed = false;
};

}