#pragma once

#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>

namespace ruffles::editor {

class Menu : public igl::opengl::glfw::imgui::ImGuiMenu
{
public:
	virtual void init(igl::opengl::glfw::Viewer* _viewer) override;
	void init_size(float width, float offset_x, float offset_y, std::string menu_name);

	void begin_menu();
	void end_menu();

	void deactivate_elements();
	void activate_elements();


private:
	float width; 
	float offset_x;
	float offset_y;
	std::string menu_name;
};

}