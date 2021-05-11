#include "editor/menu.h"

#include <igl/opengl/glfw/Viewer.h>
#include <imgui/imgui_internal.h>

namespace ruffles::editor {

void Menu::init(igl::opengl::glfw::Viewer* _viewer)
{
    ImGuiMenu::init(_viewer);

    ImGui::StyleColorsLight();
    ImGuiStyle& style = ImGui::GetStyle();
    style.FrameRounding = 0.0f;
	style.WindowRounding = 0.0f;// <- Set this on init or use ImGui::PushStyleVar()
	style.ChildRounding = 0.0f;
	style.GrabRounding = 0.0f;
	style.PopupRounding = 0.0f;
	style.ScrollbarRounding = 0.0f;
}

void Menu::init_size(float _width, float _offset_x, float _offset_y, std::string _menu_name)
{
	width = _width;
	offset_x = _offset_x;
	offset_y = _offset_y;
	menu_name = _menu_name;
}

void Menu::begin_menu()
{
	float menu_width = width; // *data_model.menu.menu_scaling();
	ImGui::SetNextWindowPos(ImVec2(offset_x, offset_y), ImGuiCond_FirstUseEver);
	//ImGui::SetNextWindowSize(ImVec2(menu_width, 0.0f), ImGuiCond_FirstUseEver);
	ImGui::SetNextWindowSizeConstraints(ImVec2(menu_width, -1.0f), ImVec2(menu_width, -1.0f));
	bool _viewer_menu_visible = true;

	ImGui::Begin(menu_name.c_str(), &_viewer_menu_visible, ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_AlwaysAutoResize);
	ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.4f);
}

void Menu::end_menu()
{
	ImGui::PopItemWidth();
	ImGui::End();
}

void Menu::deactivate_elements()
{
	ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
	ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
}

void Menu::activate_elements()
{
	ImGui::PopItemFlag();
	ImGui::PopStyleVar();
}


} //end namespace