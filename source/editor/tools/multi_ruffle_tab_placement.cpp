#include "editor/tools/multi_ruffle_tab_placement.h"

#include "editor/utils/view_utils.h"
#include "editor/utils/ruffle_view.h"
#include "editor/utils/logger.h"
#include "editor/style/colors.h"

namespace ruffles::editor {

MultiRuffleTabPlacement::MultiRuffleTabPlacement(ViewModel& view_model, DataModel& data_model) : AbstractTool(view_model, data_model)
{
	view_index = view_model.viewer.append_mesh();
	current_pair_index = 0;
	tool_type = RuffleTool::MultiRuffleConnect;
}

void MultiRuffleTabPlacement::update_view(igl::opengl::glfw::Viewer& viewer)
{
	viewer.selected_data_index = view_index;
	viewer.data().set_visible(is_enabled());

	if (!has_changed || !is_enabled())
		return;

	viewer.data().clear();

	if (view_model.ruffles_mesh.is_vertex_valid(pre_selected_vertex))
		viewer.data().add_points(view_model.ruffles_mesh.V().row(pre_selected_vertex), Colors::GRAY_LIGHT.transpose());

	for (auto pair : selected_tab_pairs)
	{
		if(view_model.ruffles_mesh.is_vertex_valid(pair.first))
			viewer.data().add_points(view_model.ruffles_mesh.V().row(pair.first), Colors::ACCENT.transpose());

		if (view_model.ruffles_mesh.is_vertex_valid(pair.second))
		{
			viewer.data().add_points(view_model.ruffles_mesh.V().row(pair.second), Colors::ACCENT.transpose());
			viewer.data().add_edges(view_model.ruffles_mesh.V().row(pair.first), view_model.ruffles_mesh.V().row(pair.second), Colors::ACCENT.transpose());
		}
	}

	//ruffles_mesh = utils::all_ruffles_mesh(data_model.parts); //TODO move this to mesh part, such that the ruffle mesh can always be retrieved and is updated only when the ruffle changed
	has_changed = false;
}

void MultiRuffleTabPlacement::update_menu(Menu& menu)
{
	//bool is_open = is_enabled();
	//if (!ImGui::CollapsingHeader("options: connect ruffles", &is_open, ImGuiTreeNodeFlags_None))
	//	return;
	if (!ImGui::CollapsingHeader("options: connect ruffles", auto_header_open()))
		return;

	ImGui::Indent();

	std::string info = "number of connectors: " + std::to_string(selected_tab_pairs.size());
	ImGui::Text(info.c_str());

	if (ImGui::Button("delete"))
		if (is_enabled() && selected_tab_pairs.size() > 0)
			selected_tab_pairs.pop_back();
	
	ImGui::Unindent();
}

bool MultiRuffleTabPlacement::callback_mouse_move(igl::opengl::glfw::Viewer& viewer, int mouse_x, int mouse_y)
{
	if (!is_enabled())
		return false;
	if (view_model.is_mouse_down)
		return false;

	pre_selected_vertex = utils::get_vertex_from_screen(viewer, mouse_x, mouse_y, view_model.ruffles_mesh.V(), view_model.ruffles_mesh.F());
	has_changed = true;

	return has_changed;
}

bool MultiRuffleTabPlacement::callback_mouse_up(igl::opengl::glfw::Viewer& viewer, int button, int modifier)
{
	if (!is_enabled())
		return false;
	if (!view_model.ruffles_mesh.is_vertex_valid(pre_selected_vertex))
		return false;

	int tab_index = current_pair_index / 2;
	int pair_index = current_pair_index % 2;

	if (pair_index == 0)
		selected_tab_pairs.push_back(std::pair(pre_selected_vertex, -1));
	else
		selected_tab_pairs[tab_index].second = pre_selected_vertex;

	current_pair_index++;
	has_changed = true;

	//TODO store the ruffle indices as you need them for adding the connectors

	return has_changed;
}

}