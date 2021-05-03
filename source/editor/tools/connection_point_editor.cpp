#include "editor/tools/connection_point_editor.h"

#include "editor/utils/ruffle_view.h"
#include "editor/utils/view_utils.h"
#include "editor/utils/logger.h"
#include "editor/style/colors.h"

namespace ruffles::editor {

ConnectionPointEditor::ConnectionPointEditor(ViewModel& view_model, DataModel& data_model) : AbstractTool(view_model, data_model) 
{
	view_index = view_model.viewer.append_mesh();
	tool_type = RuffleTool::ConnectionPointEdit;

	view_model.viewer.data().point_size = 15;
}

void ConnectionPointEditor::update_view(igl::opengl::glfw::Viewer& viewer)
{
	viewer.selected_data_index = view_index;
	viewer.data().set_visible(is_enabled());

	if (!is_enabled())
		return;
	if (!view_model.has_selected_part_changed && !has_changed)
		return;

	viewer.data().clear();

	if (view_model.ruffles_mesh.is_vertex_valid(pre_selected_vertex))
		viewer.data().add_points(view_model.ruffles_mesh.V().row(pre_selected_vertex), Colors::GRAY_LIGHT.transpose());
	
	if (view_model.ruffles_mesh.is_vertex_valid(selected_vertex))
		viewer.data().add_points(view_model.ruffles_mesh.V().row(selected_vertex), Colors::ACCENT.transpose());

	//ruffles_mesh = utils::all_ruffles_mesh(data_model.parts); //TODO replace this by creating the mesh as needed
	has_changed = false;
}

void ConnectionPointEditor::update_menu(Menu& menu)
{
	//bool is_open = is_enabled();
	//if (!ImGui::CollapsingHeader("options: connection moving", &is_open, ImGuiTreeNodeFlags_None))
	//	return;
	if (!ImGui::CollapsingHeader("options: connection moving", auto_header_open()))
		return;

	ImGui::Indent();
	ImGui::Unindent();
}

bool ConnectionPointEditor::callback_mouse_move(igl::opengl::glfw::Viewer& viewer, int mouse_x, int mouse_y)
{
	if (!is_enabled())
		return false;
	if (view_model.is_mouse_down)
		return false;

	//TODO this should snap to connection points, 
	//TODO get_vertex_from_screen() nees a mesh (V,F) (you can create a mesh that consists of the connection point of all ruffles)

	pre_selected_vertex = utils::get_vertex_from_screen(viewer, mouse_x, mouse_y, view_model.ruffles_mesh.V(), view_model.ruffles_mesh.F());
	has_changed = true;

	return has_changed;
}

bool ConnectionPointEditor::callback_mouse_up(igl::opengl::glfw::Viewer& viewer, int button, int modifier)
{
	if (!is_enabled())
		return false;
	if (!view_model.ruffles_mesh.is_vertex_valid(pre_selected_vertex))
		return false;

	selected_vertex = pre_selected_vertex;
	has_changed = true;

	return has_changed;
}

bool ConnectionPointEditor::callback_key_up(igl::opengl::glfw::Viewer& viewer, unsigned int key, int modifiers)
{
	if (!is_enabled())
		return false;
	if (!view_model.ruffles_mesh.is_vertex_valid(selected_vertex))
		return false;

	//TODO extract the connection point for moving 

	if (key == GLFW_KEY_LEFT)
	{
		//TODO move connection point by -1
		write_log(0) << "TODO move connection point by -1" << linebreak;
	}
	else if (key == GLFW_KEY_RIGHT)
	{
		//TODO move connection point by +1
		write_log(0) << "TODO move connection point by +1" << linebreak;
	}

	return false;
}

}