#include "editor/tools/force_positioning.h"

#include "editor/utils/ruffle_view.h"
#include "editor/utils/view_utils.h"
#include "editor/utils/logger.h"
#include "editor/style/colors.h"


namespace ruffles::editor {

ForcePositioning::ForcePositioning(ViewModel& view_model, DataModel& data_model) : AbstractTool(view_model, data_model)
{
	view_index = view_model.viewer.append_mesh();
	tool_type = RuffleTool::ForcePosition;

	view_model.viewer.data().line_width = 3;
}

void ForcePositioning::update_view(igl::opengl::glfw::Viewer& viewer)
{
	viewer.selected_data_index = view_index;
	viewer.data().set_visible(is_enabled());

	if (!has_changed || !is_enabled())
		return;

	viewer.data().clear();

	//TODO set force_position + force_direction in data_model or model_part 

	if (view_model.ruffles_mesh.is_vertex_valid(selected_vertex))
		viewer.data().add_edges(force_position, force_position + force_direction, Colors::ACCENT.transpose());

	if (view_model.ruffles_mesh.is_vertex_valid(pre_selected_vertex))
		viewer.data().add_edges(view_model.ruffles_mesh.V().row(pre_selected_vertex), view_model.ruffles_mesh.V().row(pre_selected_vertex) + default_direction, Colors::GRAY_LIGHT.transpose());

	//ruffles_mesh = utils::all_ruffles_mesh(data_model.parts); //TODO move this to mesh part, such that the ruffle mesh can always be retrieved and is updated only when the ruffle changed
	has_changed = false;
}

void ForcePositioning::update_menu(Menu& menu)
{
	//bool is_open = is_enabled();
	//if (!ImGui::CollapsingHeader("options: apply force", &is_open, ImGuiTreeNodeFlags_None))
	//	return;
	if (!ImGui::CollapsingHeader("options: apply force", auto_header_open()))
		return;


	ImGui::Indent();

	ImGui::Text("todo: add fields for numerical input");

	ImGui::Unindent();
}

bool ForcePositioning::callback_key_up(igl::opengl::glfw::Viewer& viewer, unsigned int key, int modifiers)
{
	if (!is_enabled())
		return false;
	if (!view_model.ruffles_mesh.is_vertex_valid(selected_vertex))
		return false;

	double t = modifiers == GLFW_MOD_SHIFT ? -1 : 1;
	t *= igl::PI / 180;

	if (key == GLFW_KEY_M)
	{
		double scale = modifiers == GLFW_MOD_SHIFT ? 0.9 : 1.1;
		update_magnitude(scale);
		has_changed = true;
	}
	else if (key == GLFW_KEY_X)
	{
		update_rotation(Eigen::Vector3d(t, 0, 0));
		has_changed = true;
	}
	else if (key == GLFW_KEY_Y)
	{
		update_rotation(Eigen::Vector3d(0, t, 0));
		has_changed = true;
	}
	else if (key == GLFW_KEY_Z)
	{
		update_rotation(Eigen::Vector3d(0, 0, t));
		has_changed = true;
	}

	return has_changed;
}

bool ForcePositioning::callback_mouse_move(igl::opengl::glfw::Viewer& viewer, int mouse_x, int mouse_y)
{
	if (!is_enabled())
		return false;
	if (view_model.is_mouse_down)
		return false;

	pre_selected_vertex = utils::get_vertex_from_screen(viewer, mouse_x, mouse_y, view_model.ruffles_mesh.V(), view_model.ruffles_mesh.F());
	has_changed = true;

	return has_changed;
}

bool ForcePositioning::callback_mouse_up(igl::opengl::glfw::Viewer& viewer, int button, int modifier)
{
	if (!is_enabled())
		return false;
	if (!view_model.ruffles_mesh.is_vertex_valid(pre_selected_vertex))
		return false;

	selected_vertex = pre_selected_vertex;
	force_direction = default_direction;
	force_position = view_model.ruffles_mesh.V().row(selected_vertex);

	has_changed = true;
	return has_changed;
}

void ForcePositioning::update_magnitude(double scale)
{
	force_direction *= scale;
}

void ForcePositioning::update_rotation(Eigen::Vector3d rotate_by)
{
	Eigen::AngleAxisd rollAngle(rotate_by(2), Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd yawAngle(rotate_by(1), Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd pitchAngle(rotate_by(0), Eigen::Vector3d::UnitX());

	Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
	Eigen::Matrix3d R = q.matrix();

	force_direction = force_direction * R;
}

}