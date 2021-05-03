#include "editor/tools/plane_positioning.h"
#include "editor/tools/ruffle_optimizer.h"

#include "editor/utils/logger.h"
#include "editor/utils/view_utils.h"
#include "editor/utils/transform.h"
//#include "editor/style/colors.h"
#include "editor/style/editor_style.h"

#include "editor/tools/intersector_adapter.h"


namespace ruffles::editor {

PlanePositioning::PlanePositioning(ViewModel& view_model, DataModel& data_model) : AbstractTool(view_model, data_model)
{
	tool_type = RuffleTool::PlanePosition;
	update_selected_data();
}

void PlanePositioning::update_selected_data()
{
	if (view_model.selected_part_index < 0 || view_model.selected_part_index >= data_model.parts.size())
		return;

	selected_part = &data_model.parts[view_model.selected_part_index];

	if (selected_plane_type == PlaneType::Cutting)
		selected_plane = &selected_part->plane();
	else if (selected_plane_type == PlaneType::Ground)
		selected_plane = &selected_part->ground_plane();
}

void PlanePositioning::update_view(igl::opengl::glfw::Viewer& viewer)
{
	if (!is_enabled())
		return;
	if (!has_changed && !have_parts_changed() && !view_model.has_selected_part_changed)
		return;
	//if (!has_changed && !have_parts_changed())
	//	return;


	// clear all items
	for (int i = 0; i < cut_view_indices.size(); i++)
	{
		viewer.data_list[cut_view_indices[i]].clear();
		viewer.data_list[ground_view_indices[i]].clear();
	}


	if (view_model.is_only_selected_visible)
	{
		update_part_view(viewer, view_model.selected_part_index);

	}
	else
	{
		for (int i = 0; i < data_model.parts.size(); i++)
			update_part_view(viewer, i);
	}


	has_changed = false;
}

void PlanePositioning::update_menu(Menu& menu)
{
	update_selected_data();

	//bool is_open = is_enabled();
	//if (!ImGui::CollapsingHeader("options: plane positioning", &is_open, ImGuiTreeNodeFlags_None))
	//	return;
	if (!ImGui::CollapsingHeader("options: plane positioning", auto_header_open()))
		return;


	ImGui::Indent();

	if (ImGui::Checkbox("show planes", &is_visible))
		has_changed = true;

	if (ImGui::Checkbox("show cutting plane", &is_cutplane_visible))
		has_changed = true;
	ImGui::SameLine();
	if (ImGui::Checkbox("show ground plane", &is_groundplane_visible))
		has_changed = true;



	int choice_plane = selected_plane_type;
	if (ImGui::RadioButton("cutting plane", &choice_plane, PlaneType::Cutting))
	{
		selected_plane = &selected_part->plane();
		has_changed = true;
	}
	if (ImGui::RadioButton("ground plane", &choice_plane, PlaneType::Ground))
	{
		selected_plane = &selected_part->ground_plane();
		has_changed = true;
	}
	selected_plane_type = static_cast<PlaneType>(choice_plane);


	if (selected_part == NULL || selected_plane == NULL)
		menu.deactivate_elements();

	double rotation_step = 45.0 / 180.0 * igl::PI;
	ImGui::Text("rotate 45deg:"); ImGui::SameLine();
	if (ImGui::Button("around X"))
	{
		selected_plane->rotate_X = rotation_step;
		selected_plane->update_rotation();
		has_changed = true;
	}
	ImGui::SameLine();
	if (ImGui::Button("around Y"))
	{
		selected_plane->rotate_Y = rotation_step;
		selected_plane->update_rotation();
		has_changed = true;
	}
	ImGui::SameLine();
	if (ImGui::Button("around Z"))
	{
		selected_plane->rotate_Z = rotation_step;
		selected_plane->update_rotation();
		has_changed = true;
	}
	ImGui::SameLine();
	if (ImGui::Button("flip plane"))
	{
		selected_plane->flip();
		has_changed = true;
	}


	if (ImGui::Button("reset plane"))
	{
		selected_plane->reset();

		if(selected_plane_type == PlaneType::Cutting)
			selected_part->plane().align(selected_part->segment().V());
		else if(selected_plane_type == PlaneType::Ground)
			selected_part->ground_plane().align(selected_part->segment().V());
		
		has_changed = true;
	}


	if (selected_plane_type == PlaneType::Ground)
		menu.deactivate_elements();
		
	if (ImGui::Button("cut cross section") && selected_plane_type == PlaneType::Cutting)
		perform_cut();

	if (selected_plane_type == PlaneType::Ground)
		menu.activate_elements();


	if (selected_part == NULL || selected_plane == NULL)
		menu.activate_elements();

	ImGui::Unindent();
}

bool PlanePositioning::callback_key_up(igl::opengl::glfw::Viewer& viewer, unsigned int key, int modifiers)
{
	//write_log(4) << "PlanePositioning::callback_key_up key: " << key << ", modifier: " << modifiers << std::endl;
	if (!is_enabled())
		return false;
	if (selected_part == NULL)
		return false;
	if (selected_plane == NULL)
		return false;

	real transform_step = view_model.transform_step;
	real rotate_step = igl::PI / 180;

	if (modifiers & GLFW_MOD_CONTROL) {
		transform_step *= 0.1;
		rotate_step *= 0.1;
	}
	if (modifiers & GLFW_MOD_ALT) {
		transform_step *= 10.;
		rotate_step *= 10.;
	}

	if (modifiers & GLFW_MOD_SHIFT) {
		transform_step *= -1;
		rotate_step *= -1.;
	}

	if (key == GLFW_KEY_C && selected_plane_type == PlaneType::Cutting)
	{
		perform_cut();	
	}
	else if (key == GLFW_KEY_N)
	{
		selected_plane->translate_N += transform_step;

		selected_plane->update_translation();
		has_changed = true;
	}
	else if (key == GLFW_KEY_X)
	{
		selected_plane->rotate_X += rotate_step;

		selected_plane->update_rotation();
		has_changed = true;
	}
	else if (key == GLFW_KEY_Y)
	{
		selected_plane->rotate_Y += rotate_step;

		selected_plane->update_rotation();
		has_changed = true;
	}
	else if (key == GLFW_KEY_Z)
	{
		selected_plane->rotate_Z += rotate_step;

		selected_plane->update_rotation();
		has_changed = true;
	}

	return has_changed;
}

void PlanePositioning::update_part_view(igl::opengl::glfw::Viewer& viewer, int part_index)
{
	//add part if new
	if (cut_view_indices.size() <= part_index)
	{
		int cut_viewID = viewer.append_mesh();
		cut_view_indices.push_back(cut_viewID);

		int ground_viewID = viewer.append_mesh();
		ground_view_indices.push_back(ground_viewID);
	}

	ModelPart& current_part = data_model.parts[part_index];
	float current_line_width = part_index == view_model.selected_part_index ? style::line_selected_thickness : style::line_thickness;


	//update ground plane view
	if (is_groundplane_visible)
	{
		viewer.selected_data_index = ground_view_indices[part_index];

		viewer.data().clear();
		viewer.data().set_visible(is_visible);
		viewer.data().set_mesh(current_part.ground_plane().V, current_part.ground_plane().F);
		viewer.data().uniform_colors(Colors::GRAY_DARK, style::fill_groundplane, Colors::BLACK);
		viewer.data().grid_texture();
		viewer.data().line_width = current_line_width;
		viewer.data().line_color = Colors::to_4f(style::line_color_plane);
		viewer.data().show_lines = false;
		add_plane_wireframe(current_part.ground_plane().V, style::line_color_plane);
	}

	//update cutting plane view
	if (is_cutplane_visible)
	{
		viewer.selected_data_index = cut_view_indices[part_index];

		viewer.data().clear();
		viewer.data().set_visible(is_visible);
		viewer.data().set_mesh(current_part.plane().V, current_part.plane().F);
		viewer.data().uniform_colors(Colors::GRAY_DARK, style::fill_cuttingplane, Colors::BLACK);
		viewer.data().grid_texture();
		viewer.data().line_width = current_line_width;
		viewer.data().line_color = Colors::to_4f(style::line_color_plane);
		viewer.data().show_lines = false;
		add_plane_wireframe(current_part.plane().V, style::line_color_plane);
	}

	//update outline
	if (current_part.cutline().rows() > 2)
	{
		Eigen::MatrixXd edges_start, edges_end;
		utils::get_curve_edges(current_part.cutline(), edges_start, edges_end);
		view_model.viewer.data().add_edges(edges_start, edges_end, style::line_color_cutline.transpose());
	}
}

bool PlanePositioning::have_parts_changed()
{
	return cut_view_indices.size() != data_model.parts.size();
}

void PlanePositioning::update_plane(igl::opengl::glfw::Viewer& viewer, Plane& plane)
{
	//viewer.data().clear();
	//viewer.data().set_visible(is_visible);


	////update plane
	//viewer.data().set_mesh(plane.V, plane.F);
	//viewer.data().uniform_colors(Colors::GRAY_DARK, Colors::GRAY_LIGHT, Colors::BLACK);
	//viewer.data().grid_texture();
}

void PlanePositioning::perform_cut() 
{
	auto cutline = selected_part->plane().cut(selected_part->segment());
	selected_part->cutline(cutline);
	has_changed = true;

	for (auto element : view_model.elements) {
		if (auto x = dynamic_cast<RuffleOptimizer*>(element)) {
			x->mark_changed();
		}
	}
}

void PlanePositioning::add_plane_wireframe(Eigen::MatrixXd V, Eigen::Vector3d color)
{
	Eigen::MatrixXd edges_start, edges_end;
	utils::get_curve_edges(V, edges_start, edges_end);

	view_model.viewer.data().add_edges(edges_start, edges_end, color.transpose());
}

}