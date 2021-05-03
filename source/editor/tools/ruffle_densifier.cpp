#include "editor/tools/ruffle_densifier.h"

#include "editor/utils/view_utils.h"
#include "editor/utils/ruffle_view.h"
#include "editor/utils/logger.h"
#include "editor/style/editor_style.h"

#include "editor/tools/ruffle_optimizer.h"


namespace ruffles::editor {

RuffleDensifier::RuffleDensifier(ViewModel& view_model, DataModel& data_model) : AbstractTool(view_model, data_model)
{
	view_index = view_model.viewer.append_mesh();
	tool_type = RuffleTool::DensityEdit;

	viewer.data().point_size = style::points_large;
}

void RuffleDensifier::update_view(igl::opengl::glfw::Viewer& viewer)
{
	viewer.selected_data_index = view_index;
	viewer.data().set_visible(is_enabled());

	if (!has_changed || !is_enabled())
		return;

	viewer.data().clear();

	if (view_model.ruffles_mesh.is_vertex_valid(pre_selected_vertex))
		viewer.data().add_points(view_model.ruffles_mesh.V().row(pre_selected_vertex), style::point_color.transpose());

	//TODO if time, highlight the ruffle section (by rendering the selected ruffle mesh (V,F)) instead of the vertex

	//ruffles_mesh = utils::all_ruffles_mesh(data_model.parts); //TODO move this to mesh part, such that the ruffle mesh can always be retrieved and is updated only when the ruffle changed
	has_changed = false;
}

void RuffleDensifier::update_menu(Menu& menu)
{
	//bool is_open = is_enabled();
	//if (!ImGui::CollapsingHeader("options: connect ruffles", &is_open, ImGuiTreeNodeFlags_None))
	//	return;
	if (!ImGui::CollapsingHeader("options: densify ruffles", auto_header_open()))
		return;


	ImGui::Indent();
	ImGui::Unindent();
}

bool RuffleDensifier::callback_key_up(igl::opengl::glfw::Viewer& viewer, unsigned int key, int modifiers)
{
	if (!is_enabled())
		return false;


	return false;
}

bool RuffleDensifier::callback_mouse_move(igl::opengl::glfw::Viewer& viewer, int mouse_x, int mouse_y)
{
	if (!is_enabled())
		return false;
	if (view_model.is_mouse_down)
		return false;

	pre_selected_vertex = utils::get_vertex_from_screen(viewer, mouse_x, mouse_y, view_model.ruffles_mesh.V(), view_model.ruffles_mesh.F());
	has_changed = true;

	return has_changed;
}

bool RuffleDensifier::callback_mouse_up(igl::opengl::glfw::Viewer& viewer, int button, int modifier)
{
	if (!is_enabled())
		return false;
	if (!view_model.ruffles_mesh.is_vertex_valid(pre_selected_vertex))
		return false;


	//TODO select appropriate ruffle form the ruffle vertex index
	auto [part, vertex] = utils::get_simmesh_vertex(data_model.parts, pre_selected_vertex);
	auto &ruffle = part->ruffle();

	for (auto it = ruffle.sections.begin(); it != ruffle.sections.end(); ++it) {
		bool found = false;
		for (auto &seg : it->mesh_segments) {
			if (seg->start == vertex) {
				found = true;
				break;
			}
		}

		if (found) {
			ruffle.densify(it);
			break;
		}
	}

	for (auto element : view_model.elements) {
		if (auto x = dynamic_cast<RuffleOptimizer*>(element)) {
			x->mark_changed();
		}
	}


	has_changed = true;

	return has_changed;
}

}