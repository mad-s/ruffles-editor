#include "tool_selector.h"

namespace ruffles::editor {


void ToolSelector::update_view(igl::opengl::glfw::Viewer& viewer)
{
	//empty on purpose
}

void ToolSelector::update_menu(Menu& menu)
{
	if (!ImGui::CollapsingHeader("tool", ImGuiTreeNodeFlags_DefaultOpen))
		return;

	int choice_tool = view_model.active_tool;
	
	ImGui::RadioButton("partition mesh", &choice_tool, RuffleTool::MeshPartition);
	ImGui::RadioButton("plane cutting", &choice_tool, RuffleTool::PlanePosition);
	ImGui::RadioButton("adapt density", &choice_tool, RuffleTool::DensityEdit);
	ImGui::RadioButton("apply force", &choice_tool, RuffleTool::ForcePosition);
	ImGui::RadioButton("move connection point", &choice_tool, RuffleTool::ConnectionPointEdit);
	ImGui::RadioButton("connect ruffles", &choice_tool, RuffleTool::MultiRuffleConnect);
	ImGui::RadioButton("change lengths", &choice_tool, RuffleTool::ChangeLengths);

	view_model.active_tool = static_cast<RuffleTool>(choice_tool);


	ImGui::Spacing();

	if (ImGui::InputInt("select part", &view_model.selected_part_index))
	{
		view_model.selected_part_index = std::clamp(view_model.selected_part_index, -1, (int)data_model.parts.size() - 1);
		view_model.has_selected_part_changed = true;
	}

	ImGui::SameLine();

	if (ImGui::Checkbox("show only selected", &view_model.is_only_selected_visible))
		view_model.has_selected_part_changed = true;


	ImGui::Spacing();
	ImGui::Spacing();

	//if (!ImGui::CollapsingHeader("tool options", ImGuiTreeNodeFlags_DefaultOpen))
	//	return;
}

}
