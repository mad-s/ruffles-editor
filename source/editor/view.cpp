#include "editor/view.h"

#include <functional>
#include <typeinfo>
#include <filesystem>

#include "editor/tools/tool_selector.h"
#include "editor/elements/mesh_renderer.h"
#include "editor/tools/plane_positioning.h"
#include "editor/tools/mesh_loader.h"
#include "editor/tools/segmenter.h"
#include "editor/tools/ruffle_optimizer.h"
#include "editor/tools/connection_point_editor.h"
#include "editor/tools/multi_ruffle_tab_placement.h"
#include "editor/tools/ruffle_densifier.h"
#include "editor/tools/force_positioning.h"
#include "editor/tools/change_lengths.h"

#include "editor/serializer.h"
#include "editor/utils/filesystem_io.h"
#include "editor/utils/logger.h"

namespace fs = std::filesystem;
namespace ruffles::editor {

bool View::launch()
{
	::LOG_LEVEL = 4;

	if (is_launched)
		return false;

	bool success = initialize();
	if (!success)
		return false;

	is_launched = true;
	view_model.viewer.launch(true, false, "Metamaterial Ruffles Editor");

	return true;
}

bool View::initialize()
{
	if (is_initialized)
		return false;

	register_callbacks();

	//intitialize viewer
	view_model.viewer.core().is_animating = true;
	view_model.viewer.core().animation_max_fps = 30.;

	view_model.viewer.core().background_color.setOnes();
	view_model.viewer.core().set_rotation_type(igl::opengl::ViewerCore::ROTATION_TYPE_TRACKBALL);

	const int width = 1920;
	const int height = 1080;
	const float scale = 1.0;
	view_model.viewer.resize(width*scale, height*scale);

	const auto coordinate_indicator = Eigen::MatrixXd::Identity(3, 3);
	view_model.viewer.data().add_edges(Eigen::MatrixXd::Zero(3, 3), coordinate_indicator * 0.5, coordinate_indicator);


	//initialize & attach menu
	view_model.menu.init_size(width*scale*0.25f, 0.0f, 0.0f, "Ruffles");
	view_model.viewer.plugins.push_back(&view_model.menu);


	initialize_view_elements();

	is_initialized = true;
	return true;
}

void View::initialize_view_elements()
{
	view_model.target_renderer = new MeshRenderer(view_model.viewer, view_model.menu);
	try_load_model();


	//data_model.segmenter = new Segmenter(view_model, data_model);
	Segmenter* segmenter = new Segmenter(view_model, data_model);

	view_model.add_element(new MeshLoader(view_model, data_model));
	view_model.add_element(view_model.target_renderer);

	view_model.add_element(new ToolSelector(view_model, data_model));
	view_model.add_element(segmenter);
	view_model.add_element(new PlanePositioning(view_model, data_model));
	view_model.add_element(new RuffleDensifier(view_model, data_model));
	view_model.add_element(new ForcePositioning(view_model, data_model));
	view_model.add_element(new ConnectionPointEditor(view_model, data_model));
	view_model.add_element(new MultiRuffleTabPlacement(view_model, data_model));
	view_model.add_element(new RuffleOptimizer(view_model, data_model));
	auto* change_lengths = new SomeTool(view_model, data_model);
	view_model.add_element(change_lengths);
	

	////DEBUG (masonry)
	//std::vector< std::vector <int>> init_segment_vertices;
	//init_segment_vertices.push_back({ 25, 461, 466, 552, 550, 561, 559, 700, 560, 3282, 3284, 3285, 3286, 2831, 2834, 2835, 2715, 3525, 3358, 3359, 1135, 1216, 982, 959, 960, 935, 980, 418, 789, 502, 597, 19, -1 });
	//init_segment_vertices.push_back({ 1, 2450, 756, 2453, 2459, 2464, 2462, 2467, 2460, 3546, 3545, 3529, 3530, 3369, 3362, 3365, 3525, -1 });
	//segmenter->initialize_with_segments(init_segment_vertices);
}

void View::register_callbacks()
{
	//called each frame
	view_model.viewer.callback_pre_draw = std::bind(&View::callback_update_view, this, std::placeholders::_1);
	view_model.menu.callback_draw_viewer_window = std::bind(&View::callback_update_menu, this);
	////menu.callback_draw_viewer_menu = callback_update_menu; // draws libigl default menu for mesh interaction
	////menu.callback_draw_custom_window = callback_update_debug_menu; //can add additional menu, if needed

	//keyboard interaction (return true if handled)
	view_model.viewer.callback_key_down = std::bind(&View::callback_key_down, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	view_model.viewer.callback_key_up   = std::bind(&View::callback_key_up, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

	//mouse interaction (return true if handled)
	view_model.viewer.callback_mouse_down = std::bind(&View::callback_mouse_down, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	view_model.viewer.callback_mouse_move = std::bind(&View::callback_mouse_move, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	view_model.viewer.callback_mouse_up   = std::bind(&View::callback_mouse_up, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
}


bool View::callback_key_down(igl::opengl::glfw::Viewer& viewer, unsigned int key, int modifiers)
{
	if (key == GLFW_KEY_ESCAPE) {
		// don't close window on escape
		glfwSetWindowShouldClose(viewer.window, GL_FALSE);
	}
	for (auto& element : view_model.elements)
		element->callback_key_down(viewer, key, modifiers);

	return false;
}

bool View::callback_key_up(igl::opengl::glfw::Viewer& viewer, unsigned int key, int modifiers)
{
	if (key == GLFW_KEY_Q)
		view_model.active_tool = RuffleTool::None;


	for (auto& element : view_model.elements)
		element->callback_key_up(viewer, key, modifiers);

	return false;
}

bool View::callback_mouse_down(igl::opengl::glfw::Viewer& viewer, int button, int modifier)
{
	view_model.is_mouse_down = true;

	for (auto& element : view_model.elements)
		element->callback_mouse_down(viewer, button, modifier);

	return false;
}

bool View::callback_mouse_move(igl::opengl::glfw::Viewer& viewer, int mouse_x, int mouse_y)
{
	for (auto& element : view_model.elements)
		element->callback_mouse_move(viewer, mouse_x, mouse_y);

	return false;
}

bool View::callback_mouse_up(igl::opengl::glfw::Viewer& viewer, int button, int modifier)
{
	view_model.is_mouse_down = false;

	for (auto& element : view_model.elements)
		element->callback_mouse_up(viewer, button, modifier);

	return false;
}


bool View::callback_update_view(igl::opengl::glfw::Viewer& viewer)
{
	for (auto& element : view_model.elements)
		element->update_view(viewer);
	
	view_model.has_selected_part_changed = false;
	return false;
}

void View::callback_update_menu()
{
	view_model.menu.begin_menu();

	ImGui::InputInt("log level", &LOG_LEVEL);


	if (ImGui::Button("load scene"))
	{
		std::string scene_file = igl::file_dialog_open();
		if (scene_file.length() == 0)
			return;

		load_scene(scene_file);
	}
	ImGui::SameLine();
	if (ImGui::Button("save scene"))
	{
		std::string auto_path = utils::get_auto_filepath(data_model.absolute_target_path());
		
		Serializer serializer;
		serializer.serialize(auto_path + "__scene", data_model);
	}


	for(int i = 0; i < view_model.elements.size(); i++) //somehow for in loop didnt work
		view_model.elements[i]->update_menu(view_model.menu);

	view_model.menu.end_menu();
}

void View::load_scene(const std::string scene_file)
{
	if(scene_file.empty())
		return;

	data_model.clear();
	//TODO clear view

	Serializer serializer;
	serializer.deserialize(scene_file, data_model);

	try_load_model();
}

void View::try_load_model()
{
	if (data_model.target().is_valid() && view_model.target_renderer != NULL)
		view_model.target_renderer->add_mesh(data_model.target());
}

}