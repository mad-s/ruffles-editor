#include "editor/elements/mesh_renderer.h"

#include "editor/utils/transform.h"
#include "editor/utils/logger.h"
#include "editor/style/editor_style.h"
//#include "editor/style/colors.h"


namespace ruffles::editor {

void color_array(Eigen::Vector3d& color_vector, float* color_array);
void color_vector(float* color_array, Eigen::Vector3d& color_vector);



void MeshRenderer::update_view(igl::opengl::glfw::Viewer& viewer)
{
	//check if mesh changed

	if (!has_changed || !is_valid())
		return;
	if (pending_add_mesh)
		add_mesh_to_view(viewer);


	//update mesh
	viewer.selected_data_index = view_index;

	//viewer.data().clear();
	//viewer.data().set_mesh(V, F);
	//viewer.data().set_colors(color);
	viewer.data().uniform_colors(Colors::GRAY_DARK, fill, Colors::BLACK);
	viewer.data().grid_texture();

	viewer.data().show_faces = show_fill;


	has_changed = false;
}

void MeshRenderer::update_menu(Menu& menu)
{
	//ImGui::Text("Mesh render settings...");
	
	//ColorPicker3
	//if (ImGui::ColorEdit3("Fill", color_menu, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_PickerHueWheel))
	if (ImGui::ColorEdit3("Fill", color_menu, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_PickerHueBar))
	{
		color_vector(color_menu, fill);
		has_changed = true;
	}

	if (ImGui::Checkbox("show fill", &show_fill))
		has_changed = true;


}

void MeshRenderer::add_mesh(Mesh& mesh)
{
	//if(this->mesh != nullptr)
	//	view_mode

	this->mesh = &mesh;

	has_changed = true;
	pending_add_mesh = true;
}

bool MeshRenderer::is_valid()
{
	return mesh != NULL && mesh->is_valid();
}

void MeshRenderer::add_mesh_to_view(igl::opengl::glfw::Viewer& viewer)
{
	if (!is_valid())
		return;


	bool align_camera = true;
	bool do_center_mesh = false;
	if (do_center_mesh)
		utils::center_mesh(mesh->V());

	if(view_index != -1) //if there was another mesh loaded, then erase that first
		view_index = viewer.erase_mesh(view_index);

	view_index = viewer.append_mesh();
	viewer.data().clear();
	viewer.data().set_mesh(mesh->V(), mesh->F());
	viewer.data().compute_normals();


	//TODO expose default style 
	//fill = Colors::GRAY_ULTRALIGHT;
	fill = style::fill_mesh;
	color_array(fill, color_menu);

	viewer.data().point_size = style::points_small;
	viewer.data().line_width = style::wire_thickness;
	viewer.data().line_color = Colors::to_4f(style::wire_color);
	viewer.data().show_faces = false;
	viewer.data().double_sided = true;
	viewer.data().face_based = true;
	//viewer.data().point_size = 5.0;
	//viewer.data().line_width = 0.5;
	//viewer.data().line_color = Eigen::Vector4f(Colors::GRAY_MID(0), Colors::GRAY_MID(1), Colors::GRAY_MID(2), 1.0f);
	//viewer.data().show_faces = false;
	//viewer.data().double_sided = true;
	//viewer.data().face_based = true;

	if (align_camera)
		viewer.core().align_camera_center(mesh->V(), mesh->F());
	
	pending_add_mesh = false;
}


void color_array(Eigen::Vector3d& color_vector, float* color_array)
{
	for (int i = 0; i < 3; i++)
		color_array[i] = color_vector(i);
}

void color_vector(float* color_array, Eigen::Vector3d& color_vector)
{
	for (int i = 0; i < 3; i++)
		color_vector(i) = color_array[i];
}

}