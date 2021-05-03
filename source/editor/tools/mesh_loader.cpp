#include "editor/tools/mesh_loader.h"

#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>

#include "editor/elements/mesh_renderer.h"
#include "model/data_model.h"

#include "editor/utils/concatenate_meshes.h"
#include "editor/utils/filesystem_io.h"
#include "editor/utils/transform.h"
#include "editor/utils/logger.h"

namespace ruffles::editor {

MeshLoader::MeshLoader(ViewModel& view_model, DataModel& data_model) : AbstractTool(view_model, data_model)
{
	if(data_model.has_target_file())
		load_mesh(data_model.absolute_target_path()); //try load mesh if set by default
}

void MeshLoader::update_menu(Menu& menu)
{
	if(ImGui::Button("load .obj"))
	{
		std::string filename = igl::file_dialog_open();
		if (filename.length() == 0)
			return;

		data_model.target_file(filename);
		load_mesh(data_model.absolute_target_path());
	}
	ImGui::SameLine();
	if (ImGui::Button("write .obj"))
	{
		std::string filename = igl::file_dialog_save();
		if (filename.length() == 0)
			return;

		write_mesh(filename);
	}

	////try load model
	//if (!data_model.has_target_file() && !data_model.target().is_valid())
	//	load_mesh(data_model.absolute_target_path());
}

void MeshLoader::update_view(igl::opengl::glfw::Viewer& viewer)
{
	//empty on purpose
}

bool MeshLoader::load_mesh(std::string file)
{
	write_log(3) << "try load model file: " << file << std::endl;

	if (file.empty())
		return false;

	Eigen::MatrixXd V, N;
	Eigen::MatrixXi F;

	bool success = igl::readOBJ(file, V, F);
	if (!success)
	{
		write_log(1) << "error at loading target. (filename: " << file << ")" << std::endl;
		std::string empty = "";
		data_model.target_file(empty);
		return success;
	}
	write_log(3) << "successfully loaded target. (dimensions: " << utils::get_dimensions(V).transpose() << ")" << linebreak << std::endl;

	V *= data_model.scale;
	data_model.clear();
	//TODO clear view

	data_model.target(V, F);
	view_model.selected_part_index = 0;
	view_model.target_renderer->add_mesh(data_model.target());

	auto dim = utils::get_dimensions(data_model.target().V());
	view_model.transform_step = 1; //dim.maxCoeff() * 0.025;

	return true;
}

bool MeshLoader::write_mesh(std::string file)
{
	write_log(3) << "try write model file: " << file << linebreak << std::endl;

	if (file.empty())
		return false;

	std::vector<Eigen::MatrixXd> Vs;
	std::vector<Eigen::MatrixXi> Fs;
	for (auto& part : data_model.parts)
	{
		Vs.push_back(part.segment().V());
		Fs.push_back(part.segment().F());
	}

	Eigen::MatrixXd V;
	Eigen::MatrixXi F;
	utils::concatenate_meshes(Vs, Fs, V, F);

	bool success = igl::writeOBJ(file, V, F);
	if (!success)
	{
		write_log(1) << "error at loading target. (filename: " << file << ")" << std::endl;
		return success;
	}

	return true;
}

}