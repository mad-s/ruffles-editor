#include "editor/serializer.h"
#include <igl/serialize.h>


namespace ruffles::model {

void Serializer::serialize(const std::string& scene_file, DataModel& data_model)
{
	//igl::serialize(data_model.models_folder, "models_folder", scene_file);
	//igl::serialize(data_model.target_file, "target_file", scene_file);
	
	igl::serialize(data_model.target(), "target_mesh", scene_file);
	//igl::serialize(data_model.target().V(), "target_mesh_V", scene_file);
	//igl::serialize(data_model.target().F(), "target_mesh_F", scene_file);
	
	const int number_parts = data_model.parts.size();
	igl::serialize(number_parts, "number_parts", scene_file);

	for (int i = 0; i < data_model.parts.size(); i++)
	{
		ModelPart& part = data_model.parts[i];
		
		igl::serialize(part.segment(), "sub_mesh_" + to_string(i), scene_file);
		igl::serialize(part.plane(), "cuttting_plane_" + to_string(i), scene_file);
		igl::serialize(part.ground_plane(), "ground_plane_" + to_string(i), scene_file);
		igl::serialize(part.cutline(), "cutline_" + to_string(i), scene_file);

		igl::serialize(part.ruffle(), "ruffle_" + to_string(i), scene_file);
	}
}

void Serializer::deserialize(const std::string& scene_file, DataModel& data_model)
{
	//data_model.clear();
	data_model.do_auto_update = false;

	//igl::deserialize(data_model.models_folder, "models_folder", scene_file);
	//igl::deserialize(data_model.target_file, "target_file", scene_file);

	//igl::deserialize(data_model.target().V(), "target_mesh_V", scene_file);
	//igl::deserialize(data_model.target().F(), "target_mesh_F", scene_file);

	Mesh target_mesh;
	igl::deserialize(target_mesh, "target_mesh", scene_file);
	data_model.target(target_mesh);

	int number_parts;
	igl::deserialize(number_parts, "number_parts", scene_file);

	for (int i = 0; i < number_parts; i++)
	{
		Mesh sub_mesh;
		igl::deserialize(sub_mesh, "sub_mesh_" + to_string(i), scene_file);
		data_model.parts.emplace_back(sub_mesh);

		ModelPart& part = data_model.parts[i];

		Plane cutting_plane;
		igl::deserialize(cutting_plane, "cuttting_plane_" + to_string(i), scene_file);
		part._plane = cutting_plane;

		Plane ground_plane;
		igl::deserialize(ground_plane, "ground_plane_" + to_string(i), scene_file);
		part._ground_plane = ground_plane;

		Eigen::MatrixXd cutline;
		igl::deserialize(cutline, "cutline_" + to_string(i), scene_file);
		part.cutline(cutline);

		Ruffle ruffle;
		igl::deserialize(ruffle, "ruffle_" + to_string(i), scene_file);
		part.ruffle(std::move(ruffle));
	}
}

}