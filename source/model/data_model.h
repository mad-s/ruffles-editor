#pragma once

#include "model/model_part.h"
//#include "editor/tools/segmenter.h"

namespace ruffles::model {

class DataModel 
{
public:
	Mesh& target();
	void target(Mesh& value);
	void target(Eigen::MatrixXd& V, Eigen::MatrixXi& F);

	//Segmenter* segmenter = NULL; //TODO (low prio) decouple from view and use only data object 

	//TODO add graph
	std::vector<ModelPart> parts;
	bool do_auto_update = true;


	std::string absolute_target_path();
	void models_folder(std::string& value);
	void target_file(std::string& value);	
	bool has_target_file();


	void update_parts(Eigen::VectorXi face_component_labels);
	void clear();

	void update_parts_graph(int root);

	real scale = 0.1;

private:
	Mesh _target;
	std::string _models_folder = ""; //absolute path to the models folder
	std::string _target_file = "";	//relative path to the model file

	void clear_parts();
};

}