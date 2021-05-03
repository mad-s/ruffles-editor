#include <iostream>
#include <string>
#include <filesystem>

#include "model/data_model.h"
#include "editor/view.h"
#include "editor/utils/filesystem_io.h"
#include "editor/utils/logger.h"

#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>
#include <igl/facet_components.h>
#include <igl/remove_duplicate_vertices.h>
#include <igl/remove_unreferenced.h>

namespace fs = std::filesystem;


ruffles::model::DataModel data;
ruffles::editor::View view(data);


//TODO (low prio) move these helper methods somewhere else
void repair(std::string file)
{
	Eigen::MatrixXd V;
	Eigen::MatrixXi F;
	bool success = igl::readOBJ(file, V, F);

	Eigen::VectorXi C;
	igl::facet_components(F, C);
	write_log(0) << "facet_components before: " << C.maxCoeff() + 1 << linebreak;

	Eigen::MatrixXd newV; 
	Eigen::MatrixXi newF, newVI, newVJ;
	int nv = V.rows();
	igl::remove_duplicate_vertices(V, F, 1e-7, newV, newVI, newVJ, newF);
	write_log(0) << "# duplicate vertices removed: " << newV.rows() - nv << linebreak;


	igl::facet_components(newF, C);
	write_log(0) << "facet_components after: " << C.maxCoeff() + 1 << linebreak;

	nv = newV.rows();
	Eigen::VectorXi I;
	igl::remove_unreferenced(newV, newF, newV, newF, I);
	write_log(0) << "# unreferenced vertices removed: " << newV.rows() - nv << linebreak;

	std::string folder = ruffles::utils::get_folder_path(file);
	success = igl::writeOBJ(folder + "/repaired.obj", newV, newF);
	exit(EXIT_SUCCESS);
}


int main(int argc, char* argv[]) 
{
	std::string models_folder = ruffles::utils::get_folder_path(__FILE__) + "/../../models/";
	data.models_folder(models_folder);

	std::string file = "";
	
	if (argc > 1) {
		file = argv[1];
		if (argc > 2) {
			data.scale = atof(argv[2]);
		}
	} else {
		//mesh files
		//mesh_file = "/teddy/teddy.obj";
		//file = "teddy/teddy_segmented.obj";
		//file = "furniture/pouf_15cm.obj";
		//mesh_file = "/architecture/masonry-fig7.obj";
		//mesh_file = "/sphere/sphere_cut.obj";

		//scene files
		//file = models_folder + "teddy/2020-09-13_16-59-32__teddy_partitioned_30cm__scene";
	}

	//TODO add nicer switching if needed
	if(fs::path(file).has_extension())
		data.target_file(file);
	else
		view.load_scene(file);
	
	view.launch();
}
