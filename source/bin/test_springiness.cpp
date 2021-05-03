#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/file_dialog_open.h>
#include <igl/file_dialog_save.h>


#include "common/common.h"
#include "common/imgui.h"

#include "ruffle/ruffle.h"
#include "simulation/verlet.h"
#include "simulation/lbfgs.h"
#include "simulation/combination.h"

#include "optimization/target_shape.h"
#include "optimization/particle_swarm.h"
#include "optimization/heuristic.h"

#include "visualization/visualization.h"

#include <unordered_map>

#include <chrono>

namespace ruffles {
	int inner_main(int argc, char *argv[]);
}
int main(int argc, char *argv[]) {
	try {
		return ruffles::inner_main(argc, argv);
	} catch (char const *x) {
		std::cerr << "Error: " << std::string(x) << std::endl;
	}
	return 1;
}


namespace ruffles {

using simulation::SimulationMesh;
using visualization::Visualization;

int inner_main(int argc, char *argv[]) {
	//Ruffle ruffle = Ruffle::create_ruffle_stack(4, 0.5, 1.0, 0.1);
	//Ruffle ruffle = Ruffle::create_horizontal_stack(4, 4., 3., 0.5);
	// wide stack:
	//Ruffle ruffle = Ruffle::create_ruffle_stack(2, 3., 10.67, 0.5);
	// default stack
	Ruffle ruffle = Ruffle::create_ruffle_stack(2, 3., 5.28, 0.5);
	ruffle.simulator.reset(new simulation::Combination(ruffle.simulation_mesh));
	ruffle.simulation_mesh.density = 0.160; // 160g paper
	ruffle.simulation_mesh.k_bend = 105000; // 160g paper
	ruffle.simulation_mesh.update_vertex_mass();
	
	/*
	std::next(ruffle.sections.begin(), 0)->length = 5.53; // ???
	std::next(ruffle.sections.begin(), 1)->length = 4.32; //  ???
	std::next(ruffle.sections.begin(), 2)->length = 6.11;
	std::next(ruffle.sections.begin(), 3)->length = 4.59;
	std::next(ruffle.sections.begin(), 4)->length = 5.28;
	std::next(ruffle.sections.begin(), 5)->length = 4.32;
	std::next(ruffle.sections.begin(), 6)->length = 6.19;
	std::next(ruffle.sections.begin(), 7)->length = 4.59; // ???
	std::next(ruffle.sections.begin(), 8)->length = 5.71; // ???
	*/
	ruffle.update_simulation_mesh();

	for (int i = 0; i < 10; i++)
		ruffle.physics_solve();
/*
	ruffle.densify(std::next(ruffle.sections.begin(), 7));
	ruffle.densify(std::next(ruffle.sections.begin(), 5));
	ruffle.densify(std::next(ruffle.sections.begin(), 3));
	ruffle.densify(std::next(ruffle.sections.begin(), 1));
*/
	ruffle.simulation_mesh.generate_air_mesh();
	for (int i = 0; i < 10; i++)
		ruffle.physics_solve();

	real y_max = 0.;
	for (auto v : ruffle.simulation_mesh.vertices) {
		y_max = max(y_max, ruffle.simulation_mesh.get_vertex_position(v)(1));
	}
	ruffle.simulation_mesh.ub(1) = y_max;

	real f = 0.;
	auto get_force = [&]() {
		f = 0.;
		VectorX grad = VectorX::Zero(ruffle.simulation_mesh.dof());
		ruffle.simulation_mesh.energy(ruffle.simulation_mesh.x, &grad);
		for (int i = 0; i < ruffle.simulation_mesh.dof(); i += 2) {
			Vector2 pos = ruffle.simulation_mesh.x.segment<2>(i);
			if (pos(1) > y_max - 1e-3) {
				if (grad(i+1) > 0.) {
					cout << "What???" << endl;
				} else {
					f += -grad(i+1);
				}
			}
		}
	};
	get_force();

	real y_max2 = 0.;

	igl::opengl::glfw::Viewer viewer;
	viewer.core().background_color.setOnes();


	bool show_forces = false;
	Visualization vis;
	auto update_visualization = [&]() {
		y_max2 = 0.;
		for (auto v : ruffle.simulation_mesh.vertices) {
			y_max2 = max(y_max2, ruffle.simulation_mesh.get_vertex_position(v)(1));
		}
		vis.clear();

		auto &mesh = ruffle.simulation_mesh;
		std::unordered_map<listref<SimulationMesh::Vertex>, int, listref_hash<SimulationMesh::Vertex>> indices;

		int i = 0;
		for (auto it = mesh.vertices.begin(); it != mesh.vertices.end(); ++it) {
			Vector2 pos = mesh.get_vertex_position(*it);
			vis.push_vertex((Vector3() << pos, 0.).finished());
			vis.push_vertex((Vector3() << pos, 4.).finished());
			indices.insert({it, i});
			i++;
		}

		for (auto seg : mesh.segments) {
			int a = indices[seg.start];
			int b = indices[seg.end];

			vis.push_face(2*a, 2*b, 2*b+1);
			vis.push_face(2*b+1, 2*a+1, 2*a);
		}
/*
		for (auto it = mesh.air_mesh.cdt.finite_faces_begin(); it != mesh.air_mesh.cdt.finite_faces_end(); ++it) {
			int a = it->vertex(0)->info();
			int b = it->vertex(1)->info();
			int c = it->vertex(2)->info();

			vis.push_face(2*a,2*b,2*c);
		}

*/

/*
		for (auto y : {mesh.lb(1), mesh.ub(1)}) {
			if (std::isfinite(y)) {
				int a = vis.push_vertex(-100, y, -10);
				int b = vis.push_vertex(-100, y, +00);
				int c = vis.push_vertex(+100, y, +00);
				int d = vis.push_vertex(+100, y, -10);
				vis.push_face(a,b,c);
				vis.push_face(c,d,a);
			}
		}
		*/

		vis.compress();

		viewer.data().clear();
		viewer.data().clear_labels();

		if (show_forces) {
			MatrixX p1(mesh.dof() / 2, 2);
			MatrixX p2(mesh.dof() / 2, 2);

			VectorX grad = VectorX::Zero(mesh.dof());
			mesh.energy(mesh.x, &grad);
			VectorX y = mesh.x - 0.003 * grad / mesh.k_global;
			for (int i = 0; i < mesh.dof(); i += 2) {
				p1.row(i/2) << mesh.x.segment<2>(i).transpose();
				p2.row(i/2) << y.segment<2>(i).transpose();
			}
			viewer.data().add_edges(p1, p2, Eigen::RowVector3d(0.8,0.,0.));
		}

		i = 0;
		for (auto it = mesh.vertices.begin(); it != mesh.vertices.end(); ++it) {
			Vector2 pos = mesh.get_vertex_position(*it);
			viewer.data().add_label((Vector3() << pos, 0.).finished().cast<double>(), to_string(i));
			i++;
		}

		viewer.data().set_mesh(vis.V.cast<double>(), vis.F);
		//viewer.data().set_colors(vis.C.cast<double>());

		viewer.data().uniform_colors(Vector3(0.3, 0.3, 0.3), Vector3(207.0/255.0, 233.0/255.0, 243.0/255.0), Vector3(0,0,0));

		viewer.data().double_sided = true;
		viewer.data().face_based = true;

	};
	update_visualization();

	igl::opengl::glfw::imgui::ImGuiMenu menu;
	viewer.plugins.push_back(&menu);


	bool solve_loop = false;
	viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer &viewer) -> bool {
		if (solve_loop) {
		ruffle.physics_solve();

			get_force();
		update_visualization();
		}
		return false;
	};
	menu.callback_draw_viewer_menu = [&]() {
		menu.draw_viewer_menu();

		ImGui::Separator();

		ImGui::Checkbox("solve loop", &solve_loop);
		ImGui::Checkbox("show forces", &show_forces);

		if (ImGui::InputReal("y_max", &y_max)) {
			ruffle.simulation_mesh.ub(1) = y_max;
			update_visualization();
		}

		if (ImGui::Button("Do it!")) {
			auto cmd = "scrot -u /tmp/spring_" + to_string(y_max) + "cm.png";
			system(cmd.c_str());
			cmd = "echo " + to_string(y_max) + "," + to_string(f * 1e-4 / ruffle.simulation_mesh.k_global) + " >> /tmp/data.csv";
			std::cout << "y_max: " << y_max << " -> f = " << (f * 1e-4 / ruffle.simulation_mesh.k_global) << std::endl;
			system(cmd.c_str());

			y_max -= 0.1;
			ruffle.simulation_mesh.ub(1) = y_max;
			update_visualization();
		}

		if (ImGui::Button("solve")) {
			ruffle.physics_solve();
			get_force();
			update_visualization();
		}

		ImGui::Text("f = %f N", f * 1e-4 / ruffle.simulation_mesh.k_global );

		ImGui::Text("y_max2 = %f", y_max2);

	};

	viewer.launch();
	return 0;
}
}
