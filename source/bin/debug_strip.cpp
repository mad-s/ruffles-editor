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
#include "simulation/simulation_mesh.h"


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
	(void)argc;
	(void)argv;

	real k_B = 1e6;
	real k_G = 1;
	real length = 20.; // 10cm
	real width = 1.; // 2cm
	real density = 0.005; // 80 g / m^2 = 8 mg / cm^2
	real h= 0.1; // 5mm

	real y_min = 0.;

	SimulationMesh mesh = SimulationMesh::generate_horizontal_strip(length, h);
	auto simulator = simulation::Verlet(mesh);
	auto lbfgs = simulation::LBFGS(mesh);
	auto update_mesh = [&]() {
		mesh = SimulationMesh::generate_horizontal_strip(length, h);
		for (auto &vx : mesh.vertices) {
			vx.width = width;
		}
		mesh.k_global = k_G;
		mesh.k_bend = k_B;
		mesh.density = density;
		mesh.update_vertex_mass();
		simulator.reset(mesh);
	};
	update_mesh();

	igl::opengl::glfw::Viewer viewer;

	Visualization vis;
	auto update_visualization = [&]() {
		vis.clear();

		std::unordered_map<listref<SimulationMesh::Vertex>, int, listref_hash<SimulationMesh::Vertex>> indices;

		y_min = infinity;
		int i = 0;
		for (auto it = mesh.vertices.begin(); it != mesh.vertices.end(); ++it) {
			Vector2 pos = mesh.get_vertex_position(*it);
			vis.push_vertex((Vector3() << pos, 0.).finished());
			vis.push_vertex((Vector3() << pos, it->width).finished());
			indices.insert({it, i});
			i++;
			y_min = min(y_min, pos.y());
		}

		for (auto seg : mesh.segments) {
			int a = indices[seg.start];
			int b = indices[seg.end];

			vis.push_face(2*a, 2*b, 2*b+1);
			vis.push_face(2*b+1, 2*a+1, 2*a);
		}

		for (auto it = mesh.air_mesh.cdt.finite_faces_begin(); it != mesh.air_mesh.cdt.finite_faces_end(); ++it) {
			int a = it->vertex(0)->info();
			int b = it->vertex(1)->info();
			int c = it->vertex(2)->info();

			vis.push_face(2*a,2*b,2*c);
		}

		vis.compress();

		viewer.data().clear();
		viewer.data().clear_labels();
		i = 0;
		for (auto it = mesh.vertices.begin(); it != mesh.vertices.end(); ++it) {
			Vector2 pos = mesh.get_vertex_position(*it);
			viewer.data().add_label((Vector3() << pos, 0.).finished().cast<double>(), to_string(i));
			i++;
		}

		viewer.data().set_mesh(vis.V.cast<double>(), vis.F);
		viewer.data().set_colors(vis.C.cast<double>());
	};
	update_visualization();

	bool simulating = false;
	bool multiple_steps = true;
	int step_count = 0;
	viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer &viewer) -> bool {
		(void)viewer;
		if (simulating) {
			auto start = std::chrono::high_resolution_clock::now();
			do {
				bool converged = simulator.step(mesh);
				step_count++;
				if (converged) {
					cout << "Converged in " << step_count << endl;
					simulating = false;
					step_count = 0;
					break;
				}
			} while (multiple_steps && std::chrono::high_resolution_clock::now() - start < std::chrono::milliseconds(16));

			update_visualization();
		}
		return false;
	};

	igl::opengl::glfw::imgui::ImGuiMenu menu;
	viewer.plugins.push_back(&menu);
	menu.callback_draw_viewer_menu = [&]() {
		menu.draw_viewer_menu();


		if (ImGui::InputReal("k_B", &k_B)) {
			mesh.k_bend = k_B;
		}
		ImGui::InputReal("density", &density);
		ImGui::InputReal("length", &length);
		ImGui::InputReal("width", &width);
		if (ImGui::InputReal("k_global", &k_G)) {
			mesh.k_global = k_G;
		}
		ImGui::InputReal("h", &h);

		//ImGui::InputReal("dt", &simulator.dt);
		if (ImGui::Button("update")) {
			update_mesh();
			update_visualization();
		}
		if (ImGui::CollapsingHeader("Physics stuff")) {
			if (ImGui::Button(simulating ? "Stop simulation" : "Start simulation", ImVec2(-1,0))) {
				simulating = !simulating;
				viewer.core().is_animating ^= 1;
			}
			if (ImGui::Button("Step physics")) {
				simulator.step(mesh);
				update_visualization();
			}
			if (ImGui::Button("Step LBFGS")) {
				lbfgs.step(mesh);
				update_visualization();
			}
		}
		ImGui::Text("y_min: %f", y_min);


	};

	viewer.launch();
	return 0;
}
}
