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
#include "simulation/line_search.h"


#include "visualization/visualization.h"

#include "optimization/heuristic.h"

#include "output/create_svg.h"

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

	Polygon target_polygon;
	int n = 16;
	real r_outer = 9;
	real r_inner = 5;
	real r_guideline = 7.;


	MatrixX line(n,2);
	for (int i = 0; i < n; i++) {
		real phi = M_PI*i/(n-1);
		target_polygon.push_back(Point(r_outer*std::cos(phi), r_outer*std::sin(phi)));
		line.row(i) << r_guideline*std::cos(phi), r_guideline*std::sin(phi)+0.01;
	}
	// align x
	line(0,0) = line(1,0);
	line(n-1,0) = line(n-2,0);
	dbg(line);
	for (int i = n-1; i >=0; i--) {
		real phi = M_PI*i/(n-1);
		target_polygon.push_back(Point(r_inner*std::cos(phi), r_inner*std::sin(phi)));
	}
	
	/*
	target_polygon.push_back(Point(0,0));
	target_polygon.push_back(Point(4,0));
	target_polygon.push_back(Point(4,5));
	target_polygon.push_back(Point(6,8));
	target_polygon.push_back(Point(10,9));
	target_polygon.push_back(Point(10,13));
	target_polygon.push_back(Point(4,11));
	target_polygon.push_back(Point(0,6));
	*/
	/*
	target_polygon.push_back(Point(-2,0.));
	target_polygon.push_back(Point(8,0.));
	target_polygon.push_back(Point(8,10.));
	target_polygon.push_back(Point(-2,10.));
	*/
	/*
	target_polygon.push_back(Point(-1,0.));
	target_polygon.push_back(Point(2.,0.));
	target_polygon.push_back(Point(0.5,3.5));
	*/
	/*
	int n = 32;
	for (int i = 0; i < n; i++) {
		real phi = 2*M_PI*(i + 0.5)/n;
		target_polygon.push_back(Point(std::sin(phi)+0.5, 1.-std::cos(phi)-(1.-std::cos(2*M_PI*0.5/n))));
	}
	*/

	optimization::TargetShape target(target_polygon);

	//Ruffle ruffle = Ruffle::create_ruffle_stack(3, 2.5, 5., 0.5);
	Ruffle ruffle = Ruffle::create_stack_along_curve(line, target, 0.5);
	//ruffle.simulation_mesh.gravity = Vector2(0, 9.81);
	//ruffle.simulation_mesh.generate_air_mesh();
	for (auto &vx : ruffle.simulation_mesh.vertices) {
		vx.width = 8.;
		vx.z = {0., 8.};
	}
	ruffle.simulation_mesh.update_vertex_mass();
	dbg(ruffle.simulation_mesh.total_mass());
	//ruffle.simulator.reset(new simulation::Verlet(ruffle.simulation_mesh));
	ruffle.simulator.reset(new simulation::LBFGS(ruffle.simulation_mesh));

	
	optimization::Heuristic heuristic(target);


	igl::opengl::glfw::Viewer viewer;

	Visualization vis;
	auto update_visualization = [&]() {
		vis.clear();


		auto &mesh = ruffle.simulation_mesh;
		std::unordered_map<listref<SimulationMesh::Vertex>, int, listref_hash<SimulationMesh::Vertex>> indices;

		int i = 0;
		for (auto it = mesh.vertices.begin(); it != mesh.vertices.end(); ++it) {
			Vector2 pos = mesh.get_vertex_position(*it);
			vis.push_vertex((Vector3() << pos, -0.5*it->width).finished());
			vis.push_vertex((Vector3() << pos, +0.5*it->width).finished());
			indices.insert({it, i});
			i++;
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

		int n_old = vis.nv;
		for (int i = 0; i < target.V.rows(); ++i) {
			vis.push_vertex(target.V.row(i).transpose());
		}
		for (int i = 0; i < target.F.rows(); ++i) {
			auto tri = target.F.row(i).array() + n_old;
			vis.push_face(tri(0), tri(1), tri(2));
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
				bool converged = ruffle.simulator->step(ruffle.simulation_mesh);
				step_count++;
				if (converged) {
					cout << "Converged in " << step_count << endl;
					simulating = false;
					step_count = 0;
					break;
				}
				//step_count++;
				//if (should_update_air_mesh) {
				//	problem.update_air_mesh(opt->position());
				//}
			} while (multiple_steps && std::chrono::high_resolution_clock::now() - start < std::chrono::milliseconds(16));
			//total_step_duration += chrono::high_resolution_clock::now() - start;

			update_visualization();
		}
		return false;
	};

	igl::opengl::glfw::imgui::ImGuiMenu menu;
	viewer.plugins.push_back(&menu);
	menu.callback_draw_viewer_menu = [&]() {
		menu.draw_viewer_menu();

		if (ImGui::CollapsingHeader("Material")) {
			ImGui::InputReal("k_global", &ruffle.simulation_mesh.k_global);
			ImGui::InputReal("k_bend", &ruffle.simulation_mesh.k_bend);
			ImGui::InputScalarN("gravity", ImGuiDataType_Double, ruffle.simulation_mesh.gravity.data(), 2);
		}

		if (ImGui::CollapsingHeader("Section lengths")){
			int i = 0;
			for (auto &sec : ruffle.sections) {
				ImGui::PushID(i);
				ImGui::InputReal("", &sec.length);
				ImGui::PopID();
				i++;
			}
			if (ImGui::Button("Update simulation mesh with lengths")) {
				ruffle.update_simulation_mesh();
			}
		}

		if (ImGui::CollapsingHeader("Change topology")) {
			static int index = 0;
			ImGui::InputInt("index", &index);

			if (ImGui::Button("Subdivide section #i")) {
				ruffle.subdivide(std::next(ruffle.sections.begin(), index));
				update_visualization();
			}
			if (ImGui::Button("Dissolve connection point #i")) {
				ruffle.dissolve_connection_point(std::next(ruffle.connection_points.begin(), index));
				update_visualization();
			}

			if (ImGui::Button("Densify section #i")) {
				ruffle.densify3(std::next(ruffle.sections.begin(), index));
				update_visualization();
			}
			if (ImGui::Button("Undensify section #i")) {
				ruffle.undensify(std::next(ruffle.sections.begin(), index));
				update_visualization();
			}
			if (ImGui::Button("Densify2 section #i")) {
				ruffle.densify2(std::next(ruffle.sections.begin(), index));
				update_visualization();
			}
		}

		if (ImGui::CollapsingHeader("Physics stuff")) {
			if (ImGui::Button("Solve physics")) {
				ruffle.physics_solve();
				update_visualization();
			}
			if (ImGui::Button(simulating ? "Stop simulation" : "Start simulation", ImVec2(-1,0))) {
				simulating = !simulating;
				viewer.core().is_animating ^= 1;
			}
			if (ImGui::Button("Step physics")) {
				ruffle.simulator->step(ruffle.simulation_mesh);
				update_visualization();
			}
			if (ImGui::Button("Project Air mesh")) {
				ruffle.simulation_mesh.air_mesh.project(ruffle.simulation_mesh.x);
				update_visualization();
			}
			if (ImGui::Button("Regen Air mesh")) {
				ruffle.simulation_mesh.air_mesh.clear();
				ruffle.simulation_mesh.generate_air_mesh();
				update_visualization();
			}
		}

		if (ImGui::CollapsingHeader("Optimization stuff")) {
			if (ImGui::Button("Step heuristic")) {
				heuristic.step(ruffle);
				update_visualization();
			}
			if (ImGui::Button("Intersect cylinder H")) {
				for (auto &vx : ruffle.simulation_mesh.vertices) {
					Vector2 xy = ruffle.simulation_mesh.get_vertex_position(vx);
					real dy = xy.y() - 5.;
					real w = 0.;
					if (dy*dy < 5.*5.) {
						w = 2.*sqrt(5.*5.-dy*dy);
					}
					w = max(0.1, w);
					vx.width = w;
				}
				update_visualization();
			}
			if (ImGui::Button("Intersect cylinder V")) {
				for (auto &vx : ruffle.simulation_mesh.vertices) {
					Vector2 xy = ruffle.simulation_mesh.get_vertex_position(vx);
					real dy = xy.x() - 3.;
					real w = 0.;
					if (dy*dy < 5.*5.) {
						w = 2.*sqrt(5.*5.-dy*dy);
					}
					w = max(0.1, w);
					vx.width = w;
				}
				update_visualization();
			}

			if (ImGui::Button("Export SVG")) {
				auto svg = create_svg(ruffle);
				svg->SaveFile("/tmp/test.svg");
			}
		}

		if (ImGui::CollapsingHeader("Debug stuff")) {
			if (ImGui::Button("dbg tangents")) {
				int i = 0;
				for (auto it = ruffle.connection_points.begin(); it != ruffle.connection_points.end(); ++it, ++i) {
					Vector2 tan = ruffle.get_tangent(*it);
					cout << "tan(" << i << ") = " << tan.transpose() << endl;
				}
			}
			if (ImGui::Button("dbg outline")) {
				for (auto outline_sec : ruffle.outline_sections) {
					cout << outline_sec << endl;
				}
			}
			if (ImGui::Button("clone")) {
				Ruffle cloned = ruffle.clone();
				ruffle = std::move(cloned);
				update_visualization();
			}
		}




	};

	viewer.launch();
	return 0;
}
}
