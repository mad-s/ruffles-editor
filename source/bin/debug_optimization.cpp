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

#include "optimization/target_shape.h"
#include "optimization/particle_swarm.h"
#include "optimization/heuristic.h"

#include "visualization/visualization.h"

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

	Ruffle ruffle = Ruffle::create_ruffle_stack(3, 3., 5., 0.5);
	for (auto &vx : ruffle.simulation_mesh.vertices) {
		vx.width = 5.;
	}
	ruffle.simulation_mesh.generate_air_mesh();
	ruffle.simulator.reset(new simulation::LBFGS(ruffle.simulation_mesh));
	ruffle.physics_solve();

	Polygon triangle;
	triangle.push_back(Point(-2,0.));
	triangle.push_back(Point(8,0.));
	triangle.push_back(Point(8,10.));
	triangle.push_back(Point(-2,10.));
	/*
	triangle.push_back(Point(-1,0.));
	triangle.push_back(Point(2.,0.));
	triangle.push_back(Point(0.5,3.5));
	*/
/*
	int n = 32;
	for (int i = 0; i < n; i++) {
		real phi = 2*M_PI*(i + 0.5)/n;
		triangle.push_back(Point(std::sin(phi)+0.5, 1.-std::cos(phi)-(1.-std::cos(2*M_PI*0.5/n))));
	}
	*/

	optimization::TargetShape target(triangle);
	//optimization::ParticleSwarm pso(target, start_ruffle, 10);
	optimization::Heuristic heuristic(target);

	igl::opengl::glfw::Viewer viewer;

	int viewed_ruffle = 0;

	Visualization vis;
	auto update_visualization = [&]() {
		vis.clear();

		auto &mesh = ruffle.simulation_mesh;
		std::unordered_map<listref<SimulationMesh::Vertex>, int, listref_hash<SimulationMesh::Vertex>> indices;

		int i = 0;
		for (auto it = mesh.vertices.begin(); it != mesh.vertices.end(); ++it) {
			Vector2 pos = mesh.get_vertex_position(*it);
			vis.push_vertex((Vector3() << pos, -0.5*it->width).finished());
			vis.push_vertex((Vector3() << pos,  0.5*it->width).finished());
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

	igl::opengl::glfw::imgui::ImGuiMenu menu;
	viewer.plugins.push_back(&menu);
	menu.callback_draw_viewer_menu = [&]() {
		menu.draw_viewer_menu();

		/*
		if (ImGui::InputInt("View ruffle #i", &viewed_ruffle)) {
			if (viewed_ruffle >= (int)pso.particles.size()) {
				viewed_ruffle = pso.particles.size()-1;
			}
			update_visualization();
		}
		*/

		if (ImGui::Button("Step optimization")) {
			//pso.step();
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
				ruffle.densify(std::next(ruffle.sections.begin(), index));
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

		if (ImGui::Button("reset sim")) {
			ruffle.simulator->reset(ruffle.simulation_mesh);
		}
		if (ImGui::Button("solve physics")) {
			ruffle.physics_solve();
			update_visualization();
		}
		/*
		if (ImGui::Button("Debug particles")) {
			for (auto &particle : pso.particles) {
				dbg(particle.x);
			}
		}
		*/
	};

	viewer.launch();
	return 0;
}
}
