#include "editor/tools/ruffle_optimizer.h"

#include "editor/utils/logger.h"
#include "editor/utils/view_utils.h"
#include "editor/utils/ruffle_view.h"
#include "editor/style/editor_style.h"

#include "common/clone_helper.h"
#include "common/imgui.h"
#include "output/create_svg.h"


namespace ruffles::editor {

RuffleOptimizer::RuffleOptimizer(ViewModel& view_model, DataModel& data_model) : AbstractTool(view_model, data_model)
{
	//empty on purpose
}

void RuffleOptimizer::update_view(igl::opengl::glfw::Viewer& viewer)
{
	if (view_model.selected_part_index != prev_selected_part) {
		has_changed = true;
		prev_selected_part = view_model.selected_part_index;
	}
	if (!has_changed && !have_parts_changed())
		return;

	for (int i = 0; i < data_model.parts.size(); i++)
		update_part_view(viewer, i);


	has_changed = false;
}

void RuffleOptimizer::update_menu(Menu& menu)
{
	part = view_model.selected_part_index >= 0 && view_model.selected_part_index < data_model.parts.size() ? &data_model.parts[view_model.selected_part_index] : NULL;

	ImGui::Spacing();
	ImGui::Spacing();

	if (part == NULL) {
		//menu.deactivate_elements();
		return;
	}

	if (ImGui::Checkbox("Show ruffle", &is_visible))
	{
		has_changed = true;
	}


	if (ImGui::Button("Intersect with target mesh")) {
		part->intersect_ruffle();
		has_changed = true;
	}

	if (ImGui::Button("Export SVGs")) {
		int i = 0;
		for (auto &part : data_model.parts) {
			auto svg = create_svg(part.ruffle());
			auto fname = "/tmp/part" + to_string(i) + ".svg";
			svg->SaveFile(fname.c_str());
			i++;
		}
	}

	ImGui::Indent();
	
	if (!ImGui::CollapsingHeader("Optimization control", ImGuiTreeNodeFlags_DefaultOpen)) {
		return;
	}

	if (ImGui::Button("Step heuristic")) {
		optimization::Heuristic heuristic(part->target());
		heuristic.step(part->ruffle());
		has_changed = true;
	}

	if (ImGui::Button("Step heuristic (outer)")) {
		optimization::Heuristic heuristic(part->target());
		heuristic.step_outer(part->ruffle());
		has_changed = true;
	}

	if (ImGui::Button("Step heuristic (inner)")) {
		optimization::Heuristic heuristic(part->target());
		heuristic.step_inner(part->ruffle());
		has_changed = true;
	}

	if (ImGui::Button("Physics solve")) {
		part->ruffle().physics_solve();
		has_changed = true;
	}

	if (ImGui::Button("(Re-)Generate air mesh")) {
		part->ruffle().simulation_mesh.generate_air_mesh();
		has_changed = true;
	}
/*
	if (ImGui::Button("perturb")) {
		part->ruffle().simulation_mesh.perturb(0.1 * part->ruffle().h);
		has_changed = true;
	}

	static double dt = 1e-12;
	ImGui::InputDouble("step size", &dt);
	if (ImGui::Button("GD step")) {
		auto &mesh = part->ruffle().simulation_mesh;
		VectorX grad = VectorX::Zero(mesh.dof());
		real e = mesh.energy(mesh.x, &grad);
		dbg(e);
		mesh.x += dt * grad;
		
		has_changed = true;
	}

	*/

	if (part) {
		//ImGui::InputDouble("factor", &part->ruffle().simulation_mesh.k_global);
		ImGui::InputDouble("k_M", &part->ruffle().simulation_mesh.lambda_membrane);
		ImGui::InputDouble("k_B", &part->ruffle().simulation_mesh.k_bend);
		ImGui::InputDouble("k_AM", &part->ruffle().simulation_mesh.lambda_air_mesh);

		ImGui::Separator();

		ImGui::InputReal("u0_ratio", &part->u0_ratio);
		ImGui::InputReal("swidth", &part->step_width);
		ImGui::InputReal("sheight", &part->step_height);
		ImGui::InputInt("count", &part->stack_count);
		if (ImGui::Button("Reinitialize")) {
			part->reinit_ruffle();
			has_changed=true;
		}

		ImGui::Separator();

		ImGui::Text("Ruffle: %lu CPs, %lu Sections", part->ruffle().connection_points.size(), part->ruffle().sections.size());
		ImGui::Text("SimMesh: %lu vertices, %lu segments", part->ruffle().simulation_mesh.vertices.size(), part->ruffle().simulation_mesh.segments.size());
		ImGui::Text("AirMesh: %lu tris", part->ruffle().simulation_mesh.air_mesh.cdt.number_of_faces());
		ImGui::Text("Last physics solve time: %f ms", part->ruffle().last_physics_solve_time * 1000.);
		ImGui::Text("Avg solve time: %f ms", part->ruffle().physics_solve_total_time * 1000. / part->ruffle().physics_solve_count);
		if (ImGui::Button("Reset avg")) {
			part->ruffle().physics_solve_total_time = 0;
			part->ruffle().physics_solve_count = 0;
		}
	}


	if (part == NULL)
		menu.activate_elements();

	ImGui::Unindent();
}

bool RuffleOptimizer::callback_key_up(igl::opengl::glfw::Viewer& viewer, unsigned int key, int modifiers)
{
	if (part == NULL)
		if (part == NULL)
		return false;

	return false;
}

void RuffleOptimizer::update_part_view(igl::opengl::glfw::Viewer& viewer, int part_index)
{
	//add part if new
	if (view_indices.size() <= part_index)
	{
		int viewID = viewer.append_mesh();
		view_indices.push_back(viewID);
	}


	view_model.ruffles_mesh = utils::all_ruffles_mesh(data_model.parts);
	if (ruffle_mesh_view_index == -1) {
		ruffle_mesh_view_index = viewer.append_mesh();
	}
	viewer.selected_data_index = ruffle_mesh_view_index;
	viewer.data().clear();
	viewer.data().set_mesh(view_model.ruffles_mesh.V(), view_model.ruffles_mesh.F());
	//viewer.data().uniform_colors(Colors::GRAY_LIGHT, Colors::GRAY_LIGHT, Colors::BLACK);
	viewer.data().uniform_colors(Colors::GRAY_DARK, style::fill_ruffle, Colors::BLACK);
	
	viewer.data().point_size = style::points_small;
	viewer.data().line_width = style::wire_thickness;
	viewer.data().line_color = Colors::to_4f(style::wire_color);
	viewer.data().double_sided = true;
	viewer.data().face_based = true;


	//update view
	viewer.selected_data_index = view_indices[part_index];
	ModelPart& current_part = data_model.parts[part_index];

	viewer.data().clear();
	viewer.data().set_visible(is_visible);
	

	// show mesh
	auto &mesh = current_part.ruffle().simulation_mesh;
	auto &target = current_part.target();

	using ruffles::simulation::SimulationMesh;
	std::unordered_map<listref<SimulationMesh::Vertex>, int, listref_hash<SimulationMesh::Vertex>> indices;

/*
	MatrixX V(2*mesh.vertices.size(), 3);
	int i = 0;
	for (auto it = mesh.vertices.begin(); it != mesh.vertices.end(); ++it, i++) {
		indices.emplace(it, i);
		Vector2 uv = mesh.get_vertex_position(*it);
		Vector3 xyz = target.origin + uv(0)*target.u_dir + uv(1)*target.v_dir;
		Vector3 n = target.u_dir.cross(target.v_dir);
		V.row(2*i+0) << (xyz+it->z.front()*n).transpose();
		V.row(2*i+1) << (xyz+it->z.back()*n).transpose();
	}
	MatrixXi F(2*mesh.segments.size(), 3);
	i = 0;
	for (auto &seg : mesh.segments) {
		int a = indices[seg.start];
		int b = indices[seg.end];
		F.row(2*i+0) << 2*a, 2*b, 2*b+1;
		F.row(2*i+1) << 2*b+1, 2*a+1, 2*a;
		i++;
	}
	
	viewer.data().set_mesh(V, F);
	*/

	for (auto &[v, m] : mesh.extra_mass) {
		Vector2 uv = mesh.get_vertex_position(*v);
		Vector3 xyz = target.origin + uv(0)*target.u_dir + uv(1)*target.v_dir;
		viewer.data().add_label(xyz, "M");
	}
	viewer.data().show_custom_labels = true;
	/*
	viewer.data().uniform_colors(
		part_index == view_model.selected_part_index ? Colors::YELLOW : Colors::GRAY_LIGHT,
		part_index == view_model.selected_part_index ? Colors::YELLOW : Colors::GRAY_LIGHT,
		Colors::BLACK);
	*/
	
	viewer.data().grid_texture();

	//update outline
	Eigen::MatrixXd edges_start(2,3), edges_end(2,3), colors(2,3);
	edges_start << target.origin.transpose(),
	               target.origin.transpose();
	edges_end   << target.u_dir.transpose(),
	               target.v_dir.transpose();
	edges_end += edges_start;
	colors << 1., 0., 1.,
	          0., 1., 1.;
	view_model.viewer.data().add_edges(edges_start, edges_end, colors);

}

bool RuffleOptimizer::have_parts_changed()
{
	return view_indices.size() != data_model.parts.size();
}


}