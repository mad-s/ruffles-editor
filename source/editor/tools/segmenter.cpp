#include "editor/tools/segmenter.h"

#include <stack>
#include <algorithm>

#include <igl/dijkstra.h>

#include <igl/edges.h>
#include <igl/edge_flaps.h>
#include <igl/unique_edge_map.h>

#include "editor/utils/view_utils.h"
#include "editor/utils/list.h"
#include "editor/utils/logger.h"


using namespace std;

namespace ruffles::editor {

//TODO move to utils

std::vector<int> get_invalid_indices(const std::vector<int>& list, const int& invalid_mark)
{
	std::vector<int> unlabled;

	int offset = 0;
	bool is_done = false;

	while (!is_done)
	{
		auto iterator = std::find(list.begin() + offset, list.end(), invalid_mark);
		is_done = iterator == list.end();
		if (is_done)
			continue;

		offset = std::distance(list.begin(), iterator);
		unlabled.push_back(offset);
		offset++;
	}

	return unlabled;
}

void dfs(const std::vector<std::vector<int>>& search_list, const int& start_index, std::vector<int>& obstacles, std::vector<int>& out_component)
{
	// Initially mark all verices as not visited 
	std::vector<bool> visited(search_list.size(), false);

	// Create a stack for DFS 
	std::stack<int> stack;

	// Push the current source node. 
	stack.push(start_index);

	int index;
	out_component.clear();

	while (!stack.empty())
	{
		// Pop a vertex from stack 
		index = stack.top();
		stack.pop();

		////is excluding obstacles
		//std::vector<int>::iterator iterator = std::find(obstacles.begin(), obstacles.end(), index);
		//bool is_contained = iterator != obstacles.end();
		//if (is_contained)
		//	continue;

		// Stack may contain same vertex twice. So we need to print the popped item only if it is not visited. 
		if (!visited[index])
		{
			visited[index] = true;
			out_component.push_back(index);
		}

		//is including obstacles
		std::vector<int>::iterator iterator = std::find(obstacles.begin(), obstacles.end(), index);
		bool is_contained = iterator != obstacles.end();
		if (is_contained)
			continue;


		// Get all adjacent vertices of the popped vertex s 
		// If a adjacent has not been visited, then push it to the stack. 
		for (auto i = search_list[index].begin(); i != search_list[index].end(); ++i)
			if (!visited[*i])
				stack.push(*i);
	}
}

//TODO remove! only for temp debug
bool debug_label_segments = false;
bool debug_labels_visible = false;

Segmenter::Segmenter(ViewModel& view_model, DataModel& data_model) : AbstractTool(view_model, data_model)
{
	view_index = view_model.viewer.append_mesh();
	tool_type = RuffleTool::MeshPartition;

	//set style
	view_model.viewer.data().point_size = 10;
	view_model.viewer.data().line_width = 5;
}

void Segmenter::initialize_with_segments(std::vector<std::vector<int>> segments)
{
	selected_vertices = segments;
	segment_index = selected_vertices.size() - 1;

	Eigen::VectorXi C = label_faces();
	data_model.update_parts(C);
	
	//TODO remove! only for temp debug
	debug_label_segments = true;
	debug_labels_visible = false;
}

void Segmenter::update_view(igl::opengl::glfw::Viewer& viewer)
{
	//if (!has_changed)
	//	return;
	//if (!(debug_label_segments && !debug_labels_visible)) //TODO remove! only for temp debug
	//	return;

	viewer.selected_data_index = view_index;
	viewer.data().clear();

	if (is_selecting && pre_selected_vertex >= 0)
		viewer.data().add_points(data_model.target().V().row(pre_selected_vertex), Eigen::RowVector3d(0.5, 0.5, 0.5));

	//current preselected path part
	Eigen::MatrixXd preview_segement_points = get_preview_segement_points();
	utils::show_curve(viewer, preview_segement_points, Eigen::RowVector3d(0.35, 0.35, 0.35), true);

	//vector<Eigen::MatrixXd> all_features;
	int size = selected_vertices.size();
	for (int i = 0; i < selected_vertices.size(); i++)
	{
		utils::show_curve(viewer, get_segement_points_at(i), Eigen::RowVector3d(0.5, 0.5, 0.5), true);
		//all_features.push_back(get_segement_points_at(i));
	}

	//render_highlighted_curves(all_features, segment_selection.segment_index, render_color_features, true);


	//TODO (alex) clean, this is only for debug
	//if (selected_vertices.size() > 0 && debug_label_segments && !debug_label_segments)
	if (selected_vertices.size() > 0)
	{
		Eigen::VectorXi C = label_faces();
		//std::vector<int> fi_unlabled = get_invalid_indices(std::vector<int>(C.data(), C.data() + C.size()), -1);
		//write_log(0) << "unlabeled faces: " << list_to_string(fi_unlabled) << linebreak;

		//for (int fi : fi_unlabled)
		//{
		//	auto face = data_model.target().F().row(fi);
		//	Eigen::RowVector3d p = data_model.target().V().row(face(0));
		//	p += data_model.target().V().row(face(1));
		//	p += data_model.target().V().row(face(2));

		//	p = p / 3;
		//	viewer.data().add_label(p, std::to_string(fi));
		//}

		Eigen::MatrixXd colors;
		igl::colormap(igl::COLOR_MAP_TYPE_JET, C, 0.0, C.maxCoeff(), colors);

		viewer.data().show_lines = false;
		viewer.data().set_mesh(data_model.target().V(), data_model.target().F());
		viewer.data().set_colors(colors);

		//debug_labels_visible = true;


		/*
		Eigen::VectorXi fl = Eigen::Map<Eigen::VectorXi>(face_lables.data(), face_lables.size());
		Eigen::MatrixXd colors;
		igl::colormap(igl::COLOR_MAP_TYPE_JET, fl, 0.0, fl.maxCoeff(), colors);
		
		viewer.data().set_mesh(data_model.target().V(), data_model.target().F());
		viewer.data().set_colors(colors);
		*/
	}
}

void Segmenter::update_menu(Menu& menu)
{
	//bool is_open = is_enabled();
	//if (!ImGui::CollapsingHeader("options: mesh partitioning", &is_open))
		//return;
	if (!ImGui::CollapsingHeader("options: mesh partitioning", auto_header_open()))
		return;


	ImGui::Indent();

	if (ImGui::Button("delete current segment"))
		delete_segment();

	if (ImGui::Button("print segments"))
	{
		for (auto segment : selected_vertices)
			write_log(0) << list_to_string(segment, ", ") << linebreak;
	}

	if (ImGui::Button("cut segments"))
	{
		Eigen::VectorXi C = label_faces();
		data_model.update_parts(C);
	}

	ImGui::Unindent();
}


bool Segmenter::callback_key_up(igl::opengl::glfw::Viewer& viewer, unsigned int key, int modifiers)
{
	if (!is_enabled())
		return false;

	//write_log(4) << "Segmenter::callback_key_up key: " << key << ", modifier: " << modifiers << std::endl;

	if (key == GLFW_KEY_LEFT_CONTROL || key == GLFW_KEY_RIGHT_CONTROL)
	{
		is_selecting = false;
		toggle_segmentation(false);
	}
	else if (key == 'F')
	{
		finalize_segment();
	}

	return false;
}

bool Segmenter::callback_key_down(igl::opengl::glfw::Viewer& viewer, unsigned int key, int modifiers)
{
	if (!is_enabled())
		return false;

	//write_log(4) << "Segmenter::callback_key_down key: " << key << ", modifier: " << modifiers << std::endl;

	if (key == GLFW_KEY_LEFT_CONTROL || key == GLFW_KEY_RIGHT_CONTROL)
	{
		is_selecting = true;
		toggle_segmentation(true);
	}

	return false;
}

bool Segmenter::callback_mouse_move(igl::opengl::glfw::Viewer& viewer, int mouse_x, int mouse_y)
{
	if (!is_enabled())
		return false;

	//write_log(4) << "Segmenter::callback_mouse_move x = " << mouse_x << ", y = " << mouse_y << std::endl;
	if (!is_selecting)
		return false;

	pre_selected_vertex = utils::get_vertex_from_screen(viewer, mouse_x, mouse_y, data_model.target().V(), data_model.target().F());
	find_edge_path();

	return true;
}

bool Segmenter::callback_mouse_up(igl::opengl::glfw::Viewer& viewer, int button, int modifier)
{
	if (!is_enabled())
		return false;

	//write_log(4) << "Segmenter::callback_mouse_up button = " << button << ", modifier = " << modifier << std::endl;

	if (!is_selecting)
		return false;

	add_path();
	return true;
}

Eigen::VectorXi Segmenter::label_faces()
{
	if (selected_vertices.size() < 1)
		return Eigen::VectorXi::Zero(data_model.target().F().rows(), data_model.target().F().cols());

	//get cleaned flattend segment_vertices
	std::vector<int> segment_vertices = utils::flatten_list(selected_vertices);

	write_log(0) << "segment_obstacles before remove: " << list_to_string(segment_vertices) << linebreak;

	std::remove_if(segment_vertices.begin(), segment_vertices.end(), [](int i) { return i < 0; });

	if (segment_vertices.back() < 0)
		segment_vertices.pop_back();

	segment_vertices = utils::remove_duplicates(segment_vertices);

	write_log(0) << "segment_obstacles after remove: " << list_to_string(segment_vertices) << linebreak;

	std::vector<int> vertex_lables = std::vector<int>(data_model.target().V().rows(), -1);
	std::vector<int> face_lables = std::vector<int>(data_model.target().F().rows(), -1);

	
	//get face-based labels for connected components
	int current_label = 0;
	int vi = utils::index_of(-1, vertex_lables);

	bool is_done = false;
	while (!is_done)
	{
		std::vector<int> current_component;
		dfs(data_model.target().adjacency_VV(), vi, segment_vertices, current_component);

		//copy over
		for (int i = 0; i < current_component.size(); i++)
		{
			int v = current_component[i];
			vertex_lables[v] = current_label;

			auto neighbor_faces = data_model.target().adjacency_VF().at(v);

			for (int j = 0; j < neighbor_faces.size(); j++)
			{
				int fi = neighbor_faces[j];
				Eigen::RowVectorXi face_vi = data_model.target().F().row(fi);
				std::vector<int> face_vi_vec(face_vi.data(), face_vi.data() + face_vi.size());

				std::vector<int> intersection = utils::intersect(face_vi_vec, current_component);

				//only label face if ALL face vertices are contained in component
				if(intersection.size() == 3) //TODO check if at border, then it doesnt have to have 3 neighbors
					face_lables[fi] = current_label;
			}
		}

		current_label++;
		current_component.clear();

		vi = utils::index_of(-1, vertex_lables);
		is_done = vi == -1;
	}

	//TODO (alex) fix: do face not vertex labelling, because some faces can remain unlabeled
	std::vector<int> unlabeled = get_invalid_indices(face_lables, -1);
	if (unlabeled.size() > 0)
	{
		write_log(0) << linebreak << "unlabeled faces: " << list_to_string(unlabeled) << linebreak << linebreak;

		//fixing temporarily until changing labelling
		for (int unlabled_fi : unlabeled)
			face_lables[unlabled_fi] = 0;
	}


	//write_log(0) << "face_lables: " << list_to_string(face_lables) << linebreak;
	Eigen::VectorXi C = Eigen::Map<Eigen::VectorXi>(face_lables.data(), face_lables.size());
	return C;
}

void Segmenter::toggle_segmentation(bool is_selecting)
{
	write_log(0) << "toggle feature selection: " <<
		"is selecting: " << boolalpha << is_selecting <<
		",number of features: " << selected_vertices.size() <<
		", current index: " << segment_index;
	if (selected_vertices.size() > 0)
		write_log(0) << ", feature size: " << selected_vertices.back().size() << endl;
	else
		write_log(0) << endl;


	if (selected_vertices.size() < 1)
	{
		selected_vertices.push_back(vector<int>());
		segment_index = 0;
		return;
	}

	if (is_selecting && segment_index < 0 && selected_vertices.size() > 0)
		segment_index = selected_vertices.size() - 1;

	std::vector<int>* current_loop = &selected_vertices[segment_index];

	if (!is_selecting && current_loop->size() < 1)
	{
		selected_vertices.pop_back();
		segment_index--;
		return;
	}

	if (is_selecting && current_loop->size() > 1 && current_loop->back() == end_signifier)
	{
		selected_vertices.push_back(vector<int>());
		segment_index++;
		return;
	}


	if (is_selecting && current_loop->size() < 1)
		return;

	if (is_selecting && current_loop->front() == current_loop->back()) //if current loop is closed
		finalize_segment();
}

void Segmenter::finalize_segment()
{
	std::vector<int>* current_loop = &selected_vertices[segment_index];

	if (current_loop->size() < 1)
		return;

	if (current_loop->front() == current_loop->back() && current_loop->size() > 1) //if current loop is closed
		current_loop->pop_back();

	if (current_loop->back() != end_signifier)
		current_loop->push_back(end_signifier); //signifies that this crease is final

	selected_vertices.push_back(vector<int>());
	segment_index++;
}

bool Segmenter::add_path()
{
	if (pre_selected_vertex < 0)
		return false;

	std::vector<int>* current_loop = &selected_vertices[segment_index];
	int loop_size = current_loop->size();

	if (loop_size < 1) //if segment is empty, add selected vertex
	{
		current_loop->push_back(pre_selected_vertex);

		write_log(0) << "adding first index to loop: " << list_to_string(*current_loop) << endl;
		return false;
	}

	if (pre_segment_path.size() <= 0)
		return false;

	reverse(pre_segment_path.begin(), pre_segment_path.end());
	current_loop->insert(current_loop->end(), pre_segment_path.begin() + 1, pre_segment_path.end());
	pre_segment_path.clear();

	write_log(0) << endl << "ADD regularly" << endl;

	int first = current_loop->front();
	int last = current_loop->back();

	if (first == last) //is closing loop
	{
		write_log(0) << endl << "CLOSED segmentation loop! " << list_to_string(*current_loop) << endl << endl;

		finalize_segment();
		return true;
	}

	write_log(0) << endl << "current_loop: " << list_to_string(*current_loop) << endl << endl;
	return true;
}

void Segmenter::add_segment()
{
	selected_vertices.push_back(vector<int>());
	segment_index++;
}

void Segmenter::add_segment(std::vector<int>& vertex_indices)
{
	selected_vertices.push_back(vertex_indices);
	segment_index = selected_vertices.size() - 1;
	finalize_segment();
}

void Segmenter::delete_segment()
{
	selected_vertices.erase(selected_vertices.begin() + segment_index);
	segment_index--;
}

void Segmenter::find_edge_path()
{
	write_log(0) << "segmentation: find_edge_path() pre_selected_vertex = " << pre_selected_vertex << linebreak;
	if (pre_selected_vertex < 0)
		return;

	int loop_index = selected_vertices.size() - 1;
	write_log(0) << "segmentation: loop_index = " << loop_index << linebreak;
	if (loop_index < 0)
		return;

	std::vector<int> *current_loop = &selected_vertices[loop_index];
	int loop_size = current_loop->size();

	if (loop_size < 1)
		return;

	int source = current_loop->at(loop_size - 1);
	write_log(0) << "segmentation: find path from " << source << " to " << pre_selected_vertex;

	std::set<int> targets{ pre_selected_vertex };
	Eigen::VectorXd min_distance;
	Eigen::VectorXi previous;
	igl::dijkstra(source, targets, data_model.target().adjacency_VV(), min_distance, previous); //compute distances
	igl::dijkstra(pre_selected_vertex, previous, pre_segment_path); //backtrack to get shortest path

	write_log(0) << "pre_segment_path: " << list_to_string(pre_segment_path) << std::endl;
	//log_list(6, pre_segment_path, "  --> dijkstra path: ", false);
}

Eigen::MatrixXd Segmenter::get_segement_points_at(int index)
{
	if (index < 0 || index > selected_vertices.size() - 1)
		return Eigen::MatrixXd();

	auto& loop = selected_vertices[index];
	if (loop.size() < 1)
		return Eigen::MatrixXd();

	int size = loop.back() == end_signifier ? loop.size() - 1 : loop.size();

	Eigen::MatrixXd segment_points(size, 3);
	for (int j = 0; j < size; j++)
		segment_points.row(j) = data_model.target().V().row(loop[j]);

	return segment_points;
}

Eigen::MatrixXd Segmenter::get_preview_segement_points()
{
	Eigen::MatrixXd preview_segement_points;
	utils::index_to_value(pre_segment_path, data_model.target().V(), preview_segement_points);
	//write_log(0) << "    indices pre_segment_path: " << list_to_string(pre_segment_path) << linebreak;
	//write_log(0) << "    lookup: preview_segement_points: " << linebreak << preview_segement_points << linebreak << linebreak;

	return preview_segement_points;
}

std::string Segmenter::to_string()
{
	stringstream stream;
	stream << endl << "segments on target: " << endl;

	for (int i = 0; i < selected_vertices.size(); i++)
		write_log(0) << linebreak << "selected_vertices: " << list_to_string(selected_vertices[i]) << linebreak;

	stream << endl;
	return stream.str();
}

}