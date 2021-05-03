#pragma once

#include "editor/tools/abstract_tool.h"
#include "model/data_model.h"

using namespace ruffles::model;
namespace ruffles::editor {

class Segmenter : public AbstractTool
{
public:
	Segmenter(ViewModel& view_model, DataModel& data_model);

	int segment_index = -1;
	std::vector<std::vector<int>> selected_vertices;

	virtual void update_view(igl::opengl::glfw::Viewer& viewer) override;
	virtual void update_menu(Menu& menu) override;

	virtual bool callback_key_up(igl::opengl::glfw::Viewer& viewer, unsigned int key, int modifiers) override;
	virtual bool callback_key_down(igl::opengl::glfw::Viewer& viewer, unsigned int key, int modifiers) override;
	virtual bool callback_mouse_move(igl::opengl::glfw::Viewer& viewer, int mouse_x, int mouse_y) override;
	virtual bool callback_mouse_up(igl::opengl::glfw::Viewer& viewer, int button, int modifier) override;

	void initialize_with_segments(std::vector<std::vector<int>> segments);
	std::string to_string();


private:
	int pre_selected_vertex = -1;
	std::vector<int> pre_segment_path;
	const int end_signifier = -1; //TODO remove this, this leads to problems!

	int view_index = -1;
	bool is_selecting = false;

	void toggle_segmentation(bool is_selecting);
	void finalize_segment();
	Eigen::VectorXi label_faces();

	bool add_path();

	void find_edge_path();
	void add_segment();
	void add_segment(std::vector<int>& vertex_indices);
	void delete_segment();

	Eigen::MatrixXd get_segement_points_at(int index);
	Eigen::MatrixXd get_preview_segement_points();
};
}