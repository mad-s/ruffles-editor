#pragma once

#include "editor/tools/abstract_tool.h"
#include "model/model_part.h"

using namespace ruffles::model;
namespace ruffles::editor {

	class ForcePositioning : public AbstractTool
	{

	public:
		ForcePositioning(ViewModel& view_model, DataModel& data_model);

		virtual void update_view(igl::opengl::glfw::Viewer& viewer) override;
		virtual void update_menu(Menu& menu) override;

		virtual bool callback_key_up(igl::opengl::glfw::Viewer& viewer, unsigned int key, int modifiers) override;
		virtual bool callback_mouse_move(igl::opengl::glfw::Viewer& viewer, int mouse_x, int mouse_y) override;
		virtual bool callback_mouse_up(igl::opengl::glfw::Viewer& viewer, int button, int modifier) override;


	private:
		int view_index = -1;

		int pre_selected_vertex = -1;
		int selected_vertex = -1;
		
		Eigen::RowVector3d force_direction = Eigen::RowVector3d::UnitY();
		Eigen::RowVector3d force_position = Eigen::RowVector3d::Zero();
		Eigen::RowVector3d default_direction = Eigen::RowVector3d::UnitY();


		void update_magnitude(double scale);
		void update_rotation(Eigen::Vector3d rotate_by);
	};

}