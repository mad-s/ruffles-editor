#pragma once

#include "editor/elements/abstract_element.h"
#include "model/mesh_model.h"

using namespace ruffles::model;
namespace ruffles::editor {

class MeshRenderer : public AbstractElement
{

public:
	MeshRenderer(igl::opengl::glfw::Viewer& viewer, Menu& menu) : AbstractElement(viewer, menu) {};

	virtual void update_view(igl::opengl::glfw::Viewer& viewer) override;
	virtual void update_menu(Menu& menu) override;

	void add_mesh(Mesh& mesh);

private:
	Mesh* mesh = NULL;

	int view_index = -1;
	bool pending_add_mesh = false;

	bool show_fill;
	float color_menu[3];
	Eigen::Vector3d fill;


	void add_mesh_to_view(igl::opengl::glfw::Viewer& viewer);
	bool is_valid();

};

}