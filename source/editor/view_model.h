#pragma once

#include <igl/opengl/glfw/Viewer.h>
#include "editor/menu.h"
#include "editor/elements/abstract_element.h"
#include "editor/elements/mesh_renderer.h"

#include "model/mesh_model.h"

namespace ruffles::editor {

//enum UIState
//{
//	Initialize,
//	Segment,
//	CrossSection
//};

enum RuffleTool
{
	None,
	MeshPartition,
	PlanePosition,
	ForcePosition,
	DensityEdit,
	ConnectionPointEdit,
	MultiRuffleConnect,
	ChangeLengths,
	Unset
};

struct ViewModel
{
public:
	ViewModel() 
	{
		active_tool = RuffleTool::None;
	};
	~ViewModel();

	igl::opengl::glfw::Viewer viewer;
	Menu menu;

	//UIState current_state;
	RuffleTool active_tool;
	bool is_mouse_down = false;

	MeshRenderer* target_renderer;
	int selected_part_index;
	bool has_selected_part_changed;
	bool is_only_selected_visible = true;

	double transform_step = 0.025;


	Mesh ruffles_mesh;

	//UI element list for updating
	std::vector<AbstractElement*> elements;
	void add_element(AbstractElement* element);


private:

};

}