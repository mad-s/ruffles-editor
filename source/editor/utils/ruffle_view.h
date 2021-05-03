#pragma once

#include "model/model_part.h"

using namespace ruffles::model;
namespace ruffles::utils 
{
	Mesh ruffle_mesh(ModelPart& part);
	Mesh all_ruffles_mesh(std::vector<ModelPart>& parts);
	pair<ModelPart*, listref<ruffles::simulation::SimulationMesh::Vertex>> get_simmesh_vertex(std::vector<ModelPart>& parts, int i);
}