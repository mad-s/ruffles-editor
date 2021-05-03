#include "ruffle_view.h"

#include "simulation/simulation_mesh.h"
#include "editor/utils/concatenate_meshes.h"

namespace ruffles::utils
{

Mesh ruffle_mesh(ModelPart& part)
{
	auto& mesh = part.ruffle().simulation_mesh;
	auto& target = part.target();

	using ruffles::simulation::SimulationMesh;
	std::unordered_map<listref<SimulationMesh::Vertex>, int, listref_hash<SimulationMesh::Vertex>> indices;

	MatrixX V(2 * mesh.vertices.size(), 3);
	int i = 0;
	for (auto it = mesh.vertices.begin(); it != mesh.vertices.end(); ++it, i++) {
		indices.emplace(it, i);
		Vector2 uv = mesh.get_vertex_position(*it);
		Vector3 xyz = target.origin + uv(0) * target.u_dir + uv(1) * target.v_dir;
		Vector3 n = target.u_dir.cross(target.v_dir);
		if (part.apex) {
			n = (xyz - *part.apex).normalized();
			xyz = *part.apex;
		}
		V.row(2 * i + 0) << (xyz + it->z.front() * n).transpose();
		V.row(2 * i + 1) << (xyz + it->z.back() * n).transpose();
	}
	MatrixXi F(2 * mesh.segments.size(), 3);
	i = 0;
	for (auto& seg : mesh.segments) {
		int a = indices[seg.start];
		int b = indices[seg.end];
		F.row(2 * i + 0) << 2 * a, 2 * b, 2 * b + 1;
		F.row(2 * i + 1) << 2 * b + 1, 2 * a + 1, 2 * a;
		i++;
	}

    return Mesh(V, F);
}

Mesh all_ruffles_mesh(std::vector<ModelPart>& parts)
{
	std::vector<Eigen::MatrixXd> V_list;
	std::vector<Eigen::MatrixXi> F_list;

	for (ModelPart& part : parts)
	{
		Mesh ruffle = utils::ruffle_mesh(part);
		V_list.push_back(ruffle.V());
		F_list.push_back(ruffle.F());
	}


	Eigen::MatrixXd ruffles_V;
	Eigen::MatrixXi ruffles_F;
	utils::concatenate_meshes(V_list, F_list, ruffles_V, ruffles_F);

	return Mesh(ruffles_V, ruffles_F);
}

pair<ModelPart*, listref<ruffles::simulation::SimulationMesh::Vertex>> get_simmesh_vertex(std::vector<ModelPart>& parts, int i) {
	for (ModelPart &part : parts) {
		auto &mesh = part.ruffle().simulation_mesh;
		int nv = 2*mesh.vertices.size();
		if (i < nv) {
			return {&part, std::next(mesh.vertices.begin(), i/2)};
		} else {
			i -= nv;
		}
	}

	panic("at the disco");
}

}