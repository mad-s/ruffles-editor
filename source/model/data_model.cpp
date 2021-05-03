#include "data_model.h"

#include <igl/facet_components.h>
#include <igl/remove_unreferenced.h>
#include <igl/boundary_loop.h>
#include <igl/topological_hole_fill.h>

#include "editor/utils/filesystem_io.h"
#include "editor/utils/logger.h"

#include <map>
#include <set>

using namespace Eigen;
namespace ruffles::model {


Mesh& DataModel::target()
{ 
	return _target; 
}

void DataModel::target(Mesh& value)
{
	_target.clear();
	_target.V(value.V());
	_target.F(value.F());

	if (!do_auto_update)
		return;

	//setup parts models
	Eigen::VectorXi C;
	igl::facet_components(_target.F(), C);
	update_parts(C);
}

void DataModel::target(Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
	Mesh mesh;
	mesh.V(V);
	mesh.F(F);

	target(mesh);
}

void DataModel::update_parts(Eigen::VectorXi C)
{
	const int n = C.maxCoeff() + 1;
	write_log(4) << "data_model.update_parts with " << n << " component(s)" << std::endl;

	//get faces from component labels
	std::vector<Eigen::MatrixXi> face_components(n);
	for (int i = 0; i < n; i++)
	{
		int nf = (C.array() == i).count();
		face_components[i].resize(nf, 3);

		write_log(4) << "  label " << i << " count = " << nf << std::endl;
	}

	std::vector<int> label_length(n, 0);
	for (int i = 0; i < C.rows(); i++)
	{
		int label = C(i);
		face_components[label].row(label_length[label]) = _target.F().row(i);
		label_length[label]++;
	}

	//delete all parts
	clear_parts();


	// identify shared vertices between components and construct graph
	real min_y = infinity;
	vector<std::set<int>> edges(n);

	// yay c++
	std::map<array<real,3>, int> dedup_vertices;

	for (int i = 0; i < _target.V().rows(); i++) {
		Vector3 xyz = _target.V().row(i);
		array<real,3> xyz_array {xyz.x(), xyz.y(), xyz.z()};
		min_y = min(min_y, xyz.y());
		int c = C(_target.adjacency_VF()[i].front());
		auto it = dedup_vertices.find(xyz_array);
		if (it != dedup_vertices.end()) {
			edges[c].emplace(it->second);
			edges[it->second].emplace(c);
		} else {
			dedup_vertices.emplace(xyz_array,c);
		}
	}

	// parts on ground are "rooted" and form their own tree
	vector<bool> rooted(n, false);

	//add new parts based on components
	Eigen::VectorXi I;
	for (int i = 0; i < n; i++)
	{
		Mesh mesh;
		igl::remove_unreferenced(_target.V(), face_components[i], mesh.V(), mesh.F(), I);

		auto &V = mesh.V();
		auto &F = mesh.F();

		// any part within 0.5cm of ground is rooted
		rooted[i] = V.col(1).minCoeff() < min_y + 0.5;

		vector<vector<int>> bnd;
		igl::boundary_loop(F, bnd);
		
		VectorX dummy; // idk what this is supposed to be, this parameter is unused
		MatrixXi F_new;
		igl::topological_hole_fill(F, dummy, bnd, F_new);
		
		int n = V.rows();
		V.conservativeResize(n+bnd.size(), 3);
		
		for (int i = 0; i < bnd.size(); i++) {
			Vector3 mean = Vector3::Zero();
			for (auto x : bnd[i]) {
				mean += V.row(x).transpose();
			}
			mean /= bnd[i].size();
			V.row(n+i) = mean.transpose();
		}

		mesh.F(F_new);
		// this should have called update, we don't need to feed V again (it was modified inplace)

		parts.emplace_back(mesh);
		auto &part = parts.back();

		if (!rooted[i]) {
			// align "ground" plane through boundary loop
			assert(!bnd.empty());
			vector<int> &longest_boundary = bnd.front();;
			int longest_boundary_length = 0;
			for (auto &loop : bnd) {
				if (loop.size() > longest_boundary_length) {
					longest_boundary_length = loop.size();
					longest_boundary = loop;
				}
			}
			MatrixX V_bnd(longest_boundary_length, 3);
			for (int i = 0; i < longest_boundary_length; i++) {
				V_bnd.row(i) = V.row(longest_boundary[i]);
			}

			part.ground_plane().align_pca(V_bnd);

			Vector3 centroid = part.segment().V().colwise().mean().transpose();
			if ((centroid - part.ground_plane().origin()).dot(part.ground_plane().normal()) < 0) {
				part.ground_plane().flip();
			}

		}
	}






	// dfs
	vector<bool> vis(n, false);
	vector<int> stack;
	for (int i = 0; i < n; i++) {
		if (rooted[i]) {
			stack.push_back(i);
			while (!stack.empty()) {
				int x = stack.back();
				stack.pop_back();
				if (vis[i]) continue;
				vis[i] = true;
				for (auto y : edges[x]) {
					if (!rooted[y]) {
						dbg(x);
						dbg(y);
						parts[x].add_child(&parts[y]);
						stack.push_back(y);
					}
				}
			}
		}
	}

	for (auto &part : parts) {
		part.update_extra_masses();
	}

	if (parts.size() == 1) {
		//parts[0].apex = Vector3(0., 10., 30);
	}
}

void DataModel::update_parts_graph(int root_component) {
	
}

void DataModel::clear()
{
	_target.clear();
	clear_parts();
}

void DataModel::clear_parts()
{
	for (auto& part : parts)
		part.clear();

	parts.clear();
}

std::string DataModel::absolute_target_path()
{
	return _models_folder + _target_file;
}

void DataModel::models_folder(std::string& value)
{
	_models_folder = utils::get_generic_path(value);

	if (!_target_file.empty())
		utils::try_get_path_relative_to(_target_file, _models_folder, _target_file);
}

void DataModel::target_file(std::string& value)
{
	_target_file = utils::get_generic_path(value);

	if (!_models_folder.empty())
		utils::try_get_path_relative_to(_target_file, _models_folder, _target_file);
}
bool DataModel::has_target_file()
{
	return !_target_file.empty();
}

}
