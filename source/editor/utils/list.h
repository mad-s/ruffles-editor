#pragma once

#include <vector>
#include <algorithm>
#include <Eigen/Dense>

namespace ruffles::utils {
	
	template<typename T>
	int index_of(const T& element, const std::vector<T>& list)
	{
		auto iterator = std::find(list.begin(), list.end(), element);
		if (iterator == list.end()) //not found
			return -1;

		int index = std::distance(list.begin(), iterator);
		return index;
	}

	template<typename T>
	bool is_contained(const T& element, const std::vector<T>& list)
	{
		auto iterator = std::find(list.begin(), list.end(), element);
		bool is_in_list = iterator != list.end();

		return is_in_list;
	}

	template<typename T>
	std::vector<T> flatten_list(std::vector<std::vector<T>> list)
	{
		std::vector<T> flat;
		for (auto l : list)
			flat.insert(flat.end(), l.begin(), l.end());

		return flat;
	}

	template<typename T>
	std::vector<T> remove_duplicates(std::vector<T>& list)
	{
		std::sort(list.begin(), list.end());
		list.erase(std::unique(list.begin(), list.end()), list.end());
		return list;
	}

	template<typename T>
	std::vector<T> intersect(std::vector<T>& list1, std::vector<T>& list2)
	{
		std::vector<T> intersection_list(list1.size() + list2.size());

		std::sort(list1.begin(), list1.end());
		std::sort(list2.begin(), list2.end());

		auto iterator = std::set_intersection(list1.begin(), list1.end(), list2.begin(), list2.end(), intersection_list.begin());
		intersection_list.resize(iterator - intersection_list.begin());

		return intersection_list;
	}


	void index_to_value(const std::vector<int>& indices, const Eigen::MatrixXd& lookup, Eigen::MatrixXd& out_values);

	//template<typename DerivedLookup>
	//void index_to_value(const std::vector<int>& indices, const Eigen::MatrixBase<DerivedLookup>& lookup, Eigen::MatrixBase<DerivedLookup>& out_values)
	//{
	//	out_values.resize(indices.size(), lookup.cols());
	//	for (int i = 0; i < indices.size(); i++)
	//		out_values.row(i) = lookup.row(indices[i]);
	//}


	//void index_to_value(const Eigen::VectorXi& indices, const Eigen::MatrixXi& lookup, Eigen::MatrixXi& out_values)
	//{
	//	out_values.resize(indices.size(), lookup.cols());
	//	for (int i = 0; i < indices.size(); i++)
	//		out_values.row(i) = lookup.row(indices[i]);
	//}

	//void value_to_index(const Eigen::MatrixXd& values, const Eigen::MatrixXd& lookup, std::vector<int>& out_indices)
	//{
	//	if (values.cols() != lookup.cols())
	//		return;
	//
	//	out_indices.clear();
	//	for (int i = 0; i < values.rows(); i++)
	//	{
	//		int index;
	//		get_closest_vertex(lookup, values.row(i), index);
	//		out_indices.push_back(index);
	//	}
	//}

	//std::vector<double> to_std_vector(const Eigen::VectorXd& eigen_vector)
	//{
	//	std::vector<double> std_vector(eigen_vector.data(), eigen_vector.data() + eigen_vector.size());
	//	return std_vector;
	//}

	//Eigen::VectorXd to_eigen_vector(std::vector<double>& std_vector)
	//{
	//	Eigen::VectorXd eigen_vector = Eigen::Map<Eigen::VectorXd>(std_vector.data(), std_vector.size());
	//	return eigen_vector;
	//}

}