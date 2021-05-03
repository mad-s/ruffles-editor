#include "editor/utils/list.h"

namespace ruffles::utils {
	
	void index_to_value(const std::vector<int>& indices, const Eigen::MatrixXd& lookup, Eigen::MatrixXd& out_values)
	{
		out_values.resize(indices.size(), lookup.cols());
		for (int i = 0; i < indices.size(); i++)
			out_values.row(i) = lookup.row(indices[i]);
	}

}