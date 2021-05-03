#pragma once

#include <string>

namespace ruffles::utils {

	std::string get_generic_path(const std::string& file_or_folder);
	std::string get_folder_path(const std::string& file);
	std::string get_auto_filepath(const std::string reference_filepath);
	bool try_get_path_relative_to(const std::string& filepath, const std::string& reference_folderpath, std::string& out_relative_filepath);

}