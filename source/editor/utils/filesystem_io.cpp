#include "editor/utils/filesystem_io.h"

#include "common/common.h"
#include <filesystem>
namespace fs = std::filesystem;

#include "editor/utils/logger.h"

namespace ruffles::utils {

std::string get_time_string();


std::string get_generic_path(const std::string& file_or_folder)
{
	auto path = fs::path(file_or_folder);
	return path.generic_string();
}

std::string get_folder_path(const std::string& file)
{
	auto filepath = fs::path(file);
	assert(filepath.has_parent_path());

	return filepath.parent_path().generic_string();
}

/*
reference_filepath is expected to be absolute, pointing to a file
*/
//std::string get_auto_filepath(std::string reference_filepath, bool add_timestamp, bool add_filename, char delimiter)
std::string get_auto_filepath(const std::string reference_filepath)
{
	write_log(0) << "reference_filepath" << reference_filepath << std::endl;

	fs::path filepath = fs::path(reference_filepath);
	assert(filepath.has_parent_path());
	assert(filepath.has_filename());

	std::string folder = filepath.parent_path().generic_string();
	std::string file = filepath.filename().generic_string();
	std::string filename = file.substr(0, file.size() - filepath.extension().generic_string().size());

	std::string datetime = get_time_string();
	std::string path = folder + "/" + datetime + "__" + filename;
	return path;
}

/*
filepath is expected to be absolute or relative, pointing to a file
reference_folderpath is expected to be absolute, pointing to a folder

output: 
	returns true if the relative version of *filepath* to *reference_folderpath* exists
		and sets *out_filepath* and *out_reference_folderpath* via generic format
	else returns false and leaves *out_filepath* and *out_reference_folderpath* unchanged
*/
bool try_get_path_relative_to(const std::string& filepath, const std::string& reference_folderpath, std::string& out_relative_filepath)
{
	auto path = fs::path(filepath);
	auto base = fs::path(reference_folderpath);
	assert(base.is_absolute());

	std::error_code error;
	fs::path relative_path = fs::relative(path, base, error);
	
	if (error.value() != 0)
		return false;

	if (path.is_relative() && relative_path.empty())
		out_relative_filepath = path.generic_string();
	else 
		out_relative_filepath = relative_path.generic_string();

	return true;
}

std::string get_time_string()
{
	time_t rawtime;
	time(&rawtime);
	struct tm* timeinfo = localtime(&rawtime);

	char buffer[80];
	strftime(buffer, 80, "%Y-%m-%d_%H-%M-%S", timeinfo);

	return buffer;
}

}