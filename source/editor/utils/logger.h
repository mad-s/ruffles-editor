#pragma once

#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Core>

/* Levels: (these are not enforced in any way)
	1 - error
	2 - warning
	3 - info
	4 - debug
	5 - verbose debug
	6 - ultra verbose debug
	0 - current debug (to be assigned to other level)
*/

//extern int LOG_LEVEL;
static int LOG_LEVEL = 4;

class mystreambuf : public std::streambuf
{ };

static mystreambuf nostreambuf;
static std::ostream nocout(&nostreambuf);

#define write_log(level) ((level <= LOG_LEVEL || LOG_LEVEL == 0) ? std::cout : nocout)
#define linebreak '\n'


template<typename T>
std::string list_to_string(const std::vector<T>& list, const std::string delimiter = " ", const bool multi_line = false, const bool print_index = false)
{
	std::stringstream output;
	output << std::fixed;

	for (int i = 0; i < list.size(); i++)
	{
		if (print_index)
			output << "[" << i << "]: ";

		output << list[i];

		if (i >= list.size() - 1)
			continue;

		if (multi_line)
			output << linebreak;
		else
			output << delimiter;
	}

	return output.str();
}

template<typename T>
std::string to_mathematica(const std::vector<T>& list)
{
	std::stringstream output;
	output << "{" << list_to_string(list, ", ") << "};";
	return output.str();
}

template <typename Derived>
std::string to_mathematica(const Eigen::MatrixBase<Derived>& matrix)
{
	int rows = matrix.rows();
	int cols = matrix.cols();

	if (rows < 1 || cols < 1)
		return "";

	std::stringstream output;

	output << "{";
	for (int r = 0; r < rows; r++)
	{
		if (cols == 1)
		{
			output << std::fixed << matrix(r, 0);
		}
		else
		{
			if (rows > 1)
				output << "{";

			for (int c = 0; c < cols; c++)
			{
				output << std::fixed << matrix(r, c);
				if (c < cols - 1)
					output << ", ";
			}
			if (rows > 1)
				output << "}";
		}

		if (r < rows - 1)
			output << "," << linebreak;
	}
	output << "};";
	
	return output.str();
}


#endif