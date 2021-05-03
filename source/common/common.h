#pragma once

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <vector>
#include <utility>
#include <string>

#include <iostream>

#include <memory>
#include <list>
#include <array>
#include <iterator>

#ifndef dbg
#define dbg(x) debug_impl(#x, x)
#endif
#ifndef dbg_list
#define dbg_list(x) debug_list_impl(#x, x.begin(), x.end())
#endif
#define panic(x) panic_impl(__FILE__, __LINE__, x)

namespace ruffles {

// for using with std::variant
template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;

template <typename T>
T debug_impl(std::string name, T x) {
	std::cout << name << " = " << x << std::endl;
	return x;
}
template <typename T>
void debug_list_impl(std::string name, T begin, T end) {
	std::cout << name << " = [";
	int i = 0;
	while (begin != end) {
		if (i) {
			std::cout << ", ";
		}
		std::cout << *begin;
		++begin;
		++i;
	}
	std::cout << "]" << std::endl;
}

void panic_impl(std::string fname, int line, std::string what = "");


//using real = float;
using real = double;
constexpr real infinity = std::numeric_limits<real>::infinity();
using std::string;
using std::to_string;
using std::cout;
using std::cerr;
using std::endl;

using std::vector;
using std::array;
using std::list;
template<typename T>
using listref = typename list<T>::iterator;


using std::tuple;
using std::pair;

using std::unique_ptr;
using std::shared_ptr;
//namespace chrono = std::chrono;
real max(real a, real b);
real min(real a, real b);

using Eigen::Matrix;
using Eigen::MatrixXi;
using Eigen::VectorXi;
using VectorXb = Eigen::Matrix<bool, -1, 1>;

using Vector2 = Eigen::Matrix<real, 2, 1>;
using Vector3 = Eigen::Matrix<real, 3, 1>;
using Vector4 = Eigen::Matrix<real, 4, 1>;
using Vector6 = Eigen::Matrix<real, 6, 1>;
using VectorX = Eigen::Matrix<real, -1, 1>;

using Matrix2 = Eigen::Matrix<real, 2, 2>;
using Matrix3 = Eigen::Matrix<real, 3, 3>;
using Matrix4 = Eigen::Matrix<real, 4, 4>;
using Matrix6 = Eigen::Matrix<real, 6, 6>;
using MatrixX = Eigen::Matrix<real, -1, -1>;

VectorX random_vector(int n);


real angle(Vector6 x, Vector6 *grad = nullptr, Matrix6 *hessian = nullptr);

// can't define in cpp file because template function
template<typename T>
void addHessianBlock(const T &mat, int row, int col, std::vector<Eigen::Triplet<real>> &target) {
	for (int i = 0; i < mat.rows(); i++) {
		for (int j = 0; j < mat.cols(); j++) {
			target.emplace_back(row+i, col+j, mat(i,j));
		}
	}
}

template<typename T1, typename T2>
std::ostream &operator<<(std::ostream &os, const std::pair<T1, T2> &x) {
	return os << "(" << x.first << "," << x.second << ")";
}
template<typename T, size_t N>
std::ostream &operator<<(std::ostream &os, const std::array<T, N> &x) {
	os << "[";
	std::copy(x.cbegin(), x.cend(), std::ostream_iterator<T>(os, ", "));
	os << "]";
	return os;
}
template<typename T>
std::ostream &operator<<(std::ostream &os, const std::list<T> &x) {
	os << "[";
	std::copy(x.cbegin(), x.cend(), std::ostream_iterator<T>(os, ", "));
	os << "]";
	return os;
}
template<typename T>
std::ostream &operator<<(std::ostream &os, const std::vector<T> &x) {
	os << "[";
	std::copy(x.cbegin(), x.cend(), std::ostream_iterator<T>(os, ", "));
	os << "]";
	return os;
}

template<typename It>
std::enable_if_t<std::is_same_v<It, typename std::list<typename std::iterator_traits<It>::value_type>::iterator> ||
                 std::is_same_v<It, typename std::list<typename std::iterator_traits<It>::value_type>::const_iterator>,
std::ostream &>
operator<<(std::ostream &os, const It &x) {
	return os << "&" << *x;
}


}
