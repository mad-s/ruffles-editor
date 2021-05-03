#pragma once

#include "common/common.h"

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Boolean_set_operations_2.h>

namespace ruffles {

typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef CGAL::Point_2<K> Point;
typedef CGAL::Polygon_2<K> Polygon;
typedef CGAL::Polygon_with_holes_2<K> PolygonWithHoles;

}

namespace CGAL {
	using ruffles::real;

	template<class NT>
	real to_real(const NT &x) {
		return to_double(x);
	}
}

