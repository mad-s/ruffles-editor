#include "optimization/target_shape.h"

#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>

namespace ruffles::optimization {

typedef CGAL::Triangulation_data_structure_2<
		CGAL::Triangulation_vertex_base_with_info_2<int, K>,
		CGAL::Constrained_triangulation_face_base_2<K>
> TriangulationDataStructure;

typedef CGAL::Triangulation_2<K, TriangulationDataStructure> Triangulation;
typedef CGAL::Constrained_Delaunay_triangulation_2<K,TriangulationDataStructure> CDT;


using ruffles::K;
typedef CGAL::Direction_2<K> Direction;
typedef CGAL::Ray_2<K> Ray;
typedef CGAL::Line_2<K> Line;
typedef CGAL::Segment_2<K> Segment;

TargetShape::TargetShape() {}

TargetShape::TargetShape(Polygon target) : target(target) {
	CDT cdt;
	unsigned n = target.size();
	vector<CDT::Vertex_handle> vertices;

	V.resize(n, 3);
	for (unsigned i = 0; i < n; i++) {
		auto vh = cdt.insert(target.vertex(i));
		vh->info() = i;
		vertices.push_back(vh);

		V.row(i) <<
			CGAL::to_real(target.vertex(i).x()),
			CGAL::to_real(target.vertex(i).y()),
			0.;
	}
	for (unsigned i = 0; i < n; i++) {
		cdt.insert_constraint(vertices[i], vertices[(i+1)%n]);
	}

	vector<array<int, 3>> faces;
	for (auto it = cdt.finite_faces_begin(); it != cdt.finite_faces_end(); ++it) {
		Point centroid = CGAL::centroid(
			it->vertex(0)->point(),
			it->vertex(1)->point(),
			it->vertex(2)->point()
		);
		if (target.has_on_positive_side(centroid)) {
			faces.push_back({
				it->vertex(0)->info(),
				it->vertex(1)->info(),
				it->vertex(2)->info()});
		}
	}

	F.resize(faces.size(), 3);
	for (unsigned i = 0; i < faces.size(); i++) {
		F.row(i) <<
			faces[i][0],
			faces[i][1],
			faces[i][2];
	}

}

TargetShape::TargetShape(Eigen::MatrixXd &cut_shape, Vector3 origin, Vector3 u_dir, Vector3 v_dir) : origin(origin), u_dir(u_dir), v_dir(v_dir), V(cut_shape) {
	assert(cut_shape.cols() == 3);

	// TODO: set F?

	real ou = origin.dot(u_dir);
	real ov = origin.dot(v_dir);
	for (int i = 0; i < cut_shape.rows(); i++) {
		Vector3 x = cut_shape.row(i).transpose();
		real u = x.dot(u_dir) - ou;
		real v = x.dot(v_dir) - ov;
		target.push_back(Point(u,v));
	}
}

real TargetShape::energy(Ruffle &ruffle) {
	Polygon outline;
	for (auto outline_section : ruffle.outline_sections) {
		auto &segments = outline_section.section->mesh_segments;
		auto push_vertex = [&](simulation::SimulationMesh::Vertex &v) {
			Vector2 x = ruffle.simulation_mesh.get_vertex_position(v);
			outline.push_back(Point(x(0), x(1)));
		};
		if (outline_section.reversed) {
			for (auto it = segments.rbegin(); it != segments.rend(); ++it) {
				push_vertex(*(*it)->end);
			}
		} else {
			for (auto it = segments.begin(); it != segments.end(); ++it) {
				push_vertex(*(*it)->start);
			}
		}
	}

	real outline_area = CGAL::to_real(outline.area());
	// cache?
	real target_area =  CGAL::to_real(target.area());

	vector<PolygonWithHoles> intersection;
	CGAL::intersection(target, outline, std::back_inserter(intersection));
	real intersection_area = 0.;
	for (auto its : intersection) {
		intersection_area += CGAL::to_real(its.outer_boundary().area());
	}

	return k*(target_area-intersection_area) + lambda*(outline_area-intersection_area);
}

array<real,2> TargetShape::intersect_horizontal(real height) {
	Line line(0,1,-height);

	// appearently we have to implement every little thing ourselves :/
	real x_min = infinity;
	real x_max = -infinity;
	for (auto it = target.edges_begin(); it != target.edges_end(); ++it) {
		auto its = CGAL::intersection(line, *it);
		if (its) {
			if (Segment* seg = boost::get<Segment>(&*its)) {
				real x0 = CGAL::to_real(seg->vertex(0).x());
				real x1 = CGAL::to_real(seg->vertex(1).x());
				x_min = min(x_min, min(x0,x1));
				x_max = max(x_max, max(x0,x1));
			}
			if (Point* point = boost::get<Point>(&*its)) {
				x_min = min(x_min, CGAL::to_real(point->x()));
				x_max = max(x_max, CGAL::to_real(point->x()));
			}
		}
	}

	return {x_min, x_max};
}

real TargetShape::height() {
	real y_min = +infinity;
	real y_max = -infinity;
	for (auto it = target.vertices_begin(); it != target.vertices_end(); ++it) {
		real y = CGAL::to_real(it->y());
		y_min = min(y_min, y);
		y_max = max(y_max, y);
	}
	y_min = max(0., y_min);
	return y_max - y_min;
}

real TargetShape::avg_width() {
	return abs(CGAL::to_real(target.area())) / height();
}

real TargetShape::signed_distance(Vector2 pos) {
	real min_dist = infinity;
	Point point(pos.x(), pos.y());
	for (auto it = target.edges_begin(); it != target.edges_end(); ++it) {
		auto dist2 = CGAL::squared_distance(point, *it);
		real dist = sqrt(CGAL::to_real(dist2));

		min_dist = min(min_dist, dist);
	}
	if (target.has_on_unbounded_side(point)) {
		min_dist *= -1;
	}
	return min_dist;
}

real TargetShape::raycast(Vector2 ro, Vector2 rd) {
	real t = infinity;

	Point source(ro.x(), ro.y());
	Ray ray(source, Direction(rd.x(), rd.y()));
	for (auto it = target.edges_begin(); it != target.edges_end(); ++it) {
		auto its = CGAL::intersection(ray, *it);
		if (its) {
			if (Segment* seg = boost::get<Segment>(&*its)) {
				t = min(t, sqrt(CGAL::to_real(CGAL::squared_distance(source, seg->vertex(0)))));
				t = min(t, sqrt(CGAL::to_real(CGAL::squared_distance(source, seg->vertex(1)))));
			}
			if (Point* point = boost::get<Point>(&*its)) {
				t = min(t, sqrt(CGAL::to_real(CGAL::squared_distance(source, *point))));
			}
		}
	}
	return t;
}

}
