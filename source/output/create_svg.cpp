#include "create_svg.h"

#include <sstream>

#include "common/clone_helper.h"


namespace ruffles {


constexpr real min_width = 4.; // 2cm
constexpr real max_connector_width = 2.;

using tinyxml2::XMLDocument;

unique_ptr<XMLDocument> create_svg(Ruffle &ruffle) {
    unique_ptr<XMLDocument> doc(new XMLDocument());
    auto oldroot = doc->NewElement("svg");
    oldroot->SetAttribute("xmlns", "http://www.w3.org/2000/svg");
    doc->InsertEndChild(oldroot);
    auto root = doc->NewElement("g");
    root->SetAttribute("fill", "none");
    root->SetAttribute("stroke", "red");
    root->SetAttribute("stroke-width", "0.001");
    oldroot->InsertEndChild(root);

    vector<Vector2> points_upper, points_lower;
    vector<Vector2> hole_points_upper, hole_points_lower;
    bool first = true;
    real l = 0.;
    real min_z = infinity;
    real max_z = -infinity;
    auto add_vertex = [&](auto vx) {
        real z0 = vx->z.front();
        real z1 = vx->z.back();
        if (z1 < z0 + min_width) {
            real d = min_width-(z1-z0);
            z0 -= 0.5*d;
            z1 += 0.5*d;
        }
        min_z = min(min_z, z0);
        max_z = max(max_z, z1);
        points_upper.emplace_back(l, z0);
        points_lower.emplace_back(l, z1);
    };

    std::unordered_map<listref<Ruffle::ConnectionPoint>,int,listref_hash<Ruffle::ConnectionPoint>> seen;
    int last = 0;
    auto add_connection = [&](auto cp) {
        if (cp->connecting_segments[0].size() + cp->connecting_segments[1].size() <= 2) {
            return;
        }
        auto z0 = cp->mesh_vertex->z.front();
        auto z1 = cp->mesh_vertex->z.back();
        real zm = 0.5*(z0+z1);
        real w = max(min_width, z1-z0);
        auto el = doc->NewElement("g");
        real connector_width = min(max_connector_width, 0.38 * w);
        string transform = "translate(" + to_string(l) + " " + to_string(zm) + ") scale(" + to_string(1./42.52 * connector_width) + ")";
        el->SetAttribute("transform", transform.c_str());
        
        bool which;
        int label;
        if (auto it = seen.find(cp); it != seen.end()) {
            label = it->second;
            which = false;
        } else {
            which = true;
            label = ++last;
            seen.emplace(cp, label);
        }

        auto connector = doc->NewElement(which ? "path" : "polyline");
        if (which) {
            connector->SetAttribute("d", "M-4.62,20.89h4.83v6.04c0,1.3,0.89,2.44,2.15,2.75s2.58-0.28,3.19-1.43L19.73,1.32 c0.22-0.41,0.33-0.87,0.33-1.32c0-0.45-0.11-0.91-0.33-1.32L5.56-28.25c-0.61-1.15-1.92-1.74-3.19-1.43 c-1.27,0.31-2.15,1.45-2.15,2.75v5.3h-4.83");
        } else {
            connector->SetAttribute("points", "-5.97,-27.62 0,-21.65 0,20.87 -5.97,26.84");
        }
        
        el->InsertEndChild(connector);
        auto text = doc->NewElement("text");
        text->SetAttribute("stroke", "#00ff00");
        text->SetAttribute("fill", "none");
        text->SetText(label);
        el->InsertEndChild(text);

        auto line = doc->NewElement("line");
        line->SetAttribute("y1", -100);
        line->SetAttribute("y2",  100);
        line->SetAttribute("stroke", "black");
        el->InsertEndChild(line);

        root->InsertEndChild(el);
    };
    for (auto section : ruffle.sections) {
        if (first) {
            add_connection(section.start);
        }
        for (auto segment : section.mesh_segments) {
            if (first) {
                first = false;
                add_vertex(segment->start);
            }
            //l += segment->length;
            l += (ruffle.simulation_mesh.get_vertex_position(*segment->end)
                - ruffle.simulation_mesh.get_vertex_position(*segment->start)).norm();
            add_vertex(segment->end);
        }
        add_connection(section.end);
    }


    vector<Vector2> points;

    for (auto it = points_upper.begin(); it != points_upper.end(); ++it) {
        points.push_back(*it);
    }
    points.push_back(points_upper.back() + Vector2(1.,0));
    points.push_back(points_lower.back() + Vector2(1.,0));
    for (auto it = points_lower.rbegin(); it != points_lower.rend(); ++it) {
        points.push_back(*it);
    }
    points.push_back(points_lower.front() + Vector2(-1.,0));
    points.push_back(points_upper.front() + Vector2(-1.,0));


    std::ostringstream oss;
    for (auto x : points) {
        oss << x.x() << "," << x.y() << " ";
    }
    auto points_text = oss.str();

    dbg(points_text);

    auto polygon = doc->NewElement("polygon");
    polygon->SetAttribute("points", points_text.c_str());
    root->InsertEndChild(polygon);
    real height = max_z - min_z;
    height += 0.2; // 1mm margin both sides
    l += 2.2; // 1cm connector + 1mm margin both sides
    auto width_attr = to_string(l) + "cm";
    auto height_attr = to_string(height) + "cm";
    auto viewbox_attr = "0 0 " + to_string(l) + " " + to_string(height);
    auto translate_attr = "translate(1.1 " + to_string(-min_z+0.1) + ")";
    root->SetAttribute("transform", translate_attr.c_str());
    oldroot->SetAttribute("width", width_attr.c_str());
    oldroot->SetAttribute("height", height_attr.c_str());
    oldroot->SetAttribute("viewBox", viewbox_attr.c_str());
    return doc;
}

}
