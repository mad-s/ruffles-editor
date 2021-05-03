#include "model/model_part.h"

#include "simulation/lbfgs.h"
#include "optimization/heuristic.h"
#include "simulation/combination.h"
#include "editor/utils/logger.h"

#include <igl/Hit.h>
#include <igl/ray_mesh_intersect.h>

namespace ruffles::model {

    using optimization::TargetShape;
    using simulation::LBFGS;
    using simulation::Combination;

    ModelPart::ModelPart(Mesh& segment) : _segment(segment)
    {
        _plane.align(_segment.V());
        _plane.translate_N=0.01;
        _plane.update_translation();

        _ground_plane.align(_segment.V());
        _ground_plane.rotate_X = 0.5*M_PI;
        _ground_plane.update_rotation();
        real t_min = infinity;
        for (int i = 0; i < segment.V().rows(); i++) {
            real t = segment.V().row(i).dot(_ground_plane.normal());
            t_min = min(t_min, t);
        }
        t_min -= _ground_plane.origin().dot(_ground_plane.normal());
        _ground_plane.translate_N = t_min+0.01;
        _ground_plane.update_translation();
        auto x = _plane.cut(_segment);
        cutline(x);
    }



    Mesh& ModelPart::segment()
    {
        return _segment;
    }

    void ModelPart::segment(Mesh& value)
    {
        _segment = value;
        //update: recut outline with reference plane (if exists)
    }

    Eigen::MatrixXd& ModelPart::cutline()
    {
        return target_shape.V;
    }

    void ModelPart::cutline(Eigen::MatrixXd& value)
    {
        assert(value.rows() > 0);
        Vector3 origin = value.row(0).transpose(); // temporary origin
        //Vector3 u_direction = ground_plane.normal().cross(_plane.normal());
        Vector3 u_direction = _ground_plane.normal().cross(_plane.normal());
        u_direction.normalize();
        Vector3 v_direction = _plane.normal().cross(u_direction);
        v_direction.normalize();
        



        real ground_v = (_ground_plane.origin().dot(_ground_plane.normal()) - origin.dot(_ground_plane.normal())) / (v_direction.dot(_ground_plane.normal()));
        vector<real> ground_u;

        Vector2 bl;
        real bl_val = infinity;

        Vector3 last_x = value.bottomRows<1>().transpose() - origin;
        for (int i = 0; i < value.rows(); i++) {
            Vector3 x = value.row(i).transpose() - origin;
            real u = u_direction.dot(x);
            real v = v_direction.dot(x);
            real last_u = u_direction.dot(last_x);
            real last_v = v_direction.dot(last_x);
            if (last_v < ground_v && v > ground_v || last_v > ground_v && v < ground_v) {
                real a = abs(last_v-ground_v);
                real b = abs(v-ground_v);
                real t = a/(a+b);
                real u_its = t*(u) + (1-t)*(last_u);
                ground_u.push_back(u_its);
            }
            real val = v + 0.1 * u;
            if (val < bl_val) {
                bl = Vector2(u,v);
                bl_val = val;
            }
            last_x = x;
        }
        std::sort(ground_u.begin(), ground_u.end());
        dbg_list(ground_u);
        if (!ground_u.empty()) {
            real u0 = (1.-u0_ratio)*ground_u.front() + u0_ratio*ground_u.back();
            dbg(u0);
            origin += u0*u_direction + ground_v*v_direction;
        } else {
            origin += bl(0) * u_direction + bl(1) * v_direction;
            // move origin up a bit
            origin += 0.5 * v_direction;
        }
        
        target_shape = TargetShape(
            value, origin, u_direction, v_direction
        );

        //update: re-init ruffle (?)
        real height = target_shape.height();
        real width = 0.7*target_shape.avg_width();
        if (std::isnan(width))
        {
            write_log(1) << "at ruffle creation: target_shape.avg_width() is " << width << linebreak;
            return;
        }

        step_width = 0.666*width;
        stack_count = max(1, round(height / (0.5*step_width)));
        step_height = height/stack_count;
        
        reinit_ruffle();

 
        //_ruffle.simulator.reset(new Combination(_ruffle.simulation_mesh));
        _ruffle.physics_solve();

        heuristic = optimization::Heuristic(target_shape);
    }

    vector<real> ModelPart::intersect_at(Vector2 uv) {
        Vector3 ro = target_shape.origin + uv(0)*target_shape.u_dir + uv(1)*target_shape.v_dir;
        Vector3 rd = target_shape.u_dir.cross(target_shape.v_dir);

        real offset = 1000.;

        if (apex) {
            rd = ro; // prev origin = point on curve
            ro = *apex;
            rd = rd - ro;
            rd.normalize();
            offset = 0.;
        }
        
        ro -= offset * rd; // we want hits even with t<0
        vector<igl::Hit> hits;

        igl::ray_mesh_intersect(ro, rd, _segment.V(), _segment.F(), hits);

        vector<real> res;
        for (auto &hit : hits) {
            res.push_back(hit.t - offset);
        }
        return res;
    }

    void ModelPart::intersect_ruffle() {
        auto &mesh = _ruffle.simulation_mesh;
        for (auto &v : mesh.vertices) {
            Vector2 uv = mesh.get_vertex_position(v);
            vector<real> hits = intersect_at(uv);
            constexpr real min_width = 1.0;
            // TODO: z0-z1 for vertices instead of just width
            if (hits.size() >= 2) {
                v.width = max(min_width, hits.back() - hits.front());
                v.z = hits;
            } else {
                assert(hits.size() == 0);
                //v.width = min_width; // min width
                // TODO: take z from previous vertices?
                v.z.clear();
            }
        }
        mesh.interpolate_missing_z();
    }

    TargetShape& ModelPart::target()
    {
        return target_shape;
    }

    Ruffle& ModelPart::ruffle() {
        return _ruffle;
    }
    
    void ModelPart::ruffle(Ruffle &&ruffle) {
        _ruffle = std::move(ruffle);
    }

    Plane& ModelPart::plane()
    {
        return _plane;
    }

    Plane& ModelPart::ground_plane()
    {
        return _ground_plane;
    }

    void ModelPart::add_child(ModelPart *child) {
        _children.push_back(child);
        update_extra_masses();
    }
    void ModelPart::clear_children() {
        _children.clear();
        update_extra_masses();
    }

    void ModelPart::update_extra_masses() {
        _ruffle.simulation_mesh.extra_mass.clear();
        for (auto c : _children) {
            auto &plane = c->ground_plane();
            auto &mesh = _ruffle.simulation_mesh;

            listref<simulation::SimulationMesh::Vertex> best;
            real best_value = -infinity;

            for (auto it = mesh.vertices.begin(); it != mesh.vertices.end(); ++it) {
                Vector2 uv = _ruffle.simulation_mesh.get_vertex_position(*it);
                Vector3 xyz = target().origin + uv(0)*target().u_dir + uv(1)*target().v_dir;
                real value = xyz.dot(plane.normal());
                if (value > best_value) {
                    best_value = value;
                    best = it;
                }
            }

            mesh.extra_mass.emplace_back(best, c->ruffle().simulation_mesh.total_mass());
        }
    }

    void ModelPart::update()
    {
        //TODO implement if needed
    }

    void ModelPart::clear()
    {
        //TODO implement if needed
        _segment.clear();
        _plane.clear();
        _ground_plane.clear();

        //TODO clear target_shape
        //TODO clear ruffle
    }

    void ModelPart::reinit_ruffle() {
        _ruffle = Ruffle::create_ruffle_stack(stack_count, step_height, step_width, h);
        // set ruffle gravity properly
        Vector3 gravity(0., -981., 0.); // TODO: set from where?
        _ruffle.simulation_mesh.gravity = Vector2(gravity.dot(target_shape.u_dir), gravity.dot(target_shape.v_dir));
        
        _ruffle.simulator.reset(new LBFGS(_ruffle.simulation_mesh));
    }
}
