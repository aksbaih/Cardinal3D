
#pragma once

#include "../lib/mathlib.h"

namespace PT {

struct Trace {

    Trace(bool hit_ = false, float distance_ = 0.f, Vec3 position_ = Vec3(), Vec3 normal_ = Vec3(),
          Vec3 origin_ = Vec3(), int material_ = 0)
        : hit(hit_), distance(distance_), position(position_), normal(normal_), origin(origin_),
          material(material_) {
    }

    bool hit = false;
    float distance = 0.0f;
    Vec3 position, normal, origin;
    int material = 0;

    static Trace min(const Trace& l, const Trace& r) {
        if(l.hit && r.hit) {
            if(l.distance < r.distance) return l;
            return r;
        }
        if(l.hit) return l;
        if(r.hit) return r;
        return {};
    }

    void transform(const Mat4& transform, const Mat4& norm) {
        position = transform * position;
        origin = transform * origin;
        normal = norm.rotate(normal).unit();
        distance = (position - origin).norm();
    }
};

} // namespace PT
