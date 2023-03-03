
#include "../lib/mathlib.h"
#include "debug.h"
#include <iostream>

bool BBox::contains(Vec3 point) const {
    if(point.x >= min.x && point.y >= min.y && point.z >= min.z && point.x <= max.x &&
       point.y <= max.y && point.z <= max.z)
        return true;
    return false;
}

bool BBox::hit(const Ray& ray, Vec2& times) const {
    return true;

    // TODO (PathTracer):
    // Implement ray - bounding box intersection test
    // If the ray intersected the bounding box within the range given by
    // [times.x,times.y], update times with the new intersection times.

    float tmin = std::numeric_limits<float>::max();

    if(ray.dir.x != 0.f) {
        float t1 = (min.x - ray.point.x) / ray.dir.x;
        float t2 = (max.x - ray.point.x) / ray.dir.x;

        if(!(t1 < times.x || t1 > times.y) && contains(ray.at(t1))) tmin = std::min(tmin, t1);
        if(!(t2 < times.x || t2 > times.y) && contains(ray.at(t2))) tmin = std::min(tmin, t2);
    }

    if(ray.dir.y != 0.f) {
        float t1 = (min.y - ray.point.y) / ray.dir.y;
        float t2 = (max.y - ray.point.y) / ray.dir.y;

        if(!(t1 < times.x || t1 > times.y) && contains(ray.at(t1))) tmin = std::min(tmin, t1);
        if(!(t2 < times.x || t2 > times.y) && contains(ray.at(t2))) tmin = std::min(tmin, t2);
    }

    if(ray.dir.z != 0.f) {
        float t1 = (min.z - ray.point.z) / ray.dir.z;
        float t2 = (max.z - ray.point.z) / ray.dir.z;

        if(!(t1 < times.x || t1 > times.y) && contains(ray.at(t1))) tmin = std::min(tmin, t1);
        if(!(t2 < times.x || t2 > times.y) && contains(ray.at(t2))) tmin = std::min(tmin, t2);
    }

    return (tmin >= times.x) && (tmin <= times.y) && contains(ray.at(tmin));
}
