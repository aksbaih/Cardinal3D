
#include "../rays/shapes.h"
#include "debug.h"

namespace PT {

const char* Shape_Type_Names[(int)Shape_Type::count] = {"None", "Sphere"};

BBox Sphere::bbox() const {

    BBox box;
    box.enclose(Vec3(-radius));
    box.enclose(Vec3(radius));
    return box;
}

Trace Sphere::hit(const Ray& ray) const {

    // TODO (PathTracer): Task 2
    // Intersect this ray with a sphere of radius Sphere::radius centered at the origin.

    // If the ray intersects the sphere twice, ret should
    // represent the first intersection, but remember to respect
    // ray.dist_bounds! For example, if there are two intersections,
    // but only the _later_ one is within ray.dist_bounds, you should
    // return that one!

    /* This formula is derived from from writing the implicit formula for a sphere. Namely, the
     * intersection point is constrained to have norm r. Solutions found using the quadratic
     * formula. */

    /* Quadratic system setup. */
    const float a = ray.dir.norm_squared();
    const float b = 2.f * dot(ray.point, ray.dir);
    const float c = ray.point.norm_squared() - radius * radius;

    const float rootTerm = b * b - 4.f * a * c;
    const float smallerRoot = (-1.f * b - sqrtf(rootTerm)) / (2.f * a);
    const float biggerRoot = (-1.f * b + sqrtf(rootTerm)) / (2.f * a);

    /* Decide on intersections. */
    if(rootTerm >= -0.f && smallerRoot >= ray.dist_bounds.x && smallerRoot <= ray.dist_bounds.y)
        return Trace{true, smallerRoot, ray.point + smallerRoot * ray.dir,
                     (ray.point + smallerRoot * ray.dir).normalize(), ray.point};
    if(rootTerm >= -0.f && biggerRoot >= ray.dist_bounds.x && biggerRoot <= ray.dist_bounds.y)
        return Trace{true, biggerRoot, ray.point + biggerRoot * ray.dir,
                     (ray.point + biggerRoot * ray.dir).normalize(), ray.point};
    return Trace{false};
}

} // namespace PT
