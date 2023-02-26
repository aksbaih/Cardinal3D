
#include "../util/camera.h"
#include "../rays/samplers.h"
#include "debug.h"

Ray Camera::generate_ray(Vec2 screen_coord) const {

    // TODO (PathTracer): Task 1
    //
    // The input screen_coord is a normalized screen coordinate [0,1]^2
    //
    // You need to transform this 2D point into a 3D position on the sensor plane, which is
    // located one unit away from the pinhole in camera space (aka view space).
    //
    // You'll need to compute this position based on the vertial field of view
    // (vert_fov) of the camera, and the aspect ratio of the output image (aspect_ratio).
    //
    // Tip: compute the ray direction in view space and use
    // the camera space to world space transform (iview) to transform the ray back into world space.

    const float sensorHeight = 2.f * tanf(Radians(vert_fov) * 0.5f);
    const float sensorWidth = sensorHeight * aspect_ratio;
    const Vec2 sensorDims(sensorWidth, sensorHeight);
    const Vec2 sensorPoint(screen_coord * sensorDims - 0.5f * sensorDims);
    const Vec3 sensorPointCameraSpace(sensorPoint.x, sensorPoint.y, -1.f);
    /* Ray starts from pinhole (origin) in the direction of the sensor point. */
    Ray ray(Vec3(0.f, 0.f, 0.f), sensorPointCameraSpace);
    /* But we want it in the world space instead. */
    ray.transform(iview);
    return ray;
}
