
#include "../rays/tri_mesh.h"
#include "debug.h"

namespace PT {

BBox Triangle::bbox() const {

    // TODO (PathTracer): Task 2
    // compute the bounding box of the triangle

    // Beware of flat/zero-volume boxes! You may need to
    // account for that here, or later on in BBox::intersect

    BBox box;
    return box;
}

Trace Triangle::hit(const Ray& ray) const {

    // Vertices of triangle - has postion and surface normal
    // See rays/tri_mesh.h for a description of this struct

    Tri_Mesh_Vert v_0 = vertex_list[v0];
    Tri_Mesh_Vert v_1 = vertex_list[v1];
    Tri_Mesh_Vert v_2 = vertex_list[v2];

    // here just to avoid unused variable warnings, students should remove the following three
    // lines.
    (void)v_0;
    (void)v_1;
    (void)v_2;

    // TODO (PathTracer): Task 2
    // Intersect this ray with a triangle defined by the above three points.
    // Intersection should yield a ray t-value, and a hit point (u,v) on the surface of the triangle

    // You'll need to fill in a "Trace" struct describing information about the hit (or lack of hit)

    /* Refer to https://stanford-cs248.github.io/Cardinal3D/pathtracer/ray_triangle_intersection for
     * documentation on the following formulae. */
    const Vec3 e1(v_1.position - v_0.position);
    const Vec3 e2(v_2.position - v_0.position);
    const Vec3 s(ray.point - v_0.position);
    const Vec3 sXe2(cross(s, e2));
    const Vec3 e1Xd(cross(e1, ray.dir));
    const float denomFactor = 1.f / dot(e1Xd, e2);
    const float u = -1.f * dot(sXe2, ray.dir) * denomFactor;
    const float v = dot(e1Xd, s) * denomFactor;
    const float t = -1.f * dot(sXe2, e1) * denomFactor;
    /* Barycentric coords of hit point. */
    const float bc1 = u;
    const float bc2 = v;
    const float bc0 = 1.f - bc1 - bc2;

    /* Cases of no hit. */
    if(isinf(denomFactor) || bc0 < 0.f || bc1 < 0.f || bc2 < 0.f || t < ray.dist_bounds.x ||
       t > ray.dist_bounds.y)
        return Trace{false};

    /* Otherwise, populate the hit appropriately. */
    return Trace{true, t, ray.point + ray.dir * t,
                 bc0 * v_0.normal + bc1 * v_1.normal + bc2 * v_2.normal, ray.point};
}

Triangle::Triangle(Tri_Mesh_Vert* verts, unsigned int v0, unsigned int v1, unsigned int v2)
    : vertex_list(verts), v0(v0), v1(v1), v2(v2) {
}

void Tri_Mesh::build(const GL::Mesh& mesh) {

    verts.clear();
    triangles.clear();

    for(const auto& v : mesh.verts()) {
        verts.push_back({v.pos, v.norm});
    }

    const auto& idxs = mesh.indices();

    std::vector<Triangle> tris;
    for(size_t i = 0; i < idxs.size(); i += 3) {
        tris.push_back(Triangle(verts.data(), idxs[i], idxs[i + 1], idxs[i + 2]));
    }

    triangles.build(std::move(tris), 4);
}

Tri_Mesh::Tri_Mesh(const GL::Mesh& mesh) {
    build(mesh);
}

Tri_Mesh Tri_Mesh::copy() const {
    Tri_Mesh ret;
    ret.verts = verts;
    ret.triangles = triangles.copy();
    return ret;
}

BBox Tri_Mesh::bbox() const {
    return triangles.bbox();
}

Trace Tri_Mesh::hit(const Ray& ray) const {
    Trace t = triangles.hit(ray);
    return t;
}

size_t Tri_Mesh::visualize(GL::Lines& lines, GL::Lines& active, size_t level,
                           const Mat4& trans) const {
    return triangles.visualize(lines, active, level, trans);
}

} // namespace PT
