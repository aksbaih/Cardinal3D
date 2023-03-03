
#include "../rays/bvh.h"
#include "debug.h"
#include <algorithm> // std::sort
#include <iostream>
#include <stack>

namespace PT {

template<typename Primitive>
Vec3 BVH<Primitive>::cost_fn(BBox parentBb, BBox bucketBbs[3][NUM_BUCKETS],
                             int primCounts[3][NUM_BUCKETS], size_t partition) const {
    BBox leftx, rightx, lefty, righty, leftz, rightz;
    size_t leftcntx = 0, rightcntx = 0, leftcnty = 0, rightcnty = 0, leftcntz = 0, rightcntz = 0;
    for(size_t i = 0; i <= partition; i++) {
        if(primCounts[0][i]) leftx.enclose(bucketBbs[0][i]);
        leftcntx += primCounts[0][i];
        lefty.enclose(bucketBbs[1][i]);
        leftcnty += primCounts[1][i];
        leftz.enclose(bucketBbs[2][i]);
        leftcntz += primCounts[2][i];
    }
    for(size_t i = partition + 1; i < NUM_BUCKETS; i++) {
        rightx.enclose(bucketBbs[0][i]);
        rightcntx += primCounts[0][i];
        righty.enclose(bucketBbs[1][i]);
        rightcnty += primCounts[1][i];
        rightz.enclose(bucketBbs[2][i]);
        rightcntz += primCounts[2][i];
    }

    Vec3 cost{1 + (8 * leftcntx * leftx.surface_area() / parentBb.surface_area()) +
                  (8 * rightcntx * rightx.surface_area() / parentBb.surface_area()),
              1 + (8 * leftcnty * lefty.surface_area() / parentBb.surface_area()) +
                  (8 * rightcnty * righty.surface_area() / parentBb.surface_area()),
              1 + (8 * leftcntz * leftz.surface_area() / parentBb.surface_area()) +
                  (8 * rightcntz * rightz.surface_area() / parentBb.surface_area())};

    return cost;
}

template<typename Primitive>
void BVH<Primitive>::bvh_recurse(size_t max_leaf_size, size_t parentAddr) {
    Node& parent = nodes[parentAddr];
    if(parent.size <= max_leaf_size) return;

    // Check along x,y,z axis
    size_t bestIndex;
    size_t bestAxis;
    float lowestCost = std::numeric_limits<float>::max();
    BBox parentBbox = parent.bbox;

    // Step 1: choose ideal partition
    int primCounts[3][NUM_BUCKETS];
    BBox bucketBbs[3][NUM_BUCKETS];
    for(size_t i = 0; i < 3; i++) {
        for(size_t j = 0; j < NUM_BUCKETS; j++) {
            primCounts[i][j] = 0;
            bucketBbs[i][j] = BBox{};
        }
    }
    Vec3 steps = (parentBbox.max - parentBbox.min) / NUM_BUCKETS;

    for(size_t i = parent.start; i < parent.start + parent.size; i++) {
        Primitive& prim = primitives[i];
        BBox primBb = prim.bbox();
        Vec3 bucket = (primBb.center() - parentBbox.min) / steps;
        primCounts[0][(size_t)bucket.x]++;
        primCounts[1][(size_t)bucket.y]++;
        primCounts[2][(size_t)bucket.z]++;
        bucketBbs[0][(size_t)bucket.x].enclose(primBb);
        bucketBbs[1][(size_t)bucket.y].enclose(primBb);
        bucketBbs[2][(size_t)bucket.z].enclose(primBb);
    }

    for(size_t partitionIndex = 0; partitionIndex < NUM_BUCKETS - 1; partitionIndex++) {
        Vec3 cost = cost_fn(parentBbox, bucketBbs, primCounts, partitionIndex);
        if(cost.x < lowestCost) {
            bestAxis = 0;
            bestIndex = partitionIndex;
            lowestCost = cost.x;
        }
        if(cost.y < lowestCost) {
            bestAxis = 1;
            bestIndex = partitionIndex;
            lowestCost = cost.y;
        }
        if(cost.x < lowestCost) {
            bestAxis = 2;
            bestIndex = partitionIndex;
            lowestCost = cost.z;
        }
    }

    // Step 2: sort based on best axis
    auto it1 = primitives.begin() + parent.start;
    auto it2 = it1 + parent.size;
    std::sort(it1, it2, [bestAxis](const Primitive& p1, const Primitive& p2) {
        BBox b1 = p1.bbox();
        BBox b2 = p2.bbox();
        float coord1 = bestAxis == 0   ? b1.center().x
                       : bestAxis == 1 ? b1.center().y
                                       : b1.center().z;
        float coord2 = bestAxis == 0   ? b2.center().x
                       : bestAxis == 1 ? b2.center().y
                                       : b2.center().z;
        return coord1 < coord2;
    });

    // Step 3: compute left and right bounding boxes
    BBox split_leftBox, split_rightBox;
    size_t span_leftBox = 0, span_rightBox = 0;
    size_t start_leftBox = parent.start, start_rightBox;

    for(size_t i = 0; i <= bestIndex; i++) {
        split_leftBox.enclose(bucketBbs[bestAxis][i]);
        span_leftBox += primCounts[bestAxis][i];
    }
    for(size_t i = bestIndex + 1; i < NUM_BUCKETS; i++) {
        split_rightBox.enclose(bucketBbs[bestAxis][i]);
        span_rightBox += primCounts[bestAxis][i];
    }
    start_rightBox = start_leftBox + span_leftBox;

    // create child nodes
    size_t node_addr_l = new_node();
    size_t node_addr_r = new_node();
    nodes[parentAddr].l = node_addr_l;
    nodes[parentAddr].r = node_addr_r;

    nodes[node_addr_l].bbox = split_leftBox;
    nodes[node_addr_l].start = start_leftBox;
    nodes[node_addr_l].size = span_leftBox;

    nodes[node_addr_r].bbox = split_rightBox;
    nodes[node_addr_r].start = start_rightBox;
    nodes[node_addr_r].size = span_rightBox;

    bvh_recurse(max_leaf_size, node_addr_l);
    bvh_recurse(max_leaf_size, node_addr_r);
}

// construct BVH hierarchy given a vector of prims
template<typename Primitive>
void BVH<Primitive>::build(std::vector<Primitive>&& prims, size_t max_leaf_size) {

    // NOTE (PathTracer):
    // This BVH is parameterized on the type of the primitive it contains. This allows
    // us to build a BVH over any type that defines a certain interface. Specifically,
    // we use this to both build a BVH over triangles within each Tri_Mesh, and over
    // a variety of Objects (which might be Tri_Meshes, Spheres, etc.) in Pathtracer.
    //
    // The Primitive interface must implement these two functions:
    //      BBox bbox() const;
    //      Trace hit(const Ray& ray) const;
    // Hence, you may call bbox() and hit() on any value of type Primitive.

    // Keep these two lines of code in your solution. They clear the list of nodes and
    // initialize member variable 'primitives' as a vector of the scene prims
    nodes.clear();
    primitives = std::move(prims);

    // TODO (PathTracer): Task 3
    // Modify the code ahead to construct a BVH from the given vector of primitives and maximum leaf
    // size configuration.
    //
    // Please use the SAH as described in class.  We recomment the binned build from lecture.
    // In general, here is a rough sketch:
    //
    //  For each axis X,Y,Z:
    //     Try possible splits along axis, evaluate SAH for each
    //  Take minimum cost across all axes.
    //  Partition primitives into a left and right child group
    //  Compute left and right child bboxes
    //  Make the left and right child nodes.
    //
    //
    // While a BVH is conceptually a tree structure, the BVH class uses a single vector (nodes)
    // to store all the nodes. Therefore, BVH nodes don't contain pointers to child nodes,
    // but rather the indices of the
    // child nodes in this array. Hence, to get the child of a node, you have to
    // look up the child index in this vector (e.g. nodes[node.l]). Similarly,
    // to create a new node, don't allocate one yourself - use BVH::new_node, which
    // returns the index of a newly added node.
    //
    // As an example of how to make nodes, the starter code below builds a BVH with a
    // root node that encloses all the primitives and its two descendants at Level 2.
    // For now, the split is hardcoded such that the first primitive is put in the left
    // child of the root, and all the other primitives are in the right child.
    // There are no further descendants.

    // edge case
    if(primitives.empty()) {
        return;
    }

    // compute bounding box for all primitives
    BBox bb;
    for(size_t i = 0; i < primitives.size(); ++i) {
        bb.enclose(primitives[i].bbox());
    }

    // set up root node (root BVH). Notice that it contains all primitives.
    root_idx = new_node();
    Node& node = nodes[root_idx];
    node.bbox = bb;
    node.start = 0;
    node.size = primitives.size();

    bvh_recurse(max_leaf_size, root_idx);
}

template<typename Primitive> Trace BVH<Primitive>::hit(const Ray& ray) const {
    // return hit(ray, root_idx);
    Trace ret;
    const Node& node = nodes[root_idx];
    for(size_t i = node.start; i < node.start + node.size; i++) {
        const Primitive& prim = primitives[i];
        Trace hit = prim.hit(ray);
        ret = Trace::min(ret, hit);
    }
    return ret;
}

template<typename Primitive> Trace BVH<Primitive>::hit(const Ray& ray, size_t nodeIdx) const {

    // TODO (PathTracer): Task 3
    // Implement ray - BVH intersection test. A ray intersects
    // with a BVH aggregate if and only if it intersects a primitive in
    // the BVH that is not an aggregate.

    const Node& node = nodes[nodeIdx];
    Trace ret;

    Vec2 times{};

    if(node.is_leaf()) {
        for(size_t i = node.start; i < node.start + node.size; i++) {
            const Primitive& prim = primitives[i];
            Trace hit = prim.hit(ray);
            ret = Trace::min(ret, hit);
        }
    } else {
        times = ray.dist_bounds;
        if(nodes[node.l].bbox.hit(ray, times)) ret = Trace::min(ret, hit(ray, node.l));
        times = ray.dist_bounds;
        if(nodes[node.r].bbox.hit(ray, times)) ret = Trace::min(ret, hit(ray, node.r));
    }
    return ret;
}

template<typename Primitive>
BVH<Primitive>::BVH(std::vector<Primitive>&& prims, size_t max_leaf_size) {
    build(std::move(prims), max_leaf_size);
}

template<typename Primitive> BVH<Primitive> BVH<Primitive>::copy() const {
    BVH<Primitive> ret;
    ret.nodes = nodes;
    ret.primitives = primitives;
    ret.root_idx = root_idx;
    return ret;
}

template<typename Primitive> bool BVH<Primitive>::Node::is_leaf() const {
    return l == r;
}

template<typename Primitive>
size_t BVH<Primitive>::new_node(BBox box, size_t start, size_t size, size_t l, size_t r) {
    Node n;
    n.bbox = box;
    n.start = start;
    n.size = size;
    n.l = l;
    n.r = r;
    nodes.push_back(n);
    return nodes.size() - 1;
}

template<typename Primitive> BBox BVH<Primitive>::bbox() const {
    return nodes[root_idx].bbox;
}

template<typename Primitive> std::vector<Primitive> BVH<Primitive>::destructure() {
    nodes.clear();
    return std::move(primitives);
}

template<typename Primitive> void BVH<Primitive>::clear() {
    nodes.clear();
    primitives.clear();
}

template<typename Primitive>
size_t BVH<Primitive>::visualize(GL::Lines& lines, GL::Lines& active, size_t level,
                                 const Mat4& trans) const {

    std::stack<std::pair<size_t, size_t>> tstack;
    tstack.push({root_idx, 0});
    size_t max_level = 0;

    if(nodes.empty()) return max_level;

    while(!tstack.empty()) {

        auto [idx, lvl] = tstack.top();
        max_level = std::max(max_level, lvl);
        const Node& node = nodes[idx];
        tstack.pop();

        Vec3 color = lvl == level ? Vec3(1.0f, 0.0f, 0.0f) : Vec3(1.0f);
        GL::Lines& add = lvl == level ? active : lines;

        BBox box = node.bbox;
        box.transform(trans);
        Vec3 min = box.min, max = box.max;

        auto edge = [&](Vec3 a, Vec3 b) { add.add(a, b, color); };

        edge(min, Vec3{max.x, min.y, min.z});
        edge(min, Vec3{min.x, max.y, min.z});
        edge(min, Vec3{min.x, min.y, max.z});
        edge(max, Vec3{min.x, max.y, max.z});
        edge(max, Vec3{max.x, min.y, max.z});
        edge(max, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{max.x, min.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, min.y, max.z});

        if(node.l && node.r) {
            tstack.push({node.l, lvl + 1});
            tstack.push({node.r, lvl + 1});
        } else {
            for(size_t i = node.start; i < node.start + node.size; i++) {
                size_t c = primitives[i].visualize(lines, active, level - lvl, trans);
                max_level = std::max(c, max_level);
            }
        }
    }
    return max_level;
}

} // namespace PT
