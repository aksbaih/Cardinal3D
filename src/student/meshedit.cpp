
#include "../geometry/halfedge.h"
#include "debug.h"
#include <iostream>
#include <queue>
#include <set>
#include <unordered_map>

/* Note on local operation return types:

    The local operations all return a std::optional<T> type. This is used so that your
    implementation can signify that it does not want to perform the operation for
    whatever reason (e.g. you don't want to allow the user to erase the last vertex).

    An optional can have two values: std::nullopt, or a value of the type it is
    parameterized on. In this way, it's similar to a pointer, but has two advantages:
    the value it holds need not be allocated elsewhere, and it provides an API that
    forces the user to check if it is null before using the value.

    In your implementaiton, if you have successfully performed the operation, you can
    simply return the required reference:

            ... collapse the edge ...
            return collapsed_vertex_ref;

    And if you wish to deny the operation, you can return the null optional:

            return std::nullopt;

    Note that the stubs below all reject their duties by returning the null optional.
*/

/*
    This method should replace the given vertex and all its neighboring
    edges and faces with a single face, returning the new face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_vertex(Halfedge_Mesh::VertexRef v) {
    /* Cannot erase vertices on boundary by definition. */
    if(v->on_boundary()) return std::nullopt;
    /* Loop over edges touching the vertex and erase them leaving the last one. */
    while(v->degree() > 1) erase_edge(v->halfedge()->edge());
    /* Finally, remove the last edge inside the final face. */
    v->halfedge()->next()->vertex()->halfedge() = v->halfedge()->next();
    v->halfedge()->twin()->prev()->next() = v->halfedge()->next();
    FaceRef finalFace = v->halfedge()->face();
    finalFace->halfedge() = v->halfedge()->next();
    /* Erase dangling elements. */
    erase(v->halfedge()->twin());
    erase(v->halfedge()->edge());
    erase(v->halfedge());
    erase(v);
    return finalFace;
}

/*
    This method should erase the given edge and return an iterator to the
    merged face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_edge(Halfedge_Mesh::EdgeRef e) {
    if(e->on_boundary()) /* Boundary edge case. */
        return std::nullopt;
    else if(e->halfedge()->vertex()->degree() == 1) /* Final edge for a vertex case. */
        return erase_vertex(e->halfedge()->vertex());
    else if(e->halfedge()->twin()->vertex()->degree() == 1)
        return erase_vertex(e->halfedge()->twin()->vertex());

    HalfedgeRef removedHalfedge = e->halfedge();
    HalfedgeRef removedHalfedgeTwin = removedHalfedge->twin();
    FaceRef finalFace = removedHalfedgeTwin->face();
    FaceRef removedFace = removedHalfedge->face();

    /* Fix dependent halfedge in finalFace. */
    HalfedgeRef updateHalfedge = removedHalfedgeTwin->next();
    while(updateHalfedge->next() != removedHalfedgeTwin) updateHalfedge = updateHalfedge->next();
    updateHalfedge->next() = removedHalfedge->next();

    /* Fix dependent halfedges in removedFace. */
    while(updateHalfedge->next() != removedHalfedge) {
        updateHalfedge = updateHalfedge->next();
        updateHalfedge->face() = finalFace;
    }
    updateHalfedge->next() = removedHalfedgeTwin->next();

    /* Fix dependent vertices. */
    removedHalfedge->next()->vertex()->halfedge() = removedHalfedge->next();
    removedHalfedgeTwin->next()->vertex()->halfedge() = removedHalfedgeTwin->next();

    /* Fix dependent face. */
    finalFace->halfedge() = removedHalfedgeTwin->next();

    /* Erase dangling elements. */
    erase(removedHalfedge);
    erase(removedHalfedgeTwin);
    erase(removedFace);
    erase(e);

    return finalFace;
}

/*
    Replaces a degenerate 2-gon by a single edge.
 */
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::erase_2gon(Halfedge_Mesh::FaceRef f) {
    /* Ignore non 2-gons including single edge 2-gons. */
    if(f->degree() != 2 || f->halfedge()->edge()->id() == f->halfedge()->next()->edge()->id())
        return std::nullopt;

    HalfedgeRef removedHalfedge = f->halfedge();
    HalfedgeRef removedHalfedgeTwin = removedHalfedge->twin();
    EdgeRef removedEdge = removedHalfedge->edge();
    FaceRef finalFace = removedHalfedgeTwin->face();
    HalfedgeRef finalHalfedge = removedHalfedge->next();

    /* Fix halfedges. */
    finalHalfedge->next() = removedHalfedgeTwin->next();
    removedHalfedgeTwin->prev()->next() = finalHalfedge;

    /* Fix vertices. */
    finalHalfedge->vertex()->halfedge() = finalHalfedge;
    finalHalfedge->twin()->vertex()->halfedge() = finalHalfedge->twin();

    /* Fix faces. */
    finalFace->halfedge() = finalHalfedge;
    finalHalfedge->face() = finalFace;

    /* Erase dangling elements. */
    erase(removedHalfedge);
    erase(removedHalfedgeTwin);
    erase(removedEdge);
    erase(f);

    return finalHalfedge->edge();
}

/*
    This method should collapse the given edge and return an iterator to
    the new vertex created by the collapse.
    Rejects a collapse that results in a 0-degree vertex or that results in a non manifold fan.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_edge(Halfedge_Mesh::EdgeRef e) {
    /* Reject collapsing a singular edge (to avoid 0-degree vertex). */
    if(e->halfedge()->vertex()->degree() == 1 && e->halfedge()->twin()->vertex()->degree() == 1)
        return std::nullopt;

    /* Reject collapsing int hourglass-like non-manifold fan by rejecting collapses with overlapping
     * neighbors not on the same collapsable face. */
    std::set<VertexRef> neighborVertices;
    HalfedgeRef currHalfedge = e->halfedge();
    do {
        neighborVertices.insert(currHalfedge->twin()->vertex());
        currHalfedge = currHalfedge->twin()->next();
    } while(currHalfedge != e->halfedge());
    currHalfedge = e->halfedge()->next();
    do {
        VertexRef currVertex = currHalfedge->twin()->vertex();
        /* Allow overlap on the common faces. */
        if(currVertex != e->halfedge()->next()->twin()->vertex() &&
           currVertex != e->halfedge()->twin()->next()->twin()->vertex() &&
           neighborVertices.find(currVertex) != neighborVertices.end())
            return std::nullopt;
        currHalfedge = currHalfedge->twin()->next();
    } while(currHalfedge != e->halfedge()->next());

    HalfedgeRef collapsedHalfedge = e->halfedge();
    FaceRef collapsedFace = collapsedHalfedge->face();
    HalfedgeRef collapsedHalfedgeTwin = collapsedHalfedge->twin();
    FaceRef collapsedFaceTwin = collapsedHalfedgeTwin->face();
    VertexRef removedVertex = collapsedHalfedgeTwin->vertex();
    VertexRef finalVertex = collapsedHalfedge->vertex();

    /* Update one halfedge per iter going clockwise around removedVertex. */
    HalfedgeRef updateHalfedge = collapsedHalfedge->next();
    while(updateHalfedge != collapsedHalfedgeTwin) {
        updateHalfedge->vertex() = finalVertex;
        finalVertex->halfedge() = updateHalfedge;
        updateHalfedge = updateHalfedge->twin()->next();
    }

    /* Fix dependent halfedges. */
    updateHalfedge = collapsedHalfedge;
    while(updateHalfedge->next() != collapsedHalfedge) updateHalfedge = updateHalfedge->next();
    updateHalfedge->next() = collapsedHalfedge->next();
    updateHalfedge = collapsedHalfedgeTwin;
    while(updateHalfedge->next() != collapsedHalfedgeTwin) updateHalfedge = updateHalfedge->next();
    updateHalfedge->next() = collapsedHalfedgeTwin->next();

    /* Fix dependent faces. */
    if(collapsedFace->halfedge() == collapsedHalfedge)
        collapsedFace->halfedge() = collapsedHalfedge->next();
    if(collapsedFaceTwin->halfedge() == collapsedHalfedgeTwin)
        collapsedFaceTwin->halfedge() = collapsedHalfedgeTwin->next();

    /* Remove any resulting 2-gons. */
    if(collapsedFace->degree() == 2) erase_2gon(collapsedFace);
    if(collapsedFaceTwin->degree() == 2) erase_2gon(collapsedFaceTwin);

    /* Calculate final vertex geometry. */
    finalVertex->pos = 0.5 * (finalVertex->pos + removedVertex->pos);

    /* Erase dangling elements. */
    erase(removedVertex);
    erase(collapsedHalfedge);
    erase(collapsedHalfedgeTwin);
    erase(e);

    return finalVertex;
}

/*
    This method should collapse the given face and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_face(Halfedge_Mesh::FaceRef f) {

    (void)f;
    return std::nullopt;
}

/*
    This method should flip the given edge and return an iterator to the
    flipped edge.
*/
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::flip_edge(Halfedge_Mesh::EdgeRef e) {
    if(e->on_boundary()) return std::nullopt;
    /* Jump each halfedge by one edge. */
    for(HalfedgeRef halfedge : {e->halfedge(), e->halfedge()->twin()}) {
        HalfedgeRef jumpHalfedge = halfedge->twin()->next();
        /* Detach. */
        halfedge->prev()->next() = jumpHalfedge;
        halfedge->vertex()->halfedge() = jumpHalfedge;
        /* Attache. */
        halfedge->vertex() = jumpHalfedge->twin()->vertex();
        jumpHalfedge->face() = halfedge->face();
        halfedge->twin()->next() = jumpHalfedge->next();
        jumpHalfedge->next() = halfedge;
        /* Fix face. */
        halfedge->face()->halfedge() = halfedge;
    }
    return e;
}

/*
    This method should split the given edge and return an iterator to the
    newly inserted vertex. The halfedge of this vertex should point along
    the edge that was split, rather than the new edges.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::split_edge(Halfedge_Mesh::EdgeRef e) {
    /* Operation only for triangle faces possibly on a boundary. */
    if((!e->halfedge()->is_boundary() && e->halfedge()->face()->degree() != 3) ||
       (!e->halfedge()->twin()->is_boundary() && e->halfedge()->twin()->face()->degree() != 3))
        return std::nullopt;
    /* Note: Any element starting with newXXX is forward from the new vertex. */
    HalfedgeRef originalHalfedge = e->halfedge();
    HalfedgeRef originalHalfedgeTwin = originalHalfedge->twin();
    /* Create a new vertex. */
    VertexRef newVertex = new_vertex();
    newVertex->pos =
        0.5 * (originalHalfedge->vertex()->pos + originalHalfedge->twin()->vertex()->pos);
    /* Create a new forward halfedge and wire it. */
    HalfedgeRef newHalfedge = new_halfedge();
    newHalfedge->vertex() = newVertex;
    newVertex->halfedge() = newHalfedge;
    newHalfedge->face() = originalHalfedge->face();
    newHalfedge->next() = originalHalfedge->next();
    originalHalfedge->next() = newHalfedge;
    /* Create a new backward halfedge and wire it. */
    HalfedgeRef newTwinHalfedge = new_halfedge();
    newTwinHalfedge->vertex() = newVertex;
    newTwinHalfedge->face() = originalHalfedgeTwin->face();
    newTwinHalfedge->next() = originalHalfedgeTwin->next();
    originalHalfedgeTwin->next() = newTwinHalfedge;
    /* Wire up new twins into new edges. */
    EdgeRef newEdge = new_edge();
    newEdge->halfedge() = newHalfedge;
    newHalfedge->twin() = originalHalfedgeTwin;
    originalHalfedgeTwin->twin() = newHalfedge;
    newHalfedge->edge() = newEdge;
    originalHalfedgeTwin->edge() = newEdge;
    originalHalfedge->twin() = newTwinHalfedge;
    newTwinHalfedge->twin() = originalHalfedge;
    newTwinHalfedge->edge() = e;
    /* At this point, we successfully added the new vertex into the mesh. */

    /* Now create the two new edges from the new vertex. */
    for(HalfedgeRef startingHalfedge : {newHalfedge, newTwinHalfedge}) {
        /* Don't split the boundary face. */
        if(startingHalfedge->is_boundary()) continue;
        /* Start by creating the twin. */
        HalfedgeRef newEdgeHalfedgeTwin = new_halfedge();
        newEdgeHalfedgeTwin->vertex() = newVertex;
        newEdgeHalfedgeTwin->next() = startingHalfedge->next()->next();
        startingHalfedge->prev()->next() = newEdgeHalfedgeTwin;
        newEdgeHalfedgeTwin->face() = newEdgeHalfedgeTwin->next()->face();
        newEdgeHalfedgeTwin->face()->halfedge() = newEdgeHalfedgeTwin;
        /* Now create the other halfedge, creating a new face. */
        HalfedgeRef newEdgeHalfedge = new_halfedge();
        FaceRef newEdgeFace = new_face();
        startingHalfedge->face() = newEdgeFace;
        startingHalfedge->next()->face() = newEdgeFace;
        newEdgeHalfedge->vertex() = startingHalfedge->next()->next()->vertex();
        newEdgeHalfedge->next() = startingHalfedge;
        startingHalfedge->next()->next() = newEdgeHalfedge;
        newEdgeHalfedge->face() = newEdgeHalfedge->next()->face();
        newEdgeHalfedge->face()->halfedge() = newEdgeHalfedge;
        /* Finally, twin up the halfedges into a new edge. */
        EdgeRef newEdgeSplit = new_edge();
        newEdgeHalfedge->twin() = newEdgeHalfedgeTwin;
        newEdgeHalfedgeTwin->twin() = newEdgeHalfedge;
        newEdgeSplit->halfedge() = newEdgeHalfedge;
        newEdgeHalfedge->edge() = newEdgeSplit;
        newEdgeHalfedgeTwin->edge() = newEdgeSplit;
    }

    return newVertex;
}

/* Note on the beveling process:

    Each of the bevel_vertex, bevel_edge, and bevel_face functions do not represent
    a full bevel operation. Instead, they should update the _connectivity_ of
    the mesh, _not_ the positions of newly created vertices. In fact, you should set
    the positions of new vertices to be exactly the same as wherever they "started from."

    When you click on a mesh element while in bevel mode, one of those three functions
    is called. But, because you may then adjust the distance/offset of the newly
    beveled face, we need another method of updating the positions of the new vertices.

    This is where bevel_vertex_positions, bevel_edge_positions, and
    bevel_face_positions come in: these functions are called repeatedly as you
    move your mouse, the position of which determins the normal and tangent offset
    parameters. These functions are also passed an array of the original vertex
    positions: for  bevel_vertex, it has one element, the original vertex position,
    for bevel_edge,  two for the two vertices, and for bevel_face, it has the original
    position of each vertex in halfedge order. You should use these positions, as well
    as the normal and tangent offset fields to assign positions to the new vertices.

    Finally, note that the normal and tangent offsets are not relative values - you
    should compute a particular new position from them, not a delta to apply.
*/

/*
    This method should replace the vertex v with a face, corresponding to
    a bevel operation. It should return the new face.  NOTE: This method is
    responsible for updating the *connectivity* of the mesh only---it does not
    need to update the vertex positions.  These positions will be updated in
    Halfedge_Mesh::bevel_vertex_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_vertex(Halfedge_Mesh::VertexRef v) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)v;
    return std::nullopt;
}

/*
    This method should replace the edge e with a face, corresponding to a
    bevel operation. It should return the new face. NOTE: This method is
    responsible for updating the *connectivity* of the mesh only---it does not
    need to update the vertex positions.  These positions will be updated in
    Halfedge_Mesh::bevel_edge_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_edge(Halfedge_Mesh::EdgeRef e) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)e;
    return std::nullopt;
}

/*
    This method should replace the face f with an additional, inset face
    (and ring of faces around it), corresponding to a bevel operation. It
    should return the new face.  NOTE: This method is responsible for updating
    the *connectivity* of the mesh only---it does not need to update the vertex
    positions. These positions will be updated in
    Halfedge_Mesh::bevel_face_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_face(Halfedge_Mesh::FaceRef f) {
    if(f->degree() < 3) return std::nullopt;
    /* This is the new face with ring of faces around it. */
    FaceRef mainNewFace = new_face();
    /* In each iteration, pick a halfedge in the original face, create a new face with 4 halfedges.
     * Chain them. Create one vertex in the top-left and connect it within the new face. Create a
     * twin for the top halfedge and combine them into an edge in the main new face. Further
     * processing is needed in the next loop. */
    HalfedgeRef halfedge = f->halfedge();
    std::queue<HalfedgeRef> revisitSideHalfedges;
    do {
        revisitSideHalfedges.push(halfedge);
        FaceRef face = new_face();
        face->halfedge() = halfedge;
        HalfedgeRef nextHalfedge = halfedge->next();
        /* Top left corner. */
        VertexRef vertex = new_vertex();
        vertex->pos = halfedge->vertex()->pos;
        /* Right, Top, Left. */
        HalfedgeRef halfedges[] = {new_halfedge(), new_halfedge(), new_halfedge()};
        /* Top. */
        HalfedgeRef topTwin = new_halfedge();
        EdgeRef topEdge = new_edge();
        /* Chain the halfedges. */
        halfedge->next() = halfedges[0];
        halfedge->face() = face;
        halfedges[0]->vertex() = halfedge->twin()->vertex();
        halfedges[0]->face() = face;
        halfedges[0]->next() = halfedges[1];
        halfedges[1]->face() = face;
        halfedges[1]->twin() = topTwin;
        topTwin->twin() = halfedges[1];
        topTwin->vertex() = vertex;
        vertex->halfedge() = topTwin;
        topTwin->face() = mainNewFace;
        mainNewFace->halfedge() = topTwin;
        topEdge->halfedge() = topTwin;
        topTwin->edge() = topEdge;
        halfedges[1]->edge() = topEdge;
        halfedges[1]->next() = halfedges[2];
        halfedges[2]->face() = face;
        halfedges[2]->vertex() = vertex;
        halfedges[2]->next() = halfedge;
        /* Move on to the next halfedge face. */
        halfedge = nextHalfedge;
    } while(halfedge != f->halfedge());

    /* Wrap around to the first halfedge at the end of the queue. */
    revisitSideHalfedges.push(f->halfedge());

    /* Now, do a second pass to combine halfedges on the sides into edges and to fill in the missing
     * vertices and to chain the new face halfedges. */
    while(!revisitSideHalfedges.empty()) {
        HalfedgeRef currentHalfedge = revisitSideHalfedges.front();
        revisitSideHalfedges.pop();
        if(revisitSideHalfedges.empty()) break;
        HalfedgeRef nextHalfedge = revisitSideHalfedges.front();
        /* Twin up Right halfedge and construct into an edge. */
        EdgeRef newEdge = new_edge();
        HalfedgeRef rightHalfedge = currentHalfedge->next();
        HalfedgeRef rightHalfedgeTwin = nextHalfedge->prev();
        rightHalfedge->twin() = rightHalfedgeTwin;
        rightHalfedgeTwin->twin() = rightHalfedge;
        rightHalfedge->edge() = newEdge;
        rightHalfedgeTwin->edge() = newEdge;
        newEdge->halfedge() = rightHalfedge;
        /* Fill up missing vertices. */
        rightHalfedge->vertex() = nextHalfedge->vertex();
        rightHalfedge->next()->vertex() = rightHalfedgeTwin->vertex();
        /* Chain the top face halfedges. */
        rightHalfedge->next()->twin()->next() = rightHalfedgeTwin->prev()->twin();
    }

    /* Erase the original face and return the new one. */
    erase(f);
    return mainNewFace;
}

/*
    Compute new vertex positions for the vertices of the beveled vertex.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the original vertex position and its associated outgoing edge
    to compute a new vertex position along the outgoing edge.
*/
void Halfedge_Mesh::bevel_vertex_positions(const std::vector<Vec3>& start_positions,
                                           Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Compute new vertex positions for the vertices of the beveled edge.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the orig array) to compute an offset vertex position.

    Note that there is a 1-to-1 correspondence between halfedges in
    newHalfedges and vertex positions
    in orig.  So, you can write loops of the form

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vector3D pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_edge_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Compute new vertex positions for the vertices of the beveled face.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the start_positions array) to compute an offset vertex
    position.

    Note that there is a 1-to-1 correspondence between halfedges in
    new_halfedges and vertex positions
    in orig. So, you can write loops of the form

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vec3 pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_face_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset,
                                         float normal_offset) {

    if(flip_orientation) normal_offset = -normal_offset;
    // if(tangent_offset < 0.01) tangent_offset = 0.01;
    tangent_offset = exp(tangent_offset);
    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    Vec3 centroid(0, 0, 0);
    for(size_t i = 0; i < start_positions.size(); i++) centroid += start_positions[i];
    centroid = centroid / start_positions.size();

    Vec3 normal(face->normal());
    normal.normalize();

    for(size_t i = 0; i < new_halfedges.size(); i++) {
        Vec3 pi = start_positions[i];
        new_halfedges[i]->vertex()->pos =
            (pi - centroid) * tangent_offset + centroid + normal * normal_offset;
    }
}

/*
    Splits all non-triangular faces into triangles.
*/
void Halfedge_Mesh::triangulate() {
    /* Ensure a face is a triangle per iteration possibly creating new polygon faces to revisit in a
     * future iteration. */
    for(FaceRef f = faces_begin(); f != faces_end(); f++) {
        if(f->degree() < 4) continue;
        HalfedgeRef faceHalfedge = f->halfedge();
        /* Create a new edge and face. */
        HalfedgeRef newHalfedge = new_halfedge();
        HalfedgeRef newHalfedgeTwin = new_halfedge();
        EdgeRef newEdge = new_edge();
        FaceRef newFace = new_face();
        /* Wire up the halfedges. */
        newHalfedge->vertex() = faceHalfedge->next()->twin()->vertex();
        newHalfedgeTwin->vertex() = faceHalfedge->vertex();
        newHalfedgeTwin->next() = faceHalfedge->next()->next();
        newHalfedge->next() = faceHalfedge;
        faceHalfedge->prev()->next() = newHalfedgeTwin;
        faceHalfedge->next()->next() = newHalfedge;
        /* Fix the faces. */
        newHalfedge->face() = f;
        f->halfedge() = newHalfedge;
        newFace->halfedge() = newHalfedgeTwin;
        HalfedgeRef updateHalfedge = newHalfedgeTwin;
        do {
            updateHalfedge->face() = newFace;
            updateHalfedge = updateHalfedge->next();
        } while(updateHalfedge != newHalfedgeTwin);
        /* Twin up the new edge. */
        newHalfedge->twin() = newHalfedgeTwin;
        newHalfedgeTwin->twin() = newHalfedge;
        newHalfedge->edge() = newEdge;
        newHalfedgeTwin->edge() = newEdge;
        newEdge->halfedge() = newHalfedge;
    }
}

/* Note on the quad subdivision process:

        Unlike the local mesh operations (like bevel or edge flip), we will perform
        subdivision by splitting *all* faces into quads "simultaneously."  Rather
        than operating directly on the halfedge data structure (which as you've
        seen is quite difficult to maintain!) we are going to do something a bit nicer:
           1. Create a raw list of vertex positions and faces (rather than a full-
              blown halfedge mesh).
           2. Build a new halfedge mesh from these lists, replacing the old one.
        Sometimes rebuilding a data structure from scratch is simpler (and even
        more efficient) than incrementally modifying the existing one.  These steps are
        detailed below.

  Step I: Compute the vertex positions for the subdivided mesh.
        Here we're going to do something a little bit strange: since we will
        have one vertex in the subdivided mesh for each vertex, edge, and face in
        the original mesh, we can nicely store the new vertex *positions* as
        attributes on vertices, edges, and faces of the original mesh. These positions
        can then be conveniently copied into the new, subdivided mesh.
        This is what you will implement in linear_subdivide_positions() and
        catmullclark_subdivide_positions().

  Steps II-IV are provided (see Halfedge_Mesh::subdivide()), but are still detailed
  here:

  Step II: Assign a unique index (starting at 0) to each vertex, edge, and
        face in the original mesh. These indices will be the indices of the
        vertices in the new (subdivided mesh).  They do not have to be assigned
        in any particular order, so long as no index is shared by more than one
        mesh element, and the total number of indices is equal to V+E+F, i.e.,
        the total number of vertices plus edges plus faces in the original mesh.
        Basically we just need a one-to-one mapping between original mesh elements
        and subdivided mesh vertices.

  Step III: Build a list of quads in the new (subdivided) mesh, as tuples of
        the element indices defined above. In other words, each new quad should be
        of the form (i,j,k,l), where i,j,k and l are four of the indices stored on
        our original mesh elements.  Note that it is essential to get the orientation
        right here: (i,j,k,l) is not the same as (l,k,j,i).  Indices of new faces
        should circulate in the same direction as old faces (think about the right-hand
        rule).

  Step IV: Pass the list of vertices and quads to a routine that clears
        the internal data for this halfedge mesh, and builds new halfedge data from
        scratch, using the two lists.
*/

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos.  The values of the positions are based on
    simple linear interpolation, e.g., the edge midpoints and face
    centroids.
*/
void Halfedge_Mesh::linear_subdivide_positions() {

    // For each vertex, assign Vertex::new_pos to
    // its original position, Vertex::pos.

    for(VertexRef v = vertices_begin(); v != vertices_end(); v++) v->new_pos = v->pos;

    // For each edge, assign the midpoint of the two original
    // positions to Edge::new_pos.

    for(EdgeRef e = edges_begin(); e != edges_end(); e++) e->new_pos = e->center();

    // For each face, assign the centroid (i.e., arithmetic mean)
    // of the original vertex positions to Face::new_pos. Note
    // that in general, NOT all faces will be triangles!

    for(FaceRef f = faces_begin(); f != faces_end(); f++) f->new_pos = f->center();
}

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos.  The values of the positions are based on
    the Catmull-Clark rules for subdivision.

    Note: this will only be called on meshes without boundary
*/
void Halfedge_Mesh::catmullclark_subdivide_positions() {

    // The implementation for this routine should be
    // a lot like Halfedge_Mesh:linear_subdivide_positions:(),
    // except that the calculation of the positions themsevles is
    // slightly more involved, using the Catmull-Clark subdivision
    // rules. (These rules are outlined in the Developer Manual.)

    // Faces

    for(FaceRef f = faces_begin(); f != faces_end(); f++) f->new_pos = f->center();

    // Edges

    for(EdgeRef e = edges_begin(); e != edges_end(); e++) {
        const Vec3 AF =
            (e->halfedge()->face()->new_pos + e->halfedge()->twin()->face()->new_pos) / 2.f;
        const Vec3 ME = e->center();
        e->new_pos = (AF + ME) / 2.f;
    }

    // Vertices

    for(VertexRef v = vertices_begin(); v != vertices_end(); v++) {
        /* Compute F and n. */
        int n = 0;
        Vec3 F(0.f, 0.f, 0.f);
        Vec3 R(0.f, 0.f, 0.f);
        HalfedgeCRef currHalfedge = v->halfedge();
        do {
            n++;
            R += currHalfedge->edge()->center();
            F += currHalfedge->face()->new_pos;
            currHalfedge = currHalfedge->twin()->next();
        } while(currHalfedge != v->halfedge());
        R /= (float)n;
        F /= (float)n;
        v->new_pos = (F + 2.f * R + (float)(n - 3) * v->pos) / (float)n;
    }
}

/*
        This routine should increase the number of triangles in the mesh
        using Loop subdivision. Note: this is will only be called on triangle meshes.
*/
void Halfedge_Mesh::loop_subdivide() {

    // Compute new positions for all the vertices in the input mesh, using
    // the Loop subdivision rule, and store them in Vertex::new_pos.
    // -> At this point, we also want to mark each vertex as being a vertex of the
    //    original mesh. Use Vertex::is_new for this.
    // -> Next, compute the updated vertex positions associated with edges, and
    //    store it in Edge::new_pos.
    // -> Next, we're going to split every edge in the mesh, in any order.  For
    //    future reference, we're also going to store some information about which
    //    subdivided edges come from splitting an edge in the original mesh, and
    //    which edges are new, by setting the flat Edge::is_new. Note that in this
    //    loop, we only want to iterate over edges of the original mesh.
    //    Otherwise, we'll end up splitting edges that we just split (and the
    //    loop will never end!)
    // -> Now flip any new edge that connects an old and new vertex.
    // -> Finally, copy the new vertex positions into final Vertex::pos.

    // Each vertex and edge of the original surface can be associated with a
    // vertex in the new (subdivided) surface.
    // Therefore, our strategy for computing the subdivided vertex locations is to
    // *first* compute the new positions
    // using the connectivity of the original (coarse) mesh; navigating this mesh
    // will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse.  We
    // will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.

    // Compute updated positions for all the vertices in the original mesh, using
    // the Loop subdivision rule.

    for(VertexRef v = vertices_begin(); v != vertices_end(); v++) {
        v->is_new = false;
        const float n = (float)v->degree();
        const float u = n == 3.f ? 3.f / 16.f : 3.f / 8.f / n;
        /* Start with own position contribution. */
        v->new_pos = (1.f - n * u) * v->pos;
        /* Add weighted sum of neighbors. */
        HalfedgeRef currHalfedge = v->halfedge();
        do {
            v->new_pos += u * currHalfedge->twin()->vertex()->pos;
            currHalfedge = currHalfedge->twin()->next();
        } while(currHalfedge != v->halfedge());
    }

    // Next, compute the updated vertex positions associated with edges.

    for(EdgeRef e = edges_begin(); e != edges_end(); e++) {
        e->is_new = false;
        e->new_pos = 3.f / 8.f * e->halfedge()->vertex()->pos +
                     3.f / 8.f * e->halfedge()->twin()->vertex()->pos +
                     1.f / 8.f * e->halfedge()->next()->twin()->vertex()->pos +
                     1.f / 8.f * e->halfedge()->twin()->next()->twin()->vertex()->pos;
    }

    // Next, we're going to split every edge in the mesh, in any order. For
    // future reference, we're also going to store some information about which
    // subdivided edges come from splitting an edge in the original mesh, and
    // which edges are new.
    // In this loop, we only want to iterate over edges of the original
    // mesh---otherwise, we'll end up splitting edges that we just split (and
    // the loop will never end!)

    for(EdgeRef e = edges_begin(); e != edges_end(); e++) {
        /* Original old edges are always between two old vertices. */
        if(e->halfedge()->vertex()->is_new || e->halfedge()->twin()->vertex()->is_new) continue;
        std::optional<VertexRef> newVertex = split_edge(e);
        if(!newVertex) continue; /* Failed to subdivide; no new edges. */
        (*newVertex)->new_pos = e->new_pos;
        (*newVertex)->halfedge()->edge()->is_new = false;
    }

    // Finally, flip any new edge that connects an old and new vertex.

    for(EdgeRef e = edges_begin(); e != edges_end(); e++) {
        if(!e->is_new) continue;
        if(e->halfedge()->vertex()->is_new != e->halfedge()->twin()->vertex()->is_new) flip_edge(e);
    }

    // Copy the updated vertex positions to the subdivided mesh.

    for(VertexRef v = vertices_begin(); v != vertices_end(); v++) v->pos = v->new_pos;
}

/*
    Isotropic remeshing. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if this is not a triangle mesh)
*/
bool Halfedge_Mesh::isotropic_remesh() {

    // Compute the mean edge length.
    // Repeat the four main steps for 5 or 6 iterations
    // -> Split edges much longer than the target length (being careful about
    //    how the loop is written!)
    // -> Collapse edges much shorter than the target length.  Here we need to
    //    be EXTRA careful about advancing the loop, because many edges may have
    //    been destroyed by a collapse (which ones?)
    // -> Now flip each edge if it improves vertex degree
    // -> Finally, apply some tangential smoothing to the vertex positions

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate are called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return false;
}

/* Helper type for quadric simplification */
struct Edge_Record {
    Edge_Record() {
    }
    Edge_Record(std::unordered_map<Halfedge_Mesh::VertexRef, Mat4>& vertex_quadrics,
                Halfedge_Mesh::EdgeRef e)
        : edge(e) {

        // Compute the combined quadric from the edge endpoints.
        // -> Build the 3x3 linear system whose solution minimizes the quadric error
        //    associated with these two endpoints.
        // -> Use this system to solve for the optimal position, and store it in
        //    Edge_Record::optimal.
        // -> Also store the cost associated with collapsing this edge in
        //    Edge_Record::cost.

        const Mat4 Q = vertex_quadrics[e->halfedge()->vertex()] +
                       vertex_quadrics[e->halfedge()->twin()->vertex()];
        const Mat4 system(
            Vec4(Q[0][0], Q[0][1], Q[0][2], 0.f), Vec4(Q[0][1], Q[1][1], Q[1][2], 0.f),
            Vec4(Q[0][2], Q[1][2], Q[2][2], 0.f), Vec4(Q[0][3], Q[1][3], Q[2][3], 1.f));
        /* Assuming this matrix is invertible. A better solution should check the determinant and
         * possibly resort to a heuristic point on the edge. */
        const Vec4 solution = system.inverse()[3];
        optimal = Vec3(solution[0], solution[1], solution[2]);
        cost = dot((Q * solution), solution);
    }
    Halfedge_Mesh::EdgeRef edge;
    Vec3 optimal;
    float cost;
};

/* Comparison operator for Edge_Records so std::set will properly order them */
bool operator<(const Edge_Record& r1, const Edge_Record& r2) {
    if(r1.cost != r2.cost) {
        return r1.cost < r2.cost;
    }
    Halfedge_Mesh::EdgeRef e1 = r1.edge;
    Halfedge_Mesh::EdgeRef e2 = r2.edge;
    return &*e1 < &*e2;
}

/** Helper type for quadric simplification
 *
 * A PQueue is a minimum-priority queue that
 * allows elements to be both inserted and removed from the
 * queue.  Together, one can easily change the priority of
 * an item by removing it, and re-inserting the same item
 * but with a different priority.  A priority queue, for
 * those who don't remember or haven't seen it before, is a
 * data structure that always keeps track of the item with
 * the smallest priority or "score," even as new elements
 * are inserted and removed.  Priority queues are often an
 * essential component of greedy algorithms, where one wants
 * to iteratively operate on the current "best" element.
 *
 * PQueue is templated on the type T of the object
 * being queued.  For this reason, T must define a comparison
 * operator of the form
 *
 *    bool operator<( const T& t1, const T& t2 )
 *
 * which returns true if and only if t1 is considered to have a
 * lower priority than t2.
 *
 * Basic use of a PQueue might look
 * something like this:
 *
 *    // initialize an empty queue
 *    PQueue<myItemType> queue;
 *
 *    // add some items (which we assume have been created
 *    // elsewhere, each of which has its priority stored as
 *    // some kind of internal member variable)
 *    queue.insert( item1 );
 *    queue.insert( item2 );
 *    queue.insert( item3 );
 *
 *    // get the highest priority item currently in the queue
 *    myItemType highestPriorityItem = queue.top();
 *
 *    // remove the highest priority item, automatically
 *    // promoting the next-highest priority item to the top
 *    queue.pop();
 *
 *    myItemType nextHighestPriorityItem = queue.top();
 *
 *    // Etc.
 *
 *    // We can also remove an item, making sure it is no
 *    // longer in the queue (note that this item may already
 *    // have been removed, if it was the 1st or 2nd-highest
 *    // priority item!)
 *    queue.remove( item2 );
 *
 */
template<class T> struct PQueue {
    void insert(const T& item) {
        queue.insert(item);
    }
    void remove(const T& item) {
        if(queue.find(item) != queue.end()) {
            queue.erase(item);
        }
    }
    const T& top(void) const {
        return *(queue.begin());
    }
    void pop(void) {
        queue.erase(queue.begin());
    }
    size_t size() {
        return queue.size();
    }
    bool empty() {
        return queue.empty();
    }

    std::set<T> queue;
};

/*
    Mesh simplification. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if you can't simplify the mesh any
    further without destroying it.)
*/
bool Halfedge_Mesh::simplify() {
    std::unordered_map<VertexRef, Mat4> vertex_quadrics;
    std::unordered_map<FaceRef, Mat4> face_quadrics;
    std::unordered_map<EdgeRef, Edge_Record> edge_records;
    PQueue<Edge_Record> edge_queue;

    // Compute initial quadrics for each face by simply writing the plane equation
    // for the face in homogeneous coordinates. These quadrics should be stored
    // in face_quadrics
    // -> Compute an initial quadric for each vertex as the sum of the quadrics
    //    associated with the incident faces, storing it in vertex_quadrics
    // -> Build a priority queue of edges according to their quadric error cost,
    //    i.e., by building an Edge_Record for each edge and sticking it in the
    //    queue. You may want to use the above PQueue<Edge_Record> for this.
    // -> Until we reach the target edge budget, collapse the best edge. Remember
    //    to remove from the queue any edge that touches the collapsing edge
    //    BEFORE it gets collapsed, and add back into the queue any edge touching
    //    the collapsed vertex AFTER it's been collapsed. Also remember to assign
    //    a quadric to the collapsed vertex, and to pop the collapsed edge off the
    //    top of the queue.

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate are called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    /* First, compute face quadrics. */
    for(FaceRef f = faces_begin(); f != faces_end(); f++) {
        /* Think of distance query as translation of plane then projection onto its normal. */
        const Mat4 T = Mat4::translate(-1.f * f->halfedge()->vertex()->pos);
        Vec4 N(f->normal(), 0.f);
        N.normalize();
        Mat4 NNT = outer(N, N); /* outer(N, N) = NN^T. */
        NNT[3][3] = 1.f;        /* Fix homogenous coord. */
        const Mat4 Q = T.T() * NNT * T;
        face_quadrics[f] = Q;
    }

    /* Second, compute vertex quadrics. */
    for(VertexRef v = vertices_begin(); v != vertices_end(); v++) {
        /* Quadric of a vertex is the sum of quadrics of faces around it. */
        Mat4 vertexQuadric = Mat4::Zero;
        HalfedgeRef currHalfedge = v->halfedge();
        do {
            vertexQuadric += face_quadrics[currHalfedge->face()];
            currHalfedge = currHalfedge->twin()->next();
        } while(currHalfedge != v->halfedge());
        vertex_quadrics[v] = vertexQuadric;
    }

    /* Now construct a PQueue of edges. */
    PQueue<Edge_Record> edgesPQueue;
    for(EdgeRef e = edges_begin(); e != edges_end(); e++) {
        edge_records[e] = Edge_Record(vertex_quadrics, e);
        edgesPQueue.insert(edge_records[e]);
        e->is_considered = false;
    }

    /* Collapse edges until faces reduce by a factor of 4. */
    Size targetSize = n_faces() / 4;
    if(targetSize < 2) return false;
    while(!edgesPQueue.empty() && n_faces() > targetSize) {
        EdgeRef edgeToCollapse = edgesPQueue.top().edge;
        edgesPQueue.pop();
        edgeToCollapse->is_considered = true;

        /* Remove immediate edge neighbors temporarily. */
        std::set<EdgeRef> neighbors;
        HalfedgeRef currHalfedge = edgeToCollapse->halfedge()->next();
        while(currHalfedge->edge() != edgeToCollapse) {
            neighbors.insert(currHalfedge->edge());
            currHalfedge = currHalfedge->twin()->next();
        }
        currHalfedge = edgeToCollapse->halfedge()->twin()->next();
        while(currHalfedge->edge() != edgeToCollapse) {
            neighbors.insert(currHalfedge->edge());
            currHalfedge = currHalfedge->twin()->next();
        }
        for(const EdgeRef& e : neighbors) edgesPQueue.remove(edge_records[e]);

        /* Collapse edge. */
        const Mat4 collapsedEdgeQuadric =
            vertex_quadrics[edgeToCollapse->halfedge()->vertex()] +
            vertex_quadrics[edgeToCollapse->halfedge()->twin()->vertex()];
        std::optional<VertexRef> collapsedVertex = collapse_edge_erase(edgeToCollapse);

        /* Re-Add neighboring edges if touching final vertex or if collapse failed. */
        if(collapsedVertex) {
            /* Add the new collapsedVertex to quadrics. */
            vertex_quadrics[*collapsedVertex] = collapsedEdgeQuadric;
            /* Add un-collapsed neighbors. */
            currHalfedge = (*collapsedVertex)->halfedge();
            do {
                if(neighbors.find(currHalfedge->edge()) != neighbors.end() &&
                   !currHalfedge->edge()->is_considered) {
                    edge_records[currHalfedge->edge()] =
                        Edge_Record(vertex_quadrics, currHalfedge->edge());
                    edgesPQueue.insert(edge_records[currHalfedge->edge()]);
                }
                currHalfedge = currHalfedge->twin()->next();
            } while(currHalfedge != (*collapsedVertex)->halfedge());
        } else {
            /* Add all removed neighbors if collapse fails. */
            for(const EdgeRef& e : neighbors) {
                if(e->is_considered) continue;
                edge_records[e] = Edge_Record(vertex_quadrics, e);
                edgesPQueue.insert(edge_records[e]);
            }
        }
    }

    return true;
}
