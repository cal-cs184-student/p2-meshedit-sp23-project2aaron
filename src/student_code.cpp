#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL {

    float lerp(float p0, float p1, float t) {
        return (1 - t) * p0 + t * p1;
    }
    /**
     * Evaluates one step of the de Casteljau's algorithm using the given points and
     * the scalar parameter t (class member).
     *
     * @param points A vector of points in 2D
     * @return A vector containing intermediate points or the final interpolated vector
     */
    std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points) {
        // TODO Part 1.
        std::vector<Vector2D> nextStep;
        for (int i = 0; i < points.size() - 1; i++) {
            Vector2D v1 = points[i];
            Vector2D v2 = points[i + 1];
            Vector2D interpolated;
            interpolated.x = lerp(v1.x, v2.x, t);
            interpolated.y = lerp(v1.y, v2.y, t);
            nextStep.push_back(interpolated);
        }
        return nextStep;
    }


    /**
     * Evaluates one step of the de Casteljau's algorithm using the given points and
     * the scalar parameter t (function parameter).
     *
     * @param points    A vector of points in 3D
     * @param t         Scalar interpolation parameter
     * @return A vector containing intermediate points or the final interpolated vector
     */
  std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
      std::vector<Vector3D> nextStep;
      for (int i = 0; i < points.size() - 1; i++) {
          Vector3D v1 = points[i];
          Vector3D v2 = points[i + 1];
          Vector3D interpolated;
          interpolated.x = lerp(v1.x, v2.x, t);
          interpolated.y = lerp(v1.y, v2.y, t);
          interpolated.z = lerp(v1.z, v2.z, t);
          nextStep.push_back(interpolated);
      }
      return nextStep;
  }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
    std::vector<Vector3D> tempPoints = points;
    for (int i = 0; i < points.size() - 1; i++)
    {
        tempPoints = evaluateStep(tempPoints, t);
    }
    return tempPoints[0];
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {  
    // TODO Part 2.
    std::vector<Vector3D> interpolatedPoints;
    for (int i = 0; i < controlPoints.size(); i++) {
        Vector3D point = evaluate1D(controlPoints[i], u);
        interpolatedPoints.push_back(point);
    }
    return evaluate1D(interpolatedPoints, v);
  }

  Vector3D faceNormal(Vector3D v1, Vector3D v2, Vector3D v3 ) {
      Vector3D dir1 = v1 - v2;
      Vector3D dir2 = v1 - v3;
      return cross(dir1, dir2);
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.
    Vector3D adjacentNormals;
    HalfedgeCIter h = this->halfedge();   // get the outgoing half-edge of the vertex
    do {
        HalfedgeCIter h_twin = h->twin(); // get the opposite half-edge
        VertexCIter v = h_twin->vertex(); // vertex is the 'source' of the half-edge, so
        // h->vertex() is v, whereas h_twin->vertex() is the neighboring vertex
        Vector3D normal = faceNormal(v->position, h->vertex()->position, h_twin->next()->next()->vertex()->position);
        adjacentNormals += normal * normal.norm();
        h = h_twin->next();               // move to the next outgoing half-edge of the vertex
    } while(h != this->halfedge());
    adjacentNormals.normalize();
    //cout << adjacentNormals;
    return adjacentNormals;
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
    HalfedgeIter h = e0->halfedge();
    VertexIter b = h->vertex();
    VertexIter c = h->next()->vertex();
    VertexIter a = h->next()->next()->vertex();
    VertexIter d = h->twin()->next()->next()->vertex();
    FaceIter f1 = h->face();
    FaceIter f2 = h->twin()->face();
    if (f1->isBoundary() || f2->isBoundary()) {
        return EdgeIter();
    }
    HalfedgeIter htwin = h->twin();
    HalfedgeIter ac = h->next();
    HalfedgeIter ab = ac->next();
    HalfedgeIter bd = htwin->next();
    HalfedgeIter cd = bd->next();

    cd->setNeighbors(ac, cd->twin(), d, cd->edge(), f1);
    ac->setNeighbors(h, ac->twin(), c, ac->edge(), f1);
    h->setNeighbors(cd, htwin, a, h->edge(), f1);
    htwin->setNeighbors(ab, h, d, htwin->edge(), f2);
    ab->setNeighbors(bd, ab->twin(), a, ab->edge(), f2);
    bd->setNeighbors(htwin, bd->twin(), b, bd->edge(), f2);

    f1->halfedge() = h;
    f2->halfedge() = htwin;
    c->halfedge() = ac;
    b->halfedge() = bd;
    d->halfedge() = htwin;
    a->halfedge() = h;
    /**
    HalfedgeMesh::check_for(f1);
    cout << "f1^ f2v";
    HalfedgeMesh::check_for(f2); */
    return h->edge();
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
      HalfedgeIter h = e0->halfedge();
      VertexIter b = h->vertex();
      VertexIter c = h->next()->vertex();
      VertexIter a = h->next()->next()->vertex();
      VertexIter d = h->twin()->next()->next()->vertex();
      FaceIter f1 = h->face();
      FaceIter f2 = h->twin()->face();
      if (f1->isBoundary() || f2->isBoundary()) {
          return VertexIter();
      }
      HalfedgeIter htwin = h->twin();
      HalfedgeIter ac = h->next();
      HalfedgeIter ab = ac->next();
      HalfedgeIter bd = htwin->next();
      HalfedgeIter cd = bd->next();

      //Create the new elements
      HalfedgeIter v = newHalfedge();
      HalfedgeIter vtwin = newHalfedge();
      HalfedgeIter v2 = newHalfedge();
      HalfedgeIter v2twin = newHalfedge();
      HalfedgeIter h2 = newHalfedge();
      HalfedgeIter h2twin = newHalfedge();

      VertexIter m = newVertex();
      //cout << (c->position + b->position) / 2;
      m->position = (c->position + b->position)/2;
      m->halfedge() = h;
      a->halfedge() = v;
      b->halfedge() = h2;
      c->halfedge() = htwin;
      d->halfedge() = v2twin;

      FaceIter f3 = newFace();
      FaceIter f4 = newFace();
      f1->halfedge() = v;
      f2->halfedge() = v2;
      f3->halfedge() = v2twin;
      f4->halfedge() = vtwin;

      EdgeIter vedge = newEdge();
      EdgeIter v2edge = newEdge();
      EdgeIter h2edge = newEdge();
      vedge->halfedge() = v;
      v2edge->halfedge() = v2;
      h2edge->halfedge() = h2;

      //Set halfedge neighbors
      h->setNeighbors(ac, htwin, m, h->edge(), f1);
      htwin->setNeighbors(v2, h, c, htwin->edge(), f2);
      v->setNeighbors(h, vtwin, a, vedge, f1);
      vtwin->setNeighbors(ab, v, m, vedge, f4);
      h2->setNeighbors(vtwin, h2twin, b, h2edge, f4);
      h2twin->setNeighbors(bd, h2, m, h2edge, f3);
      v2->setNeighbors(cd, v2twin, m, v2edge, f2);
      v2twin->setNeighbors(h2twin, v2, d, v2edge, f3);
      cd->setNeighbors(htwin, cd->twin(), d, cd->edge(), f2);
      ac->setNeighbors(v, ac->twin(), c, ac->edge(), f1);
      ab->setNeighbors(h2, ab->twin(), a, ab->edge(), f4);
      bd->setNeighbors(v2twin, bd->twin(), b, bd->edge(), f3);
    return m;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.

    // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // a vertex of the original mesh.
    
    // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
    
    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)
    
    // 4. Flip any new edge that connects an old and new vertex.

    // 5. Copy the new vertex positions into final Vertex::position.

      for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
          Vector3D totalsum = Vector3D(0, 0, 0);
          HalfedgeCIter h = v->halfedge();   // get the outgoing half-edge of the vertex
          float n = 0;
          do {
              HalfedgeCIter h_twin = h->twin(); // get the opposite half-edge
              VertexCIter o = h_twin->vertex(); // v is the outside vertex
              totalsum += o->position;
              h = h_twin->next();               // move to the next outgoing half-edge of the vertex
              n++;
          } while(h != v->halfedge());
          float u = 1;
          if (n == 3) {
              u = 3/16;
          } else {
              u = 3/(8*n);
          }
          v->newPosition = (1 - n * u) * v->position + u * totalsum;
          //cout << (v->newPosition - v->position);
          //cout << n;
          v->isNew = false;
      }
      int size = 0;
      for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
          HalfedgeIter h = e->halfedge();
          VertexIter a = h->vertex();
          VertexIter b = h->twin()->vertex();
          VertexIter d = h->next()->next()->vertex();
          VertexIter c = h->twin()->next()->next()->vertex();
          e->newPosition = 3/8 * (a->position + b->position) + 1/8 * (c->position + d->position);
          size++;
      }
      //set<EdgeIter> badlist;
      EdgeIter e = mesh.edgesBegin();
      for (int i = 0; i < size; i++) {
          /*
          if (std::find(badlist.begin(), badlist.end(), e) != badlist.end()) {
              e++;
              continue;
          } else { */
          VertexIter v = mesh.splitEdge(e);
          HalfedgeIter h = v->halfedge();
          h->edge()->isNew = false;
          h->twin()->next()->edge()->isNew = true;
          h->twin()->next()->twin()->next()->edge()->isNew = false;
          h->twin()->next()->twin()->next()->twin()->next()->edge()->isNew = true;
          /*
          badlist.insert(h->edge());
          badlist.insert(h->twin()->next()->edge());
          badlist.insert(h->twin()->next()->twin()->next()->edge());
          badlist.insert(h->twin()->next()->twin()->next()->twin()->next()->edge()); */
          v->isNew = true;
          e++;
          //}
      }
      for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
          HalfedgeIter h = e->halfedge();
          if ((h->vertex()->isNew && !h->twin()->vertex()->isNew) || (!h->vertex()->isNew && h->twin()->vertex()->isNew)) {
              if (h->edge()->isNew) {
                  mesh.flipEdge(e);
              }
          }
          e->isNew = false;
          e->newPosition = 0;
      }
      for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
          if (!v->isNew) {
              v->position = v->newPosition;
              v->newPosition = 0;
          }
          v->isNew = false;
      }
  }
}
