#ifndef COLLISION_H
#define COLLISION_H

#define MAX_FACE_VERTICES 32

#include <glm/glm.hpp>
#include <map>
#include <vector>
#define GLM_ENABLE_EXPERIMENTAL
#include <algorithm>
#include <cmath>
#include <float.h>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <string.h>

#include "Utility.h"

class Collision {
public:
  static const float epsilon;

  struct Face;

  struct HalfEdge {
    glm::vec3* startVertex;
    HalfEdge*  next;
    HalfEdge*  twin;
    Face*      face;
  };

  struct Face {
    glm::vec3 normal;
    HalfEdge* startEdge;
  };

  struct Polyhedron {
    std::vector<glm::vec3> vertices;
    std::vector<HalfEdge>  edges;
    std::vector<Face>      faces;
    glm::vec3              centroid;
    glm::vec3              maxHalfWidth;
  };

  struct ContactPoint {
    float     separation;
    glm::vec3 position;
  };

  struct ContactManifold {
    glm::vec3                 normal;
    std::vector<ContactPoint> points;
  };

  enum class IsStatic {
    null,
    A,
    B
  };

private:
  struct TemporalCoherenceState {
    enum Status { null,
                  separated,
                  overlapping };
    enum SeparatingAxisType { face,
                              edge };
    enum OverlappingFeaturesType { faceFace,
                                   edgeEdge };
    struct SeparatingFaceAxis {
      Face*       face;
      Polyhedron* otherPolyhedron;
    };
    struct SeparatingEdgeAxis {
      HalfEdge* edgeA;
      HalfEdge* edgeB;
    };
    struct OverlappingFaces {
      Face* faceA;
      Face* faceB;
    };
    struct OverlappingEdges {
      HalfEdge* edgeA;
      HalfEdge* edgeB;
    };
    Status                  status;
    SeparatingAxisType      separatingAxisType;
    SeparatingFaceAxis      separatingFaceAxis;
    SeparatingEdgeAxis      separatingEdgeAxis;
    OverlappingFeaturesType overlappingFeaturesType;
    OverlappingFaces        overlappingFaces;
    OverlappingEdges        overlappingEdges;
  };

  static std::map<Polyhedron*, std::map<Polyhedron*, TemporalCoherenceState>> temporalCoherenceStateMap;

  static void setTemporalCoherenceStateSeparatingFaceAxis(TemporalCoherenceState* state, Face* face, Polyhedron* otherPolyhedron);
  static void setTemporalCoherenceStateSeparatingEdgeAxis(TemporalCoherenceState* state, HalfEdge* edgeA, HalfEdge* edgeB);
  static void setTemporalCoherenceStateOverlappingFaces(TemporalCoherenceState* state, Face* faceA, Face* faceB);
  static void setTemporalCoherenceStateOverlappingEdges(TemporalCoherenceState* state, HalfEdge* edgeA, HalfEdge* edgeB);

  struct FaceDirectionQuery {
    Face* face;
    float separation;
  };

  static FaceDirectionQuery getFaceDirectionOverlap(Polyhedron* A, Polyhedron* B);
  static FaceDirectionQuery getSingleFaceDirectionOverlap(Face* A, Polyhedron* B);
  static int                getSupportPoint(Polyhedron* polyhedron, glm::vec3 direction);
  static float              pointToPlaneDistance(glm::vec3 point, glm::vec3 planeNormal, glm::vec3 pointOnPlane);
  static bool               isSeparatingFaceAxis(Face* face, Polyhedron* other);

  struct EdgeDirectionQuery {
    glm::vec3 separationDirection; // unit vector to the direction in which polyhedron B has to be pushed to resolve overlap
    float     separation; // separation is negative if there is overlap
    HalfEdge* edgeA; // edge on Polyhedron A that can be used (with edge on polyhedron B) to find the contact point
    HalfEdge* edgeB; // edge on Polyhedron B that can be used (with edge on polyhedron A) to find the contact point
  };

  static EdgeDirectionQuery getEdgeDirectionOverlap(Polyhedron* A, Polyhedron* B);
  static bool               buildMinkowskiFace(HalfEdge* A, HalfEdge* B);
  static bool               isMinkowskiFace(glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 d);

  struct EdgeSeparationDistanceQuery {
    glm::vec3 separationDirection;
    float     separation;
  };

  static EdgeSeparationDistanceQuery getEdgeSeparationDistance(HalfEdge* edgeA, HalfEdge* edgeB, glm::vec3 centroidA);
  static bool                        isSeparatingEdgeAxis(HalfEdge* edgeA, HalfEdge* edgeB, glm::vec3* centroidA);
  static Face*                       getIncidentFace(Face* reference, Polyhedron* p);
  static int                         clipPolygon(glm::vec3* clippedPolygon, glm::vec3* vertices, int verticesSize, HalfEdge* referenceFaceEdge);
  static int                         clipPolygon2(glm::vec3* clippedPolygon, glm::vec3* vertices, int verticesSize, HalfEdge* referenceFaceEdge);
  static int                         getFaceVertices(glm::vec3* faceVertices, HalfEdge* startEdge);
  static float                       triangleArea(glm::vec3* p1, glm::vec3* p2, glm::vec3* q, glm::vec3* n);
  static void                        getFaceContactPoints(std::vector<ContactPoint>* points, Face* referenceFace, Face* incidentFace, bool referenceFaceIsInPlanePolyhedron);
  static ContactManifold             getFaceContactManifold(Polyhedron* A, Polyhedron* B, FaceDirectionQuery* queryA, FaceDirectionQuery* queryB);
  static ContactManifold             getEdgeContactManifold(Polyhedron* A, Polyhedron* B, EdgeDirectionQuery* query, IsStatic isStatic);
  static glm::vec3                   getNearestPointOnLineA(glm::vec3 startA, glm::vec3 endA, glm::vec3 startB, glm::vec3 endB);
  static glm::vec3                   getNearestPointOnLineB(glm::vec3 startA, glm::vec3 endA, glm::vec3 startB, glm::vec3 endB);
  static glm::vec3                   getNearestPointBetweenLines(glm::vec3 startA, glm::vec3 endA, glm::vec3 startB, glm::vec3 endB);

  struct PointToLineSegmentDistanceQuery {
    float     distance;
    glm::vec3 nearestPointOnLineSegment;
    glm::vec3 normal;
    bool      pointOverlapsLineSegment;
  };

  static PointToLineSegmentDistanceQuery pointToLineSegmentDistance(glm::vec3 point, glm::vec3 p1, glm::vec3 p2);
  static void                            reduceContactPoints(ContactPoint* reducedContactPoints, ContactPoint* contactPoints, int contactPointsSize, glm::vec3* faceNormal);

public:
  static glm::vec3       linePlaneIntersection(glm::vec3 lineStartPoint, glm::vec3 lineEndPoint, glm::vec3 planeNormal, glm::vec3 planeVertex);
  static bool            joinContactManifoldNormals(std::vector<ContactManifold>* contactManifolds, glm::vec3* joinedNormal);
  static bool            joinContactManifolds(std::vector<ContactManifold>* contactManifolds, ContactManifold* reducedManifold);
  static ContactManifold getPolyhedronContactManifold(Polyhedron* A, Polyhedron* B, IsStatic isStatic, bool useTemporalCoherence = false); // both polyhedrons are in world space
  static ContactManifold getSphereBoxContactManifold(glm::vec3 positionA, float radiusA, glm::vec3 halfWidthB, glm::mat4 modelMatrixB, glm::mat4 modelMatrixInverseB); // polyhedron B is in local space and is required to be an axis-aligned box
  static ContactManifold getSphereCylinderContactManifold(glm::vec3 positionA, float radiusA, float radiusB, float halfHeightB, glm::mat4 modelMatrixB, glm::mat4 modelMatrixInverseB); // polyhedron B is in local space and is required to be an axis-aligned upright cylinder
  static ContactManifold getSphereSphereContactManifold(glm::vec3 positionA, float radiusA, glm::vec3 positionB, float radiusB);
  static ContactManifold getSpherePlaneContactManifold(glm::vec3 positionA, float radiusA, Polyhedron* B);
  static void            constructHitbox(Polyhedron* p, glm::vec3 halfWidth);
  static void            constructCylinder(Polyhedron* p, float radius, float height, int edges);
  static void            constructPlane(Polyhedron* p, std::vector<glm::vec3>* vertices, glm::vec3* normal);
  static void            copyPolyhedron(Polyhedron* source, Polyhedron* target);
  static void            transformPolyhedron(Polyhedron* p, Polyhedron* pResult, glm::mat4 m);
  static glm::vec3       getSupportPointCoordinate(Polyhedron* polyhedron, glm::vec3 direction);
};

#endif
