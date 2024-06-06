
#include "Collision.h"

const float                                                                                           Collision::epsilon = 0.00001f;
std::map<Collision::Polyhedron*, std::map<Collision::Polyhedron*, Collision::TemporalCoherenceState>> Collision::temporalCoherenceStateMap =
  std::map<Collision::Polyhedron*, std::map<Collision::Polyhedron*, Collision::TemporalCoherenceState>>();

void Collision::setTemporalCoherenceStateSeparatingFaceAxis(TemporalCoherenceState* state, Face* face, Polyhedron* otherPolyhedron) {
  state->status = TemporalCoherenceState::Status::separated;
  state->separatingAxisType = TemporalCoherenceState::SeparatingAxisType::face;
  state->separatingFaceAxis.otherPolyhedron = otherPolyhedron;
  state->separatingFaceAxis.face = face;
}

void Collision::setTemporalCoherenceStateSeparatingEdgeAxis(TemporalCoherenceState* state, HalfEdge* edgeA, HalfEdge* edgeB) {
  state->status = TemporalCoherenceState::Status::separated;
  state->separatingAxisType = TemporalCoherenceState::SeparatingAxisType::edge;
  state->separatingEdgeAxis.edgeA = edgeA;
  state->separatingEdgeAxis.edgeB = edgeB;
}

void Collision::setTemporalCoherenceStateOverlappingFaces(TemporalCoherenceState* state, Face* faceA, Face* faceB) {
  state->status = TemporalCoherenceState::Status::overlapping;
  state->overlappingFeaturesType = TemporalCoherenceState::OverlappingFeaturesType::faceFace;
  state->overlappingFaces.faceA = faceA;
  state->overlappingFaces.faceB = faceB;
}

void Collision::setTemporalCoherenceStateOverlappingEdges(TemporalCoherenceState* state, HalfEdge* edgeA, HalfEdge* edgeB) {
  state->status = TemporalCoherenceState::Status::overlapping;
  state->overlappingFeaturesType = TemporalCoherenceState::OverlappingFeaturesType::edgeEdge;
  state->overlappingEdges.edgeA = edgeA;
  state->overlappingEdges.edgeB = edgeB;
}

Collision::FaceDirectionQuery Collision::getFaceDirectionOverlap(Polyhedron* A, Polyhedron* B) {
  int   maxIndex = -1;
  float maxSeparation = -FLT_MAX;

  for(int i = 0; i < A->faces.size(); i++) {
    int   supportVertexIndex = getSupportPoint(B, -A->faces[i].normal);
    float separation = pointToPlaneDistance(B->vertices[supportVertexIndex], A->faces[i].normal, *(A->faces[i].startEdge->startVertex)); // any vertex of the face will do
    if(separation > 0.0f) { // early exit since at least one separating axis exists
      return FaceDirectionQuery{&(A->faces[i]), separation};
    }
    if(separation > maxSeparation) {
      maxIndex = i;
      maxSeparation = separation;
    }
  }
  return FaceDirectionQuery{&(A->faces[maxIndex]), maxSeparation};
}

Collision::FaceDirectionQuery Collision::getSingleFaceDirectionOverlap(Face* A, Polyhedron* B) {
  int   supportVertexIndex = getSupportPoint(B, -A->normal);
  float separation = pointToPlaneDistance(B->vertices[supportVertexIndex], A->normal, *(A->startEdge->startVertex)); // any vertex of the face will do
  return FaceDirectionQuery{A, separation};
}

int Collision::getSupportPoint(Polyhedron* polyhedron, glm::vec3 direction) {
  int   maxIndex = -1;
  float maxProjection = -FLT_MAX;
  for(int i = 0; i < polyhedron->vertices.size(); i++) {
    float projection = glm::dot(direction, polyhedron->vertices[i]);
    if(projection > maxProjection) {
      maxIndex = i;
      maxProjection = projection;
    }
  }
  return maxIndex;
}

float Collision::pointToPlaneDistance(glm::vec3 point, glm::vec3 planeNormal, glm::vec3 pointOnPlane) {
  return glm::dot(planeNormal, (point - pointOnPlane));
}

bool Collision::isSeparatingFaceAxis(Face* face, Polyhedron* other) {
  int   supportVertexIndex = getSupportPoint(other, -face->normal);
  float separation = pointToPlaneDistance(other->vertices[supportVertexIndex], face->normal, *(face->startEdge->startVertex));
  if(separation > 0.0f) {
    return true;
  }
  return false;
}

Collision::EdgeDirectionQuery Collision::getEdgeDirectionOverlap(Polyhedron* A, Polyhedron* B) {
  glm::vec3 maxSeparationDirection;
  float     maxSeparation = -FLT_MAX;
  int       maxIndexA = -1;
  int       maxIndexB = -1;

  // skip half of the half-edges to iterate unique edges because the twin is always stored after the edge
  for(int i = 0; i < A->edges.size(); i += 2) {
    for(int j = 0; j < B->edges.size(); j += 2) {
      if(buildMinkowskiFace(&(A->edges[i]), &(B->edges[j]))) {
        EdgeSeparationDistanceQuery edgeSeparationDistanceQuery = getEdgeSeparationDistance(&(A->edges[i]), &(B->edges[j]), A->centroid);
        if(edgeSeparationDistanceQuery.separation > 0.0f) { // early exit since at least one separating axis exists
          return EdgeDirectionQuery{glm::vec3(1.0f, 0.0f, 0.0f), edgeSeparationDistanceQuery.separation, &(A->edges[i]), &(B->edges[j])};
        }
        if(edgeSeparationDistanceQuery.separation > maxSeparation) {
          maxSeparationDirection = edgeSeparationDistanceQuery.separationDirection;
          maxSeparation = edgeSeparationDistanceQuery.separation;
          maxIndexA = i;
          maxIndexB = j;
        }
      }
    }
  }
  return EdgeDirectionQuery{maxSeparationDirection, maxSeparation, &(A->edges[maxIndexA]), &(B->edges[maxIndexB])};
}

bool Collision::buildMinkowskiFace(HalfEdge* A, HalfEdge* B) {
  // face normals define vertices of the two arcs in the Gauss map
  // negate normals C and D to account for Minkowski difference
  return isMinkowskiFace(A->face->normal,
                         A->twin->face->normal,
                         -B->face->normal,
                         -B->twin->face->normal);
}

bool Collision::isMinkowskiFace(glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 d) {
  // test if arcs AB and CD intersect on the unit sphere

  glm::vec3 BxA = glm::cross(b, a);
  glm::vec3 DxC = glm::cross(d, c);

  float CBA = glm::dot(c, BxA);
  float DBA = glm::dot(d, BxA);
  float ADC = glm::dot(a, DxC);
  float BDC = glm::dot(b, DxC);

  return ((CBA * DBA < 0.0f) && (ADC * BDC < 0.0f) && (CBA * BDC > 0.0f));
}

Collision::EdgeSeparationDistanceQuery Collision::getEdgeSeparationDistance(HalfEdge* edgeA, HalfEdge* edgeB, glm::vec3 centroidA) {
  glm::vec3 a = glm::normalize(*(edgeA->twin->startVertex) - *(edgeA->startVertex));
  glm::vec3 b = glm::normalize(*(edgeB->twin->startVertex) - *(edgeB->startVertex));
  glm::vec3 AxB = glm::cross(a, b);

  // skip near parallel edges
  const float kTolerance = epsilon;
  float       len = glm::length(AxB);
  if(len < kTolerance) {
    return EdgeSeparationDistanceQuery{glm::vec3(1.0f, 0.0f, 0.0f), -FLT_MAX};
  }

  // ensure consistent normal orientation (from A to B)
  glm::vec3 normal = AxB / len;
  if(glm::dot(normal, (*(edgeA->startVertex)) - centroidA) < 0.0f) {
    normal = -normal;
  }

  return EdgeSeparationDistanceQuery{normal, pointToPlaneDistance(*(edgeB->startVertex), normal, *(edgeA->startVertex))};
}

bool Collision::isSeparatingEdgeAxis(HalfEdge* edgeA, HalfEdge* edgeB, glm::vec3* centroidA) {
  EdgeSeparationDistanceQuery edgeSeparationDistanceQuery = getEdgeSeparationDistance(edgeA, edgeB, *centroidA);
  if(edgeSeparationDistanceQuery.separation > 0.0f) {
    return true;
  }
  return false;
}

Collision::Face* Collision::getIncidentFace(Face* reference, Polyhedron* p) {
  // incident face is the most anti-parallel face of the polyhedron to the reference face (minimising face)
  Face* minFace = &(p->faces[0]);
  float minDp = glm::dot(p->faces[0].normal, reference->normal);
  for(int i = 1; i < p->faces.size(); i++) {
    float dp = glm::dot(p->faces[i].normal, reference->normal);
    if(dp < minDp) {
      minFace = &(p->faces[i]);
      minDp = dp;
    }
  }
  return minFace;
}

int Collision::clipPolygon(glm::vec3* clippedPolygon, glm::vec3* vertices, int verticesSize, HalfEdge* referenceFaceEdge) {
  // optimised version using raw arrays
  // Sutherland–Hodgman algorithm
  memcpy(clippedPolygon, vertices, verticesSize * sizeof(glm::vec3));
  int       clippedPolygonSize = verticesSize;
  glm::vec3 inputPolygon[2 * MAX_FACE_VERTICES];
  HalfEdge* currentEdge = referenceFaceEdge;
  do {
    memcpy(inputPolygon, clippedPolygon, clippedPolygonSize * sizeof(glm::vec3));
    int inputPolygonSize = clippedPolygonSize;
    clippedPolygonSize = 0;
    glm::vec3 s = inputPolygon[inputPolygonSize - 1];
    glm::vec3 vertex = *(currentEdge->startVertex);
    glm::vec3 normal = currentEdge->twin->face->normal;
    for(int i = 0; i < inputPolygonSize; i++) {
      glm::vec3 e = inputPolygon[i];
      if(pointToPlaneDistance(e, normal, vertex) < 0.0f) {
        if(pointToPlaneDistance(s, normal, vertex) >= 0.0f) {
          clippedPolygon[clippedPolygonSize] = linePlaneIntersection(s, e, normal, vertex);
          clippedPolygonSize++;
        }
        clippedPolygon[clippedPolygonSize] = e;
        clippedPolygonSize++;
      } else if(pointToPlaneDistance(s, normal, vertex) < 0.0f) {
        clippedPolygon[clippedPolygonSize] = linePlaneIntersection(s, e, normal, vertex);
        clippedPolygonSize++;
      }
      s = e;
    }
    currentEdge = currentEdge->next;
  } while(currentEdge != referenceFaceEdge);
  return clippedPolygonSize;
}

int Collision::clipPolygon2(glm::vec3* clippedPolygon, glm::vec3* vertices, int verticesSize, HalfEdge* referenceFaceEdge) {
  // similar to clipPolygon but calculates the clipping planes correctly for polyhedrons with 2 faces (planes)
  memcpy(clippedPolygon, vertices, verticesSize * sizeof(glm::vec3));
  int       clippedPolygonSize = verticesSize;
  glm::vec3 inputPolygon[2 * MAX_FACE_VERTICES];
  HalfEdge* currentEdge = referenceFaceEdge;
  do {
    memcpy(inputPolygon, clippedPolygon, clippedPolygonSize * sizeof(glm::vec3));
    int inputPolygonSize = clippedPolygonSize;
    clippedPolygonSize = 0;
    glm::vec3 s = inputPolygon[inputPolygonSize - 1];
    glm::vec3 vertex = *(currentEdge->startVertex);
    glm::vec3 normal = glm::cross(*(currentEdge->twin->startVertex) - vertex, currentEdge->face->normal); // calculate the clipping face normal
    for(int i = 0; i < inputPolygonSize; i++) {
      glm::vec3 e = inputPolygon[i];
      if(pointToPlaneDistance(e, normal, vertex) < 0.0f) {
        if(pointToPlaneDistance(s, normal, vertex) >= 0.0f) {
          clippedPolygon[clippedPolygonSize] = linePlaneIntersection(s, e, normal, vertex);
          clippedPolygonSize++;
        }
        clippedPolygon[clippedPolygonSize] = e;
        clippedPolygonSize++;
      } else if(pointToPlaneDistance(s, normal, vertex) < 0.0f) {
        clippedPolygon[clippedPolygonSize] = linePlaneIntersection(s, e, normal, vertex);
        clippedPolygonSize++;
      }
      s = e;
    }
    currentEdge = currentEdge->next;
  } while(currentEdge != referenceFaceEdge);
  return clippedPolygonSize;
}

int Collision::getFaceVertices(glm::vec3* faceVertices, HalfEdge* startEdge) {
  HalfEdge* currentEdge = startEdge;
  int       faceVerticesSize = 0;
  do {
    faceVertices[faceVerticesSize] = *(currentEdge->startVertex);
    faceVerticesSize++;
    currentEdge = currentEdge->next;
  } while(currentEdge != startEdge);
  return faceVerticesSize;
}

float Collision::triangleArea(glm::vec3* p1, glm::vec3* p2, glm::vec3* q, glm::vec3* n) {
  return 0.5f * glm::dot(glm::cross(*p1 - *q, *p2 - *q), *n);
}

void Collision::getFaceContactPoints(std::vector<ContactPoint>* points, Face* referenceFace, Face* incidentFace, bool referenceFaceIsInPlanePolyhedron) {
  glm::vec3 incidentFaceVertices[MAX_FACE_VERTICES];
  int       incidentFaceVerticesSize = getFaceVertices(incidentFaceVertices, incidentFace->startEdge);
  glm::vec3 polygon[2 * MAX_FACE_VERTICES]; // 2 * max size of input vertices (worst case is that each vertex causes 2 vertices in clipped polygon)
  int       polygonSize;
  if(referenceFaceIsInPlanePolyhedron) {
    polygonSize = clipPolygon2(polygon, incidentFaceVertices, incidentFaceVerticesSize, referenceFace->startEdge);
  } else {
    polygonSize = clipPolygon(polygon, incidentFaceVertices, incidentFaceVerticesSize, referenceFace->startEdge);
  }
  // contact points are all the vertices of clipped incidentFace below reference face
  // move the contact points onto the reference face to improve stability
  if(polygonSize <= 4) {
    for(int i = 0; i < polygonSize; i++) {
      float d = pointToPlaneDistance(polygon[i], referenceFace->normal, *(referenceFace->startEdge->startVertex));
      if(d < 0.0f) {
        points->emplace_back(ContactPoint{d, polygon[i] + referenceFace->normal * -d});
      }
    }
  } else {
    // reduce contact manifold to maximum of 4 points
    ContactPoint contactPoints[2 * MAX_FACE_VERTICES];
    int          contactPointsSize = 0;
    for(int i = 0; i < polygonSize; i++) {
      float d = pointToPlaneDistance(polygon[i], referenceFace->normal, *(referenceFace->startEdge->startVertex));
      if(d < 0.0f) {
        contactPoints[contactPointsSize] = ContactPoint{d, polygon[i] + referenceFace->normal * -d};
        contactPointsSize++;
      }
    }
    if(contactPointsSize > 4) {
      ContactPoint reducedContactPoints[4];
      reduceContactPoints(reducedContactPoints, contactPoints, contactPointsSize, &(referenceFace->normal));
      for(int i = 0; i < 4; i++) {
        points->emplace_back(reducedContactPoints[i]);
      }
    } else {
      for(int i = 0; i < contactPointsSize; i++) {
        points->emplace_back(contactPoints[i]);
      }
    }
  }
}

Collision::ContactManifold Collision::getFaceContactManifold(Polyhedron* A, Polyhedron* B, FaceDirectionQuery* queryA, FaceDirectionQuery* queryB) {
  ContactManifold contactManifold;
  Face*           referenceFace;
  Polyhedron*     incidentPolyhedron;
  bool            referenceFaceIsInB;
  bool            referenceFaceIsInPlanePolyhedron = false;
  if(queryA->separation > queryB->separation - epsilon) { // add a bias to prefer A if both are equally minimising faces to avoid flip-flopping
    referenceFace = queryA->face;
    referenceFaceIsInB = false;
    incidentPolyhedron = B;
    if(A->faces.size() == 2) referenceFaceIsInPlanePolyhedron = true;
  } else {
    referenceFace = queryB->face;
    referenceFaceIsInB = true;
    incidentPolyhedron = A;
    if(B->faces.size() == 2) referenceFaceIsInPlanePolyhedron = true;
  }
  contactManifold.normal = referenceFace->normal;
  if(referenceFaceIsInB) { // normal always away from A
    contactManifold.normal = -contactManifold.normal;
  }
  Face* incidentFace = getIncidentFace(referenceFace, incidentPolyhedron);
  getFaceContactPoints(&contactManifold.points, referenceFace, incidentFace, referenceFaceIsInPlanePolyhedron);
  return contactManifold;
}

Collision::ContactManifold Collision::getEdgeContactManifold(Polyhedron* A, Polyhedron* B, EdgeDirectionQuery* query, IsStatic isStatic) {
  ContactManifold contactManifold;
  if(isStatic == IsStatic::A) {
    // returning point on line B is more consistent with face clipping when B is movable and A is static level geometry
    contactManifold.points.emplace_back(ContactPoint{query->separation, getNearestPointOnLineB(*(query->edgeA->startVertex), *(query->edgeA->twin->startVertex), *(query->edgeB->startVertex), *(query->edgeB->twin->startVertex))});
  } else if(isStatic == IsStatic::B) {
    // returning point on line A is more consistent with face clipping when A is movable and B is static level geometry
    contactManifold.points.emplace_back(ContactPoint{query->separation, getNearestPointOnLineA(*(query->edgeA->startVertex), *(query->edgeA->twin->startVertex), *(query->edgeB->startVertex), *(query->edgeB->twin->startVertex))});
  } else {
    // return the middle point between points on lines, no matter which is A and B
    contactManifold.points.emplace_back(ContactPoint{query->separation, getNearestPointBetweenLines(*(query->edgeA->startVertex), *(query->edgeA->twin->startVertex), *(query->edgeB->startVertex), *(query->edgeB->twin->startVertex))});
  }
  contactManifold.normal = query->separationDirection;
  return contactManifold;
}

glm::vec3 Collision::getNearestPointOnLineA(glm::vec3 startA, glm::vec3 endA, glm::vec3 startB, glm::vec3 endB) {
  glm::vec3 dA = endA - startA;
  glm::vec3 dB = endB - startB;
  glm::vec3 dAxdB = glm::cross(dA, dB);
  if(glm::length2(dAxdB) < epsilon) { // check if the edges are (near) parallel to avoid division by zero in case that dA x dB is a zero vector
    return startA + (0.5f * dA);
  }
  glm::vec3 n2 = glm::cross(dB, dAxdB);
  return startA + (glm::dot(startB - startA, n2) / glm::dot(dA, n2) * dA);
}

glm::vec3 Collision::getNearestPointOnLineB(glm::vec3 startA, glm::vec3 endA, glm::vec3 startB, glm::vec3 endB) {
  return getNearestPointOnLineA(startB, endB, startA, endA);
}

glm::vec3 Collision::getNearestPointBetweenLines(glm::vec3 startA, glm::vec3 endA, glm::vec3 startB, glm::vec3 endB) {
  // get the nearest points to each line on both lines
  glm::vec3 dA = endA - startA;
  glm::vec3 dB = endB - startB;
  glm::vec3 dAxdB = glm::cross(dA, dB);
  if(glm::length2(dAxdB) < epsilon) { // check if the edges are (near) parallel to avoid division by zero in case that dA x dB is a zero vector
    return 0.5f * (startA + (0.5f * dA) + startB + (0.5f * dB));
  }
  glm::vec3 n2 = glm::cross(dB, dAxdB);
  glm::vec3 pointOnA = startA + (glm::dot(startB - startA, n2) / glm::dot(dA, n2) * dA);
  glm::vec3 n1 = glm::cross(dA, glm::cross(dB, dA));
  glm::vec3 pointOnB = startB + (glm::dot(startA - startB, n1) / glm::dot(dB, n1) * dB);
  return 0.5f * (pointOnA + pointOnB);
}

Collision::PointToLineSegmentDistanceQuery Collision::pointToLineSegmentDistance(glm::vec3 point, glm::vec3 p1, glm::vec3 p2) {
  // assume p1 != p2
  glm::vec3 line = p2 - p1;
  bool      sign1 = (pointToPlaneDistance(point, line, p1) < 0);
  bool      sign2 = (pointToPlaneDistance(point, line, p2) < 0);
  bool      pointBetweenEndPoints = (sign1 != sign2);
  glm::vec3 p1ToPoint = point - p1;
  glm::vec3 p2ToPoint = point - p2;
  glm::vec3 lineNormal = -glm::normalize(glm::cross(line, glm::cross(line, p1ToPoint)));
  if(Utility::isNan(lineNormal)) { // point overlaps the line
    if(pointBetweenEndPoints) {
      return PointToLineSegmentDistanceQuery{0.0f, point, glm::vec3(0.0f), true};
    }
    float dp1 = glm::length(p1ToPoint);
    float dp2 = glm::length(p2ToPoint);
    if(dp1 < dp2) {
      return PointToLineSegmentDistanceQuery{dp1, p1, p1ToPoint / dp1, false};
    }
    return PointToLineSegmentDistanceQuery{dp2, p2, p2ToPoint / dp2, false};
  }
  float distanceToLine = pointToPlaneDistance(point, lineNormal, p1);
  if(pointBetweenEndPoints) {
    // distanceToLine is always positive because lineNormal direction depends on which side of the line the point is
    return PointToLineSegmentDistanceQuery{distanceToLine, point + (-lineNormal * distanceToLine), lineNormal, false};
  }
  float dp1 = glm::length(p1ToPoint);
  float dp2 = glm::length(p2ToPoint);
  if(dp1 < dp2) {
    return PointToLineSegmentDistanceQuery{dp1, p1, p1ToPoint / dp1, false};
  }
  return PointToLineSegmentDistanceQuery{dp2, p2, p2ToPoint / dp2, false};
}

void Collision::reduceContactPoints(ContactPoint* reducedContactPoints, ContactPoint* contactPoints, int contactPointsSize, glm::vec3* faceNormal) {
  float     maxProjection = -FLT_MAX;
  int       indexA = 0;
  glm::vec3 direction = Utility::getOneOrthogonalVector(*faceNormal);
  for(int i = 1; i < contactPointsSize; i++) {
    float projection = glm::dot(direction, contactPoints[i].position);
    if(projection > maxProjection) {
      maxProjection = projection;
      indexA = i;
    }
  }
  reducedContactPoints[0] = contactPoints[indexA];
  float maxDistance = 0.0f;
  int   indexB = 0;
  for(int i = 0; i < contactPointsSize; i++) {
    float distance = glm::length2(contactPoints[i].position - contactPoints[indexA].position);
    if(distance > maxDistance) {
      maxDistance = distance;
      indexB = i;
    }
  }
  reducedContactPoints[1] = contactPoints[indexB];
  float maxArea = 0.0f;
  int   indexC = 0;
  for(int i = 0; i < contactPointsSize; i++) {
    float area = triangleArea(&contactPoints[indexA].position, &contactPoints[indexB].position, &contactPoints[i].position, faceNormal);
    if(area > maxArea) {
      maxArea = area;
      indexC = i;
    }
  }
  reducedContactPoints[2] = contactPoints[indexC];
  float maxNegativeArea = 0.0f;
  int   indexD = 0;
  for(int i = 0; i < contactPointsSize; i++) { // edge A-B
    float area = triangleArea(&contactPoints[indexA].position, &contactPoints[indexB].position, &contactPoints[i].position, faceNormal);
    if(area < maxNegativeArea) {
      maxNegativeArea = area;
      indexD = i;
    }
  }
  for(int i = 0; i < contactPointsSize; i++) { // edge B-C
    float area = triangleArea(&contactPoints[indexB].position, &contactPoints[indexC].position, &contactPoints[i].position, faceNormal);
    if(area < maxNegativeArea) {
      maxNegativeArea = area;
      indexD = i;
    }
  }
  for(int i = 0; i < contactPointsSize; i++) { // edge B-C
    float area = triangleArea(&contactPoints[indexC].position, &contactPoints[indexA].position, &contactPoints[i].position, faceNormal);
    if(area < maxNegativeArea) {
      maxNegativeArea = area;
      indexD = i;
    }
  }
  reducedContactPoints[3] = contactPoints[indexD];
}

glm::vec3 Collision::linePlaneIntersection(glm::vec3 lineStartPoint, glm::vec3 lineEndPoint, glm::vec3 planeNormal, glm::vec3 planeVertex) {
  glm::vec3 lineDirectionVector = lineEndPoint - lineStartPoint;
  float     denominator = glm::dot(lineDirectionVector, planeNormal);
  if(std::abs(denominator) < epsilon) {
    // plane and line are nearly parallel, just take the average of endpoints (intersection is assumed when calling this function)
    return lineStartPoint + (0.5f * (lineEndPoint - lineStartPoint));
  }
  float d = glm::dot(planeVertex - lineStartPoint, planeNormal) / denominator;
  return d * lineDirectionVector + lineStartPoint;
}

bool Collision::joinContactManifoldNormals(std::vector<ContactManifold>* contactManifolds, glm::vec3* joinedNormal) {
  glm::vec3 normal(0.0f, 0.0f, 0.0f);
  for(int i = 0; i < contactManifolds->size(); i++) {
    normal += (*contactManifolds)[i].normal;
  }
  normal = glm::normalize(normal);
  if(Utility::isNan(normal)) {
    return false;
  }
  *joinedNormal = normal;
  return true;
}

bool Collision::joinContactManifolds(std::vector<ContactManifold>* contactManifolds, ContactManifold* reducedManifold) {
  glm::vec3 joinedNormal;
  if(!joinContactManifoldNormals(contactManifolds, &joinedNormal)) {
    return false;
  }
  std::vector<ContactPoint> points;
  for(int i = 0; i < contactManifolds->size(); i++) {
    for(int j = 0; j < (*contactManifolds)[i].points.size(); j++) {
      points.emplace_back((*contactManifolds)[i].points[j]);
      // calculate new contact point depth from surface (assume the surface is continuous and orthogonal to the old normal)
      if(std::abs(glm::dot(joinedNormal, (*contactManifolds)[i].normal)) > epsilon &&
         glm::length2(glm::cross(joinedNormal, (*contactManifolds)[i].normal)) > epsilon) { // skip if old and new normals are nearly orthogonal or parallel
        glm::vec3 separationLineStart = points.back().position + (*contactManifolds)[i].normal * points.back().separation;
        glm::vec3 separationLineEnd = separationLineStart + joinedNormal;
        glm::vec3 newContactPoint = linePlaneIntersection(separationLineStart, separationLineEnd, (*contactManifolds)[i].normal, points.back().position);
        points.back().position = newContactPoint;
        points.back().separation = -glm::length(newContactPoint - separationLineStart);
      }
    }
  }
  // reduce contact points to 4 points that maximize the contact area
  reducedManifold->normal = joinedNormal;
  if(points.size() > 4) {
    reducedManifold->points.resize(4);
    reduceContactPoints(reducedManifold->points.data(), points.data(), points.size(), &joinedNormal);
  } else {
    reducedManifold->points = points;
  }
  return true;
}

Collision::ContactManifold Collision::getPolyhedronContactManifold(Polyhedron* A, Polyhedron* B, IsStatic isStatic, bool useTemporalCoherence) {
  TemporalCoherenceState* state = nullptr;
  if(useTemporalCoherence) {
    auto it = temporalCoherenceStateMap.find(A);
    if(it == temporalCoherenceStateMap.end()) {
      temporalCoherenceStateMap[A] = std::map<Polyhedron*, TemporalCoherenceState>();
    }
    auto it2 = temporalCoherenceStateMap[A].find(B);
    if(it2 == temporalCoherenceStateMap[A].end()) {
      temporalCoherenceStateMap[A][B] = TemporalCoherenceState{TemporalCoherenceState::Status::null};
    }
    state = &(temporalCoherenceStateMap[A][B]);
    if(state->status == TemporalCoherenceState::Status::separated) {
      // check if the separating axis of last frame is still separating
      if(state->separatingAxisType == TemporalCoherenceState::SeparatingAxisType::face) {
        if(isSeparatingFaceAxis(state->separatingFaceAxis.face, state->separatingFaceAxis.otherPolyhedron)) {
          return ContactManifold{};
        }
      } else if(state->separatingAxisType == TemporalCoherenceState::SeparatingAxisType::edge) {
        if(isSeparatingEdgeAxis(state->separatingEdgeAxis.edgeA, state->separatingEdgeAxis.edgeB, &(A->centroid))) {
          return ContactManifold{};
        }
      }
    } else if(state->status == TemporalCoherenceState::Status::overlapping) {
      // try to rebuild contact from previous features
      if(state->overlappingFeaturesType == TemporalCoherenceState::OverlappingFeaturesType::faceFace) {
        FaceDirectionQuery faceDirectionQueryA = getSingleFaceDirectionOverlap(state->overlappingFaces.faceA, B);
        FaceDirectionQuery faceDirectionQueryB = getSingleFaceDirectionOverlap(state->overlappingFaces.faceB, A);
        if(faceDirectionQueryA.separation < 0.0f && faceDirectionQueryB.separation < 0.0f) {
          ContactManifold contactManifold = getFaceContactManifold(A, B, &faceDirectionQueryA, &faceDirectionQueryB);
          if(contactManifold.points.size() > 0) {
            return contactManifold;
          }
        }
      } else if(state->overlappingFeaturesType == TemporalCoherenceState::OverlappingFeaturesType::edgeEdge) {
        EdgeSeparationDistanceQuery edgeSeparationDistanceQuery = getEdgeSeparationDistance(state->overlappingEdges.edgeA, state->overlappingEdges.edgeB, A->centroid);
        if(edgeSeparationDistanceQuery.separation < 0.0f) {
          // check that the manifold normal is in allowed range to avoid forming otherwise valid edge contacts where some polyhedron faces are clipping
          glm::vec3 flippedNormal = -edgeSeparationDistanceQuery.separationDirection;
          glm::vec3 combinedVector = glm::normalize(state->overlappingEdges.edgeB->face->normal + state->overlappingEdges.edgeB->twin->face->normal);
          if(!Utility::isNan(combinedVector)) { // in case of a 2-sided plane
            float allowedRange = glm::dot(combinedVector, state->overlappingEdges.edgeB->face->normal);
            bool  angleOkB = (glm::dot(combinedVector, flippedNormal) > allowedRange);
            combinedVector = glm::normalize(state->overlappingEdges.edgeA->face->normal + state->overlappingEdges.edgeA->twin->face->normal);
            if(!Utility::isNan(combinedVector)) { // in case of a 2-sided plane
              allowedRange = glm::dot(combinedVector, state->overlappingEdges.edgeA->face->normal);
              bool angleOkA = (glm::dot(combinedVector, edgeSeparationDistanceQuery.separationDirection) > allowedRange);
              if(angleOkA && angleOkB) {
                EdgeDirectionQuery edgeDirectionQuery = EdgeDirectionQuery{
                  edgeSeparationDistanceQuery.separationDirection,
                  edgeSeparationDistanceQuery.separation,
                  state->overlappingEdges.edgeA,
                  state->overlappingEdges.edgeB};
                ContactManifold contactManifold = getEdgeContactManifold(A, B, &edgeDirectionQuery, isStatic);
                if(contactManifold.points.size() > 0) {
                  return contactManifold;
                }
              }
            }
          }
        }
      }
    }
  }

  // state undetermined or not separated anymore
  FaceDirectionQuery faceDirectionQueryA = getFaceDirectionOverlap(A, B); // faces A
  if(faceDirectionQueryA.separation > 0.0f) { // a separating axis exists
    if(useTemporalCoherence) setTemporalCoherenceStateSeparatingFaceAxis(state, faceDirectionQueryA.face, B);
    return ContactManifold{};
  }
  FaceDirectionQuery faceDirectionQueryB = getFaceDirectionOverlap(B, A); // faces B
  if(faceDirectionQueryB.separation > 0.0f) { // a separating axis exists
    if(useTemporalCoherence) setTemporalCoherenceStateSeparatingFaceAxis(state, faceDirectionQueryB.face, A);
    return ContactManifold{};
  }
  EdgeDirectionQuery edgeDirectionQuery = getEdgeDirectionOverlap(A, B); // edges A and B
  if(edgeDirectionQuery.separation > 0.0f) { // a separating axis exists
    if(useTemporalCoherence) setTemporalCoherenceStateSeparatingEdgeAxis(state, edgeDirectionQuery.edgeA, edgeDirectionQuery.edgeB);
    return ContactManifold{};
  }

  // add a bias to prefer face contact if face and edge contacts are equally minimising to avoid flip-flopping
  if(faceDirectionQueryA.separation + epsilon > edgeDirectionQuery.separation ||
     faceDirectionQueryB.separation + epsilon > edgeDirectionQuery.separation) {
    ContactManifold contactManifold = getFaceContactManifold(A, B, &faceDirectionQueryA, &faceDirectionQueryB);
    if(useTemporalCoherence) setTemporalCoherenceStateOverlappingFaces(state, faceDirectionQueryA.face, faceDirectionQueryB.face);
    return contactManifold;
  }
  ContactManifold contactManifold = getEdgeContactManifold(A, B, &edgeDirectionQuery, isStatic);
  if(useTemporalCoherence) setTemporalCoherenceStateOverlappingEdges(state, edgeDirectionQuery.edgeA, edgeDirectionQuery.edgeB);
  return contactManifold;
}

Collision::ContactManifold Collision::getSphereBoxContactManifold(glm::vec3 positionA, float radiusA, glm::vec3 halfWidth, glm::mat4 modelMatrixB, glm::mat4 modelMatrixInverseB) {
  // test for overlap in local space of B
  glm::vec3 sphereCenter = glm::vec3(modelMatrixInverseB * glm::vec4(positionA, 1));
  glm::vec3 nearestPointInBox(
    std::max(-halfWidth.x, std::min(sphereCenter.x, halfWidth.x)),
    std::max(-halfWidth.y, std::min(sphereCenter.y, halfWidth.y)),
    std::max(-halfWidth.z, std::min(sphereCenter.z, halfWidth.z)));
  glm::vec3 offset = sphereCenter - nearestPointInBox;
  float     offsetLength = glm::length(offset);
  if(offsetLength < radiusA) { // sphere overlaps the box
    ContactManifold contactManifold;
    if(offsetLength <= 0.0f) { // sphere center is inside the box
      float     distances[6];
      glm::vec3 normals[6];
      distances[0] = std::abs(halfWidth.x - sphereCenter.x);
      normals[0] = glm::vec3(1.0f, 0.0f, 0.0f);
      distances[1] = std::abs(-halfWidth.x - sphereCenter.x);
      normals[1] = glm::vec3(-1.0f, 0.0f, 0.0f);
      distances[2] = std::abs(halfWidth.y - sphereCenter.y);
      normals[2] = glm::vec3(0.0f, 1.0f, 0.0f);
      distances[3] = std::abs(-halfWidth.y - sphereCenter.y);
      normals[3] = glm::vec3(0.0f, -1.0f, 0.0f);
      distances[4] = std::abs(halfWidth.z - sphereCenter.z);
      normals[4] = glm::vec3(0.0f, 0.0f, 1.0f);
      distances[5] = std::abs(-halfWidth.z - sphereCenter.z);
      normals[5] = glm::vec3(0.0f, 0.0f, -1.0f);
      float     minDistance = distances[0];
      glm::vec3 minNormal = normals[0];
      for(int i = 1; i < 6; i++) {
        if(distances[i] < minDistance) {
          minDistance = distances[i];
          minNormal = normals[i];
        }
      }
      contactManifold.points.emplace_back(ContactPoint{-minDistance, minNormal * minDistance});
      contactManifold.normal = -minNormal; // normal away from A (sphere)
    } else {
      contactManifold.points.emplace_back(ContactPoint{-(radiusA - offsetLength), nearestPointInBox});
      contactManifold.normal = -offset / offsetLength; // normal away from A (sphere)
    }
    // return contact manifold in world space
    contactManifold.points[0].position = glm::vec3(modelMatrixB * glm::vec4(contactManifold.points[0].position, 1));
    contactManifold.normal = glm::vec3(modelMatrixB * glm::vec4(contactManifold.normal, 0));
    return contactManifold;
  }
  return ContactManifold{};
}

Collision::ContactManifold Collision::getSphereCylinderContactManifold(glm::vec3 positionA, float radiusA, float radiusB, float halfHeightB, glm::mat4 modelMatrixB, glm::mat4 modelMatrixInverseB) {
  // test for overlap in local space of B
  glm::vec3 sphereCenter = glm::vec3(modelMatrixInverseB * glm::vec4(positionA, 1));
  glm::vec3 nearestPointInCylinderRadius;
  glm::vec3 nearestPointInCylinderTop;
  glm::vec3 nearestPointInCylinderBottom;
  glm::vec3 planeDirection = glm::vec3(1.0f, 0.0f, 0.0f);
  glm::vec3 sphereXzPlaneCenter = glm::vec3(sphereCenter.x, 0.0f, sphereCenter.z);
  if(glm::length2(sphereXzPlaneCenter) > epsilon) {
    planeDirection = glm::normalize(sphereXzPlaneCenter);
  }
  nearestPointInCylinderRadius = planeDirection * radiusB;
  nearestPointInCylinderRadius.y = glm::clamp(sphereCenter.y, -halfHeightB, halfHeightB);
  bool sphereCenterOutsideRadius = (glm::length2(sphereXzPlaneCenter) > radiusB * radiusB);
  if(sphereCenterOutsideRadius) {
    nearestPointInCylinderTop = glm::vec3(nearestPointInCylinderRadius.x, halfHeightB, nearestPointInCylinderRadius.z);
    nearestPointInCylinderBottom = glm::vec3(nearestPointInCylinderRadius.x, -halfHeightB, nearestPointInCylinderRadius.z);
  } else {
    nearestPointInCylinderTop = glm::vec3(sphereCenter.x, halfHeightB, sphereCenter.z);
    nearestPointInCylinderBottom = glm::vec3(sphereCenter.x, -halfHeightB, sphereCenter.z);
  }
  glm::vec3 nearestPointInCylinderSurface = nearestPointInCylinderRadius;
  if(glm::length2(sphereCenter - nearestPointInCylinderTop) < glm::length2(sphereCenter - nearestPointInCylinderSurface)) {
    nearestPointInCylinderSurface = nearestPointInCylinderTop;
  }
  if(glm::length2(sphereCenter - nearestPointInCylinderBottom) < glm::length2(sphereCenter - nearestPointInCylinderSurface)) {
    nearestPointInCylinderSurface = nearestPointInCylinderBottom;
  }
  bool      centerOverlap = (sphereCenter.y < halfHeightB && sphereCenter.y > -halfHeightB && !sphereCenterOutsideRadius);
  glm::vec3 offset = sphereCenter - nearestPointInCylinderSurface;
  float     offsetLength = glm::length(offset);
  if(offsetLength < radiusA || centerOverlap) { // sphere overlaps the cylinder
    ContactManifold contactManifold;
    contactManifold.points.emplace_back(ContactPoint{-std::abs(radiusA - offsetLength), nearestPointInCylinderSurface});
    // normal away from A (sphere)
    if(offsetLength > epsilon) { // avoid division by zero
      contactManifold.normal = -offset / offsetLength;
      if(centerOverlap) {
        contactManifold.normal *= -1.0f;
      }
    } else if(nearestPointInCylinderSurface == nearestPointInCylinderRadius) {
      if(glm::length2(sphereXzPlaneCenter) > epsilon) {
        contactManifold.normal = glm::normalize(-sphereXzPlaneCenter);
      } else {
        contactManifold.normal = glm::vec3(0.0f, -1.0f, 0.0f);
      }
    } else if(nearestPointInCylinderSurface == nearestPointInCylinderTop) {
      contactManifold.normal = glm::vec3(0.0f, -1.0f, 0.0f);
    } else { // nearestPointInCylinderBottom
      contactManifold.normal = glm::vec3(0.0f, 1.0f, 0.0f);
    }
    // return contact manifold in world space
    contactManifold.points[0].position = glm::vec3(modelMatrixB * glm::vec4(contactManifold.points[0].position, 1));
    contactManifold.normal = glm::vec3(modelMatrixB * glm::vec4(contactManifold.normal, 0));
    return contactManifold;
  }
  return ContactManifold{};
}

Collision::ContactManifold Collision::getSphereSphereContactManifold(glm::vec3 positionA, float radiusA, glm::vec3 positionB, float radiusB) {
  glm::vec3 vector = positionB - positionA;
  float     length2 = glm::length2(vector);
  if(length2 > epsilon && length2 < (radiusA + radiusB) * (radiusA + radiusB)) {
    ContactManifold contactManifold;
    contactManifold.normal = glm::normalize(vector);
    glm::vec3 surfacePointA = positionA + radiusA * contactManifold.normal;
    glm::vec3 surfacePointB = positionB + radiusB * -contactManifold.normal;
    contactManifold.points.emplace_back(ContactPoint{-glm::length(surfacePointA - surfacePointB), surfacePointB});
    return contactManifold;
  }
  return ContactManifold{};
}

Collision::ContactManifold Collision::getSpherePlaneContactManifold(glm::vec3 positionA, float radiusA, Polyhedron* B) {
  const glm::vec3 planeNormal = B->faces[0].normal;
  const glm::vec3 planeVertex = B->vertices[0];
  const float     dist = pointToPlaneDistance(positionA, planeNormal, planeVertex);
  if(std::abs(dist) > radiusA) {
    return ContactManifold{};
  }

  const glm::vec3 intersectionPoint = linePlaneIntersection(positionA, positionA - planeNormal, planeNormal, planeVertex);

  bool isInside = true;
  int  c = B->vertices.size();
  bool sign = (pointToPlaneDistance(intersectionPoint, glm::cross(B->vertices[1] - planeVertex, planeNormal), planeVertex) < 0);
  for(int i = 1; i < c; i++) {
    if(i == c - 1) {
      if((pointToPlaneDistance(intersectionPoint, glm::cross(B->vertices[0] - B->vertices[i], planeNormal), B->vertices[i]) < 0) != sign) {
        isInside = false;
        break;
      }
    } else {
      if((pointToPlaneDistance(intersectionPoint, glm::cross(B->vertices[i + 1] - B->vertices[i], planeNormal), B->vertices[i]) < 0) != sign) {
        isInside = false;
        break;
      }
    }
  }

  if(isInside) {
    ContactManifold contactManifold;
    // always make the contact push the sphere to the direction of plane normal
    contactManifold.points.emplace_back(ContactPoint{dist - radiusA, intersectionPoint});
    contactManifold.normal = -planeNormal;
    return contactManifold;
  }

  PointToLineSegmentDistanceQuery minDistanceQuery = pointToLineSegmentDistance(positionA, B->vertices[0], B->vertices[1]);
  for(int i = 1; i < c; i++) {
    PointToLineSegmentDistanceQuery query;
    if(i == c - 1) {
      query = pointToLineSegmentDistance(positionA, B->vertices[i], B->vertices[0]);
    } else {
      query = pointToLineSegmentDistance(positionA, B->vertices[i], B->vertices[i + 1]);
    }
    if(query.distance < minDistanceQuery.distance) {
      minDistanceQuery = query;
    }
  }
  if(minDistanceQuery.distance > radiusA) {
    return ContactManifold{};
  }
  ContactManifold contactManifold;
  if(minDistanceQuery.pointOverlapsLineSegment) {
    contactManifold.points.emplace_back(ContactPoint{-radiusA, minDistanceQuery.nearestPointOnLineSegment});
    contactManifold.normal = -planeNormal;
    return contactManifold;
  }
  if(dist > 0.0f) {
    contactManifold.points.emplace_back(ContactPoint{dist - radiusA, intersectionPoint});
    contactManifold.normal = -planeNormal;
  } else {
    contactManifold.points.emplace_back(ContactPoint{-dist - radiusA, intersectionPoint});
    contactManifold.normal = planeNormal;
  }
  return contactManifold;
}

void Collision::constructHitbox(Polyhedron* p, glm::vec3 halfWidth) {
  p->vertices.reserve(8);
  p->edges.reserve(24);
  p->faces.reserve(6);

  // +z face
  p->vertices.emplace_back(glm::vec3(halfWidth.x, halfWidth.y, halfWidth.z));
  p->vertices.emplace_back(glm::vec3(-halfWidth.x, halfWidth.y, halfWidth.z));
  p->vertices.emplace_back(glm::vec3(-halfWidth.x, -halfWidth.y, halfWidth.z));
  p->vertices.emplace_back(glm::vec3(halfWidth.x, -halfWidth.y, halfWidth.z));

  // -z face
  p->vertices.emplace_back(glm::vec3(-halfWidth.x, halfWidth.y, -halfWidth.z));
  p->vertices.emplace_back(glm::vec3(halfWidth.x, halfWidth.y, -halfWidth.z));
  p->vertices.emplace_back(glm::vec3(halfWidth.x, -halfWidth.y, -halfWidth.z));
  p->vertices.emplace_back(glm::vec3(-halfWidth.x, -halfWidth.y, -halfWidth.z));

  // half-edges travel counter-clockwise and the twin is stored after the half-edge
  // +z face
  p->edges.emplace_back(HalfEdge{&(p->vertices[0]), nullptr, nullptr, nullptr});
  p->edges.emplace_back(HalfEdge{&(p->vertices[1]), nullptr, nullptr, nullptr}); // twin
  p->edges.emplace_back(HalfEdge{&(p->vertices[1]), nullptr, nullptr, nullptr});
  p->edges.emplace_back(HalfEdge{&(p->vertices[2]), nullptr, nullptr, nullptr}); // twin
  p->edges.emplace_back(HalfEdge{&(p->vertices[2]), nullptr, nullptr, nullptr});
  p->edges.emplace_back(HalfEdge{&(p->vertices[3]), nullptr, nullptr, nullptr}); // twin
  p->edges.emplace_back(HalfEdge{&(p->vertices[3]), nullptr, nullptr, nullptr});
  p->edges.emplace_back(HalfEdge{&(p->vertices[0]), nullptr, nullptr, nullptr}); // twin

  //-z face
  p->edges.emplace_back(HalfEdge{&(p->vertices[4]), nullptr, nullptr, nullptr});
  p->edges.emplace_back(HalfEdge{&(p->vertices[5]), nullptr, nullptr, nullptr}); // twin
  p->edges.emplace_back(HalfEdge{&(p->vertices[5]), nullptr, nullptr, nullptr});
  p->edges.emplace_back(HalfEdge{&(p->vertices[6]), nullptr, nullptr, nullptr}); // twin
  p->edges.emplace_back(HalfEdge{&(p->vertices[6]), nullptr, nullptr, nullptr});
  p->edges.emplace_back(HalfEdge{&(p->vertices[7]), nullptr, nullptr, nullptr}); // twin
  p->edges.emplace_back(HalfEdge{&(p->vertices[7]), nullptr, nullptr, nullptr});
  p->edges.emplace_back(HalfEdge{&(p->vertices[4]), nullptr, nullptr, nullptr}); // twin

  // +y face missing edges
  p->edges.emplace_back(HalfEdge{&(p->vertices[0]), nullptr, nullptr, nullptr});
  p->edges.emplace_back(HalfEdge{&(p->vertices[5]), nullptr, nullptr, nullptr}); // twin
  p->edges.emplace_back(HalfEdge{&(p->vertices[4]), nullptr, nullptr, nullptr});
  p->edges.emplace_back(HalfEdge{&(p->vertices[1]), nullptr, nullptr, nullptr}); // twin

  // -y face missing edges
  p->edges.emplace_back(HalfEdge{&(p->vertices[3]), nullptr, nullptr, nullptr});
  p->edges.emplace_back(HalfEdge{&(p->vertices[6]), nullptr, nullptr, nullptr}); // twin
  p->edges.emplace_back(HalfEdge{&(p->vertices[7]), nullptr, nullptr, nullptr});
  p->edges.emplace_back(HalfEdge{&(p->vertices[2]), nullptr, nullptr, nullptr}); // twin

  // +-x
  p->faces.emplace_back(Face{glm::vec3(1.0f, 0.0f, 0.0f), &(p->edges[17])});
  p->faces.emplace_back(Face{glm::vec3(-1.0f, 0.0f, 0.0f), &(p->edges[19])});

  // +-y
  p->faces.emplace_back(Face{glm::vec3(0.0f, 1.0f, 0.0f), &(p->edges[9])});
  p->faces.emplace_back(Face{glm::vec3(0.0f, -1.0f, 0.0f), &(p->edges[5])});

  // +-z
  p->faces.emplace_back(Face{glm::vec3(0.0f, 0.0f, 1.0f), &(p->edges[0])});
  p->faces.emplace_back(Face{glm::vec3(0.0f, 0.0f, -1.0f), &(p->edges[8])});

  // +z face
  p->edges[0].next = &(p->edges[2]);
  p->edges[2].next = &(p->edges[4]);
  p->edges[4].next = &(p->edges[6]);
  p->edges[6].next = &(p->edges[0]);

  // -z face
  p->edges[8].next = &(p->edges[10]);
  p->edges[10].next = &(p->edges[12]);
  p->edges[12].next = &(p->edges[14]);
  p->edges[14].next = &(p->edges[8]);

  p->edges[16].next = &(p->edges[9]);
  p->edges[17].next = &(p->edges[7]); // twin

  p->edges[18].next = &(p->edges[1]);
  p->edges[19].next = &(p->edges[15]); // twin

  p->edges[20].next = &(p->edges[11]);
  p->edges[21].next = &(p->edges[5]); // twin

  p->edges[22].next = &(p->edges[3]);
  p->edges[23].next = &(p->edges[13]); // twin

  // +z face missing edges
  p->edges[1].next = &(p->edges[16]);
  p->edges[3].next = &(p->edges[19]); // twin
  p->edges[5].next = &(p->edges[23]); // twin
  p->edges[7].next = &(p->edges[20]);

  // -z face missing edges
  p->edges[9].next = &(p->edges[18]);
  p->edges[11].next = &(p->edges[17]); // twin
  p->edges[13].next = &(p->edges[21]); // twin
  p->edges[15].next = &(p->edges[22]);

  for(int i = 0; i < p->edges.size(); i += 2) {
    p->edges[i].twin = &(p->edges[i + 1]);
    p->edges[i + 1].twin = &(p->edges[i]);
  }

  // +z face
  p->edges[0].face = &(p->faces[4]);
  p->edges[2].face = &(p->faces[4]);
  p->edges[4].face = &(p->faces[4]);
  p->edges[6].face = &(p->faces[4]);

  // -z face
  p->edges[8].face = &(p->faces[5]);
  p->edges[10].face = &(p->faces[5]);
  p->edges[12].face = &(p->faces[5]);
  p->edges[14].face = &(p->faces[5]);

  p->edges[16].face = &(p->faces[2]); // +y
  p->edges[17].face = &(p->faces[0]); // +x

  p->edges[18].face = &(p->faces[2]); // +y
  p->edges[19].face = &(p->faces[1]); // -x

  p->edges[20].face = &(p->faces[0]); // +x
  p->edges[21].face = &(p->faces[3]); // -y

  p->edges[22].face = &(p->faces[1]); // -x
  p->edges[23].face = &(p->faces[3]); // -y

  // +z face missing edges
  p->edges[1].face = &(p->faces[2]); // +y
  p->edges[3].face = &(p->faces[1]); // -x
  p->edges[5].face = &(p->faces[3]); // -y
  p->edges[7].face = &(p->faces[0]); // +x

  // -z face missing edges
  p->edges[9].face = &(p->faces[2]); // +y
  p->edges[11].face = &(p->faces[0]); // +x
  p->edges[13].face = &(p->faces[3]); // -y
  p->edges[15].face = &(p->faces[1]); // -x

  p->centroid = glm::vec3(0.0f, 0.0f, 0.0f);
  p->maxHalfWidth = halfWidth;
}

void Collision::constructCylinder(Polyhedron* p, float radius, float height, int edges) {
  const int c = edges;
  p->vertices.reserve(2 * c);
  p->edges.reserve(2 * (3 * c));
  p->faces.reserve(c + 2);

  // +y face
  for(int i = 0; i < c; i++) {
    p->vertices.emplace_back(glm::rotateY(glm::vec3(radius, height * 0.5f, 0.0f), float(i) / float(c) * Utility::tau));
  }
  // -y face
  for(int i = 0; i < c; i++) {
    p->vertices.emplace_back(glm::rotateY(glm::vec3(radius, height * -0.5f, 0.0f), float(i) / float(c) * Utility::tau));
  }
  // half-edges travel counter-clockwise and the twin is stored after the half-edge
  for(int i = 0; i < c; i++) {
    p->edges.emplace_back(HalfEdge{&(p->vertices[i]), nullptr, nullptr, nullptr});
    p->edges.emplace_back(HalfEdge{&(p->vertices[c + i]), nullptr, nullptr, nullptr}); // twin
  }
  for(int i = 0; i < c; i++) {
    p->edges.emplace_back(HalfEdge{&(p->vertices[i]), nullptr, nullptr, nullptr});
    if(i == c - 1) {
      p->edges.emplace_back(HalfEdge{&(p->vertices[0]), nullptr, nullptr, nullptr}); // twin
    } else {
      p->edges.emplace_back(HalfEdge{&(p->vertices[i + 1]), nullptr, nullptr, nullptr}); // twin
    }
  }
  for(int i = 0; i < c; i++) {
    if(i == c - 1) {
      p->edges.emplace_back(HalfEdge{&(p->vertices[c]), nullptr, nullptr, nullptr});
    } else {
      p->edges.emplace_back(HalfEdge{&(p->vertices[c + i + 1]), nullptr, nullptr, nullptr});
    }
    p->edges.emplace_back(HalfEdge{&(p->vertices[c + i]), nullptr, nullptr, nullptr}); // twin
  }

  for(int i = 0; i < c; i++) {
    p->faces.emplace_back(Face{glm::rotateY(glm::vec3(1.0f, 0.0f, 0.0f), ((float(i) + 0.5f) / float(c)) * Utility::tau), &(p->edges[2 * i])});
  }
  p->faces.emplace_back(Face{glm::vec3(0.0f, 1.0f, 0.0f), &(p->edges[2 * c])});
  p->faces.emplace_back(Face{glm::vec3(0.0f, -1.0f, 0.0f), &(p->edges[4 * c])});

  for(int i = 0; i < 2 * c; i += 2) {
    // vertical edges
    p->edges[i].next = &(p->edges[4 * c + i + 1]);
    if(i == 0) {
      p->edges[i + 1].next = &(p->edges[2 * c + 2 * c - 2 + 1]);
    } else {
      p->edges[i + 1].next = &(p->edges[2 * c + i - 2 + 1]);
    }
    // +y face edges
    if(i == 2 * (c - 1)) {
      p->edges[2 * c + i].next = &(p->edges[2 * c]);
    } else {
      p->edges[2 * c + i].next = &(p->edges[2 * c + i + 2]);
    }
    p->edges[2 * c + i + 1].next = &(p->edges[i]);
    // -y face edges
    if(i == 0) {
      p->edges[4 * c + i].next = &(p->edges[4 * c + 2 * c - 2]);
    } else {
      p->edges[4 * c + i].next = &(p->edges[4 * c + i - 2]);
    }
    if(i == 2 * (c - 1)) {
      p->edges[4 * c + i + 1].next = &(p->edges[1]);
    } else {
      p->edges[4 * c + i + 1].next = &(p->edges[i + 2 + 1]);
    }
  }

  for(int i = 0; i < p->edges.size(); i += 2) {
    p->edges[i].twin = &(p->edges[i + 1]);
    p->edges[i + 1].twin = &(p->edges[i]);
  }

  for(int i = 0; i < c; i++) {
    // vertical edges
    p->edges[i * 2].face = &(p->faces[i]);
    if(i == 0) {
      p->edges[i * 2 + 1].face = &(p->faces[c - 1]);
    } else {
      p->edges[i * 2 + 1].face = &(p->faces[i - 1]);
    }
    // +y face edges
    p->edges[2 * c + i * 2].face = &(p->faces[c]);
    p->edges[2 * c + i * 2 + 1].face = &(p->faces[i]);
    // -y face edges
    p->edges[4 * c + i * 2].face = &(p->faces[c + 1]);
    p->edges[4 * c + i * 2 + 1].face = &(p->faces[i]);
  }

  p->centroid = glm::vec3(0.0f, 0.0f, 0.0f);
  p->maxHalfWidth = glm::vec3(radius, height * 0.5f, radius);
}

void Collision::constructPlane(Polyhedron* p, std::vector<glm::vec3>* vertices, glm::vec3* normal) {
  // construct a 2-sided plane polyhedron
  const int c = vertices->size();
  p->vertices.reserve(c);
  p->edges.reserve(c);
  p->faces.reserve(2);
  for(int i = 0; i < c; i++) {
    p->vertices.emplace_back((*vertices)[i]);
  }
  // half-edges travel counter-clockwise and the twin is stored after the half-edge
  for(int i = 0; i < c; i++) {
    p->edges.emplace_back(HalfEdge{&(p->vertices[i]), nullptr, nullptr, nullptr});
    if(i == c - 1) {
      p->edges.emplace_back(HalfEdge{&(p->vertices[0]), nullptr, nullptr, nullptr}); // twin
    } else {
      p->edges.emplace_back(HalfEdge{&(p->vertices[i + 1]), nullptr, nullptr, nullptr}); // twin
    }
  }
  // assume vertices are in counter-clockwise order (the normal is facing the correct way)
  p->faces.emplace_back(Face{*normal, &(p->edges[0])});
  p->faces.emplace_back(Face{-(*normal), &(p->edges[1])});

  for(int i = 0; i < 2 * c; i += 2) {
    if(i == 2 * c - 2) {
      p->edges[i].next = &(p->edges[0]);
    } else {
      p->edges[i].next = &(p->edges[i + 2]);
    }
    p->edges[i].face = &(p->faces[0]);
  }
  for(int i = 1; i < 2 * c; i += 2) {
    if(i == 2 * c - 1) {
      p->edges[1].next = &(p->edges[i]);
    } else {
      p->edges[i + 2].next = &(p->edges[i]);
    }
    p->edges[i].face = &(p->faces[1]);
  }

  for(int i = 0; i < p->edges.size(); i += 2) {
    p->edges[i].twin = &(p->edges[i + 1]);
    p->edges[i + 1].twin = &(p->edges[i]);
  }

  glm::vec3 average = glm::vec3(0.0f, 0.0f, 0.0f);
  for(int i = 0; i < c; i++) {
    average += p->vertices[i];
  }
  p->centroid = average / (float)c;

  float     maxDistanceSquared = 0.0f;
  glm::vec3 maxV;
  for(int i = 0; i < c; i++) {
    glm::vec3 v = p->vertices[i] - p->centroid;
    float     distanceSquared = glm::length2(v);
    if(distanceSquared > maxDistanceSquared) {
      maxDistanceSquared = distanceSquared;
      maxV = v;
    }
  }
  p->maxHalfWidth = glm::vec3(std::abs(maxV.x), std::abs(maxV.y), std::abs(maxV.z));
}

void Collision::copyPolyhedron(Polyhedron* source, Polyhedron* target) {
  target->vertices = source->vertices;
  target->edges = source->edges;
  target->faces = source->faces;
  target->centroid = source->centroid;
  target->maxHalfWidth = source->maxHalfWidth;
  // convert pointers
  std::vector<glm::vec3>::size_type vertexIndex;
  std::vector<HalfEdge>::size_type  edgeIndex;
  std::vector<Face>::size_type      faceIndex;
  for(int i = 0; i < source->edges.size(); i++) {
    vertexIndex = source->edges[i].startVertex - &(source->vertices[0]);
    target->edges[i].startVertex = &(target->vertices[vertexIndex]);
    edgeIndex = source->edges[i].next - &(source->edges[0]);
    target->edges[i].next = &(target->edges[edgeIndex]);
    edgeIndex = source->edges[i].twin - &(source->edges[0]);
    target->edges[i].twin = &(target->edges[edgeIndex]);
    faceIndex = source->edges[i].face - &(source->faces[0]);
    target->edges[i].face = &(target->faces[faceIndex]);
  }
  for(int i = 0; i < source->faces.size(); i++) {
    edgeIndex = source->faces[i].startEdge - &(source->edges[0]);
    target->faces[i].startEdge = &(target->edges[edgeIndex]);
  }
  transformPolyhedron(source, target, glm::mat4(1.0f));
}

void Collision::transformPolyhedron(Polyhedron* p, Polyhedron* pResult, glm::mat4 m) {
  for(int i = 0; i < p->vertices.size(); i++) {
    pResult->vertices[i] = glm::vec3(m * glm::vec4(p->vertices[i], 1));
  }
  for(int i = 0; i < p->faces.size(); i++) {
    pResult->faces[i].normal = glm::vec3(m * glm::vec4(p->faces[i].normal, 0));
  }
  pResult->centroid = glm::vec3(m * glm::vec4(p->centroid, 1));
}

glm::vec3 Collision::getSupportPointCoordinate(Polyhedron* polyhedron, glm::vec3 direction) {
  return polyhedron->vertices[getSupportPoint(polyhedron, direction)];
}
