//
// Copyright (c) 2024 Contributors
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#ifndef DETOURMULTINAVMESHQUERY_H
#define DETOURMULTINAVMESHQUERY_H

#include "DetourMultiNavMesh.h"
#include "DetourStatus.h"

/// Maximum number of polygons in a single segment's corridor.
static const int DT_MULTI_MAX_PATH_POLYS = 256;

/// Maximum number of segments in a multi-navmesh path.
static const int DT_MULTI_MAX_PATH_SEGMENTS = 64;

/// Maximum number of high-level graph nodes (2 per link + start + end).
static const int DT_MULTI_MAX_GRAPH_NODES = DT_MULTI_NAVMESH_MAX_LINKS * 2 + 2;

/// A segment of a multi-navmesh path, representing traversal within a single mesh.
struct dtMultiNavMeshPathSegment
{
	int meshIndex;								///< Which navmesh this segment is on.
	float startPos[3];							///< Start position in world space. [(x, y, z)]
	float endPos[3];							///< End position in world space. [(x, y, z)]
	dtPolyRef polys[DT_MULTI_MAX_PATH_POLYS];	///< Polygon corridor within this mesh.
	int polyCount;								///< Number of polygons in the corridor.
	int linkIndex;								///< Index of the link used to transition to next segment. (-1 for last segment.)
};

/// Provides pathfinding across multiple navmesh instances.
/// Uses a two-level A* approach: high-level graph between meshes,
/// then local paths within each mesh.
class dtMultiNavMeshQuery
{
public:
	dtMultiNavMeshQuery();
	~dtMultiNavMeshQuery();

	/// Initializes the query object.
	///  @param[in]		multiNavMesh	The multi-navmesh to query against.
	/// @return The status flags for the operation.
	dtStatus init(const dtMultiNavMesh* multiNavMesh);

	/// Finds a path from start to end across multiple navmeshes.
	///  @param[in]		startMesh		Index of the starting navmesh.
	///  @param[in]		startRef		Polygon reference on the start mesh.
	///  @param[in]		startPos		Start position in world space. [(x, y, z)]
	///  @param[in]		endMesh			Index of the ending navmesh.
	///  @param[in]		endRef			Polygon reference on the end mesh.
	///  @param[in]		endPos			End position in world space. [(x, y, z)]
	///  @param[in]		filter			The polygon filter to apply.
	///  @param[out]	segments		Output array of path segments.
	///  @param[out]	segmentCount	Number of segments in the output.
	///  @param[in]		maxSegments		Maximum segments the output array can hold.
	/// @return The status flags for the operation.
	dtStatus findPath(int startMesh, dtPolyRef startRef, const float* startPos,
					  int endMesh, dtPolyRef endRef, const float* endPos,
					  const dtQueryFilter* filter,
					  dtMultiNavMeshPathSegment* segments, int* segmentCount,
					  int maxSegments) const;

	/// Finds the nearest polygon across all navmeshes.
	///  @param[in]		worldPos		World position to search near. [(x, y, z)]
	///  @param[in]		halfExtents		Search extents in world space. [(x, y, z)]
	///  @param[in]		filter			The polygon filter to apply.
	///  @param[out]	meshIndex		Index of the mesh containing the nearest poly.
	///  @param[out]	polyRef			Reference of the nearest polygon.
	///  @param[out]	nearestPt		The nearest point on the polygon. [(x, y, z)] [opt]
	/// @return The status flags for the operation.
	dtStatus findNearestPoly(const float* worldPos, const float* halfExtents,
							 const dtQueryFilter* filter,
							 int* meshIndex, dtPolyRef* polyRef, float* nearestPt) const;

private:
	dtMultiNavMeshQuery(const dtMultiNavMeshQuery&);
	dtMultiNavMeshQuery& operator=(const dtMultiNavMeshQuery&);

	/// Node for high-level graph search.
	struct GraphNode
	{
		int meshIndex;		///< Which mesh this node is on.
		int linkIndex;		///< Which link this node represents (-1 for start/end).
		int linkSide;		///< 0 = side A of link, 1 = side B of link, -1 = start/end.
		float pos[3];		///< World-space position.
		float cost;			///< Accumulated cost from start.
		float total;		///< cost + heuristic.
		int parent;			///< Index of parent node in search.
		bool open;			///< In open list.
		bool closed;		///< In closed list.
	};

	/// Finds the high-level path (sequence of links) between meshes.
	int findHighLevelPath(int startMesh, const float* startPos,
						  int endMesh, const float* endPos,
						  int* linkPath, int* linkSides, int maxLinks) const;

	const dtMultiNavMesh* m_multiNavMesh;
};

/// Allocates a multi-navmesh query object.
dtMultiNavMeshQuery* dtAllocMultiNavMeshQuery();

/// Frees a multi-navmesh query object.
void dtFreeMultiNavMeshQuery(dtMultiNavMeshQuery* query);

#endif // DETOURMULTINAVMESHQUERY_H
