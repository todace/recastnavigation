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

#ifndef DETOURMULTINAVMESH_H
#define DETOURMULTINAVMESH_H

#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourStatus.h"

/// Maximum number of navmesh instances in a multi-navmesh.
static const int DT_MULTI_NAVMESH_MAX_MESHES = 32;

/// Maximum number of cross-mesh links.
static const int DT_MULTI_NAVMESH_MAX_LINKS = 1024;

/// Maximum number of search nodes for per-mesh queries.
static const int DT_MULTI_NAVMESH_QUERY_MAX_NODES = 2048;

/// Configuration parameters for dtMultiNavMesh.
struct dtMultiNavMeshParams
{
	int maxNavMeshes;				///< Maximum number of navmesh instances. [Limit: <= DT_MULTI_NAVMESH_MAX_MESHES]
	int maxLinks;					///< Maximum number of cross-mesh links. [Limit: <= DT_MULTI_NAVMESH_MAX_LINKS]
	float overlapDetectionRadius;	///< Radius for auto-detecting overlapping connection zones.
};

/// Represents a single navmesh instance within a multi-navmesh system.
/// Each instance has its own coordinate transform (gravity direction, origin, rotation).
struct dtNavMeshInstance
{
	dtNavMesh* navMesh;				///< The navigation mesh. (User-owned, not freed by dtMultiNavMesh.)
	dtNavMeshQuery* navQuery;		///< Pre-initialized query object for this mesh. (Owned by dtMultiNavMesh.)
	float up[3];					///< The "up" direction / gravity direction for this mesh. (e.g. {0,1,0} for floor)
	float origin[3];				///< World-space origin offset of this mesh.
	float rotation[9];				///< 3x3 rotation matrix (local-to-world, row-major).
	float invRotation[9];			///< 3x3 inverse rotation matrix (world-to-local, row-major).
	float* customVertexData;		///< Optional custom vertex data. (User-owned, not freed.)
	int customVertexStride;			///< Number of floats per vertex in customVertexData.
	int customVertexCount;			///< Number of vertices in customVertexData.
	bool active;					///< Whether this slot is in use.
};

/// A cross-mesh link connecting two navmesh instances.
struct dtMultiNavMeshLink
{
	int meshIndexA;					///< Index of the first mesh.
	int meshIndexB;					///< Index of the second mesh.
	dtPolyRef polyRefA;				///< Polygon reference on mesh A at the connection point.
	dtPolyRef polyRefB;				///< Polygon reference on mesh B at the connection point.
	float posA[3];					///< World-space position on mesh A.
	float posB[3];					///< World-space position on mesh B.
	float cost;						///< Traversal cost for crossing this link.
	bool bidirectional;				///< Whether this link can be traversed in both directions.
	unsigned int userId;			///< User-defined identifier for this link.
	bool active;					///< Whether this link slot is in use.
};

/// Manages multiple navmesh instances and cross-mesh connections.
/// Allows pathfinding across different surfaces with different gravity directions,
/// including custom user-built navmeshes with extra vertex data.
class dtMultiNavMesh
{
public:
	dtMultiNavMesh();
	~dtMultiNavMesh();

	/// Initializes the multi-navmesh system.
	///  @param[in]		params		Initialization parameters.
	/// @return The status flags for the operation.
	dtStatus init(const dtMultiNavMeshParams* params);

	/// Adds a navmesh instance with the specified transform.
	///  @param[in]		navMesh		The navmesh to add. (User retains ownership.)
	///  @param[in]		up			The "up" direction for this mesh. [(x, y, z)]
	///  @param[in]		origin		World-space origin offset. [(x, y, z)]
	///  @param[in]		rotation	3x3 rotation matrix (local-to-world, row-major). [9 floats]
	///  @param[out]	meshIndex	The index of the added mesh.
	/// @return The status flags for the operation.
	dtStatus addNavMesh(dtNavMesh* navMesh, const float* up, const float* origin,
						const float* rotation, int* meshIndex);

	/// Adds a custom navmesh with extra per-vertex data.
	///  @param[in]		navMesh			The navmesh to add. (User retains ownership.)
	///  @param[in]		vertexData		Custom per-vertex float data. (User retains ownership.)
	///  @param[in]		vertexStride	Number of floats per vertex.
	///  @param[in]		vertexCount		Number of vertices.
	///  @param[in]		up				The "up" direction for this mesh. [(x, y, z)]
	///  @param[in]		origin			World-space origin offset. [(x, y, z)]
	///  @param[in]		rotation		3x3 rotation matrix (local-to-world, row-major). [9 floats]
	///  @param[out]	meshIndex		The index of the added mesh.
	/// @return The status flags for the operation.
	dtStatus addCustomNavMesh(dtNavMesh* navMesh, const float* vertexData, int vertexStride,
							  int vertexCount, const float* up, const float* origin,
							  const float* rotation, int* meshIndex);

	/// Removes a navmesh instance and all associated links.
	///  @param[in]		meshIndex	The index of the mesh to remove.
	/// @return The status flags for the operation.
	dtStatus removeNavMesh(int meshIndex);

	/// Adds a cross-mesh link between two navmesh instances.
	///  @param[in]		meshA		Index of the first mesh.
	///  @param[in]		polyA		Polygon reference on mesh A.
	///  @param[in]		posA		World-space position on mesh A. [(x, y, z)]
	///  @param[in]		meshB		Index of the second mesh.
	///  @param[in]		polyB		Polygon reference on mesh B.
	///  @param[in]		posB		World-space position on mesh B. [(x, y, z)]
	///  @param[in]		cost		Traversal cost for this link.
	///  @param[in]		bidirectional	Whether this link can be traversed in both directions.
	///  @param[in]		userId		User-defined identifier.
	///  @param[out]	linkIndex	The index of the added link. [opt]
	/// @return The status flags for the operation.
	dtStatus addLink(int meshA, dtPolyRef polyA, const float* posA,
					 int meshB, dtPolyRef polyB, const float* posB,
					 float cost, bool bidirectional, unsigned int userId,
					 int* linkIndex);

	/// Removes a cross-mesh link.
	///  @param[in]		linkIndex	The index of the link to remove.
	/// @return The status flags for the operation.
	dtStatus removeLink(int linkIndex);

	/// Auto-detects overlapping zones between all mesh pairs and creates links.
	/// Uses the overlapDetectionRadius from init params.
	///  @param[in]		radius		Detection radius. If <= 0, uses the value from init params.
	/// @return The number of links created.
	int detectLinks(float radius);

	/// Transforms a position from local mesh coordinates to world coordinates.
	///  @param[in]		meshIndex	The mesh index.
	///  @param[in]		localPos	Local position. [(x, y, z)]
	///  @param[out]	worldPos	World position. [(x, y, z)]
	/// @return The status flags for the operation.
	dtStatus localToWorld(int meshIndex, const float* localPos, float* worldPos) const;

	/// Transforms a position from world coordinates to local mesh coordinates.
	///  @param[in]		meshIndex	The mesh index.
	///  @param[in]		worldPos	World position. [(x, y, z)]
	///  @param[out]	localPos	Local position. [(x, y, z)]
	/// @return The status flags for the operation.
	dtStatus worldToLocal(int meshIndex, const float* worldPos, float* localPos) const;

	/// Returns the number of active navmesh instances.
	int getNavMeshCount() const;

	/// Returns the maximum number of navmesh instances.
	int getMaxNavMeshes() const { return m_maxNavMeshes; }

	/// Returns the navmesh instance at the given index.
	///  @param[in]		index	The mesh index.
	/// @return The navmesh instance, or null if the index is invalid or inactive.
	const dtNavMeshInstance* getInstance(int index) const;

	/// Returns the number of active links.
	int getLinkCount() const;

	/// Returns the maximum number of links.
	int getMaxLinks() const { return m_maxLinks; }

	/// Returns the link at the given index.
	///  @param[in]		index	The link index.
	/// @return The link, or null if the index is invalid or inactive.
	const dtMultiNavMeshLink* getLink(int index) const;

	/// Returns the initialization parameters.
	const dtMultiNavMeshParams* getParams() const { return &m_params; }

private:
	dtMultiNavMesh(const dtMultiNavMesh&);
	dtMultiNavMesh& operator=(const dtMultiNavMesh&);

	/// Computes the inverse of a 3x3 rotation matrix (transpose for orthogonal matrices).
	static void transposeMatrix3x3(const float* m, float* out);

	/// Applies a 3x3 matrix to a vector.
	static void mulMatrix3x3Vec(const float* m, const float* v, float* out);

	dtMultiNavMeshParams m_params;
	dtNavMeshInstance* m_meshes;
	dtMultiNavMeshLink* m_links;
	int m_maxNavMeshes;
	int m_maxLinks;
};

/// Allocates a multi-navmesh object using the Detour allocator.
dtMultiNavMesh* dtAllocMultiNavMesh();

/// Frees a multi-navmesh object using the Detour allocator.
void dtFreeMultiNavMesh(dtMultiNavMesh* multiNavMesh);

#endif // DETOURMULTINAVMESH_H
