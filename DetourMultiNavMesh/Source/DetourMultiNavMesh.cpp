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

#include "DetourMultiNavMesh.h"
#include "DetourAlloc.h"
#include "DetourAssert.h"
#include "DetourCommon.h"
#include "DetourNavMeshQuery.h"

#include <string.h>
#include <math.h>
#include <new>

dtMultiNavMesh::dtMultiNavMesh()
	: m_meshes(0)
	, m_links(0)
	, m_maxNavMeshes(0)
	, m_maxLinks(0)
{
	memset(&m_params, 0, sizeof(m_params));
}

dtMultiNavMesh::~dtMultiNavMesh()
{
	if (m_meshes)
	{
		for (int i = 0; i < m_maxNavMeshes; ++i)
		{
			if (m_meshes[i].active && m_meshes[i].navQuery)
			{
				dtFreeNavMeshQuery(m_meshes[i].navQuery);
				m_meshes[i].navQuery = 0;
			}
		}
		dtFree(m_meshes);
	}
	if (m_links)
		dtFree(m_links);
}

dtStatus dtMultiNavMesh::init(const dtMultiNavMeshParams* params)
{
	if (!params)
		return DT_FAILURE | DT_INVALID_PARAM;

	if (params->maxNavMeshes <= 0 || params->maxNavMeshes > DT_MULTI_NAVMESH_MAX_MESHES)
		return DT_FAILURE | DT_INVALID_PARAM;

	if (params->maxLinks <= 0 || params->maxLinks > DT_MULTI_NAVMESH_MAX_LINKS)
		return DT_FAILURE | DT_INVALID_PARAM;

	m_params = *params;
	m_maxNavMeshes = params->maxNavMeshes;
	m_maxLinks = params->maxLinks;

	m_meshes = (dtNavMeshInstance*)dtAlloc(sizeof(dtNavMeshInstance) * m_maxNavMeshes, DT_ALLOC_PERM);
	if (!m_meshes)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	memset(m_meshes, 0, sizeof(dtNavMeshInstance) * m_maxNavMeshes);

	m_links = (dtMultiNavMeshLink*)dtAlloc(sizeof(dtMultiNavMeshLink) * m_maxLinks, DT_ALLOC_PERM);
	if (!m_links)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	memset(m_links, 0, sizeof(dtMultiNavMeshLink) * m_maxLinks);

	return DT_SUCCESS;
}

void dtMultiNavMesh::transposeMatrix3x3(const float* m, float* out)
{
	out[0] = m[0]; out[1] = m[3]; out[2] = m[6];
	out[3] = m[1]; out[4] = m[4]; out[5] = m[7];
	out[6] = m[2]; out[7] = m[5]; out[8] = m[8];
}

void dtMultiNavMesh::mulMatrix3x3Vec(const float* m, const float* v, float* out)
{
	out[0] = m[0]*v[0] + m[1]*v[1] + m[2]*v[2];
	out[1] = m[3]*v[0] + m[4]*v[1] + m[5]*v[2];
	out[2] = m[6]*v[0] + m[7]*v[1] + m[8]*v[2];
}

dtStatus dtMultiNavMesh::addNavMesh(dtNavMesh* navMesh, const float* up, const float* origin,
									const float* rotation, int* meshIndex)
{
	if (!navMesh || !up || !origin || !rotation || !meshIndex)
		return DT_FAILURE | DT_INVALID_PARAM;

	// Find free slot.
	int freeSlot = -1;
	for (int i = 0; i < m_maxNavMeshes; ++i)
	{
		if (!m_meshes[i].active)
		{
			freeSlot = i;
			break;
		}
	}
	if (freeSlot == -1)
		return DT_FAILURE | DT_BUFFER_TOO_SMALL;

	dtNavMeshInstance& inst = m_meshes[freeSlot];

	// Create and init query for this mesh.
	dtNavMeshQuery* query = dtAllocNavMeshQuery();
	if (!query)
		return DT_FAILURE | DT_OUT_OF_MEMORY;

	dtStatus status = query->init(navMesh, DT_MULTI_NAVMESH_QUERY_MAX_NODES);
	if (dtStatusFailed(status))
	{
		dtFreeNavMeshQuery(query);
		return status;
	}

	inst.navMesh = navMesh;
	inst.navQuery = query;
	dtVcopy(inst.up, up);
	dtVcopy(inst.origin, origin);
	memcpy(inst.rotation, rotation, sizeof(float) * 9);
	transposeMatrix3x3(rotation, inst.invRotation);
	inst.customVertexData = 0;
	inst.customVertexStride = 0;
	inst.customVertexCount = 0;
	inst.active = true;

	*meshIndex = freeSlot;
	return DT_SUCCESS;
}

dtStatus dtMultiNavMesh::addCustomNavMesh(dtNavMesh* navMesh, const float* vertexData, int vertexStride,
										  int vertexCount, const float* up, const float* origin,
										  const float* rotation, int* meshIndex)
{
	dtStatus status = addNavMesh(navMesh, up, origin, rotation, meshIndex);
	if (dtStatusFailed(status))
		return status;

	dtNavMeshInstance& inst = m_meshes[*meshIndex];
	inst.customVertexData = const_cast<float*>(vertexData);
	inst.customVertexStride = vertexStride;
	inst.customVertexCount = vertexCount;

	return DT_SUCCESS;
}

dtStatus dtMultiNavMesh::removeNavMesh(int meshIndex)
{
	if (meshIndex < 0 || meshIndex >= m_maxNavMeshes)
		return DT_FAILURE | DT_INVALID_PARAM;

	if (!m_meshes[meshIndex].active)
		return DT_FAILURE | DT_INVALID_PARAM;

	// Remove all links involving this mesh.
	for (int i = 0; i < m_maxLinks; ++i)
	{
		if (m_links[i].active &&
			(m_links[i].meshIndexA == meshIndex || m_links[i].meshIndexB == meshIndex))
		{
			m_links[i].active = false;
		}
	}

	// Free the query we created.
	if (m_meshes[meshIndex].navQuery)
	{
		dtFreeNavMeshQuery(m_meshes[meshIndex].navQuery);
		m_meshes[meshIndex].navQuery = 0;
	}

	memset(&m_meshes[meshIndex], 0, sizeof(dtNavMeshInstance));
	return DT_SUCCESS;
}

dtStatus dtMultiNavMesh::addLink(int meshA, dtPolyRef polyA, const float* posA,
								 int meshB, dtPolyRef polyB, const float* posB,
								 float cost, bool bidirectional, unsigned int userId,
								 int* linkIndex)
{
	if (meshA < 0 || meshA >= m_maxNavMeshes || !m_meshes[meshA].active)
		return DT_FAILURE | DT_INVALID_PARAM;
	if (meshB < 0 || meshB >= m_maxNavMeshes || !m_meshes[meshB].active)
		return DT_FAILURE | DT_INVALID_PARAM;
	if (!posA || !posB)
		return DT_FAILURE | DT_INVALID_PARAM;

	// Find free link slot.
	int freeSlot = -1;
	for (int i = 0; i < m_maxLinks; ++i)
	{
		if (!m_links[i].active)
		{
			freeSlot = i;
			break;
		}
	}
	if (freeSlot == -1)
		return DT_FAILURE | DT_BUFFER_TOO_SMALL;

	dtMultiNavMeshLink& link = m_links[freeSlot];
	link.meshIndexA = meshA;
	link.meshIndexB = meshB;
	link.polyRefA = polyA;
	link.polyRefB = polyB;
	dtVcopy(link.posA, posA);
	dtVcopy(link.posB, posB);
	link.cost = cost;
	link.bidirectional = bidirectional;
	link.userId = userId;
	link.active = true;

	if (linkIndex)
		*linkIndex = freeSlot;

	return DT_SUCCESS;
}

dtStatus dtMultiNavMesh::removeLink(int linkIndex)
{
	if (linkIndex < 0 || linkIndex >= m_maxLinks)
		return DT_FAILURE | DT_INVALID_PARAM;

	if (!m_links[linkIndex].active)
		return DT_FAILURE | DT_INVALID_PARAM;

	m_links[linkIndex].active = false;
	return DT_SUCCESS;
}

int dtMultiNavMesh::detectLinks(float radius)
{
	if (radius <= 0)
		radius = m_params.overlapDetectionRadius;
	if (radius <= 0)
		return 0;

	const float radiusSqr = radius * radius;
	int linksCreated = 0;

	// For each pair of active meshes, find overlapping polygon vertices.
	for (int i = 0; i < m_maxNavMeshes; ++i)
	{
		if (!m_meshes[i].active || !m_meshes[i].navMesh)
			continue;

		for (int j = i + 1; j < m_maxNavMeshes; ++j)
		{
			if (!m_meshes[j].active || !m_meshes[j].navMesh)
				continue;

			const dtNavMesh* meshA = m_meshes[i].navMesh;
			const dtNavMesh* meshB = m_meshes[j].navMesh;

			// Iterate all tiles and polygons of mesh A.
			for (int tileIdxA = 0; tileIdxA < meshA->getMaxTiles(); ++tileIdxA)
			{
				const dtMeshTile* tileA = meshA->getTile(tileIdxA);
				if (!tileA || !tileA->header)
					continue;

				for (int polyIdxA = 0; polyIdxA < tileA->header->polyCount; ++polyIdxA)
				{
					const dtPoly* polyA = &tileA->polys[polyIdxA];
					if (polyA->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
						continue;

					// Compute centroid of poly A in world space.
					float centroidA[3] = {0, 0, 0};
					for (int v = 0; v < polyA->vertCount; ++v)
					{
						dtVadd(centroidA, centroidA, &tileA->verts[polyA->verts[v]*3]);
					}
					dtVscale(centroidA, centroidA, 1.0f / (float)polyA->vertCount);

					float worldCentroidA[3];
					float tempA[3];
					mulMatrix3x3Vec(m_meshes[i].rotation, centroidA, tempA);
					dtVadd(worldCentroidA, tempA, m_meshes[i].origin);

					const dtPolyRef refA = meshA->getPolyRefBase(tileA) | (dtPolyRef)polyIdxA;

					// Iterate all tiles and polygons of mesh B.
					for (int tileIdxB = 0; tileIdxB < meshB->getMaxTiles(); ++tileIdxB)
					{
						const dtMeshTile* tileB = meshB->getTile(tileIdxB);
						if (!tileB || !tileB->header)
							continue;

						for (int polyIdxB = 0; polyIdxB < tileB->header->polyCount; ++polyIdxB)
						{
							const dtPoly* polyB = &tileB->polys[polyIdxB];
							if (polyB->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
								continue;

							// Compute centroid of poly B in world space.
							float centroidB[3] = {0, 0, 0};
							for (int v = 0; v < polyB->vertCount; ++v)
							{
								dtVadd(centroidB, centroidB, &tileB->verts[polyB->verts[v]*3]);
							}
							dtVscale(centroidB, centroidB, 1.0f / (float)polyB->vertCount);

							float worldCentroidB[3];
							float tempB[3];
							mulMatrix3x3Vec(m_meshes[j].rotation, centroidB, tempB);
							dtVadd(worldCentroidB, tempB, m_meshes[j].origin);

							const float distSqr = dtVdistSqr(worldCentroidA, worldCentroidB);
							if (distSqr <= radiusSqr)
							{
								const dtPolyRef refB = meshB->getPolyRefBase(tileB) | (dtPolyRef)polyIdxB;

								int linkIdx = -1;
								dtStatus status = addLink(i, refA, worldCentroidA,
														  j, refB, worldCentroidB,
														  sqrtf(distSqr), true, 0, &linkIdx);
								if (dtStatusSucceed(status))
									++linksCreated;
							}
						}
					}
				}
			}
		}
	}

	return linksCreated;
}

dtStatus dtMultiNavMesh::localToWorld(int meshIndex, const float* localPos, float* worldPos) const
{
	if (meshIndex < 0 || meshIndex >= m_maxNavMeshes || !m_meshes[meshIndex].active)
		return DT_FAILURE | DT_INVALID_PARAM;
	if (!localPos || !worldPos)
		return DT_FAILURE | DT_INVALID_PARAM;

	float rotated[3];
	mulMatrix3x3Vec(m_meshes[meshIndex].rotation, localPos, rotated);
	dtVadd(worldPos, rotated, m_meshes[meshIndex].origin);
	return DT_SUCCESS;
}

dtStatus dtMultiNavMesh::worldToLocal(int meshIndex, const float* worldPos, float* localPos) const
{
	if (meshIndex < 0 || meshIndex >= m_maxNavMeshes || !m_meshes[meshIndex].active)
		return DT_FAILURE | DT_INVALID_PARAM;
	if (!worldPos || !localPos)
		return DT_FAILURE | DT_INVALID_PARAM;

	float translated[3];
	dtVsub(translated, worldPos, m_meshes[meshIndex].origin);
	mulMatrix3x3Vec(m_meshes[meshIndex].invRotation, translated, localPos);
	return DT_SUCCESS;
}

int dtMultiNavMesh::getNavMeshCount() const
{
	int count = 0;
	for (int i = 0; i < m_maxNavMeshes; ++i)
	{
		if (m_meshes[i].active)
			++count;
	}
	return count;
}

const dtNavMeshInstance* dtMultiNavMesh::getInstance(int index) const
{
	if (index < 0 || index >= m_maxNavMeshes)
		return 0;
	if (!m_meshes[index].active)
		return 0;
	return &m_meshes[index];
}

int dtMultiNavMesh::getLinkCount() const
{
	int count = 0;
	for (int i = 0; i < m_maxLinks; ++i)
	{
		if (m_links[i].active)
			++count;
	}
	return count;
}

const dtMultiNavMeshLink* dtMultiNavMesh::getLink(int index) const
{
	if (index < 0 || index >= m_maxLinks)
		return 0;
	if (!m_links[index].active)
		return 0;
	return &m_links[index];
}

dtMultiNavMesh* dtAllocMultiNavMesh()
{
	void* mem = dtAlloc(sizeof(dtMultiNavMesh), DT_ALLOC_PERM);
	if (!mem)
		return 0;
	return new(mem) dtMultiNavMesh;
}

void dtFreeMultiNavMesh(dtMultiNavMesh* multiNavMesh)
{
	if (!multiNavMesh)
		return;
	multiNavMesh->~dtMultiNavMesh();
	dtFree(multiNavMesh);
}
