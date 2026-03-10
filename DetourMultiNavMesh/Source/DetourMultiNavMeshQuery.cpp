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

#include "DetourMultiNavMeshQuery.h"
#include "DetourAlloc.h"
#include "DetourAssert.h"
#include "DetourCommon.h"

#include <string.h>
#include <float.h>
#include <new>

dtMultiNavMeshQuery::dtMultiNavMeshQuery()
	: m_multiNavMesh(0)
{
}

dtMultiNavMeshQuery::~dtMultiNavMeshQuery()
{
}

dtStatus dtMultiNavMeshQuery::init(const dtMultiNavMesh* multiNavMesh)
{
	if (!multiNavMesh)
		return DT_FAILURE | DT_INVALID_PARAM;

	m_multiNavMesh = multiNavMesh;
	return DT_SUCCESS;
}

int dtMultiNavMeshQuery::findHighLevelPath(int startMesh, const float* startPos,
										   int endMesh, const float* endPos,
										   int* linkPath, int* linkSides,
										   int maxLinks) const
{
	// Build graph nodes from all active links.
	// Each link has two sides (A and B), each is a graph node.
	// Start and end positions are also graph nodes.
	GraphNode nodes[DT_MULTI_MAX_GRAPH_NODES];
	int nodeCount = 0;

	// Node 0: start
	{
		GraphNode& n = nodes[nodeCount];
		n.meshIndex = startMesh;
		n.linkIndex = -1;
		n.linkSide = -1;
		dtVcopy(n.pos, startPos);
		n.cost = 0;
		n.total = dtVdist(startPos, endPos);
		n.parent = -1;
		n.open = true;
		n.closed = false;
		nodeCount++;
	}

	// Node 1: end
	{
		GraphNode& n = nodes[nodeCount];
		n.meshIndex = endMesh;
		n.linkIndex = -1;
		n.linkSide = -1;
		dtVcopy(n.pos, endPos);
		n.cost = FLT_MAX;
		n.total = FLT_MAX;
		n.parent = -1;
		n.open = false;
		n.closed = false;
		nodeCount++;
	}

	const int endNodeIdx = 1;

	// Add nodes for each active link (two nodes per link: side A and side B).
	const int maxLinkSlots = m_multiNavMesh->getMaxLinks();
	for (int i = 0; i < maxLinkSlots && nodeCount + 1 < DT_MULTI_MAX_GRAPH_NODES; ++i)
	{
		const dtMultiNavMeshLink* link = m_multiNavMesh->getLink(i);
		if (!link)
			continue;

		// Side A
		{
			GraphNode& n = nodes[nodeCount];
			n.meshIndex = link->meshIndexA;
			n.linkIndex = i;
			n.linkSide = 0;
			dtVcopy(n.pos, link->posA);
			n.cost = FLT_MAX;
			n.total = FLT_MAX;
			n.parent = -1;
			n.open = false;
			n.closed = false;
			nodeCount++;
		}

		// Side B
		if (nodeCount < DT_MULTI_MAX_GRAPH_NODES)
		{
			GraphNode& n = nodes[nodeCount];
			n.meshIndex = link->meshIndexB;
			n.linkIndex = i;
			n.linkSide = 1;
			dtVcopy(n.pos, link->posB);
			n.cost = FLT_MAX;
			n.total = FLT_MAX;
			n.parent = -1;
			n.open = false;
			n.closed = false;
			nodeCount++;
		}
	}

	// A* on the graph.
	bool found = false;

	for (;;)
	{
		// Find the open node with lowest total cost.
		int bestIdx = -1;
		float bestTotal = FLT_MAX;
		for (int i = 0; i < nodeCount; ++i)
		{
			if (nodes[i].open && !nodes[i].closed && nodes[i].total < bestTotal)
			{
				bestTotal = nodes[i].total;
				bestIdx = i;
			}
		}

		if (bestIdx == -1)
			break; // No more open nodes.

		GraphNode& best = nodes[bestIdx];
		best.open = false;
		best.closed = true;

		// Check if we reached the end.
		if (bestIdx == endNodeIdx)
		{
			found = true;
			break;
		}

		// Expand neighbors: any node on the same mesh, or the other side of the same link.
		for (int i = 0; i < nodeCount; ++i)
		{
			if (i == bestIdx || nodes[i].closed)
				continue;

			float edgeCost = 0;
			bool isNeighbor = false;

			if (nodes[i].meshIndex == best.meshIndex)
			{
				// Same mesh: edge cost is estimated by world-space distance.
				edgeCost = dtVdist(best.pos, nodes[i].pos);
				isNeighbor = true;
			}
			else if (best.linkIndex >= 0 && best.linkIndex == nodes[i].linkIndex)
			{
				// Other side of the same link: edge cost is the link's cost.
				const dtMultiNavMeshLink* link = m_multiNavMesh->getLink(best.linkIndex);
				if (link)
				{
					// Check directionality.
					if (link->bidirectional ||
						(best.linkSide == 0 && nodes[i].linkSide == 1))
					{
						edgeCost = link->cost;
						isNeighbor = true;
					}
				}
			}

			if (!isNeighbor)
				continue;

			const float newCost = best.cost + edgeCost;
			if (newCost < nodes[i].cost)
			{
				nodes[i].cost = newCost;
				nodes[i].total = newCost + dtVdist(nodes[i].pos, endPos);
				nodes[i].parent = bestIdx;
				nodes[i].open = true;
			}
		}
	}

	if (!found)
		return 0;

	// Trace back the path and extract link indices.
	int tempPath[DT_MULTI_MAX_GRAPH_NODES];
	int tempCount = 0;
	int idx = endNodeIdx;
	while (idx != -1 && tempCount < DT_MULTI_MAX_GRAPH_NODES)
	{
		tempPath[tempCount++] = idx;
		idx = nodes[idx].parent;
	}

	// Reverse and extract links.
	int linkCount = 0;
	for (int i = tempCount - 1; i >= 0 && linkCount < maxLinks; --i)
	{
		int nodeIdx = tempPath[i];
		if (nodes[nodeIdx].linkIndex >= 0)
		{
			// Avoid duplicating a link (each link has two sides in the path).
			if (linkCount > 0 && linkPath[linkCount - 1] == nodes[nodeIdx].linkIndex)
				continue;

			linkPath[linkCount] = nodes[nodeIdx].linkIndex;
			linkSides[linkCount] = nodes[nodeIdx].linkSide;
			linkCount++;
		}
	}

	return linkCount;
}

dtStatus dtMultiNavMeshQuery::findPath(int startMesh, dtPolyRef startRef, const float* startPos,
									   int endMesh, dtPolyRef endRef, const float* endPos,
									   const dtQueryFilter* filter,
									   dtMultiNavMeshPathSegment* segments, int* segmentCount,
									   int maxSegments) const
{
	if (!segmentCount)
		return DT_FAILURE | DT_INVALID_PARAM;

	*segmentCount = 0;

	if (!m_multiNavMesh || !startPos || !endPos || !filter || !segments || maxSegments <= 0)
		return DT_FAILURE | DT_INVALID_PARAM;

	const dtNavMeshInstance* startInst = m_multiNavMesh->getInstance(startMesh);
	const dtNavMeshInstance* endInst = m_multiNavMesh->getInstance(endMesh);
	if (!startInst || !endInst)
		return DT_FAILURE | DT_INVALID_PARAM;

	// Same-mesh optimization: skip high-level search.
	if (startMesh == endMesh)
	{
		// Convert world positions to local.
		float localStart[3], localEnd[3];
		m_multiNavMesh->worldToLocal(startMesh, startPos, localStart);
		m_multiNavMesh->worldToLocal(endMesh, endPos, localEnd);

		dtMultiNavMeshPathSegment& seg = segments[0];
		seg.meshIndex = startMesh;
		dtVcopy(seg.startPos, startPos);
		dtVcopy(seg.endPos, endPos);
		seg.linkIndex = -1;

		dtStatus status = startInst->navQuery->findPath(
			startRef, endRef, localStart, localEnd, filter,
			seg.polys, &seg.polyCount, DT_MULTI_MAX_PATH_POLYS);

		if (dtStatusFailed(status))
			return status;

		*segmentCount = 1;
		return DT_SUCCESS;
	}

	// Two-level pathfinding.
	// Step 1: Find high-level path (sequence of links).
	int linkPath[DT_MULTI_MAX_PATH_SEGMENTS];
	int linkSides[DT_MULTI_MAX_PATH_SEGMENTS];
	int linkCount = findHighLevelPath(startMesh, startPos, endMesh, endPos,
									  linkPath, linkSides, DT_MULTI_MAX_PATH_SEGMENTS);

	if (linkCount == 0)
		return DT_FAILURE;

	// Step 2: Build local path segments.
	// Segment structure: start -> link[0].sideA, link[0].sideB -> link[1].sideA, ...., link[n].sideB -> end
	int segIdx = 0;

	// Current position and mesh as we walk through the links.
	int curMesh = startMesh;
	float curWorldPos[3];
	dtVcopy(curWorldPos, startPos);
	dtPolyRef curPolyRef = startRef;

	for (int i = 0; i < linkCount && segIdx < maxSegments; ++i)
	{
		const dtMultiNavMeshLink* link = m_multiNavMesh->getLink(linkPath[i]);
		if (!link)
			continue;

		// Determine which side we enter this link from.
		int entrySide;
		if (link->meshIndexA == curMesh)
			entrySide = 0;
		else if (link->meshIndexB == curMesh)
			entrySide = 1;
		else
			continue; // Shouldn't happen.

		const float* entryWorldPos = (entrySide == 0) ? link->posA : link->posB;
		const dtPolyRef entryPolyRef = (entrySide == 0) ? link->polyRefA : link->polyRefB;

		const float* exitWorldPos = (entrySide == 0) ? link->posB : link->posA;
		const dtPolyRef exitPolyRef = (entrySide == 0) ? link->polyRefB : link->polyRefA;
		const int exitMesh = (entrySide == 0) ? link->meshIndexB : link->meshIndexA;

		const dtNavMeshInstance* curInst = m_multiNavMesh->getInstance(curMesh);
		if (!curInst)
			continue;

		// Build local path within current mesh: curPos -> entry side of link.
		dtMultiNavMeshPathSegment& seg = segments[segIdx];
		seg.meshIndex = curMesh;
		dtVcopy(seg.startPos, curWorldPos);
		dtVcopy(seg.endPos, entryWorldPos);
		seg.linkIndex = linkPath[i];

		float localStart[3], localEnd[3];
		m_multiNavMesh->worldToLocal(curMesh, curWorldPos, localStart);
		m_multiNavMesh->worldToLocal(curMesh, entryWorldPos, localEnd);

		dtStatus status = curInst->navQuery->findPath(
			curPolyRef, entryPolyRef, localStart, localEnd, filter,
			seg.polys, &seg.polyCount, DT_MULTI_MAX_PATH_POLYS);

		if (dtStatusFailed(status))
		{
			// Still record the segment even if local path fails (partial result).
			seg.polyCount = 0;
		}

		segIdx++;

		// Move to the exit side of the link.
		curMesh = exitMesh;
		dtVcopy(curWorldPos, exitWorldPos);
		curPolyRef = exitPolyRef;
	}

	// Final segment: last link exit -> end position.
	if (segIdx < maxSegments && curMesh == endMesh)
	{
		const dtNavMeshInstance* curInst = m_multiNavMesh->getInstance(curMesh);
		if (curInst)
		{
			dtMultiNavMeshPathSegment& seg = segments[segIdx];
			seg.meshIndex = curMesh;
			dtVcopy(seg.startPos, curWorldPos);
			dtVcopy(seg.endPos, endPos);
			seg.linkIndex = -1;

			float localStart[3], localEnd[3];
			m_multiNavMesh->worldToLocal(curMesh, curWorldPos, localStart);
			m_multiNavMesh->worldToLocal(endMesh, endPos, localEnd);

			dtStatus status = curInst->navQuery->findPath(
				curPolyRef, endRef, localStart, localEnd, filter,
				seg.polys, &seg.polyCount, DT_MULTI_MAX_PATH_POLYS);

			if (dtStatusFailed(status))
				seg.polyCount = 0;

			segIdx++;
		}
	}

	*segmentCount = segIdx;
	return (*segmentCount > 0) ? DT_SUCCESS : DT_FAILURE;
}

dtStatus dtMultiNavMeshQuery::findNearestPoly(const float* worldPos, const float* halfExtents,
											  const dtQueryFilter* filter,
											  int* meshIndex, dtPolyRef* polyRef, float* nearestPt) const
{
	if (!m_multiNavMesh || !worldPos || !halfExtents || !filter || !meshIndex || !polyRef)
		return DT_FAILURE | DT_INVALID_PARAM;

	*meshIndex = -1;
	*polyRef = 0;

	float bestDist = FLT_MAX;

	for (int i = 0; i < m_multiNavMesh->getMaxNavMeshes(); ++i)
	{
		const dtNavMeshInstance* inst = m_multiNavMesh->getInstance(i);
		if (!inst)
			continue;

		// Transform world position to local.
		float localPos[3];
		m_multiNavMesh->worldToLocal(i, worldPos, localPos);

		// Transform half extents to local (approximate: use same extents since rotation
		// preserves distances, though axis alignment may differ).
		// For proper handling, we'd need to rotate the extents, but for simplicity
		// we use the max extent for all axes.
		float maxExtent = halfExtents[0];
		if (halfExtents[1] > maxExtent) maxExtent = halfExtents[1];
		if (halfExtents[2] > maxExtent) maxExtent = halfExtents[2];
		float localExtents[3] = { maxExtent, maxExtent, maxExtent };

		dtPolyRef ref = 0;
		float pt[3];
		dtStatus status = inst->navQuery->findNearestPoly(localPos, localExtents, filter, &ref, pt);

		if (dtStatusFailed(status) || ref == 0)
			continue;

		// Transform result back to world space and compare.
		float worldPt[3];
		m_multiNavMesh->localToWorld(i, pt, worldPt);

		const float dist = dtVdistSqr(worldPos, worldPt);
		if (dist < bestDist)
		{
			bestDist = dist;
			*meshIndex = i;
			*polyRef = ref;
			if (nearestPt)
				dtVcopy(nearestPt, worldPt);
		}
	}

	if (*polyRef == 0)
		return DT_FAILURE;

	return DT_SUCCESS;
}

dtMultiNavMeshQuery* dtAllocMultiNavMeshQuery()
{
	void* mem = dtAlloc(sizeof(dtMultiNavMeshQuery), DT_ALLOC_PERM);
	if (!mem)
		return 0;
	return new(mem) dtMultiNavMeshQuery;
}

void dtFreeMultiNavMeshQuery(dtMultiNavMeshQuery* query)
{
	if (!query)
		return;
	query->~dtMultiNavMeshQuery();
	dtFree(query);
}
