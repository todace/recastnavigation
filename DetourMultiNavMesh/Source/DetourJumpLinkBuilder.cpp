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

#include "DetourJumpLinkBuilder.h"
#include "DetourCommon.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"

#include <math.h>

namespace
{

/// Returns true if the given polygon edge has no connection at all:
/// neither an internal neighbour nor a link (external tile portal or
/// off-mesh connection) attached to that edge.
bool isBoundaryEdge(const dtMeshTile* tile, const dtPoly* poly, int edge)
{
	if (poly->neis[edge] == 0)
	{
		// No internal neighbour and not marked as a portal. Still might have
		// off-mesh links pointing out of this edge, but those don't make it
		// walkable-through, so treat as boundary.
		return true;
	}
	if (poly->neis[edge] & DT_EXT_LINK)
	{
		// Portal edge: boundary only when no link actually connects it
		// (e.g. single tile mesh, or neighbouring tile absent).
		for (unsigned int k = poly->firstLink; k != DT_NULL_LINK; k = tile->links[k].next)
		{
			if (tile->links[k].edge == edge)
				return false;
		}
		return true;
	}
	return false;
}

/// Returns true when an already-created link between the same instance pair
/// lies within minSpacing of the candidate world position.
bool hasNearbyLink(const dtMultiNavMesh* mnm, int meshA, int meshB,
				   const float* worldPos, float minSpacing)
{
	if (minSpacing <= 0)
		return false;
	const float minSpacingSqr = minSpacing * minSpacing;
	const int maxLinks = mnm->getMaxLinks();
	for (int i = 0; i < maxLinks; ++i)
	{
		const dtMultiNavMeshLink* link = mnm->getLink(i);
		if (!link)
			continue;
		const float* pos = 0;
		if (link->meshIndexA == meshA && link->meshIndexB == meshB)
			pos = link->posA;
		else if (link->meshIndexA == meshB && link->meshIndexB == meshA)
			pos = link->posB;
		else
			continue;
		if (dtVdistSqr(pos, worldPos) < minSpacingSqr)
			return true;
	}
	return false;
}

} // namespace

int dtBuildJumpLinks(dtMultiNavMesh* mnm,
					 const dtJumpLinkBuilderParams* params,
					 const dtQueryFilter* filter)
{
	if (!mnm || !params || !filter)
		return -1;
	if (params->maxJumpDistance <= 0 || params->probeSpacing <= 0)
		return -1;

	const float maxJumpSqr = params->maxJumpDistance * params->maxJumpDistance;
	const float ext = params->maxJumpDistance;
	const float halfExtents[3] = { ext, ext, ext };

	int linksCreated = 0;

	const int maxMeshes = mnm->getMaxNavMeshes();
	for (int ia = 0; ia < maxMeshes; ++ia)
	{
		const dtNavMeshInstance* instA = mnm->getInstance(ia);
		if (!instA || !instA->navMesh)
			continue;
		const dtNavMesh* meshA = instA->navMesh;

		for (int ti = 0; ti < meshA->getMaxTiles(); ++ti)
		{
			const dtMeshTile* tile = meshA->getTile(ti);
			if (!tile || !tile->header)
				continue;
			const dtPolyRef polyRefBase = meshA->getPolyRefBase(tile);

			for (int pi = 0; pi < tile->header->polyCount; ++pi)
			{
				const dtPoly* poly = &tile->polys[pi];
				if (poly->getType() != DT_POLYTYPE_GROUND)
					continue;
				const dtPolyRef refA = polyRefBase | (dtPolyRef)pi;

				for (int e = 0; e < (int)poly->vertCount; ++e)
				{
					if (!isBoundaryEdge(tile, poly, e))
						continue;

					const float* va = &tile->verts[poly->verts[e] * 3];
					const float* vb = &tile->verts[poly->verts[(e + 1) % poly->vertCount] * 3];
					const float edgeLen = dtVdist(va, vb);
					int probeCount = (int)ceilf(edgeLen / params->probeSpacing);
					if (probeCount < 1)
						probeCount = 1;

					for (int p = 0; p < probeCount; ++p)
					{
						const float t = ((float)p + 0.5f) / (float)probeCount;
						float probeLocal[3];
						dtVlerp(probeLocal, va, vb, t);
						float probeWorld[3];
						mnm->localToWorld(ia, probeLocal, probeWorld);

						// Search all other instances for a landing polygon.
						for (int ib = 0; ib < maxMeshes; ++ib)
						{
							if (ib == ia)
								continue;
							const dtNavMeshInstance* instB = mnm->getInstance(ib);
							if (!instB || !instB->navQuery)
								continue;

							const float upDot = dtVdot(instA->up, instB->up);
							if (upDot < params->minUpDot)
								continue;

							float probeLocalB[3];
							mnm->worldToLocal(ib, probeWorld, probeLocalB);

							dtPolyRef refB = 0;
							float nearestLocalB[3];
							const dtStatus st = instB->navQuery->findNearestPoly(
								probeLocalB, halfExtents, filter, &refB, nearestLocalB);
							if (dtStatusFailed(st) || refB == 0)
								continue;

							float landWorld[3];
							mnm->localToWorld(ib, nearestLocalB, landWorld);

							const float distSqr = dtVdistSqr(probeWorld, landWorld);
							if (distSqr > maxJumpSqr)
								continue;

							if (hasNearbyLink(mnm, ia, ib, probeWorld, params->minLinkSpacing))
								continue;

							const float dist = sqrtf(distSqr);
							const float cost = dist * params->costScale + params->baseCost;
							int linkIdx = -1;
							if (dtStatusSucceed(mnm->addLink(
									ia, refA, probeWorld,
									ib, refB, landWorld,
									cost, params->bidirectional, params->userId,
									&linkIdx)))
							{
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
