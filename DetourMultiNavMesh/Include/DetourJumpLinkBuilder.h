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

#ifndef DETOURJUMPLINKBUILDER_H
#define DETOURJUMPLINKBUILDER_H

#include "DetourMultiNavMesh.h"

/// Parameters controlling automatic jump-link generation between the
/// navmesh instances of a dtMultiNavMesh.
///
/// The builder walks the boundary edges of every navmesh instance (polygon
/// edges with no neighbour), probes points along those edges, and searches
/// the other instances for a nearby landing polygon. Valid candidates are
/// added as cross-mesh links, so surfaces built for different gravity
/// directions get connected exactly where their walkable areas meet or
/// where a short jump can bridge them.
struct dtJumpLinkBuilderParams
{
	/// Maximum world-space distance between a boundary-edge point and the
	/// landing point on the other mesh. [Limit: > 0]
	float maxJumpDistance;

	/// Spacing between probe points along a boundary edge. [Limit: > 0]
	float probeSpacing;

	/// Minimum world-space distance between two generated links connecting
	/// the same pair of meshes. Prevents flooding a shared edge with
	/// near-identical links. Set to 0 to keep every candidate.
	float minLinkSpacing;

	/// Minimum allowed dot product between the two instances' up vectors.
	/// -1 (default) allows any gravity relation (e.g. floor to ceiling),
	/// 0 allows up to perpendicular gravity, 0.99 restricts to nearly
	/// identical gravity.
	float minUpDot;

	/// Cost of traversing a generated link is
	/// world distance * costScale + baseCost.
	float costScale;
	float baseCost;

	/// Whether generated links can be traversed in both directions.
	bool bidirectional;

	/// User id assigned to every generated link (useful for filtering or
	/// removing auto-generated links later).
	unsigned int userId;

	dtJumpLinkBuilderParams()
		: maxJumpDistance(2.0f)
		, probeSpacing(1.0f)
		, minLinkSpacing(0.5f)
		, minUpDot(-1.0f)
		, costScale(1.0f)
		, baseCost(0.5f)
		, bidirectional(true)
		, userId(0)
	{
	}
};

/// Automatically creates jump-connection links between all pairs of navmesh
/// instances registered in the multi-navmesh.
///
/// Unlike centroid-based overlap detection, this walks the precise boundary
/// edges of each navmesh, so links are placed where an agent would actually
/// leave one surface and land on another.
///
///  @param[in,out]	multiNavMesh	The multi-navmesh to build links into.
///  @param[in]		params			Builder parameters.
///  @param[in]		filter			Query filter used when searching landing
///									polygons (may restrict areas/flags).
/// @return Number of links created, or -1 on invalid input.
int dtBuildJumpLinks(dtMultiNavMesh* multiNavMesh,
					 const dtJumpLinkBuilderParams* params,
					 const dtQueryFilter* filter);

#endif // DETOURJUMPLINKBUILDER_H
