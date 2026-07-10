#include "DetourJumpLinkBuilder.h"
#include "DetourMultiNavMesh.h"
#include "DetourMultiNavMeshQuery.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "catch2/catch_amalgamated.hpp"

#include <string.h>

namespace
{

// Flat 2-triangle quad navmesh spanning local (0,0,0)..(size,0,size).
struct QuadNavMesh
{
	dtNavMesh* navMesh;
	unsigned char* navData;
	int navDataSize;

	QuadNavMesh() : navMesh(0), navData(0), navDataSize(0) {}

	~QuadNavMesh()
	{
		if (navMesh)
			dtFreeNavMesh(navMesh);
	}

	bool create(float size)
	{
		const int sizeVx = (int)size;
		unsigned short verts[] = {
			0, 0, 0,
			(unsigned short)sizeVx, 0, 0,
			(unsigned short)sizeVx, 0, (unsigned short)sizeVx,
			0, 0, (unsigned short)sizeVx,
		};

		const int nvp = 6;
		unsigned short polys[2 * 2 * nvp];
		memset(polys, 0xff, sizeof(polys));
		polys[0] = 0; polys[1] = 1; polys[2] = 2;
		polys[nvp + 2] = 0x8001;
		polys[2 * nvp + 0] = 0; polys[2 * nvp + 1] = 2; polys[2 * nvp + 2] = 3;
		polys[2 * nvp + nvp + 0] = 0x8000;

		unsigned short polyFlags[] = { 1, 1 };
		unsigned char polyAreas[] = { 0, 0 };

		dtNavMeshCreateParams params;
		memset(&params, 0, sizeof(params));
		params.verts = verts;
		params.vertCount = 4;
		params.polys = polys;
		params.polyFlags = polyFlags;
		params.polyAreas = polyAreas;
		params.polyCount = 2;
		params.nvp = nvp;
		params.walkableHeight = 2.0f;
		params.walkableRadius = 0.5f;
		params.walkableClimb = 0.5f;
		params.cs = 1.0f;
		params.ch = 1.0f;
		params.bmin[0] = 0; params.bmin[1] = 0; params.bmin[2] = 0;
		params.bmax[0] = size; params.bmax[1] = 1.0f; params.bmax[2] = size;
		params.buildBvTree = true;

		if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
			return false;
		navMesh = dtAllocNavMesh();
		if (!navMesh)
			return false;
		return dtStatusSucceed(navMesh->init(navData, navDataSize, DT_TILE_FREE_DATA));
	}
};

void identityMatrix(float* m)
{
	memset(m, 0, sizeof(float) * 9);
	m[0] = 1.0f; m[4] = 1.0f; m[8] = 1.0f;
}

// Row-major 90-degree rotation around X: world = (lx, -lz, ly).
void rotX90Matrix(float* m)
{
	memset(m, 0, sizeof(float) * 9);
	m[0] = 1.0f;
	m[5] = -1.0f;	// row 1: (0, 0, -1)
	m[7] = 1.0f;	// row 2: (0, 1, 0)
}

struct MultiSetup
{
	dtMultiNavMesh* multi;

	MultiSetup() : multi(dtAllocMultiNavMesh())
	{
		dtMultiNavMeshParams params;
		params.maxNavMeshes = 8;
		params.maxLinks = 256;
		params.overlapDetectionRadius = 1.0f;
		REQUIRE(dtStatusSucceed(multi->init(&params)));
	}

	~MultiSetup() { dtFreeMultiNavMesh(multi); }
};

} // namespace

TEST_CASE("dtBuildJumpLinks connects adjacent coplanar meshes")
{
	QuadNavMesh meshA, meshB;
	REQUIRE(meshA.create(10.0f));
	REQUIRE(meshB.create(10.0f));

	MultiSetup s;
	float up[] = { 0, 1, 0 };
	float originA[] = { 0, 0, 0 };
	float originB[] = { 10, 0, 0 };	// B abuts A at world x = 10.
	float rot[9];
	identityMatrix(rot);

	int idxA = -1, idxB = -1;
	REQUIRE(dtStatusSucceed(s.multi->addNavMesh(meshA.navMesh, up, originA, rot, &idxA)));
	REQUIRE(dtStatusSucceed(s.multi->addNavMesh(meshB.navMesh, up, originB, rot, &idxB)));

	dtJumpLinkBuilderParams params;
	params.maxJumpDistance = 1.0f;
	params.probeSpacing = 2.0f;
	params.minLinkSpacing = 1.0f;
	dtQueryFilter filter;

	const int created = dtBuildJumpLinks(s.multi, &params, &filter);
	REQUIRE(created > 0);
	REQUIRE(s.multi->getLinkCount() == created);

	// All links must connect near the shared boundary at world x = 10.
	for (int i = 0; i < s.multi->getMaxLinks(); ++i)
	{
		const dtMultiNavMeshLink* link = s.multi->getLink(i);
		if (!link)
			continue;
		REQUIRE(link->posA[0] == Catch::Approx(10.0f).margin(params.maxJumpDistance));
		REQUIRE(link->posB[0] == Catch::Approx(10.0f).margin(params.maxJumpDistance));
	}

	// Pathfinding across the auto-generated links must work.
	dtMultiNavMeshQuery* query = dtAllocMultiNavMeshQuery();
	REQUIRE(dtStatusSucceed(query->init(s.multi)));

	const dtNavMeshInstance* instA = s.multi->getInstance(idxA);
	const dtNavMeshInstance* instB = s.multi->getInstance(idxB);
	const float halfExtents[] = { 5, 4, 5 };

	float startLocal[] = { 2, 0, 5 };
	float endLocal[] = { 8, 0, 5 };
	dtPolyRef startRef = 0, endRef = 0;
	float nearest[3];
	REQUIRE(dtStatusSucceed(instA->navQuery->findNearestPoly(startLocal, halfExtents, &filter, &startRef, nearest)));
	REQUIRE(dtStatusSucceed(instB->navQuery->findNearestPoly(endLocal, halfExtents, &filter, &endRef, nearest)));
	REQUIRE(startRef != 0);
	REQUIRE(endRef != 0);

	float startWorld[] = { 2, 0, 5 };
	float endWorld[] = { 18, 0, 5 };
	dtMultiNavMeshPathSegment segments[8];
	int segCount = 0;
	REQUIRE(dtStatusSucceed(query->findPath(idxA, startRef, startWorld,
											idxB, endRef, endWorld,
											&filter, segments, &segCount, 8)));
	REQUIRE(segCount >= 2);
	REQUIRE(segments[0].meshIndex == idxA);
	REQUIRE(segments[segCount - 1].meshIndex == idxB);

	dtFreeMultiNavMeshQuery(query);
}

TEST_CASE("dtBuildJumpLinks connects floor to perpendicular wall")
{
	QuadNavMesh floor, wall;
	REQUIRE(floor.create(10.0f));
	REQUIRE(wall.create(10.0f));

	MultiSetup s;
	float floorUp[] = { 0, 1, 0 };
	float floorOrigin[] = { 0, 0, 0 };
	float identity[9];
	identityMatrix(identity);

	// Wall: local plane rotated 90 deg around X. Local (x, 0, z) maps to
	// world (x, -z, 0), so the wall hangs below y=0 in the z=0 plane and its
	// top edge (local z = 0) coincides with the floor's z=0 boundary edge.
	float wallRot[9];
	rotX90Matrix(wallRot);
	float wallUp[] = { 0, 0, 1 };	// rotated local (0,1,0)
	float wallOrigin[] = { 0, 0, 0 };

	int idxFloor = -1, idxWall = -1;
	REQUIRE(dtStatusSucceed(s.multi->addNavMesh(floor.navMesh, floorUp, floorOrigin, identity, &idxFloor)));
	REQUIRE(dtStatusSucceed(s.multi->addNavMesh(wall.navMesh, wallUp, wallOrigin, wallRot, &idxWall)));

	dtQueryFilter filter;

	SECTION("Perpendicular gravity allowed by default")
	{
		dtJumpLinkBuilderParams params;
		params.maxJumpDistance = 1.0f;
		params.probeSpacing = 2.0f;
		params.minLinkSpacing = 1.0f;

		const int created = dtBuildJumpLinks(s.multi, &params, &filter);
		REQUIRE(created > 0);

		// Links must sit near the shared edge (world y=0, z=0).
		for (int i = 0; i < s.multi->getMaxLinks(); ++i)
		{
			const dtMultiNavMeshLink* link = s.multi->getLink(i);
			if (!link)
				continue;
			REQUIRE(fabsf(link->posA[1]) <= params.maxJumpDistance + 0.01f);
			REQUIRE(fabsf(link->posA[2]) <= params.maxJumpDistance + 0.01f);
		}
	}

	SECTION("minUpDot rejects perpendicular gravity")
	{
		dtJumpLinkBuilderParams params;
		params.maxJumpDistance = 1.0f;
		params.probeSpacing = 2.0f;
		params.minUpDot = 0.9f;	// Requires nearly parallel up vectors.

		const int created = dtBuildJumpLinks(s.multi, &params, &filter);
		REQUIRE(created == 0);
	}
}

TEST_CASE("dtBuildJumpLinks respects maxJumpDistance")
{
	QuadNavMesh meshA, meshB;
	REQUIRE(meshA.create(10.0f));
	REQUIRE(meshB.create(10.0f));

	MultiSetup s;
	float up[] = { 0, 1, 0 };
	float originA[] = { 0, 0, 0 };
	float originB[] = { 100, 0, 0 };	// Far away.
	float rot[9];
	identityMatrix(rot);

	int idxA = -1, idxB = -1;
	REQUIRE(dtStatusSucceed(s.multi->addNavMesh(meshA.navMesh, up, originA, rot, &idxA)));
	REQUIRE(dtStatusSucceed(s.multi->addNavMesh(meshB.navMesh, up, originB, rot, &idxB)));

	dtJumpLinkBuilderParams params;
	params.maxJumpDistance = 2.0f;
	params.probeSpacing = 1.0f;
	dtQueryFilter filter;

	REQUIRE(dtBuildJumpLinks(s.multi, &params, &filter) == 0);
	REQUIRE(s.multi->getLinkCount() == 0);
}

TEST_CASE("dtBuildJumpLinks minLinkSpacing limits link density")
{
	QuadNavMesh meshA, meshB;
	REQUIRE(meshA.create(10.0f));
	REQUIRE(meshB.create(10.0f));

	float up[] = { 0, 1, 0 };
	float originA[] = { 0, 0, 0 };
	float originB[] = { 10, 0, 0 };
	float rot[9];
	identityMatrix(rot);
	dtQueryFilter filter;

	int denseCount = 0;
	{
		MultiSetup s;
		int idxA, idxB;
		REQUIRE(dtStatusSucceed(s.multi->addNavMesh(meshA.navMesh, up, originA, rot, &idxA)));
		REQUIRE(dtStatusSucceed(s.multi->addNavMesh(meshB.navMesh, up, originB, rot, &idxB)));
		dtJumpLinkBuilderParams params;
		params.maxJumpDistance = 1.0f;
		params.probeSpacing = 0.5f;
		params.minLinkSpacing = 0;	// Keep everything.
		denseCount = dtBuildJumpLinks(s.multi, &params, &filter);
		REQUIRE(denseCount > 0);
	}

	int sparseCount = 0;
	{
		MultiSetup s;
		int idxA, idxB;
		REQUIRE(dtStatusSucceed(s.multi->addNavMesh(meshA.navMesh, up, originA, rot, &idxA)));
		REQUIRE(dtStatusSucceed(s.multi->addNavMesh(meshB.navMesh, up, originB, rot, &idxB)));
		dtJumpLinkBuilderParams params;
		params.maxJumpDistance = 1.0f;
		params.probeSpacing = 0.5f;
		params.minLinkSpacing = 5.0f;	// Sparse.
		sparseCount = dtBuildJumpLinks(s.multi, &params, &filter);
		REQUIRE(sparseCount > 0);
	}

	REQUIRE(sparseCount < denseCount);
}

TEST_CASE("dtBuildJumpLinks validates input")
{
	dtJumpLinkBuilderParams params;
	dtQueryFilter filter;
	REQUIRE(dtBuildJumpLinks(0, &params, &filter) == -1);

	MultiSetup s;
	REQUIRE(dtBuildJumpLinks(s.multi, 0, &filter) == -1);
	REQUIRE(dtBuildJumpLinks(s.multi, &params, 0) == -1);

	params.maxJumpDistance = 0;
	REQUIRE(dtBuildJumpLinks(s.multi, &params, &filter) == -1);
}
