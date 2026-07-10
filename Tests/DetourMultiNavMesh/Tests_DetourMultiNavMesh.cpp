#include "DetourMultiNavMesh.h"
#include "DetourMultiNavMeshQuery.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "DetourCommon.h"
#include "catch2/catch_amalgamated.hpp"

#include <string.h>
#include <math.h>

namespace
{

// Helper: Creates a simple single-tile navmesh with the given polygon layout.
// The mesh is a flat quad made of 2 triangles on the XZ plane at y=0.
// Vertices: (0,0,0), (size,0,0), (size,0,size), (0,0,size)
// Polys: 2 triangles covering the quad.
struct SimpleNavMesh
{
	dtNavMesh* navMesh;
	unsigned char* navData;
	int navDataSize;

	SimpleNavMesh() : navMesh(0), navData(0), navDataSize(0) {}

	~SimpleNavMesh()
	{
		if (navMesh)
		{
			dtFreeNavMesh(navMesh);
			navMesh = 0;
		}
		// navData is owned by navMesh after addTile with DT_TILE_FREE_DATA
	}

	bool create(float size)
	{
		// The verts are in voxel coordinates (unsigned short).
		// We use cs=1.0, ch=1.0 so voxel coords == world coords.
		const float cs = 1.0f;
		const float ch = 1.0f;
		const int sizeVx = (int)size;

		unsigned short verts[] = {
			0,			0, 0,
			(unsigned short)sizeVx, 0, 0,
			(unsigned short)sizeVx, 0, (unsigned short)sizeVx,
			0,			0, (unsigned short)sizeVx,
		};

		// Two quads (each using nvp=6, padded with 0xffff for unused verts and neighbors).
		// Poly 0: vertices 0,1,2 (triangle)
		// Poly 1: vertices 0,2,3 (triangle)
		// nvp = 6, so each poly is 2*6 = 12 unsigned shorts (6 verts + 6 neighbors).
		const int nvp = 6;
		unsigned short polys[2 * 2 * nvp];
		memset(polys, 0xff, sizeof(polys));

		// Neighbor encoding for dtNavMeshCreateParams: 0xffff = border,
		// plain value = internal neighbor poly index (the builder adds +1).

		// Poly 0: verts 0,1,2. Edge 2 (diagonal) connects to poly 1.
		polys[0] = 0; polys[1] = 1; polys[2] = 2;
		polys[nvp + 0] = 0xffff; polys[nvp + 1] = 0xffff; polys[nvp + 2] = 1;

		// Poly 1: verts 0,2,3. Edge 0 (diagonal) connects to poly 0.
		polys[2 * nvp + 0] = 0; polys[2 * nvp + 1] = 2; polys[2 * nvp + 2] = 3;
		polys[2 * nvp + nvp + 0] = 0;
		polys[2 * nvp + nvp + 1] = 0xffff;
		polys[2 * nvp + nvp + 2] = 0xffff;

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
		params.cs = cs;
		params.ch = ch;
		params.bmin[0] = 0; params.bmin[1] = 0; params.bmin[2] = 0;
		params.bmax[0] = size; params.bmax[1] = 1.0f; params.bmax[2] = size;
		params.buildBvTree = true;

		if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
			return false;

		navMesh = dtAllocNavMesh();
		if (!navMesh)
			return false;

		dtStatus status = navMesh->init(navData, navDataSize, DT_TILE_FREE_DATA);
		if (dtStatusFailed(status))
			return false;

		return true;
	}
};

// Identity rotation matrix (no rotation).
void identityMatrix(float* m)
{
	memset(m, 0, sizeof(float) * 9);
	m[0] = 1.0f; m[4] = 1.0f; m[8] = 1.0f;
}

// Rotation matrix that rotates 90 degrees around X axis (floor -> wall on Z).
// Maps: x->x, y->-z, z->y
void rotX90Matrix(float* m)
{
	memset(m, 0, sizeof(float) * 9);
	m[0] = 1.0f;				// row 0: (1, 0, 0)
	m[3+1] = 0.0f; m[3+2] = -1.0f;	// row 1: (0, 0, -1)
	m[6+1] = 1.0f; m[6+2] = 0.0f;	// row 2: (0, 1, 0)
}

// Rotation matrix that rotates 180 degrees around X axis (floor -> ceiling).
// Maps: x->x, y->-y, z->-z
void rotX180Matrix(float* m)
{
	memset(m, 0, sizeof(float) * 9);
	m[0] = 1.0f;
	m[4] = -1.0f;
	m[8] = -1.0f;
}

} // namespace

TEST_CASE("dtMultiNavMesh initialization")
{
	SECTION("Init with valid params succeeds")
	{
		dtMultiNavMesh* multi = dtAllocMultiNavMesh();
		REQUIRE(multi != 0);

		dtMultiNavMeshParams params;
		params.maxNavMeshes = 8;
		params.maxLinks = 64;
		params.overlapDetectionRadius = 1.0f;

		dtStatus status = multi->init(&params);
		REQUIRE(dtStatusSucceed(status));
		REQUIRE(multi->getNavMeshCount() == 0);
		REQUIRE(multi->getLinkCount() == 0);

		dtFreeMultiNavMesh(multi);
	}

	SECTION("Init with null params fails")
	{
		dtMultiNavMesh* multi = dtAllocMultiNavMesh();
		dtStatus status = multi->init(0);
		REQUIRE(dtStatusFailed(status));
		dtFreeMultiNavMesh(multi);
	}

	SECTION("Init with invalid params fails")
	{
		dtMultiNavMesh* multi = dtAllocMultiNavMesh();
		dtMultiNavMeshParams params;
		params.maxNavMeshes = 0;
		params.maxLinks = 10;
		params.overlapDetectionRadius = 1.0f;
		dtStatus status = multi->init(&params);
		REQUIRE(dtStatusFailed(status));
		dtFreeMultiNavMesh(multi);
	}
}

TEST_CASE("dtMultiNavMesh add and remove navmeshes")
{
	SimpleNavMesh meshA, meshB;
	REQUIRE(meshA.create(10.0f));
	REQUIRE(meshB.create(10.0f));

	dtMultiNavMesh* multi = dtAllocMultiNavMesh();
	dtMultiNavMeshParams params;
	params.maxNavMeshes = 4;
	params.maxLinks = 16;
	params.overlapDetectionRadius = 1.0f;
	REQUIRE(dtStatusSucceed(multi->init(&params)));

	SECTION("Add navmesh succeeds")
	{
		float up[] = {0, 1, 0};
		float origin[] = {0, 0, 0};
		float rot[9];
		identityMatrix(rot);

		int idx = -1;
		dtStatus status = multi->addNavMesh(meshA.navMesh, up, origin, rot, &idx);
		REQUIRE(dtStatusSucceed(status));
		REQUIRE(idx == 0);
		REQUIRE(multi->getNavMeshCount() == 1);

		const dtNavMeshInstance* inst = multi->getInstance(idx);
		REQUIRE(inst != 0);
		REQUIRE(inst->navMesh == meshA.navMesh);
		REQUIRE(inst->active == true);
	}

	SECTION("Add multiple navmeshes")
	{
		float up[] = {0, 1, 0};
		float origin1[] = {0, 0, 0};
		float origin2[] = {10, 0, 0};
		float rot[9];
		identityMatrix(rot);

		int idx1 = -1, idx2 = -1;
		REQUIRE(dtStatusSucceed(multi->addNavMesh(meshA.navMesh, up, origin1, rot, &idx1)));
		REQUIRE(dtStatusSucceed(multi->addNavMesh(meshB.navMesh, up, origin2, rot, &idx2)));
		REQUIRE(idx1 != idx2);
		REQUIRE(multi->getNavMeshCount() == 2);
	}

	SECTION("Remove navmesh")
	{
		float up[] = {0, 1, 0};
		float origin[] = {0, 0, 0};
		float rot[9];
		identityMatrix(rot);

		int idx = -1;
		REQUIRE(dtStatusSucceed(multi->addNavMesh(meshA.navMesh, up, origin, rot, &idx)));
		REQUIRE(multi->getNavMeshCount() == 1);

		REQUIRE(dtStatusSucceed(multi->removeNavMesh(idx)));
		REQUIRE(multi->getNavMeshCount() == 0);
		REQUIRE(multi->getInstance(idx) == 0);
	}

	SECTION("Remove navmesh cleans up associated links")
	{
		float up[] = {0, 1, 0};
		float origin1[] = {0, 0, 0};
		float origin2[] = {10, 0, 0};
		float rot[9];
		identityMatrix(rot);

		int idx1 = -1, idx2 = -1;
		REQUIRE(dtStatusSucceed(multi->addNavMesh(meshA.navMesh, up, origin1, rot, &idx1)));
		REQUIRE(dtStatusSucceed(multi->addNavMesh(meshB.navMesh, up, origin2, rot, &idx2)));

		float posA[] = {5, 0, 5};
		float posB[] = {15, 0, 5};
		int linkIdx = -1;
		REQUIRE(dtStatusSucceed(multi->addLink(idx1, 1, posA, idx2, 1, posB, 1.0f, true, 0, &linkIdx)));
		REQUIRE(multi->getLinkCount() == 1);

		REQUIRE(dtStatusSucceed(multi->removeNavMesh(idx1)));
		REQUIRE(multi->getLinkCount() == 0);
	}

	dtFreeMultiNavMesh(multi);
}

TEST_CASE("dtMultiNavMesh add and remove links")
{
	SimpleNavMesh meshA, meshB;
	REQUIRE(meshA.create(10.0f));
	REQUIRE(meshB.create(10.0f));

	dtMultiNavMesh* multi = dtAllocMultiNavMesh();
	dtMultiNavMeshParams params;
	params.maxNavMeshes = 4;
	params.maxLinks = 16;
	params.overlapDetectionRadius = 1.0f;
	REQUIRE(dtStatusSucceed(multi->init(&params)));

	float up[] = {0, 1, 0};
	float origin1[] = {0, 0, 0};
	float origin2[] = {20, 0, 0};
	float rot[9];
	identityMatrix(rot);

	int idx1 = -1, idx2 = -1;
	REQUIRE(dtStatusSucceed(multi->addNavMesh(meshA.navMesh, up, origin1, rot, &idx1)));
	REQUIRE(dtStatusSucceed(multi->addNavMesh(meshB.navMesh, up, origin2, rot, &idx2)));

	SECTION("Add link succeeds")
	{
		float posA[] = {5, 0, 5};
		float posB[] = {25, 0, 5};
		int linkIdx = -1;
		dtStatus status = multi->addLink(idx1, 1, posA, idx2, 1, posB, 2.5f, true, 42, &linkIdx);
		REQUIRE(dtStatusSucceed(status));
		REQUIRE(linkIdx >= 0);
		REQUIRE(multi->getLinkCount() == 1);

		const dtMultiNavMeshLink* link = multi->getLink(linkIdx);
		REQUIRE(link != 0);
		REQUIRE(link->meshIndexA == idx1);
		REQUIRE(link->meshIndexB == idx2);
		REQUIRE(link->cost == Catch::Approx(2.5f));
		REQUIRE(link->bidirectional == true);
		REQUIRE(link->userId == 42);
	}

	SECTION("Remove link succeeds")
	{
		float posA[] = {5, 0, 5};
		float posB[] = {25, 0, 5};
		int linkIdx = -1;
		REQUIRE(dtStatusSucceed(multi->addLink(idx1, 1, posA, idx2, 1, posB, 1.0f, true, 0, &linkIdx)));
		REQUIRE(multi->getLinkCount() == 1);

		REQUIRE(dtStatusSucceed(multi->removeLink(linkIdx)));
		REQUIRE(multi->getLinkCount() == 0);
	}

	SECTION("Add link with invalid mesh index fails")
	{
		float posA[] = {5, 0, 5};
		float posB[] = {25, 0, 5};
		dtStatus status = multi->addLink(99, 1, posA, idx2, 1, posB, 1.0f, true, 0, 0);
		REQUIRE(dtStatusFailed(status));
	}

	dtFreeMultiNavMesh(multi);
}

TEST_CASE("dtMultiNavMesh coordinate transforms")
{
	dtMultiNavMesh* multi = dtAllocMultiNavMesh();
	dtMultiNavMeshParams params;
	params.maxNavMeshes = 8;
	params.maxLinks = 16;
	params.overlapDetectionRadius = 1.0f;
	REQUIRE(dtStatusSucceed(multi->init(&params)));

	SimpleNavMesh mesh;
	REQUIRE(mesh.create(10.0f));

	SECTION("Identity transform (floor)")
	{
		float up[] = {0, 1, 0};
		float origin[] = {100, 200, 300};
		float rot[9];
		identityMatrix(rot);

		int idx = -1;
		REQUIRE(dtStatusSucceed(multi->addNavMesh(mesh.navMesh, up, origin, rot, &idx)));

		float local[] = {5, 0, 5};
		float world[3];
		REQUIRE(dtStatusSucceed(multi->localToWorld(idx, local, world)));
		REQUIRE(world[0] == Catch::Approx(105));
		REQUIRE(world[1] == Catch::Approx(200));
		REQUIRE(world[2] == Catch::Approx(305));

		float backLocal[3];
		REQUIRE(dtStatusSucceed(multi->worldToLocal(idx, world, backLocal)));
		REQUIRE(backLocal[0] == Catch::Approx(5));
		REQUIRE(backLocal[1] == Catch::Approx(0));
		REQUIRE(backLocal[2] == Catch::Approx(5));
	}

	SECTION("90-degree X rotation (wall)")
	{
		float up[] = {0, 0, 1};
		float origin[] = {0, 0, 0};
		float rot[9];
		rotX90Matrix(rot);

		int idx = -1;
		REQUIRE(dtStatusSucceed(multi->addNavMesh(mesh.navMesh, up, origin, rot, &idx)));

		// Row-major rotation: row0=(1,0,0), row1=(0,0,-1), row2=(0,1,0)
		// local (1,2,3) -> world (1, -3, 2)
		float local[] = {1, 2, 3};
		float world[3];
		REQUIRE(dtStatusSucceed(multi->localToWorld(idx, local, world)));
		REQUIRE(world[0] == Catch::Approx(1));
		REQUIRE(world[1] == Catch::Approx(-3));
		REQUIRE(world[2] == Catch::Approx(2));

		float backLocal[3];
		REQUIRE(dtStatusSucceed(multi->worldToLocal(idx, world, backLocal)));
		REQUIRE(backLocal[0] == Catch::Approx(1));
		REQUIRE(backLocal[1] == Catch::Approx(2));
		REQUIRE(backLocal[2] == Catch::Approx(3));
	}

	SECTION("180-degree X rotation (ceiling)")
	{
		float up[] = {0, -1, 0};
		float origin[] = {0, 10, 0};
		float rot[9];
		rotX180Matrix(rot);

		int idx = -1;
		REQUIRE(dtStatusSucceed(multi->addNavMesh(mesh.navMesh, up, origin, rot, &idx)));

		float local[] = {5, 0, 5};
		float world[3];
		REQUIRE(dtStatusSucceed(multi->localToWorld(idx, local, world)));
		REQUIRE(world[0] == Catch::Approx(5));
		REQUIRE(world[1] == Catch::Approx(10));
		REQUIRE(world[2] == Catch::Approx(-5));

		float backLocal[3];
		REQUIRE(dtStatusSucceed(multi->worldToLocal(idx, world, backLocal)));
		REQUIRE(backLocal[0] == Catch::Approx(5));
		REQUIRE(backLocal[1] == Catch::Approx(0));
		REQUIRE(backLocal[2] == Catch::Approx(5));
	}

	dtFreeMultiNavMesh(multi);
}

TEST_CASE("dtMultiNavMesh custom vertex data")
{
	SimpleNavMesh mesh;
	REQUIRE(mesh.create(10.0f));

	dtMultiNavMesh* multi = dtAllocMultiNavMesh();
	dtMultiNavMeshParams params;
	params.maxNavMeshes = 4;
	params.maxLinks = 16;
	params.overlapDetectionRadius = 1.0f;
	REQUIRE(dtStatusSucceed(multi->init(&params)));

	float up[] = {0, 1, 0};
	float origin[] = {0, 0, 0};
	float rot[9];
	identityMatrix(rot);

	// Custom vertex data: 4 vertices, 3 floats each (e.g., velocity).
	float customData[] = {
		1.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f,
		0.0f, 0.0f, 1.0f,
		1.0f, 1.0f, 1.0f,
	};

	int idx = -1;
	dtStatus status = multi->addCustomNavMesh(mesh.navMesh, customData, 3, 4,
											  up, origin, rot, &idx);
	REQUIRE(dtStatusSucceed(status));
	REQUIRE(idx >= 0);

	const dtNavMeshInstance* inst = multi->getInstance(idx);
	REQUIRE(inst != 0);
	REQUIRE(inst->customVertexData == customData);
	REQUIRE(inst->customVertexStride == 3);
	REQUIRE(inst->customVertexCount == 4);

	dtFreeMultiNavMesh(multi);
}

TEST_CASE("dtMultiNavMeshQuery initialization")
{
	dtMultiNavMesh* multi = dtAllocMultiNavMesh();
	dtMultiNavMeshParams params;
	params.maxNavMeshes = 4;
	params.maxLinks = 16;
	params.overlapDetectionRadius = 1.0f;
	REQUIRE(dtStatusSucceed(multi->init(&params)));

	SECTION("Init with valid multi-navmesh succeeds")
	{
		dtMultiNavMeshQuery* query = dtAllocMultiNavMeshQuery();
		REQUIRE(query != 0);
		REQUIRE(dtStatusSucceed(query->init(multi)));
		dtFreeMultiNavMeshQuery(query);
	}

	SECTION("Init with null fails")
	{
		dtMultiNavMeshQuery* query = dtAllocMultiNavMeshQuery();
		REQUIRE(dtStatusFailed(query->init(0)));
		dtFreeMultiNavMeshQuery(query);
	}

	dtFreeMultiNavMesh(multi);
}

TEST_CASE("dtMultiNavMeshQuery findPath same mesh")
{
	SimpleNavMesh mesh;
	REQUIRE(mesh.create(10.0f));

	dtMultiNavMesh* multi = dtAllocMultiNavMesh();
	dtMultiNavMeshParams params;
	params.maxNavMeshes = 4;
	params.maxLinks = 16;
	params.overlapDetectionRadius = 1.0f;
	REQUIRE(dtStatusSucceed(multi->init(&params)));

	float up[] = {0, 1, 0};
	float origin[] = {0, 0, 0};
	float rot[9];
	identityMatrix(rot);

	int idx = -1;
	REQUIRE(dtStatusSucceed(multi->addNavMesh(mesh.navMesh, up, origin, rot, &idx)));

	dtMultiNavMeshQuery* query = dtAllocMultiNavMeshQuery();
	REQUIRE(dtStatusSucceed(query->init(multi)));

	// Find nearest poly for start and end positions.
	const dtNavMeshInstance* inst = multi->getInstance(idx);
	REQUIRE(inst != 0);

	float startLocal[] = {2, 0, 2};
	float endLocal[] = {8, 0, 8};
	float halfExtents[] = {2, 4, 2};
	dtQueryFilter filter;

	dtPolyRef startRef = 0;
	float nearestStart[3];
	REQUIRE(dtStatusSucceed(inst->navQuery->findNearestPoly(startLocal, halfExtents, &filter, &startRef, nearestStart)));
	REQUIRE(startRef != 0);

	dtPolyRef endRef = 0;
	float nearestEnd[3];
	REQUIRE(dtStatusSucceed(inst->navQuery->findNearestPoly(endLocal, halfExtents, &filter, &endRef, nearestEnd)));
	REQUIRE(endRef != 0);

	// Find path (same mesh, identity transform so world == local).
	dtMultiNavMeshPathSegment segments[4];
	int segCount = 0;
	dtStatus status = query->findPath(idx, startRef, startLocal, idx, endRef, endLocal,
									  &filter, segments, &segCount, 4);
	REQUIRE(dtStatusSucceed(status));
	REQUIRE(segCount == 1);
	REQUIRE(segments[0].meshIndex == idx);
	REQUIRE(segments[0].polyCount > 0);
	REQUIRE(segments[0].linkIndex == -1);

	dtFreeMultiNavMeshQuery(query);
	dtFreeMultiNavMesh(multi);
}

TEST_CASE("dtMultiNavMeshQuery findPath across two meshes")
{
	SimpleNavMesh meshA, meshB;
	REQUIRE(meshA.create(10.0f));
	REQUIRE(meshB.create(10.0f));

	dtMultiNavMesh* multi = dtAllocMultiNavMesh();
	dtMultiNavMeshParams params;
	params.maxNavMeshes = 4;
	params.maxLinks = 16;
	params.overlapDetectionRadius = 1.0f;
	REQUIRE(dtStatusSucceed(multi->init(&params)));

	float up[] = {0, 1, 0};
	float originA[] = {0, 0, 0};
	float originB[] = {10, 0, 0};  // Mesh B is adjacent to mesh A.
	float rot[9];
	identityMatrix(rot);

	int idxA = -1, idxB = -1;
	REQUIRE(dtStatusSucceed(multi->addNavMesh(meshA.navMesh, up, originA, rot, &idxA)));
	REQUIRE(dtStatusSucceed(multi->addNavMesh(meshB.navMesh, up, originB, rot, &idxB)));

	// Find polys near the connection point on each mesh.
	const dtNavMeshInstance* instA = multi->getInstance(idxA);
	const dtNavMeshInstance* instB = multi->getInstance(idxB);
	REQUIRE(instA != 0);
	REQUIRE(instB != 0);

	float halfExtents[] = {5, 4, 5};
	dtQueryFilter filter;

	// Find connection polys: mesh A near (9,0,5) local, mesh B near (1,0,5) local.
	float connLocalA[] = {9, 0, 5};
	float connLocalB[] = {1, 0, 5};
	dtPolyRef connRefA = 0, connRefB = 0;
	float nearPtA[3], nearPtB[3];
	REQUIRE(dtStatusSucceed(instA->navQuery->findNearestPoly(connLocalA, halfExtents, &filter, &connRefA, nearPtA)));
	REQUIRE(dtStatusSucceed(instB->navQuery->findNearestPoly(connLocalB, halfExtents, &filter, &connRefB, nearPtB)));
	REQUIRE(connRefA != 0);
	REQUIRE(connRefB != 0);

	// Add link at the connection point (world space).
	float connWorldA[] = {9, 0, 5};  // originA + connLocalA
	float connWorldB[] = {11, 0, 5}; // originB + connLocalB
	int linkIdx = -1;
	REQUIRE(dtStatusSucceed(multi->addLink(idxA, connRefA, connWorldA,
										   idxB, connRefB, connWorldB,
										   2.0f, true, 0, &linkIdx)));

	// Find start and end polys.
	float startLocal[] = {2, 0, 2};
	float endLocal[] = {8, 0, 8};
	dtPolyRef startRef = 0, endRef = 0;
	float nearStart[3], nearEnd[3];
	REQUIRE(dtStatusSucceed(instA->navQuery->findNearestPoly(startLocal, halfExtents, &filter, &startRef, nearStart)));
	REQUIRE(dtStatusSucceed(instB->navQuery->findNearestPoly(endLocal, halfExtents, &filter, &endRef, nearEnd)));
	REQUIRE(startRef != 0);
	REQUIRE(endRef != 0);

	// Start in world space: originA + startLocal = (2, 0, 2)
	// End in world space: originB + endLocal = (18, 0, 8)
	float startWorld[] = {2, 0, 2};
	float endWorld[] = {18, 0, 8};

	dtMultiNavMeshQuery* query = dtAllocMultiNavMeshQuery();
	REQUIRE(dtStatusSucceed(query->init(multi)));

	dtMultiNavMeshPathSegment segments[8];
	int segCount = 0;
	dtStatus status = query->findPath(idxA, startRef, startWorld, idxB, endRef, endWorld,
									  &filter, segments, &segCount, 8);
	REQUIRE(dtStatusSucceed(status));
	REQUIRE(segCount >= 2);  // At least: meshA segment + meshB segment
	REQUIRE(segments[0].meshIndex == idxA);
	REQUIRE(segments[segCount - 1].meshIndex == idxB);

	dtFreeMultiNavMeshQuery(query);
	dtFreeMultiNavMesh(multi);
}

TEST_CASE("dtMultiNavMeshQuery findPath no route between meshes")
{
	SimpleNavMesh meshA, meshB;
	REQUIRE(meshA.create(10.0f));
	REQUIRE(meshB.create(10.0f));

	dtMultiNavMesh* multi = dtAllocMultiNavMesh();
	dtMultiNavMeshParams params;
	params.maxNavMeshes = 4;
	params.maxLinks = 16;
	params.overlapDetectionRadius = 1.0f;
	REQUIRE(dtStatusSucceed(multi->init(&params)));

	float up[] = {0, 1, 0};
	float originA[] = {0, 0, 0};
	float originB[] = {100, 0, 0};
	float rot[9];
	identityMatrix(rot);

	int idxA = -1, idxB = -1;
	REQUIRE(dtStatusSucceed(multi->addNavMesh(meshA.navMesh, up, originA, rot, &idxA)));
	REQUIRE(dtStatusSucceed(multi->addNavMesh(meshB.navMesh, up, originB, rot, &idxB)));

	// No links between the meshes.
	const dtNavMeshInstance* instA = multi->getInstance(idxA);
	const dtNavMeshInstance* instB = multi->getInstance(idxB);

	float halfExtents[] = {5, 4, 5};
	dtQueryFilter filter;

	float startLocal[] = {5, 0, 5};
	float endLocal[] = {5, 0, 5};
	dtPolyRef startRef = 0, endRef = 0;
	float nearPt[3];
	REQUIRE(dtStatusSucceed(instA->navQuery->findNearestPoly(startLocal, halfExtents, &filter, &startRef, nearPt)));
	REQUIRE(dtStatusSucceed(instB->navQuery->findNearestPoly(endLocal, halfExtents, &filter, &endRef, nearPt)));

	float startWorld[] = {5, 0, 5};
	float endWorld[] = {105, 0, 5};

	dtMultiNavMeshQuery* query = dtAllocMultiNavMeshQuery();
	REQUIRE(dtStatusSucceed(query->init(multi)));

	dtMultiNavMeshPathSegment segments[8];
	int segCount = 0;
	dtStatus status = query->findPath(idxA, startRef, startWorld, idxB, endRef, endWorld,
									  &filter, segments, &segCount, 8);
	REQUIRE(dtStatusFailed(status));
	REQUIRE(segCount == 0);

	dtFreeMultiNavMeshQuery(query);
	dtFreeMultiNavMesh(multi);
}

TEST_CASE("dtMultiNavMeshQuery findNearestPoly across meshes")
{
	SimpleNavMesh meshA, meshB;
	REQUIRE(meshA.create(10.0f));
	REQUIRE(meshB.create(10.0f));

	dtMultiNavMesh* multi = dtAllocMultiNavMesh();
	dtMultiNavMeshParams params;
	params.maxNavMeshes = 4;
	params.maxLinks = 16;
	params.overlapDetectionRadius = 1.0f;
	REQUIRE(dtStatusSucceed(multi->init(&params)));

	float up[] = {0, 1, 0};
	float originA[] = {0, 0, 0};
	float originB[] = {100, 0, 0};
	float rot[9];
	identityMatrix(rot);

	int idxA = -1, idxB = -1;
	REQUIRE(dtStatusSucceed(multi->addNavMesh(meshA.navMesh, up, originA, rot, &idxA)));
	REQUIRE(dtStatusSucceed(multi->addNavMesh(meshB.navMesh, up, originB, rot, &idxB)));

	dtMultiNavMeshQuery* query = dtAllocMultiNavMeshQuery();
	REQUIRE(dtStatusSucceed(query->init(multi)));

	dtQueryFilter filter;

	SECTION("Finds poly on mesh A")
	{
		float worldPos[] = {5, 0, 5};
		float halfExtents[] = {5, 4, 5};
		int meshIdx = -1;
		dtPolyRef polyRef = 0;
		float nearestPt[3];

		dtStatus status = query->findNearestPoly(worldPos, halfExtents, &filter,
												 &meshIdx, &polyRef, nearestPt);
		REQUIRE(dtStatusSucceed(status));
		REQUIRE(meshIdx == idxA);
		REQUIRE(polyRef != 0);
	}

	SECTION("Finds poly on mesh B")
	{
		float worldPos[] = {105, 0, 5};
		float halfExtents[] = {5, 4, 5};
		int meshIdx = -1;
		dtPolyRef polyRef = 0;
		float nearestPt[3];

		dtStatus status = query->findNearestPoly(worldPos, halfExtents, &filter,
												 &meshIdx, &polyRef, nearestPt);
		REQUIRE(dtStatusSucceed(status));
		REQUIRE(meshIdx == idxB);
		REQUIRE(polyRef != 0);
	}

	dtFreeMultiNavMeshQuery(query);
	dtFreeMultiNavMesh(multi);
}
