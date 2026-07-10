#include "DetourSurfaceNav.h"
#include "catch2/catch_amalgamated.hpp"

#include <string.h>
#include <math.h>

namespace
{

// Axis-aligned box as 12 triangles.
struct BoxGeometry
{
	float verts[8 * 3];
	int tris[12 * 3];

	BoxGeometry(const float* bmin, const float* bmax)
	{
		const float x0 = bmin[0], y0 = bmin[1], z0 = bmin[2];
		const float x1 = bmax[0], y1 = bmax[1], z1 = bmax[2];
		const float v[8][3] = {
			{ x0, y0, z0 }, { x1, y0, z0 }, { x1, y0, z1 }, { x0, y0, z1 },
			{ x0, y1, z0 }, { x1, y1, z0 }, { x1, y1, z1 }, { x0, y1, z1 },
		};
		memcpy(verts, v, sizeof(v));
		const int t[12][3] = {
			{ 0, 1, 2 }, { 0, 2, 3 },	// bottom
			{ 4, 6, 5 }, { 4, 7, 6 },	// top
			{ 0, 4, 5 }, { 0, 5, 1 },	// z0 side
			{ 3, 2, 6 }, { 3, 6, 7 },	// z1 side
			{ 0, 3, 7 }, { 0, 7, 4 },	// x0 side
			{ 1, 5, 6 }, { 1, 6, 2 },	// x1 side
		};
		memcpy(tris, t, sizeof(t));
	}
};

int findFaceAtWithDir(const dtSurfaceNav& nav, const float* pos, int dir, float radius)
{
	int best = -1;
	float bestDistSqr = radius * radius;
	for (int i = 0; i < nav.getFaceCount(); ++i)
	{
		const dtSurfaceFace* f = nav.getFace(i);
		if (!f->active || f->dir != dir)
			continue;
		float c[3];
		nav.getFaceCenter(i, c);
		const float dx = c[0]-pos[0], dy = c[1]-pos[1], dz = c[2]-pos[2];
		const float distSqr = dx*dx + dy*dy + dz*dz;
		if (distSqr < bestDistSqr)
		{
			bestDistSqr = distSqr;
			best = i;
		}
	}
	return best;
}

} // namespace

TEST_CASE("dtSurfaceNav build from box geometry")
{
	const float bmin[3] = { 0, 0, 0 };
	const float bmax[3] = { 4, 4, 4 };
	BoxGeometry box(bmin, bmax);

	dtSurfaceNavParams params;
	params.cellSize = 1.0f;
	params.bmin[0] = -2; params.bmin[1] = -2; params.bmin[2] = -2;
	params.bmax[0] = 6; params.bmax[1] = 6; params.bmax[2] = 6;

	dtSurfaceNav* nav = dtAllocSurfaceNav();
	REQUIRE(nav != 0);
	REQUIRE(dtStatusSucceed(nav->build(&params, box.verts, 8, box.tris, 12)));
	REQUIRE(nav->getFaceCount() > 0);

	SECTION("Solid voxels where the box walls are")
	{
		const float onWall[3] = { 0.1f, 2, 2 };
		REQUIRE(nav->isSolid(onWall));
		const float farOutside[3] = { -1.9f, 2, 2 };
		REQUIRE_FALSE(nav->isSolid(farOutside));
	}

	SECTION("Faces exist on all six sides")
	{
		const float top[3] = { 2, 4, 2 };
		const float bottom[3] = { 2, 0, 2 };
		const float sideX[3] = { 4, 2, 2 };
		REQUIRE(findFaceAtWithDir(*nav, top, DT_SURF_DIR_YP, 2.0f) >= 0);
		REQUIRE(findFaceAtWithDir(*nav, bottom, DT_SURF_DIR_YN, 2.0f) >= 0);
		REQUIRE(findFaceAtWithDir(*nav, sideX, DT_SURF_DIR_XP, 2.0f) >= 0);
	}

	SECTION("Adjacency wraps around convex edges")
	{
		// Somewhere along the border of the top surface, a top face must
		// connect to a face of another orientation (the surface wraps over
		// the box edge onto a side wall).
		bool foundWrap = false;
		for (int i = 0; i < nav->getFaceCount() && !foundWrap; ++i)
		{
			const dtSurfaceFace* f = nav->getFace(i);
			if (!f->active || f->dir != DT_SURF_DIR_YP)
				continue;
			for (int s = 0; s < 4; ++s)
			{
				if (f->neis[s] >= 0 && nav->getFace(f->neis[s])->dir != f->dir)
				{
					foundWrap = true;
					break;
				}
			}
		}
		REQUIRE(foundWrap);
	}

	SECTION("Climbing path wraps from top to bottom")
	{
		const float topPos[3] = { 2, 4, 2 };
		const float bottomPos[3] = { 2, 0, 2 };
		const int startFace = findFaceAtWithDir(*nav, topPos, DT_SURF_DIR_YP, 2.0f);
		const int endFace = findFaceAtWithDir(*nav, bottomPos, DT_SURF_DIR_YN, 2.0f);
		REQUIRE(startFace >= 0);
		REQUIRE(endFace >= 0);

		dtSurfaceNavFilter climbing;
		climbing.setClimbing();

		int path[256];
		int pathCount = 0;
		REQUIRE(dtStatusSucceed(nav->findPath(startFace, endFace, &climbing,
											  path, &pathCount, 256)));
		REQUIRE(pathCount >= 2);
		REQUIRE(path[0] == startFace);
		REQUIRE(path[pathCount - 1] == endFace);

		// The path must pass over at least one non-vertical-facing face
		// (a wall) to get from top to bottom.
		bool crossedWall = false;
		for (int i = 0; i < pathCount; ++i)
		{
			const int dir = nav->getFace(path[i])->dir;
			if (dir != DT_SURF_DIR_YP && dir != DT_SURF_DIR_YN)
				crossedWall = true;
		}
		REQUIRE(crossedWall);
	}

	SECTION("Walking filter accepts only up-facing faces")
	{
		dtSurfaceNavFilter walking;
		const float up[3] = { 0, 1, 0 };
		walking.setWalking(0, 0, up, 0.9f);

		const float topPos[3] = { 2, 4, 2 };
		const float bottomPos[3] = { 2, 0, 2 };
		const int topFace = findFaceAtWithDir(*nav, topPos, DT_SURF_DIR_YP, 2.0f);
		const int bottomFace = findFaceAtWithDir(*nav, bottomPos, DT_SURF_DIR_YN, 2.0f);
		REQUIRE(nav->passFilter(topFace, &walking));
		REQUIRE_FALSE(nav->passFilter(bottomFace, &walking));

		// Walking path from top to bottom must fail (end face filtered).
		int path[64];
		int pathCount = 0;
		REQUIRE(dtStatusFailed(nav->findPath(topFace, bottomFace, &walking,
											 path, &pathCount, 64)));
	}

	SECTION("Gravity zone flips walkability locally")
	{
		// Zone with inverted gravity around the bottom of the box: its
		// down-facing faces become walkable ceilings for that zone.
		dtSurfaceNavGravityZone zone;
		zone.bmin[0] = -2; zone.bmin[1] = -2; zone.bmin[2] = -2;
		zone.bmax[0] = 6; zone.bmax[1] = 1; zone.bmax[2] = 6;
		zone.up[0] = 0; zone.up[1] = -1; zone.up[2] = 0;

		dtSurfaceNavFilter walking;
		const float up[3] = { 0, 1, 0 };
		walking.setWalking(&zone, 1, up, 0.9f);

		const float bottomPos[3] = { 2, 0, 2 };
		const int bottomFace = findFaceAtWithDir(*nav, bottomPos, DT_SURF_DIR_YN, 2.0f);
		REQUIRE(bottomFace >= 0);
		REQUIRE(nav->passFilter(bottomFace, &walking));

		const float topPos[3] = { 2, 4, 2 };
		const int topFace = findFaceAtWithDir(*nav, topPos, DT_SURF_DIR_YP, 2.0f);
		REQUIRE(nav->passFilter(topFace, &walking));	// Outside zone: default up.
	}

	dtFreeSurfaceNav(nav);
}

TEST_CASE("dtSurfaceNav findNearestFace")
{
	const float bmin[3] = { 0, 0, 0 };
	const float bmax[3] = { 4, 4, 4 };
	BoxGeometry box(bmin, bmax);

	dtSurfaceNavParams params;
	params.cellSize = 1.0f;
	params.bmin[0] = -2; params.bmin[1] = -2; params.bmin[2] = -2;
	params.bmax[0] = 6; params.bmax[1] = 6; params.bmax[2] = 6;

	dtSurfaceNav* nav = dtAllocSurfaceNav();
	REQUIRE(dtStatusSucceed(nav->build(&params, box.verts, 8, box.tris, 12)));

	dtSurfaceNavFilter climbing;
	climbing.setClimbing();

	SECTION("Finds a face near a position above the box")
	{
		const float above[3] = { 2, 5, 2 };
		const int face = nav->findNearestFace(above, 3.0f, &climbing);
		REQUIRE(face >= 0);
		float normal[3];
		nav->getFaceNormal(face, normal);
		REQUIRE(normal[1] == Catch::Approx(1.0f));	// Top face.
	}

	SECTION("Respects maxRadius")
	{
		const float farAway[3] = { 2, 50, 2 };
		REQUIRE(nav->findNearestFace(farAway, 3.0f, &climbing) == -1);
	}

	SECTION("Walking filter restricts candidates")
	{
		dtSurfaceNavFilter walking;
		const float up[3] = { 0, 1, 0 };
		walking.setWalking(0, 0, up, 0.9f);

		// Below the box: nearest unfiltered face is the bottom, but with a
		// walking filter only top faces qualify.
		const float below[3] = { 2, -1, 2 };
		const int face = nav->findNearestFace(below, 100.0f, &walking);
		REQUIRE(face >= 0);
		float normal[3];
		nav->getFaceNormal(face, normal);
		REQUIRE(normal[1] == Catch::Approx(1.0f));
	}

	dtFreeSurfaceNav(nav);
}

TEST_CASE("dtSurfaceNav minRegionFaces culls small patches")
{
	// A large slab plus a distant single small triangle.
	float verts[] = {
		// Slab 8x8 at y=0.
		0, 0, 0,
		8, 0, 0,
		8, 0, 8,
		0, 0, 8,
		// Tiny triangle far to the side.
		20, 0, 4,
		20.4f, 0, 4,
		20.2f, 0, 4.4f,
	};
	int tris[] = {
		0, 1, 2,
		0, 2, 3,
		4, 5, 6,
	};

	dtSurfaceNavParams params;
	params.cellSize = 1.0f;
	params.bmin[0] = -1; params.bmin[1] = -2; params.bmin[2] = -1;
	params.bmax[0] = 23; params.bmax[1] = 2; params.bmax[2] = 10;
	params.minRegionFaces = 4;

	dtSurfaceNav* nav = dtAllocSurfaceNav();
	REQUIRE(dtStatusSucceed(nav->build(&params, verts, 7, tris, 3)));

	dtSurfaceNavFilter climbing;
	climbing.setClimbing();

	// The tiny triangle's faces are culled: nothing within its neighbourhood.
	const float nearTiny[3] = { 20.2f, 0.5f, 4.2f };
	REQUIRE(nav->findNearestFace(nearTiny, 2.0f, &climbing) == -1);

	// The slab survives.
	const float nearSlab[3] = { 4, 1, 4 };
	REQUIRE(nav->findNearestFace(nearSlab, 2.0f, &climbing) >= 0);

	dtFreeSurfaceNav(nav);
}

namespace
{

// Appends a box to vertex/triangle arrays. Returns updated vertex count.
int appendBox(float* verts, int nverts, int* tris, int ntris,
			  const float* bmin, const float* bmax)
{
	BoxGeometry box(bmin, bmax);
	memcpy(verts + nverts * 3, box.verts, sizeof(box.verts));
	for (int i = 0; i < 12 * 3; ++i)
		tris[ntris * 3 + i] = box.tris[i] + nverts;
	return nverts + 8;
}

} // namespace

TEST_CASE("dtSurfaceNav smoothed normals connect stairs for walking agents")
{
	// Five 1x1-step stairs ascending in +x, each 4 deep in z. Raw voxel
	// faces alternate between up-facing treads and vertical risers; the
	// smoothed normals recover the ~45-degree slope.
	const int STEPS = 5;
	float verts[STEPS * 8 * 3];
	int tris[STEPS * 12 * 3];
	int nverts = 0;
	for (int i = 0; i < STEPS; ++i)
	{
		const float bmin[3] = { (float)i, 0, 0 };
		const float bmax[3] = { (float)(i + 1), (float)(i + 1), 4 };
		nverts = appendBox(verts, nverts, tris, i * 12, bmin, bmax);
	}
	const int ntris = STEPS * 12;

	const float up[3] = { 0, 1, 0 };
	const float maxSlopeCos = 0.5f;	// Accept up to ~60 degrees.

	SECTION("Walking path climbs the stairs with smoothing enabled")
	{
		dtSurfaceNavParams params;
		params.cellSize = 1.0f;
		params.bmin[0] = -2; params.bmin[1] = -2; params.bmin[2] = -2;
		params.bmax[0] = 8; params.bmax[1] = 8; params.bmax[2] = 6;
		params.normalSmoothingHops = 2;

		dtSurfaceNav* nav = dtAllocSurfaceNav();
		REQUIRE(dtStatusSucceed(nav->build(&params, verts, nverts, tris, ntris)));

		dtSurfaceNavFilter walking;
		walking.setWalking(0, 0, up, maxSlopeCos);

		// Some riser face (vertical raw normal) must now pass the walking
		// filter thanks to its tilted smoothed normal.
		bool riserWalkable = false;
		for (int i = 0; i < nav->getFaceCount() && !riserWalkable; ++i)
		{
			const dtSurfaceFace* f = nav->getFace(i);
			if (f->active && (f->dir == DT_SURF_DIR_XP || f->dir == DT_SURF_DIR_XN) &&
				nav->passFilter(i, &walking))
				riserWalkable = true;
		}
		REQUIRE(riserWalkable);

		// Walking path from the bottom tread to the top tread must exist.
		const float bottomPos[3] = { 0.5f, 2, 2 };
		const float topPos[3] = { 4.5f, 6, 2 };
		const int startFace = nav->findNearestFace(bottomPos, 2.5f, &walking);
		const int endFace = nav->findNearestFace(topPos, 2.5f, &walking);
		REQUIRE(startFace >= 0);
		REQUIRE(endFace >= 0);

		int path[512];
		int pathCount = 0;
		REQUIRE(dtStatusSucceed(nav->findPath(startFace, endFace, &walking,
											  path, &pathCount, 512)));
		REQUIRE(pathCount >= 2);

		dtFreeSurfaceNav(nav);
	}

	SECTION("Raw normals (smoothing disabled) disconnect the stairs")
	{
		dtSurfaceNavParams params;
		params.cellSize = 1.0f;
		params.bmin[0] = -2; params.bmin[1] = -2; params.bmin[2] = -2;
		params.bmax[0] = 8; params.bmax[1] = 8; params.bmax[2] = 6;
		params.normalSmoothingHops = 0;	// Raw axis normals only.

		dtSurfaceNav* nav = dtAllocSurfaceNav();
		REQUIRE(dtStatusSucceed(nav->build(&params, verts, nverts, tris, ntris)));

		dtSurfaceNavFilter walking;
		walking.setWalking(0, 0, up, maxSlopeCos);

		const float bottomPos[3] = { 0.5f, 2, 2 };
		const float topPos[3] = { 4.5f, 6, 2 };
		const int startFace = nav->findNearestFace(bottomPos, 2.5f, &walking);
		const int endFace = nav->findNearestFace(topPos, 2.5f, &walking);
		REQUIRE(startFace >= 0);
		REQUIRE(endFace >= 0);

		// Every route between treads passes over a vertical riser, which the
		// raw-normal walking filter rejects: no path.
		int path[512];
		int pathCount = 0;
		REQUIRE(dtStatusFailed(nav->findPath(startFace, endFace, &walking,
											 path, &pathCount, 512)));

		dtFreeSurfaceNav(nav);
	}
}

TEST_CASE("dtSurfaceNav smoothed normals still reject pole shafts")
{
	// A thin 1x6x1 pole: its shaft is genuinely vertical, so even smoothed
	// normals must stay horizontal there and fail a walking filter.
	const float bmin[3] = { 0, 0, 0 };
	const float bmax[3] = { 1, 6, 1 };
	BoxGeometry pole(bmin, bmax);

	dtSurfaceNavParams params;
	params.cellSize = 1.0f;
	params.bmin[0] = -2; params.bmin[1] = -2; params.bmin[2] = -2;
	params.bmax[0] = 3; params.bmax[1] = 8; params.bmax[2] = 3;
	params.normalSmoothingHops = 2;

	dtSurfaceNav* nav = dtAllocSurfaceNav();
	REQUIRE(dtStatusSucceed(nav->build(&params, pole.verts, 8, pole.tris, 12)));

	dtSurfaceNavFilter walking;
	const float up[3] = { 0, 1, 0 };
	walking.setWalking(0, 0, up, 0.5f);

	// Mid-shaft side faces must fail the walking filter.
	bool anyMidShaftWalkable = false;
	for (int i = 0; i < nav->getFaceCount(); ++i)
	{
		const dtSurfaceFace* f = nav->getFace(i);
		if (!f->active || f->dir == DT_SURF_DIR_YP || f->dir == DT_SURF_DIR_YN)
			continue;
		float center[3];
		nav->getFaceCenter(i, center);
		if (center[1] < 2.0f || center[1] > 4.0f)
			continue;	// Only mid-height shaft faces, away from cap effects.
		if (nav->passFilter(i, &walking))
			anyMidShaftWalkable = true;
	}
	REQUIRE_FALSE(anyMidShaftWalkable);

	dtFreeSurfaceNav(nav);
}

TEST_CASE("dtSurfaceNav build validation")
{
	dtSurfaceNav* nav = dtAllocSurfaceNav();

	float verts[] = { 0, 0, 0, 1, 0, 0, 0, 0, 1 };
	int tris[] = { 0, 1, 2 };

	SECTION("Null params rejected")
	{
		REQUIRE(dtStatusFailed(nav->build(0, verts, 3, tris, 1)));
	}

	SECTION("Invalid cell size rejected")
	{
		dtSurfaceNavParams params;
		params.cellSize = 0;
		params.bmax[0] = params.bmax[1] = params.bmax[2] = 1;
		REQUIRE(dtStatusFailed(nav->build(&params, verts, 3, tris, 1)));
	}

	SECTION("Empty bounds rejected")
	{
		dtSurfaceNavParams params;
		params.cellSize = 1.0f;
		// bmin == bmax == 0 gives a zero-size grid.
		REQUIRE(dtStatusFailed(nav->build(&params, verts, 3, tris, 1)));
	}

	dtFreeSurfaceNav(nav);
}
