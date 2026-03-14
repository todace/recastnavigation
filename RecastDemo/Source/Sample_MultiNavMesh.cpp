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

#include "Sample_MultiNavMesh.h"

#include "DetourCrowd.h"
#include "DetourDebugDraw.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "InputGeom.h"
#include "RecastDebugDraw.h"
#include "SDL_opengl.h"
#include "Tool_Crowd.h"
#include "Tool_NavMeshTester.h"
#include "Tool_OffMeshConnection.h"
#include "imguiHelpers.h"

#include <imgui.h>
#include <cmath>
#include <cstring>
#include <cfloat>

// ---- GravityZoneBuildData ----

void GravityZoneBuildData::cleanup()
{
	if (navMesh) { dtFreeNavMesh(navMesh); navMesh = nullptr; }
	if (polyMesh) { rcFreePolyMesh(polyMesh); polyMesh = nullptr; }
	if (detailMesh) { rcFreePolyMeshDetail(detailMesh); detailMesh = nullptr; }
	delete[] genVerts; genVerts = nullptr;
	delete[] genTris; genTris = nullptr;
	delete[] genNormals; genNormals = nullptr;
	genVertCount = 0;
	genTriCount = 0;
	meshIndex = -1;
}

// ---- Draw mode names ----

const char* Sample_MultiNavMesh::drawModeNames[]{ "Navmesh", "Navmesh Trans", "Input Mesh" };

// ---- Constructor / Destructor ----

Sample_MultiNavMesh::Sample_MultiNavMesh()
{
	filter.setIncludeFlags(SAMPLE_POLYFLAGS_ALL ^ SAMPLE_POLYFLAGS_DISABLED);
	filter.setExcludeFlags(0);
}

Sample_MultiNavMesh::~Sample_MultiNavMesh() { cleanup(); }

void Sample_MultiNavMesh::cleanupBuildIntermediates()
{
	delete[] triAreas; triAreas = nullptr;
	rcFreeHeightField(heightfield); heightfield = nullptr;
	rcFreeCompactHeightfield(compactHeightfield); compactHeightfield = nullptr;
	rcFreeContourSet(contourSet); contourSet = nullptr;
}

void Sample_MultiNavMesh::cleanup()
{
	cleanupBuildIntermediates();
	for (int i = 0; i < numZones; ++i) zones[i].cleanup();
	numZones = 0;
	if (multiQuery) { dtFreeMultiNavMeshQuery(multiQuery); multiQuery = nullptr; }
	if (multiNavMesh) { dtFreeMultiNavMesh(multiNavMesh); multiNavMesh = nullptr; }
	navMesh = nullptr;
	pathSegmentCount = 0;
	startRef = endRef = 0;
	startMeshIndex = endMeshIndex = -1;
	multiAgent.active = false;
}

// ---- Utility ----

void Sample_MultiNavMesh::buildGLMatrix(const float* rot, const float* origin, float* m)
{
	m[0] = rot[0];  m[4] = rot[1];  m[8]  = rot[2];  m[12] = origin[0];
	m[1] = rot[3];  m[5] = rot[4];  m[9]  = rot[5];  m[13] = origin[1];
	m[2] = rot[6];  m[6] = rot[7];  m[10] = rot[8];  m[14] = origin[2];
	m[3] = 0.0f;    m[7] = 0.0f;    m[11] = 0.0f;    m[15] = 1.0f;
}

bool Sample_MultiNavMesh::intersectSegTri(
	const float* sp, const float* sq,
	const float* a, const float* b, const float* c, float& t)
{
	const float ab[3] = {b[0]-a[0], b[1]-a[1], b[2]-a[2]};
	const float ac[3] = {c[0]-a[0], c[1]-a[1], c[2]-a[2]};
	const float dir[3] = {sq[0]-sp[0], sq[1]-sp[1], sq[2]-sp[2]};
	const float n[3] = { ab[1]*ac[2]-ab[2]*ac[1], ab[2]*ac[0]-ab[0]*ac[2], ab[0]*ac[1]-ab[1]*ac[0] };
	const float d = -n[0]*dir[0] - n[1]*dir[1] - n[2]*dir[2];
	if (fabsf(d) < 1e-8f) return false;
	const float ap[3] = {sp[0]-a[0], sp[1]-a[1], sp[2]-a[2]};
	t = (n[0]*ap[0] + n[1]*ap[1] + n[2]*ap[2]) / d;
	if (t < 0.0f || t > 1.0f) return false;
	const float e[3] = { -dir[1]*ap[2]+dir[2]*ap[1], -dir[2]*ap[0]+dir[0]*ap[2], -dir[0]*ap[1]+dir[1]*ap[0] };
	float v = (ac[0]*e[0] + ac[1]*e[1] + ac[2]*e[2]) / d;
	if (v < 0.0f || v > 1.0f) return false;
	float w = -(ab[0]*e[0] + ab[1]*e[1] + ab[2]*e[2]) / d;
	if (w < 0.0f || v + w > 1.0f) return false;
	return true;
}

bool Sample_MultiNavMesh::raycastCustomGeometry(const float* src, const float* dst, float& tmin)
{
	bool hit = false;
	tmin = FLT_MAX;
	for (int i = 0; i < numZones; ++i)
	{
		const GravityZoneBuildData& zone = zones[i];
		if (zone.useInputGeom || !zone.genVerts || !zone.genTris || zone.genTriCount == 0) continue;
		const float* R = zone.rotation;
		const float* O = zone.origin;
		float sw[3] = {src[0]-O[0], src[1]-O[1], src[2]-O[2]};
		float dw[3] = {dst[0]-O[0], dst[1]-O[1], dst[2]-O[2]};
		float sL[3] = { R[0]*sw[0]+R[3]*sw[1]+R[6]*sw[2], R[1]*sw[0]+R[4]*sw[1]+R[7]*sw[2], R[2]*sw[0]+R[5]*sw[1]+R[8]*sw[2] };
		float dL[3] = { R[0]*dw[0]+R[3]*dw[1]+R[6]*dw[2], R[1]*dw[0]+R[4]*dw[1]+R[7]*dw[2], R[2]*dw[0]+R[5]*dw[1]+R[8]*dw[2] };
		for (int ti = 0; ti < zone.genTriCount; ++ti)
		{
			const float* v0 = &zone.genVerts[zone.genTris[ti*3+0]*3];
			const float* v1 = &zone.genVerts[zone.genTris[ti*3+1]*3];
			const float* v2 = &zone.genVerts[zone.genTris[ti*3+2]*3];
			float t;
			if (intersectSegTri(sL, dL, v0, v1, v2, t) && t < tmin) { tmin = t; hit = true; }
		}
	}
	return hit;
}

// ---- Generate flat grid mesh ----

void Sample_MultiNavMesh::generateFlatGrid(
	float width, float depth, int gridW, int gridH,
	float** outVerts, int* outVertCount, int** outTris, int* outTriCount, float** outNormals)
{
	const int vertsW = gridW + 1, vertsH = gridH + 1;
	const int numVerts = vertsW * vertsH, numTris = gridW * gridH * 2;
	float* verts = new float[numVerts * 3];
	int* tris = new int[numTris * 3];
	float* normals = new float[numTris * 3];
	const float cellW = width / static_cast<float>(gridW), cellH = depth / static_cast<float>(gridH);
	for (int iz = 0; iz <= gridH; ++iz)
		for (int ix = 0; ix <= gridW; ++ix)
		{
			const int idx = iz * vertsW + ix;
			verts[idx*3+0] = static_cast<float>(ix) * cellW;
			verts[idx*3+1] = 0.0f;
			verts[idx*3+2] = static_cast<float>(iz) * cellH;
		}
	int ti = 0;
	for (int iz = 0; iz < gridH; ++iz)
		for (int ix = 0; ix < gridW; ++ix)
		{
			const int v0=iz*vertsW+ix, v1=iz*vertsW+ix+1, v2=(iz+1)*vertsW+ix+1, v3=(iz+1)*vertsW+ix;
			tris[ti*3+0]=v0; tris[ti*3+1]=v3; tris[ti*3+2]=v1;
			normals[ti*3+0]=0; normals[ti*3+1]=1; normals[ti*3+2]=0; ti++;
			tris[ti*3+0]=v1; tris[ti*3+1]=v3; tris[ti*3+2]=v2;
			normals[ti*3+0]=0; normals[ti*3+1]=1; normals[ti*3+2]=0; ti++;
		}
	*outVerts = verts; *outVertCount = numVerts;
	*outTris = tris; *outTriCount = numTris; *outNormals = normals;
}

// ---- Zone transforms ----

void Sample_MultiNavMesh::setupZoneTransforms()
{
	if (!inputGeometry) return;
	const float* bmin = inputGeometry->getNavMeshBoundsMin();
	const float* bmax = inputGeometry->getNavMeshBoundsMax();
	const float floorW = bmax[0]-bmin[0], floorD = bmax[2]-bmin[2];
	numZones = 6;

	// Zone 0: Floor
	{ auto& z=zones[0]; z.up[0]=0; z.up[1]=1; z.up[2]=0;
	  z.origin[0]=0; z.origin[1]=0; z.origin[2]=0;
	  float id[9]={1,0,0,0,1,0,0,0,1}; memcpy(z.rotation,id,sizeof(id));
	  z.color=duRGBA(0,192,0,64); z.label="Floor"; z.useInputGeom=true; }

	// Zone 1: Right Wall (+X)
	{ auto& z=zones[1]; z.up[0]=-1; z.up[1]=0; z.up[2]=0;
	  z.origin[0]=bmax[0]; z.origin[1]=bmin[1]; z.origin[2]=bmin[2];
	  float r[9]={0,-1,0, 1,0,0, 0,0,1}; memcpy(z.rotation,r,sizeof(r));
	  z.color=duRGBA(0,128,255,64); z.label="Right Wall"; z.useInputGeom=false;
	  generateFlatGrid(wallHeight,floorD,SURFACE_GRID_SIZE,SURFACE_GRID_SIZE,&z.genVerts,&z.genVertCount,&z.genTris,&z.genTriCount,&z.genNormals); }

	// Zone 2: Left Wall (-X)
	{ auto& z=zones[2]; z.up[0]=1; z.up[1]=0; z.up[2]=0;
	  z.origin[0]=bmin[0]; z.origin[1]=bmin[1]; z.origin[2]=bmax[2];
	  float r[9]={0,1,0, 1,0,0, 0,0,-1}; memcpy(z.rotation,r,sizeof(r));
	  z.color=duRGBA(160,0,255,64); z.label="Left Wall"; z.useInputGeom=false;
	  generateFlatGrid(wallHeight,floorD,SURFACE_GRID_SIZE,SURFACE_GRID_SIZE,&z.genVerts,&z.genVertCount,&z.genTris,&z.genTriCount,&z.genNormals); }

	// Zone 3: Back Wall (+Z)
	{ auto& z=zones[3]; z.up[0]=0; z.up[1]=0; z.up[2]=-1;
	  z.origin[0]=bmax[0]; z.origin[1]=bmin[1]; z.origin[2]=bmax[2];
	  float r[9]={0,0,-1, 1,0,0, 0,-1,0}; memcpy(z.rotation,r,sizeof(r));
	  z.color=duRGBA(0,200,200,64); z.label="Back Wall"; z.useInputGeom=false;
	  generateFlatGrid(wallHeight,floorW,SURFACE_GRID_SIZE,SURFACE_GRID_SIZE,&z.genVerts,&z.genVertCount,&z.genTris,&z.genTriCount,&z.genNormals); }

	// Zone 4: Front Wall (-Z)
	{ auto& z=zones[4]; z.up[0]=0; z.up[1]=0; z.up[2]=1;
	  z.origin[0]=bmin[0]; z.origin[1]=bmin[1]; z.origin[2]=bmin[2];
	  float r[9]={0,0,1, 1,0,0, 0,1,0}; memcpy(z.rotation,r,sizeof(r));
	  z.color=duRGBA(255,128,0,64); z.label="Front Wall"; z.useInputGeom=false;
	  generateFlatGrid(wallHeight,floorW,SURFACE_GRID_SIZE,SURFACE_GRID_SIZE,&z.genVerts,&z.genVertCount,&z.genTris,&z.genTriCount,&z.genNormals); }

	// Zone 5: Ceiling
	{ auto& z=zones[5]; z.up[0]=0; z.up[1]=-1; z.up[2]=0;
	  z.origin[0]=bmax[0]; z.origin[1]=bmin[1]+wallHeight; z.origin[2]=bmin[2];
	  float r[9]={-1,0,0, 0,-1,0, 0,0,1}; memcpy(z.rotation,r,sizeof(r));
	  z.color=duRGBA(255,64,0,64); z.label="Ceiling"; z.useInputGeom=false;
	  generateFlatGrid(floorW,floorD,SURFACE_GRID_SIZE,SURFACE_GRID_SIZE,&z.genVerts,&z.genVertCount,&z.genTris,&z.genTriCount,&z.genNormals); }
}

// ---- Build a navmesh for one zone ----

bool Sample_MultiNavMesh::buildNavMeshForZone(
	GravityZoneBuildData& zone, const float* verts, int vertCount,
	const int* tris, int triCount, const float* boundsMin, const float* boundsMax)
{
	if (zone.navMesh) { dtFreeNavMesh(zone.navMesh); zone.navMesh=nullptr; }
	if (zone.polyMesh) { rcFreePolyMesh(zone.polyMesh); zone.polyMesh=nullptr; }
	if (zone.detailMesh) { rcFreePolyMeshDetail(zone.detailMesh); zone.detailMesh=nullptr; }
	zone.meshIndex = -1;
	cleanupBuildIntermediates();

	memset(&config, 0, sizeof(config));
	config.cs = cellSize; config.ch = cellHeight;
	config.walkableSlopeAngle = agentMaxSlope;
	config.walkableHeight = static_cast<int>(ceilf(agentHeight / config.ch));
	config.walkableClimb = static_cast<int>(floorf(agentMaxClimb / config.ch));
	config.walkableRadius = static_cast<int>(ceilf(agentRadius / config.cs));
	config.maxEdgeLen = static_cast<int>(edgeMaxLen / cellSize);
	config.maxSimplificationError = edgeMaxError;
	config.minRegionArea = static_cast<int>(rcSqr(regionMinSize));
	config.mergeRegionArea = static_cast<int>(rcSqr(regionMergeSize));
	config.maxVertsPerPoly = vertsPerPoly;
	config.detailSampleDist = detailSampleDist < 0.9f ? 0 : cellSize * detailSampleDist;
	config.detailSampleMaxError = cellHeight * detailSampleMaxError;
	rcVcopy(config.bmin, boundsMin); rcVcopy(config.bmax, boundsMax);
	rcCalcGridSize(config.bmin, config.bmax, config.cs, &config.width, &config.height);

	heightfield = rcAllocHeightfield();
	if (!heightfield || !rcCreateHeightfield(buildContext, *heightfield, config.width, config.height, config.bmin, config.bmax, config.cs, config.ch))
		return false;
	triAreas = new unsigned char[triCount];
	memset(triAreas, 0, triCount);
	rcMarkWalkableTriangles(buildContext, config.walkableSlopeAngle, verts, vertCount, tris, triCount, triAreas);
	if (!rcRasterizeTriangles(buildContext, verts, vertCount, tris, triAreas, triCount, *heightfield, config.walkableClimb))
		return false;
	if (filterLowHangingObstacles) rcFilterLowHangingWalkableObstacles(buildContext, config.walkableClimb, *heightfield);
	if (filterLedgeSpans) rcFilterLedgeSpans(buildContext, config.walkableHeight, config.walkableClimb, *heightfield);
	if (filterWalkableLowHeightSpans) rcFilterWalkableLowHeightSpans(buildContext, config.walkableHeight, *heightfield);

	compactHeightfield = rcAllocCompactHeightfield();
	if (!compactHeightfield) return false;
	if (!rcBuildCompactHeightfield(buildContext, config.walkableHeight, config.walkableClimb, *heightfield, *compactHeightfield)) return false;
	if (!rcErodeWalkableArea(buildContext, config.walkableRadius, *compactHeightfield)) return false;

	if (partitionType == SamplePartitionType::WATERSHED) {
		if (!rcBuildDistanceField(buildContext, *compactHeightfield)) return false;
		if (!rcBuildRegions(buildContext, *compactHeightfield, 0, config.minRegionArea, config.mergeRegionArea)) return false;
	} else if (partitionType == SamplePartitionType::MONOTONE) {
		if (!rcBuildRegionsMonotone(buildContext, *compactHeightfield, 0, config.minRegionArea, config.mergeRegionArea)) return false;
	} else {
		if (!rcBuildLayerRegions(buildContext, *compactHeightfield, 0, config.minRegionArea)) return false;
	}

	contourSet = rcAllocContourSet();
	if (!contourSet || !rcBuildContours(buildContext, *compactHeightfield, config.maxSimplificationError, config.maxEdgeLen, *contourSet)) return false;
	zone.polyMesh = rcAllocPolyMesh();
	if (!zone.polyMesh || !rcBuildPolyMesh(buildContext, *contourSet, config.maxVertsPerPoly, *zone.polyMesh)) return false;
	zone.detailMesh = rcAllocPolyMeshDetail();
	if (!zone.detailMesh || !rcBuildPolyMeshDetail(buildContext, *zone.polyMesh, *compactHeightfield, config.detailSampleDist, config.detailSampleMaxError, *zone.detailMesh)) return false;
	cleanupBuildIntermediates();

	if (config.maxVertsPerPoly <= DT_VERTS_PER_POLYGON)
	{
		for (int i = 0; i < zone.polyMesh->npolys; ++i) {
			if (zone.polyMesh->areas[i] == RC_WALKABLE_AREA) zone.polyMesh->areas[i] = SAMPLE_POLYAREA_GROUND;
			if (zone.polyMesh->areas[i] == SAMPLE_POLYAREA_GROUND || zone.polyMesh->areas[i] == SAMPLE_POLYAREA_GRASS || zone.polyMesh->areas[i] == SAMPLE_POLYAREA_ROAD)
				zone.polyMesh->flags[i] = SAMPLE_POLYFLAGS_WALK;
			else if (zone.polyMesh->areas[i] == SAMPLE_POLYAREA_WATER) zone.polyMesh->flags[i] = SAMPLE_POLYFLAGS_SWIM;
			else if (zone.polyMesh->areas[i] == SAMPLE_POLYAREA_DOOR) zone.polyMesh->flags[i] = SAMPLE_POLYFLAGS_WALK | SAMPLE_POLYFLAGS_DOOR;
		}
		dtNavMeshCreateParams params; memset(&params, 0, sizeof(params));
		params.verts=zone.polyMesh->verts; params.vertCount=zone.polyMesh->nverts;
		params.polys=zone.polyMesh->polys; params.polyAreas=zone.polyMesh->areas;
		params.polyFlags=zone.polyMesh->flags; params.polyCount=zone.polyMesh->npolys; params.nvp=zone.polyMesh->nvp;
		params.detailMeshes=zone.detailMesh->meshes; params.detailVerts=zone.detailMesh->verts;
		params.detailVertsCount=zone.detailMesh->nverts; params.detailTris=zone.detailMesh->tris; params.detailTriCount=zone.detailMesh->ntris;
		params.walkableHeight=agentHeight; params.walkableRadius=agentRadius; params.walkableClimb=agentMaxClimb;
		rcVcopy(params.bmin, zone.polyMesh->bmin); rcVcopy(params.bmax, zone.polyMesh->bmax);
		params.cs=config.cs; params.ch=config.ch; params.buildBvTree=true;
		unsigned char* navData=0; int navDataSize=0;
		if (!dtCreateNavMeshData(&params, &navData, &navDataSize)) return false;
		zone.navMesh = dtAllocNavMesh();
		if (!zone.navMesh) { dtFree(navData); return false; }
		if (dtStatusFailed(zone.navMesh->init(navData, navDataSize, DT_TILE_FREE_DATA))) { dtFree(navData); return false; }
	}
	return zone.navMesh != nullptr;
}

// ---- Main build ----

bool Sample_MultiNavMesh::build()
{
	if (!inputGeometry || inputGeometry->mesh.verts.empty()) { buildContext->log(RC_LOG_ERROR, "No input mesh."); return false; }
	cleanup();
	buildContext->resetTimers();
	buildContext->startTimer(RC_TIMER_TOTAL);

	const float* bmin = inputGeometry->getNavMeshBoundsMin();
	const float* bmax = inputGeometry->getNavMeshBoundsMax();
	const float floorW = bmax[0]-bmin[0], floorD = bmax[2]-bmin[2];

	setupZoneTransforms();

	for (int i = 0; i < numZones; ++i)
	{
		auto& zone = zones[i];
		buildContext->log(RC_LOG_PROGRESS, "Building zone %d: %s", i, zone.label);
		bool ok = false;
		if (zone.useInputGeom)
		{
			ok = buildNavMeshForZone(zone,
				inputGeometry->mesh.verts.data(), static_cast<int>(inputGeometry->mesh.verts.size())/3,
				inputGeometry->mesh.tris.data(), static_cast<int>(inputGeometry->mesh.tris.size())/3,
				bmin, bmax);
		}
		else
		{
			float lmin[3]={0,-0.1f,0}, lmax[3];
			if (i==1||i==2) { lmax[0]=wallHeight; lmax[1]=0.1f; lmax[2]=floorD; }
			else if (i==3||i==4) { lmax[0]=wallHeight; lmax[1]=0.1f; lmax[2]=floorW; }
			else { lmax[0]=floorW; lmax[1]=0.1f; lmax[2]=floorD; }
			ok = buildNavMeshForZone(zone, zone.genVerts, zone.genVertCount, zone.genTris, zone.genTriCount, lmin, lmax);
		}
		if (!ok) buildContext->log(RC_LOG_ERROR, "Failed zone %d (%s).", i, zone.label);
	}

	// Create multi-navmesh
	multiNavMesh = dtAllocMultiNavMesh();
	if (!multiNavMesh) return false;
	dtMultiNavMeshParams mnmParams;
	mnmParams.maxNavMeshes = MAX_GRAVITY_ZONES;
	mnmParams.maxLinks = 1024;
	mnmParams.overlapDetectionRadius = 1.0f;
	if (dtStatusFailed(multiNavMesh->init(&mnmParams))) return false;

	int addedCount = 0;
	for (int i = 0; i < numZones; ++i)
	{
		if (!zones[i].navMesh) continue;
		int meshIndex = -1;
		if (dtStatusSucceed(multiNavMesh->addNavMesh(zones[i].navMesh, zones[i].up, zones[i].origin, zones[i].rotation, &meshIndex)))
		{
			zones[i].meshIndex = meshIndex;
			addedCount++;
			buildContext->log(RC_LOG_PROGRESS, "  Added zone %d (%s) as mesh %d", i, zones[i].label, meshIndex);
		}
	}

	// DO NOT use detectLinks() - it uses polygon centroids which produce bad link positions.
	// Only use precise manual edge-based links.
	if (addedCount > 1)
		createManualLinks();

	// Init multi-navmesh query
	multiQuery = dtAllocMultiNavMeshQuery();
	if (multiQuery && dtStatusFailed(multiQuery->init(multiNavMesh)))
	{
		dtFreeMultiNavMeshQuery(multiQuery);
		multiQuery = nullptr;
	}

	// Set base class navMesh to floor
	if (zones[0].navMesh) { navMesh = zones[0].navMesh; navQuery->init(navMesh, 2048); }
	if (navMesh && crowd) crowd->init(128, agentRadius, navMesh);
	if (tool) { tool->reset(); tool->init(this); }
	initToolStates(this);

	buildContext->stopTimer(RC_TIMER_TOTAL);
	totalBuildTimeMs = static_cast<float>(buildContext->getAccumulatedTime(RC_TIMER_TOTAL)) / 1000.0f;
	buildContext->log(RC_LOG_PROGRESS, "Multi-navmesh built: %d zones, %d links, %.1fms", addedCount, multiNavMesh->getLinkCount(), totalBuildTimeMs);
	return true;
}

// ---- Manual link creation (improved) ----

void Sample_MultiNavMesh::addLinksAlongEdge(
	int zoneA, int zoneB,
	const float* edgeStart, const float* edgeEnd, int numProbes)
{
	if (!multiNavMesh || zones[zoneA].meshIndex < 0 || zones[zoneB].meshIndex < 0) return;

	// Use tight search extents - just slightly larger than agent radius erosion
	const float ext = agentRadius + 0.5f;
	const float halfExtents[3] = {ext, ext, ext};
	int linksAdded = 0;

	for (int p = 0; p < numProbes; ++p)
	{
		float t = (static_cast<float>(p) + 0.5f) / static_cast<float>(numProbes);
		float edgePos[3] = {
			edgeStart[0] + t*(edgeEnd[0]-edgeStart[0]),
			edgeStart[1] + t*(edgeEnd[1]-edgeStart[1]),
			edgeStart[2] + t*(edgeEnd[2]-edgeStart[2])
		};

		// Find nearest poly on zone A
		float localA[3];
		multiNavMesh->worldToLocal(zones[zoneA].meshIndex, edgePos, localA);
		const dtNavMeshInstance* instA = multiNavMesh->getInstance(zones[zoneA].meshIndex);
		dtPolyRef refA = 0; float nearestA[3];
		if (instA && instA->navQuery)
			instA->navQuery->findNearestPoly(localA, halfExtents, &filter, &refA, nearestA);

		// Find nearest poly on zone B
		float localB[3];
		multiNavMesh->worldToLocal(zones[zoneB].meshIndex, edgePos, localB);
		const dtNavMeshInstance* instB = multiNavMesh->getInstance(zones[zoneB].meshIndex);
		dtPolyRef refB = 0; float nearestB[3];
		if (instB && instB->navQuery)
			instB->navQuery->findNearestPoly(localB, halfExtents, &filter, &refB, nearestB);

		if (refA && refB)
		{
			float worldA[3], worldB[3];
			multiNavMesh->localToWorld(zones[zoneA].meshIndex, nearestA, worldA);
			multiNavMesh->localToWorld(zones[zoneB].meshIndex, nearestB, worldB);

			// Compute actual distance for cost
			float dx=worldA[0]-worldB[0], dy=worldA[1]-worldB[1], dz=worldA[2]-worldB[2];
			float dist = sqrtf(dx*dx + dy*dy + dz*dz);

			int linkIdx = -1;
			if (dtStatusSucceed(multiNavMesh->addLink(
				zones[zoneA].meshIndex, refA, worldA,
				zones[zoneB].meshIndex, refB, worldB,
				dist + 0.5f, true, 0, &linkIdx)))
				linksAdded++;
		}
	}

	if (linksAdded > 0)
		buildContext->log(RC_LOG_PROGRESS, "  %s <-> %s: %d links", zones[zoneA].label, zones[zoneB].label, linksAdded);
}

void Sample_MultiNavMesh::createManualLinks()
{
	if (!multiNavMesh || !inputGeometry) return;
	const float* bmin = inputGeometry->getNavMeshBoundsMin();
	const float* bmax = inputGeometry->getNavMeshBoundsMax();
	const float ceilY = bmin[1] + wallHeight;
	const int np = linkProbesPerEdge;

	buildContext->log(RC_LOG_PROGRESS, "Creating edge-based links (%d probes/edge)...", np);

	// Floor <-> walls (4 bottom edges)
	{ float s[3]={bmax[0],bmin[1],bmin[2]}; float e[3]={bmax[0],bmin[1],bmax[2]}; addLinksAlongEdge(0,1,s,e,np); }
	{ float s[3]={bmin[0],bmin[1],bmin[2]}; float e[3]={bmin[0],bmin[1],bmax[2]}; addLinksAlongEdge(0,2,s,e,np); }
	{ float s[3]={bmin[0],bmin[1],bmax[2]}; float e[3]={bmax[0],bmin[1],bmax[2]}; addLinksAlongEdge(0,3,s,e,np); }
	{ float s[3]={bmin[0],bmin[1],bmin[2]}; float e[3]={bmax[0],bmin[1],bmin[2]}; addLinksAlongEdge(0,4,s,e,np); }

	// Walls <-> ceiling (4 top edges)
	{ float s[3]={bmax[0],ceilY,bmin[2]}; float e[3]={bmax[0],ceilY,bmax[2]}; addLinksAlongEdge(1,5,s,e,np); }
	{ float s[3]={bmin[0],ceilY,bmin[2]}; float e[3]={bmin[0],ceilY,bmax[2]}; addLinksAlongEdge(2,5,s,e,np); }
	{ float s[3]={bmin[0],ceilY,bmax[2]}; float e[3]={bmax[0],ceilY,bmax[2]}; addLinksAlongEdge(3,5,s,e,np); }
	{ float s[3]={bmin[0],ceilY,bmin[2]}; float e[3]={bmax[0],ceilY,bmin[2]}; addLinksAlongEdge(4,5,s,e,np); }

	// Wall <-> wall (4 vertical corners)
	{ float s[3]={bmax[0],bmin[1],bmax[2]}; float e[3]={bmax[0],ceilY,bmax[2]}; addLinksAlongEdge(1,3,s,e,np); }
	{ float s[3]={bmax[0],bmin[1],bmin[2]}; float e[3]={bmax[0],ceilY,bmin[2]}; addLinksAlongEdge(1,4,s,e,np); }
	{ float s[3]={bmin[0],bmin[1],bmax[2]}; float e[3]={bmin[0],ceilY,bmax[2]}; addLinksAlongEdge(2,3,s,e,np); }
	{ float s[3]={bmin[0],bmin[1],bmin[2]}; float e[3]={bmin[0],ceilY,bmin[2]}; addLinksAlongEdge(2,4,s,e,np); }

	buildContext->log(RC_LOG_PROGRESS, "Total links: %d", multiNavMesh->getLinkCount());
}

// ---- Detailed path computation ----

void Sample_MultiNavMesh::computeDetailedPaths()
{
	for (int s = 0; s < pathSegmentCount; ++s)
	{
		SegmentDetail& detail = segmentDetails[s];
		detail.nstraightPath = 0;
		const dtMultiNavMeshPathSegment& seg = pathSegments[s];
		if (seg.meshIndex < 0 || seg.polyCount <= 0) continue;
		const dtNavMeshInstance* inst = multiNavMesh->getInstance(seg.meshIndex);
		if (!inst || !inst->navQuery) continue;
		float localStart[3], localEnd[3];
		multiNavMesh->worldToLocal(seg.meshIndex, seg.startPos, localStart);
		multiNavMesh->worldToLocal(seg.meshIndex, seg.endPos, localEnd);
		float localStraight[MAX_STRAIGHT_PATH * 3];
		unsigned char flags[MAX_STRAIGHT_PATH];
		dtPolyRef polys[MAX_STRAIGHT_PATH];
		int nstraight = 0;
		inst->navQuery->findStraightPath(localStart, localEnd, seg.polys, seg.polyCount,
			localStraight, flags, polys, &nstraight, MAX_STRAIGHT_PATH, 0);
		detail.nstraightPath = nstraight;
		for (int i = 0; i < nstraight; ++i)
		{
			multiNavMesh->localToWorld(seg.meshIndex, &localStraight[i*3], &detail.straightPath[i*3]);
			detail.straightPathFlags[i] = flags[i];
		}
	}
}

// ---- Path recalculation ----

void Sample_MultiNavMesh::recalcPath()
{
	pathSegmentCount = 0;
	multiAgent.active = false;
	if (!multiQuery || !sposSet || !eposSet) return;
	if (startMeshIndex < 0 || endMeshIndex < 0 || !startRef || !endRef) return;
	dtStatus status = multiQuery->findPath(startMeshIndex, startRef, spos, endMeshIndex, endRef, epos, &filter,
		pathSegments, &pathSegmentCount, MAX_PATH_SEGMENTS);
	if (dtStatusFailed(status)) pathSegmentCount = 0;
	if (pathSegmentCount > 0) computeDetailedPaths();
}

// ---- Click handling ----

void Sample_MultiNavMesh::handleMultiGravityClick(const float* rayHitPos, bool shift)
{
	if (!multiQuery) return;
	const float halfExtents[3] = {2.0f, 4.0f, 2.0f};
	int meshIndex = -1; dtPolyRef polyRef = 0; float nearestPt[3];
	dtStatus status = multiQuery->findNearestPoly(rayHitPos, halfExtents, &filter, &meshIndex, &polyRef, nearestPt);
	if (dtStatusFailed(status) || !polyRef) return;
	if (shift) { sposSet=true; rcVcopy(spos,nearestPt); startMeshIndex=meshIndex; startRef=polyRef; }
	else { eposSet=true; rcVcopy(epos,nearestPt); endMeshIndex=meshIndex; endRef=polyRef; }
	recalcPath();
}

void Sample_MultiNavMesh::onClick(const float* rayStartPos, const float* rayHitPos, bool shift)
{
	if (multiGravityToolActive) handleMultiGravityClick(rayHitPos, shift);
	else Sample::onClick(rayStartPos, rayHitPos, shift);
}

// ---- Multi-mesh agent ----

void Sample_MultiNavMesh::updateMultiAgent(float dt)
{
	if (!multiAgent.active || pathSegmentCount <= 0) return;

	// Get current target waypoint
	const SegmentDetail& detail = segmentDetails[multiAgent.currentSegment];
	if (detail.nstraightPath <= 0)
	{
		// No detailed path for this segment, try next
		const dtMultiNavMeshPathSegment& seg = pathSegments[multiAgent.currentSegment];
		float dx = seg.endPos[0]-multiAgent.pos[0];
		float dy = seg.endPos[1]-multiAgent.pos[1];
		float dz = seg.endPos[2]-multiAgent.pos[2];
		float dist = sqrtf(dx*dx + dy*dy + dz*dz);
		if (dist < 0.5f)
		{
			multiAgent.currentSegment++;
			multiAgent.currentWaypoint = 0;
			if (multiAgent.currentSegment >= pathSegmentCount) { multiAgent.active = false; return; }
		}
		else
		{
			float move = dt * multiAgent.speed;
			if (move > dist) move = dist;
			float inv = move / dist;
			multiAgent.pos[0] += dx*inv; multiAgent.pos[1] += dy*inv; multiAgent.pos[2] += dz*inv;
		}
		return;
	}

	int wp = multiAgent.currentWaypoint;
	if (wp >= detail.nstraightPath) wp = detail.nstraightPath - 1;

	const float* target = &detail.straightPath[wp * 3];
	float dx = target[0]-multiAgent.pos[0];
	float dy = target[1]-multiAgent.pos[1];
	float dz = target[2]-multiAgent.pos[2];
	float dist = sqrtf(dx*dx + dy*dy + dz*dz);

	if (dist < 0.3f)
	{
		multiAgent.currentWaypoint++;
		if (multiAgent.currentWaypoint >= detail.nstraightPath)
		{
			// Move to next segment
			multiAgent.currentSegment++;
			multiAgent.currentWaypoint = 0;
			if (multiAgent.currentSegment >= pathSegmentCount) { multiAgent.active = false; return; }
			// Update gravity for new mesh
			const dtMultiNavMeshPathSegment& newSeg = pathSegments[multiAgent.currentSegment];
			if (newSeg.meshIndex >= 0 && newSeg.meshIndex < numZones)
			{
				multiAgent.meshIndex = newSeg.meshIndex;
				rcVcopy(multiAgent.up, zones[newSeg.meshIndex].up);
				memcpy(multiAgent.rotation, zones[newSeg.meshIndex].rotation, sizeof(float)*9);
			}
		}
	}
	else
	{
		float move = dt * multiAgent.speed;
		if (move > dist) move = dist;
		float inv = move / dist;
		multiAgent.pos[0] += dx*inv; multiAgent.pos[1] += dy*inv; multiAgent.pos[2] += dz*inv;
	}
}

void Sample_MultiNavMesh::drawOrientedCylinder(const float* pos, const float* up, float radius, float height, unsigned int col)
{
	// Draw a simple cylinder oriented along the "up" direction
	const int NUM_SEG = 12;
	// Build a local frame: up is given, compute two perpendicular axes
	float fwd[3], right[3];
	// Pick an arbitrary vector not parallel to up
	float tmp[3] = {0, 0, 1};
	if (fabsf(up[0]*tmp[0]+up[1]*tmp[1]+up[2]*tmp[2]) > 0.9f)
		{ tmp[0]=1; tmp[1]=0; tmp[2]=0; }
	// right = up x tmp
	right[0] = up[1]*tmp[2]-up[2]*tmp[1]; right[1] = up[2]*tmp[0]-up[0]*tmp[2]; right[2] = up[0]*tmp[1]-up[1]*tmp[0];
	float rlen = sqrtf(right[0]*right[0]+right[1]*right[1]+right[2]*right[2]);
	if (rlen > 1e-6f) { right[0]/=rlen; right[1]/=rlen; right[2]/=rlen; }
	// fwd = right x up
	fwd[0] = right[1]*up[2]-right[2]*up[1]; fwd[1] = right[2]*up[0]-right[0]*up[2]; fwd[2] = right[0]*up[1]-right[1]*up[0];

	// Draw vertical lines of cylinder
	debugDraw.begin(DU_DRAW_LINES, 2.0f);
	for (int i = 0; i < NUM_SEG; ++i)
	{
		float a = (float)i / (float)NUM_SEG * 3.14159265f * 2.0f;
		float cs = cosf(a), sn = sinf(a);
		float px = pos[0] + (right[0]*cs + fwd[0]*sn) * radius;
		float py = pos[1] + (right[1]*cs + fwd[1]*sn) * radius;
		float pz = pos[2] + (right[2]*cs + fwd[2]*sn) * radius;
		float tx = px + up[0]*height, ty = py + up[1]*height, tz = pz + up[2]*height;
		debugDraw.vertex(px, py, pz, col);
		debugDraw.vertex(tx, ty, tz, col);
	}
	debugDraw.end();

	// Draw top and bottom circles
	unsigned int circCol = duRGBA((col&0xff), ((col>>8)&0xff), ((col>>16)&0xff), 200);
	for (int ring = 0; ring < 2; ++ring)
	{
		float off = ring == 0 ? 0.0f : height;
		debugDraw.begin(DU_DRAW_LINES, 2.0f);
		for (int i = 0; i < NUM_SEG; ++i)
		{
			float a0 = (float)i / (float)NUM_SEG * 3.14159265f * 2.0f;
			float a1 = (float)((i+1)%NUM_SEG) / (float)NUM_SEG * 3.14159265f * 2.0f;
			float x0 = pos[0] + (right[0]*cosf(a0)+fwd[0]*sinf(a0))*radius + up[0]*off;
			float y0 = pos[1] + (right[1]*cosf(a0)+fwd[1]*sinf(a0))*radius + up[1]*off;
			float z0 = pos[2] + (right[2]*cosf(a0)+fwd[2]*sinf(a0))*radius + up[2]*off;
			float x1 = pos[0] + (right[0]*cosf(a1)+fwd[0]*sinf(a1))*radius + up[0]*off;
			float y1 = pos[1] + (right[1]*cosf(a1)+fwd[1]*sinf(a1))*radius + up[1]*off;
			float z1 = pos[2] + (right[2]*cosf(a1)+fwd[2]*sinf(a1))*radius + up[2]*off;
			debugDraw.vertex(x0, y0, z0, circCol);
			debugDraw.vertex(x1, y1, z1, circCol);
		}
		debugDraw.end();
	}
}

void Sample_MultiNavMesh::renderMultiAgent()
{
	if (!multiAgent.active) return;
	drawOrientedCylinder(multiAgent.pos, multiAgent.up, agentRadius, agentHeight, duRGBA(0, 220, 255, 200));

	// Draw up arrow
	debugDraw.begin(DU_DRAW_LINES, 3.0f);
	float top[3] = { multiAgent.pos[0]+multiAgent.up[0]*agentHeight,
	                 multiAgent.pos[1]+multiAgent.up[1]*agentHeight,
	                 multiAgent.pos[2]+multiAgent.up[2]*agentHeight };
	debugDraw.vertex(top[0], top[1], top[2], duRGBA(0,255,0,255));
	debugDraw.vertex(top[0]+multiAgent.up[0]*1.5f, top[1]+multiAgent.up[1]*1.5f, top[2]+multiAgent.up[2]*1.5f, duRGBA(0,255,0,255));
	debugDraw.end();
}

void Sample_MultiNavMesh::update(float dt)
{
	Sample::update(dt);
	updateToolStates(dt);
	updateMultiAgent(dt);
}

// ---- Rendering ----

void Sample_MultiNavMesh::renderMultiGravityPath()
{
	const unsigned int startCol = duRGBA(128, 25, 0, 192);
	const unsigned int endCol = duRGBA(51, 102, 0, 192);
	debugDraw.depthMask(false);

	if (sposSet) {
		debugDraw.begin(DU_DRAW_POINTS, 14.0f);
		debugDraw.vertex(spos[0], spos[1]+0.15f, spos[2], startCol);
		debugDraw.end();
		if (startMeshIndex >= 0 && startMeshIndex < numZones) {
			debugDraw.begin(DU_DRAW_LINES, 2.0f);
			debugDraw.vertex(spos[0], spos[1], spos[2], duRGBA(0,255,0,220));
			debugDraw.vertex(spos[0]+zones[startMeshIndex].up[0]*2, spos[1]+zones[startMeshIndex].up[1]*2, spos[2]+zones[startMeshIndex].up[2]*2, duRGBA(0,255,0,220));
			debugDraw.end();
		}
	}
	if (eposSet) {
		debugDraw.begin(DU_DRAW_POINTS, 14.0f);
		debugDraw.vertex(epos[0], epos[1]+0.15f, epos[2], endCol);
		debugDraw.end();
		if (endMeshIndex >= 0 && endMeshIndex < numZones) {
			debugDraw.begin(DU_DRAW_LINES, 2.0f);
			debugDraw.vertex(epos[0], epos[1], epos[2], duRGBA(255,0,0,220));
			debugDraw.vertex(epos[0]+zones[endMeshIndex].up[0]*2, epos[1]+zones[endMeshIndex].up[1]*2, epos[2]+zones[endMeshIndex].up[2]*2, duRGBA(255,0,0,220));
			debugDraw.end();
		}
	}

	if (pathSegmentCount > 0)
	{
		const unsigned int zoneColors[] = {
			duRGBA(64,255,64,220), duRGBA(64,128,255,220), duRGBA(180,64,255,220),
			duRGBA(64,220,220,220), duRGBA(255,160,64,220), duRGBA(255,100,64,220)
		};

		for (int s = 0; s < pathSegmentCount; ++s)
		{
			const auto& seg = pathSegments[s];
			const auto& detail = segmentDetails[s];
			unsigned int segCol = (seg.meshIndex >= 0 && seg.meshIndex < 6) ? zoneColors[seg.meshIndex] : duRGBA(255,255,255,220);

			if (detail.nstraightPath >= 2)
			{
				debugDraw.begin(DU_DRAW_LINES, 3.0f);
				for (int i = 0; i < detail.nstraightPath-1; ++i) {
					debugDraw.vertex(detail.straightPath[i*3], detail.straightPath[i*3+1]+0.2f, detail.straightPath[i*3+2], segCol);
					debugDraw.vertex(detail.straightPath[(i+1)*3], detail.straightPath[(i+1)*3+1]+0.2f, detail.straightPath[(i+1)*3+2], segCol);
				}
				debugDraw.end();
				debugDraw.begin(DU_DRAW_POINTS, 6.0f);
				for (int i = 0; i < detail.nstraightPath; ++i)
					debugDraw.vertex(detail.straightPath[i*3], detail.straightPath[i*3+1]+0.2f, detail.straightPath[i*3+2], segCol);
				debugDraw.end();
			}
			else
			{
				debugDraw.begin(DU_DRAW_LINES, 3.0f);
				debugDraw.vertex(seg.startPos[0], seg.startPos[1]+0.2f, seg.startPos[2], segCol);
				debugDraw.vertex(seg.endPos[0], seg.endPos[1]+0.2f, seg.endPos[2], segCol);
				debugDraw.end();
			}

			// Link crossing
			if (seg.linkIndex >= 0 && s+1 < pathSegmentCount) {
				debugDraw.begin(DU_DRAW_LINES, 4.0f);
				debugDraw.vertex(seg.endPos[0], seg.endPos[1]+0.3f, seg.endPos[2], duRGBA(255,64,255,220));
				debugDraw.vertex(pathSegments[s+1].startPos[0], pathSegments[s+1].startPos[1]+0.3f, pathSegments[s+1].startPos[2], duRGBA(255,64,255,220));
				debugDraw.end();
				debugDraw.begin(DU_DRAW_POINTS, 10.0f);
				debugDraw.vertex(seg.endPos[0], seg.endPos[1]+0.3f, seg.endPos[2], duRGBA(255,200,0,255));
				debugDraw.vertex(pathSegments[s+1].startPos[0], pathSegments[s+1].startPos[1]+0.3f, pathSegments[s+1].startPos[2], duRGBA(255,200,0,255));
				debugDraw.end();
			}
		}

		// Corridor polygons
		for (int s = 0; s < pathSegmentCount; ++s)
		{
			const auto& seg = pathSegments[s];
			if (seg.meshIndex < 0 || seg.meshIndex >= numZones || zones[seg.meshIndex].meshIndex < 0) continue;
			const auto& zone = zones[seg.meshIndex];
			float mat[16]; buildGLMatrix(zone.rotation, zone.origin, mat);
			glPushMatrix(); glMultMatrixf(mat);
			if (zone.navMesh)
				for (int j = 0; j < seg.polyCount; ++j)
					duDebugDrawNavMeshPoly(&debugDraw, *zone.navMesh, seg.polys[j], duRGBA(255,255,0,24));
			glPopMatrix();
		}
	}

	renderMultiAgent();
	debugDraw.depthMask(true);
}

void Sample_MultiNavMesh::renderMultiGravityOverlay()
{
	if (sposSet) DrawWorldspaceText(spos[0], spos[1]+0.5f, spos[2], IM_COL32(0,0,0,220), "Start", true);
	if (eposSet) DrawWorldspaceText(epos[0], epos[1]+0.5f, epos[2], IM_COL32(0,0,0,220), "End", true);
	if (pathSegmentCount > 0)
	{
		char buf[128];
		snprintf(buf, sizeof(buf), "Path: %d segments, %d links", pathSegmentCount, pathSegmentCount > 1 ? pathSegmentCount-1 : 0);
		DrawScreenspaceText(280, 60, IM_COL32(255,255,200,220), buf);
	}
	if (multiAgent.active && multiAgent.meshIndex >= 0 && multiAgent.meshIndex < numZones)
	{
		DrawWorldspaceText(multiAgent.pos[0], multiAgent.pos[1]+agentHeight+0.5f, multiAgent.pos[2],
						   IM_COL32(0,200,255,220), zones[multiAgent.meshIndex].label, true);
	}
}

void Sample_MultiNavMesh::render()
{
	if (!inputGeometry) return;
	glEnable(GL_FOG); glDepthMask(GL_TRUE);
	const float texScale = 1.0f / (cellSize * 10.0f);

	for (int i = 0; i < numZones; ++i)
	{
		const auto& zone = zones[i];
		float mat[16]; buildGLMatrix(zone.rotation, zone.origin, mat);
		glPushMatrix(); glMultMatrixf(mat);
		if (currentDrawMode == DrawMode::MESH || currentDrawMode == DrawMode::NAVMESH)
		{
			if (zone.useInputGeom)
				duDebugDrawTriMeshSlope(&debugDraw, inputGeometry->mesh.verts.data(), inputGeometry->mesh.getVertCount(),
					inputGeometry->mesh.tris.data(), inputGeometry->mesh.normals.data(), inputGeometry->mesh.getTriCount(), agentMaxSlope, texScale);
			else if (zone.genVerts && zone.genTris)
				duDebugDrawTriMeshSlope(&debugDraw, zone.genVerts, zone.genVertCount, zone.genTris, zone.genNormals, zone.genTriCount, agentMaxSlope, texScale);
		}
		if (zone.navMesh && (currentDrawMode == DrawMode::NAVMESH || currentDrawMode == DrawMode::NAVMESH_TRANS))
			duDebugDrawNavMesh(&debugDraw, *zone.navMesh, navMeshDrawFlags);
		glPopMatrix();
	}

	if (inputGeometry && currentDrawMode != DrawMode::NAVMESH_TRANS)
		inputGeometry->drawOffMeshConnections(&debugDraw);

	glDisable(GL_FOG); glDepthMask(GL_FALSE);

	// Draw cross-mesh links
	if (drawLinks && multiNavMesh)
	{
		debugDraw.begin(DU_DRAW_LINES, 2.0f);
		for (int i = 0; i < multiNavMesh->getMaxLinks(); ++i) {
			const dtMultiNavMeshLink* link = multiNavMesh->getLink(i);
			if (!link || !link->active) continue;
			debugDraw.vertex(link->posA[0], link->posA[1], link->posA[2], duRGBA(255,200,0,128));
			debugDraw.vertex(link->posB[0], link->posB[1], link->posB[2], duRGBA(255,200,0,128));
		}
		debugDraw.end();
		debugDraw.begin(DU_DRAW_POINTS, 5.0f);
		for (int i = 0; i < multiNavMesh->getMaxLinks(); ++i) {
			const dtMultiNavMeshLink* link = multiNavMesh->getLink(i);
			if (!link || !link->active) continue;
			debugDraw.vertex(link->posA[0], link->posA[1], link->posA[2], duRGBA(255,128,0,200));
			debugDraw.vertex(link->posB[0], link->posB[1], link->posB[2], duRGBA(255,128,0,200));
		}
		debugDraw.end();
	}

	if (multiGravityToolActive) renderMultiGravityPath();
	else { if (tool) tool->render(); renderToolStates(); }
	glDepthMask(GL_TRUE);
}

void Sample_MultiNavMesh::renderOverlay()
{
	if (drawZoneLabels && multiNavMesh && inputGeometry)
	{
		const float* bmin = inputGeometry->getNavMeshBoundsMin();
		const float* bmax = inputGeometry->getNavMeshBoundsMax();
		const float floorW = bmax[0]-bmin[0], floorD = bmax[2]-bmin[2];
		for (int i = 0; i < numZones; ++i) {
			if (zones[i].meshIndex < 0) continue;
			float lc[3]={0,0,0};
			if (zones[i].useInputGeom) { lc[0]=(bmin[0]+bmax[0])*0.5f; lc[1]=(bmin[1]+bmax[1])*0.5f; lc[2]=(bmin[2]+bmax[2])*0.5f; }
			else { switch(i) { case 1: case 2: lc[0]=wallHeight*0.5f; lc[2]=floorD*0.5f; break;
			                   case 3: case 4: lc[0]=wallHeight*0.5f; lc[2]=floorW*0.5f; break;
			                   case 5: lc[0]=floorW*0.5f; lc[2]=floorD*0.5f; break; } }
			float wc[3]; multiNavMesh->localToWorld(zones[i].meshIndex, lc, wc);
			DrawWorldspaceText(wc[0], wc[1], wc[2], IM_COL32(255,255,255,200), zones[i].label, true);
		}
	}
	DrawScreenspaceText(280, 40, IM_COL32(255,255,255,192), "SHIFT+Click: Start   Click: End   (any surface)");
	if (multiGravityToolActive) renderMultiGravityOverlay();
	else { if (tool) tool->drawOverlayUI(); renderOverlayToolStates(); }
}

// ---- UI ----

void Sample_MultiNavMesh::drawSettingsUI()
{
	drawCommonSettingsUI();
	ImGui::SeparatorText("Multi-NavMesh");
	ImGui::SliderFloat("Wall Height", &wallHeight, 2.0f, 40.0f, "%.0f");
	ImGui::SliderInt("Link Probes/Edge", &linkProbesPerEdge, 5, 50);
	ImGui::Text("Build Time: %.1fms", totalBuildTimeMs);
	if (multiNavMesh) ImGui::Text("Zones: %d   Links: %d", multiNavMesh->getNavMeshCount(), multiNavMesh->getLinkCount());
}

void Sample_MultiNavMesh::drawToolsUI()
{
	ImGui::SeparatorText("Tool Selection");
	if (ImGui::RadioButton("Multi-Gravity Path", multiGravityToolActive))
	{ multiGravityToolActive = true; setTool(nullptr); }

#define TOOL(toolType, toolClass) \
	if (ImGui::RadioButton(toolNames[static_cast<uint8_t>(SampleToolType::toolType)], \
			!multiGravityToolActive && tool && tool->type() == SampleToolType::toolType)) \
	{ multiGravityToolActive = false; setTool(new (toolClass){}); }
	TOOL(NAVMESH_TESTER, NavMeshTesterTool)
	TOOL(OFFMESH_CONNECTION, OffMeshConnectionTool)
	TOOL(CROWD, CrowdTool)
#undef TOOL

	ImGui::SeparatorText("Tool Settings");

	if (multiGravityToolActive)
	{
		ImGui::TextWrapped("Multi-gravity pathfinding across floor, 4 walls, and ceiling.");
		ImGui::Separator();
		if (sposSet && startMeshIndex >= 0 && startMeshIndex < numZones)
			ImGui::Text("Start: %s", zones[startMeshIndex].label);
		else ImGui::TextDisabled("Start: not set");
		if (eposSet && endMeshIndex >= 0 && endMeshIndex < numZones)
			ImGui::Text("End: %s", zones[endMeshIndex].label);
		else ImGui::TextDisabled("End: not set");

		if (pathSegmentCount > 0)
		{
			ImGui::SeparatorText("Path Segments");
			for (int i = 0; i < pathSegmentCount; ++i) {
				const auto& seg = pathSegments[i];
				const char* ml = (seg.meshIndex >= 0 && seg.meshIndex < numZones) ? zones[seg.meshIndex].label : "?";
				ImGui::Text("  %d. %s (%d polys)", i+1, ml, seg.polyCount);
			}
		}

		ImGui::Separator();
		if (ImGui::Button("Clear Path"))
		{ sposSet=eposSet=false; startRef=endRef=0; startMeshIndex=endMeshIndex=-1; pathSegmentCount=0; multiAgent.active=false; }

		ImGui::SameLine();
		if (pathSegmentCount > 0)
		{
			if (!multiAgent.active)
			{
				if (ImGui::Button("Animate Agent"))
				{
					multiAgent.active = true;
					multiAgent.currentSegment = 0;
					multiAgent.currentWaypoint = 0;
					rcVcopy(multiAgent.pos, spos);
					multiAgent.meshIndex = startMeshIndex;
					if (startMeshIndex >= 0 && startMeshIndex < numZones) {
						rcVcopy(multiAgent.up, zones[startMeshIndex].up);
						memcpy(multiAgent.rotation, zones[startMeshIndex].rotation, sizeof(float)*9);
					}
				}
			}
			else
			{
				if (ImGui::Button("Stop Agent")) multiAgent.active = false;
				ImGui::SliderFloat("Speed", &multiAgent.speed, 1.0f, 20.0f, "%.0f");
			}
		}
	}
	else if (tool) tool->drawMenuUI();
}

void Sample_MultiNavMesh::drawDebugUI()
{
	ImGui::Text("Draw");
	if (ImGui::BeginCombo("##drawMode", drawModeNames[static_cast<int>(currentDrawMode)], 0))
	{
		for (int i = 0; i < 3; ++i) {
			bool sel = currentDrawMode == static_cast<DrawMode>(i);
			if (ImGui::Selectable(drawModeNames[i], sel)) currentDrawMode = static_cast<DrawMode>(i);
			if (sel) ImGui::SetItemDefaultFocus();
		}
		ImGui::EndCombo();
	}
	ImGui::Checkbox("Draw Links", &drawLinks);
	ImGui::Checkbox("Draw Zone Labels", &drawZoneLabels);
}

void Sample_MultiNavMesh::onMeshChanged(InputGeom* geom)
{
	Sample::onMeshChanged(geom);
	cleanup();
	if (tool) { tool->reset(); tool->init(this); }
	resetToolStates(); initToolStates(this);
}
