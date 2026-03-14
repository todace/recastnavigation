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

#pragma once

#include "DetourMultiNavMesh.h"
#include "DetourMultiNavMeshQuery.h"
#include "Recast.h"
#include "Sample.h"

#include <cstdint>

/// Maximum number of gravity zone instances in the demo.
static const int MAX_GRAVITY_ZONES = 6;

/// Grid subdivision for generated surfaces (wall, ceiling).
static const int SURFACE_GRID_SIZE = 8;

/// Represents a single gravity zone's Recast build data and navmesh.
struct GravityZoneBuildData
{
	dtNavMesh* navMesh = nullptr;
	rcPolyMesh* polyMesh = nullptr;
	rcPolyMeshDetail* detailMesh = nullptr;

	// Generated flat grid geometry (for wall/ceiling zones, not used for floor)
	float* genVerts = nullptr;     ///< xyz per vertex
	int* genTris = nullptr;        ///< 3 indices per triangle
	float* genNormals = nullptr;   ///< xyz per triangle normal
	int genVertCount = 0;
	int genTriCount = 0;

	float up[3] = {0, 1, 0};
	float origin[3] = {0, 0, 0};
	float rotation[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
	int meshIndex = -1;  ///< Index in dtMultiNavMesh, or -1 if not added.

	unsigned int color = 0;  ///< Tint color for rendering.
	const char* label = "";  ///< Display label (e.g. "Floor", "Wall", "Ceiling").
	bool useInputGeom = false;  ///< true for floor (uses loaded mesh), false for generated geometry.

	void cleanup();
};

/// Demo sample showing multi-navmesh pathfinding across surfaces with different
/// gravity directions. Builds separate navmeshes on floor, wall, and ceiling,
/// connects them with cross-mesh links, and demonstrates pathfinding that
/// crosses between surfaces with different gravity directions.
/// Also integrates standard tools (NavMeshTester, Crowd, OffMeshConnection)
/// that operate on the floor navmesh.
class Sample_MultiNavMesh : public Sample
{
	float totalBuildTimeMs = 0;
	rcConfig config{};

	// Recast intermediates (reused for each zone build, cleaned between)
	unsigned char* triAreas = nullptr;
	rcHeightfield* heightfield = nullptr;
	rcCompactHeightfield* compactHeightfield = nullptr;
	rcContourSet* contourSet = nullptr;

	// Gravity zones
	GravityZoneBuildData zones[MAX_GRAVITY_ZONES];
	int numZones = 0;

	// Multi-navmesh system
	dtMultiNavMesh* multiNavMesh = nullptr;
	dtMultiNavMeshQuery* multiQuery = nullptr;

	// Multi-mesh path testing state
	bool sposSet = false;
	float spos[3] = {};
	int startMeshIndex = -1;
	dtPolyRef startRef = 0;

	bool eposSet = false;
	float epos[3] = {};
	int endMeshIndex = -1;
	dtPolyRef endRef = 0;

	dtQueryFilter filter;

	// Path result
	static constexpr int MAX_PATH_SEGMENTS = 32;
	dtMultiNavMeshPathSegment pathSegments[MAX_PATH_SEGMENTS];
	int pathSegmentCount = 0;

	// Detailed straight path per segment (for path rendering)
	static constexpr int MAX_STRAIGHT_PATH = 128;
	struct SegmentDetail
	{
		float straightPath[MAX_STRAIGHT_PATH * 3];
		unsigned char straightPathFlags[MAX_STRAIGHT_PATH];
		int nstraightPath = 0;
	};
	SegmentDetail segmentDetails[MAX_PATH_SEGMENTS];

	// Multi-mesh path-following agent
	struct MultiMeshAgent
	{
		bool active = false;
		float pos[3] = {};
		float up[3] = {0, 1, 0};           ///< Current gravity up direction.
		float rotation[9] = {1,0,0,0,1,0,0,0,1}; ///< Current orientation matrix.
		int currentSegment = 0;             ///< Current path segment index.
		int currentWaypoint = 0;            ///< Current waypoint within segment.
		float speed = 5.0f;                 ///< Movement speed.
		int meshIndex = -1;                 ///< Current mesh the agent is on.
	};
	MultiMeshAgent multiAgent;

	// Settings
	float wallHeight = 10.0f;  ///< Height of generated walls.
	int linkProbesPerEdge = 20; ///< Number of probe points per shared edge for link creation.

	enum class DrawMode : uint8_t
	{
		NAVMESH,
		NAVMESH_TRANS,
		MESH,
	};
	DrawMode currentDrawMode = DrawMode::NAVMESH;
	static const char* drawModeNames[];

	bool drawLinks = true;
	bool drawZoneLabels = true;
	bool multiGravityToolActive = true;  ///< When true, clicks do multi-gravity pathfinding.

	void cleanup();
	void cleanupBuildIntermediates();

	/// Generate flat grid mesh geometry in local space (XZ plane, Y=0).
	static void generateFlatGrid(float width, float depth, int gridW, int gridH,
								  float** outVerts, int* outVertCount,
								  int** outTris, int* outTriCount,
								  float** outNormals);

	/// Build a navmesh from given geometry (standard Recast pipeline).
	bool buildNavMeshForZone(GravityZoneBuildData& zone,
							 const float* verts, int vertCount,
							 const int* tris, int triCount,
							 const float* boundsMin, const float* boundsMax);

	/// Set up the gravity zone transforms based on input mesh bounds.
	void setupZoneTransforms();

	/// Create manual links between adjacent zones at their shared edges.
	void createManualLinks();

	/// Add manual links along a single shared edge between two zones.
	void addLinksAlongEdge(int zoneA, int zoneB,
						   const float* edgeStart, const float* edgeEnd, int numProbes);

	/// Build 4x4 OpenGL matrix from a 3x3 rotation + origin.
	static void buildGLMatrix(const float* rot3x3, const float* origin, float* mat4x4);

	/// Ray-triangle intersection (Möller-Trumbore).
	static bool intersectSegTri(const float* sp, const float* sq,
								const float* a, const float* b, const float* c,
								float& t);

	/// Recalculate path after start/end changes.
	void recalcPath();

	/// Compute detailed straight paths within each segment for rendering.
	void computeDetailedPaths();

	/// Render the multi-gravity path with detailed segments.
	void renderMultiGravityPath();

	/// Render the multi-gravity path overlay.
	void renderMultiGravityOverlay();

	/// Handle click for multi-gravity pathfinding.
	void handleMultiGravityClick(const float* rayHitPos, bool shift);

	/// Update multi-mesh agent animation.
	void updateMultiAgent(float dt);

	/// Render multi-mesh agent with gravity-correct orientation.
	void renderMultiAgent();

	/// Draw a cylinder oriented along a given up direction.
	void drawOrientedCylinder(const float* pos, const float* up, float radius, float height, unsigned int col);

public:
	Sample_MultiNavMesh();
	~Sample_MultiNavMesh() override;
	Sample_MultiNavMesh(const Sample_MultiNavMesh&) = delete;
	Sample_MultiNavMesh& operator=(const Sample_MultiNavMesh&) = delete;
	Sample_MultiNavMesh(Sample_MultiNavMesh&&) = delete;
	Sample_MultiNavMesh& operator=(Sample_MultiNavMesh&&) = delete;

	void drawSettingsUI() override;
	void drawToolsUI() override;
	void drawDebugUI() override;

	void render() override;
	void renderOverlay() override;
	void onMeshChanged(InputGeom* geom) override;
	void onClick(const float* rayStartPos, const float* rayHitPos, bool shift) override;
	bool build() override;
	void update(float dt) override;
	bool raycastCustomGeometry(const float* src, const float* dst, float& tmin) override;
};
