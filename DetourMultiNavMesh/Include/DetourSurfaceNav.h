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

#ifndef DETOURSURFACENAV_H
#define DETOURSURFACENAV_H

#include "DetourStatus.h"

/// Face normal directions. Each exposed voxel face points along one of the
/// six axis directions.
enum dtSurfaceFaceDir
{
	DT_SURF_DIR_XP = 0,	///< +X
	DT_SURF_DIR_XN = 1,	///< -X
	DT_SURF_DIR_YP = 2,	///< +Y
	DT_SURF_DIR_YN = 3,	///< -Y
	DT_SURF_DIR_ZP = 4,	///< +Z
	DT_SURF_DIR_ZN = 5,	///< -Z
	DT_SURF_DIR_COUNT = 6
};

/// An exposed voxel face of the surface wrap. A face is a square of
/// cellSize x cellSize that an agent can stand on (subject to filtering).
struct dtSurfaceFace
{
	unsigned short x, y, z;	///< Voxel grid coordinates of the solid voxel.
	unsigned char dir;		///< Face normal direction. (See: #dtSurfaceFaceDir)
	unsigned char active;	///< Nonzero when the face survived region filtering.
	int neis[4];			///< Neighbour face index per lateral direction, -1 if none.
	int region;				///< Planar patch id the face belongs to.

	/// Smoothed unit normal: the average of raw face normals over a small
	/// surface neighbourhood. Voxel faces can only point along 6 axis
	/// directions, so raw normals cannot express intermediate slopes; the
	/// smoothed normal recovers them (a 45-degree staircase of voxel faces
	/// gets ~45-degree smoothed normals). Walking filters test this normal.
	float snormal[3];
};

/// Build parameters for the surface wrap.
struct dtSurfaceNavParams
{
	float cellSize;			///< Voxel edge length. [Limit: > 0]
	float bmin[3];			///< Build bounds minimum.
	float bmax[3];			///< Build bounds maximum.

	/// Minimum number of faces a planar patch (coplanar connected component)
	/// must contain to be kept. Culls poles, thin ledges and other surfaces
	/// too small to stay on. 1 keeps everything.
	int minRegionFaces;

	/// Graph-hop radius used to compute per-face smoothed normals
	/// (see: dtSurfaceFace::snormal). 0 disables smoothing (smoothed normal
	/// equals the raw axis normal), larger values regularize rough surfaces
	/// over a wider footprint. Typical: 2.
	int normalSmoothingHops;

	dtSurfaceNavParams()
		: cellSize(0.5f)
		, minRegionFaces(1)
		, normalSmoothingHops(2)
	{
		bmin[0] = bmin[1] = bmin[2] = 0;
		bmax[0] = bmax[1] = bmax[2] = 0;
	}
};

/// Axis-aligned gravity zone. Faces whose centers fall inside the box use
/// the zone's up direction when evaluated by a walking filter.
struct dtSurfaceNavGravityZone
{
	float bmin[3];
	float bmax[3];
	float up[3];
};

/// Filter for surface navigation queries.
///
/// Climbing agents traverse every face regardless of orientation.
/// Walking agents only accept faces whose smoothed normal opposes the local
/// gravity (dot(snormal, up) >= maxSlopeCos), where the local up direction
/// comes from the containing gravity zone or defaults to defaultUp. Testing
/// the smoothed normal (rather than the raw axis normal) keeps ramps, stairs
/// and rough-but-regular surfaces connected for walking agents while still
/// rejecting genuinely vertical structure such as walls and pole shafts.
class dtSurfaceNavFilter
{
public:
	dtSurfaceNavFilter()
		: m_zones(0)
		, m_zoneCount(0)
		, m_maxSlopeCos(0.5f)
		, m_climbing(true)
	{
		m_defaultUp[0] = 0; m_defaultUp[1] = 1; m_defaultUp[2] = 0;
	}

	/// Configure as climbing agent: all faces pass.
	void setClimbing() { m_climbing = true; }

	/// Configure as walking agent with gravity zones.
	///  @param[in]	zones			Gravity zone array (borrowed, not copied). May be null.
	///  @param[in]	zoneCount		Number of zones.
	///  @param[in]	defaultUp		Up direction outside all zones.
	///  @param[in]	maxSlopeCos		Minimum dot(faceNormal, up) to accept a face.
	void setWalking(const dtSurfaceNavGravityZone* zones, int zoneCount,
					const float* defaultUp, float maxSlopeCos)
	{
		m_climbing = false;
		m_zones = zones;
		m_zoneCount = zoneCount;
		m_defaultUp[0] = defaultUp[0];
		m_defaultUp[1] = defaultUp[1];
		m_defaultUp[2] = defaultUp[2];
		m_maxSlopeCos = maxSlopeCos;
	}

	bool isClimbing() const { return m_climbing; }
	const dtSurfaceNavGravityZone* getZones() const { return m_zones; }
	int getZoneCount() const { return m_zoneCount; }
	const float* getDefaultUp() const { return m_defaultUp; }
	float getMaxSlopeCos() const { return m_maxSlopeCos; }

private:
	const dtSurfaceNavGravityZone* m_zones;
	int m_zoneCount;
	float m_defaultUp[3];
	float m_maxSlopeCos;
	bool m_climbing;
};

/// A navigable surface wrap around collision geometry.
///
/// Built by voxelizing input triangles into a solid grid and extracting the
/// exposed voxel faces. The faces form a graph: coplanar faces connect
/// laterally, and 90-degree convex/concave transitions wrap the surface
/// around edges and into corners, so the graph covers walls, ceilings and
/// floors alike. Climbing agents path over the whole wrap; walking agents
/// apply a gravity filter. The underlying voxel grid doubles as a simplified
/// collision representation (see: #isSolid).
class dtSurfaceNav
{
public:
	dtSurfaceNav();
	~dtSurfaceNav();

	/// Builds the surface wrap from triangle geometry.
	///  @param[in]	params		Build parameters.
	///  @param[in]	verts		Vertex positions. [(x, y, z) * nverts]
	///  @param[in]	nverts		Vertex count.
	///  @param[in]	tris		Triangle vertex indices. [(a, b, c) * ntris]
	///  @param[in]	ntris		Triangle count.
	/// @return The status flags for the operation.
	dtStatus build(const dtSurfaceNavParams* params,
				   const float* verts, int nverts,
				   const int* tris, int ntris);

	/// Number of faces (including culled ones; check dtSurfaceFace::active).
	int getFaceCount() const { return m_faceCount; }

	/// Face accessor. Returns null for an invalid index.
	const dtSurfaceFace* getFace(int i) const;

	/// World-space center of a face.
	void getFaceCenter(int i, float* center) const;

	/// Raw axis-aligned unit normal of a face.
	void getFaceNormal(int i, float* normal) const;

	/// Smoothed unit normal of a face. (See: dtSurfaceFace::snormal)
	void getFaceSmoothedNormal(int i, float* normal) const;

	/// Whether a face passes the given filter (active + orientation check).
	bool passFilter(int i, const dtSurfaceNavFilter* filter) const;

	/// Finds the face passing the filter whose center is nearest to pos.
	///  @param[in]	pos			World position.
	///  @param[in]	maxRadius	Maximum search radius (<= 0 for unlimited).
	///  @param[in]	filter		Filter to apply.
	/// @return Face index, or -1 when nothing is in range.
	int findNearestFace(const float* pos, float maxRadius,
						const dtSurfaceNavFilter* filter) const;

	/// A* path over the face graph.
	///  @param[in]	startFace	Start face index.
	///  @param[in]	endFace		End face index.
	///  @param[in]	filter		Filter to apply to visited faces.
	///  @param[out]	path		Face indices from start to end.
	///  @param[out]	pathCount	Number of faces written.
	///  @param[in]	maxPath		Capacity of path.
	/// @return The status flags for the operation. DT_BUFFER_TOO_SMALL is set
	///         when the path was truncated.
	dtStatus findPath(int startFace, int endFace,
					  const dtSurfaceNavFilter* filter,
					  int* path, int* pathCount, int maxPath) const;

	/// Simplified collision query: whether the voxel containing pos is solid.
	bool isSolid(const float* pos) const;

	/// Grid dimensions in voxels.
	void getGridSize(int* w, int* h, int* d) const { *w = m_width; *h = m_height; *d = m_depth; }

	/// The build parameters used.
	const dtSurfaceNavParams* getParams() const { return &m_params; }

private:
	dtSurfaceNav(const dtSurfaceNav&);
	dtSurfaceNav& operator=(const dtSurfaceNav&);

	void purge();
	bool solidAt(int x, int y, int z) const;
	int faceIndexAt(int x, int y, int z, int dir) const;
	void buildAdjacency();
	void filterRegions(int minRegionFaces);
	void computeSmoothedNormals(int hops);

	dtSurfaceNavParams m_params;
	int m_width, m_height, m_depth;	///< Grid dimensions in voxels.
	unsigned int* m_solid;			///< Solid voxel bit grid.
	dtSurfaceFace* m_faces;			///< Face array.
	int m_faceCount;
	unsigned long long* m_faceKeys;	///< Sorted (key << 32 | index) lookup for faceIndexAt.
};

/// Allocates a surface nav object using the Detour allocator.
dtSurfaceNav* dtAllocSurfaceNav();

/// Frees a surface nav object using the Detour allocator.
void dtFreeSurfaceNav(dtSurfaceNav* nav);

#endif // DETOURSURFACENAV_H
