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

#include "DetourSurfaceNav.h"
#include "DetourAlloc.h"
#include "DetourCommon.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <new>

namespace
{

/// Maximum total voxel count of the build grid (bit grid memory cap).
const unsigned long long DT_SURF_MAX_VOXELS = 1ull << 27;

const int DIR_VEC[6][3] =
{
	{ 1, 0, 0 }, { -1, 0, 0 },
	{ 0, 1, 0 }, { 0, -1, 0 },
	{ 0, 0, 1 }, { 0, 0, -1 },
};

/// Lateral direction codes for a face, indexed by the face normal's axis.
const int LATERAL_DIRS[3][4] =
{
	{ 2, 3, 4, 5 },	// normal on x: laterals are +-y, +-z
	{ 0, 1, 4, 5 },	// normal on y: laterals are +-x, +-z
	{ 0, 1, 2, 3 },	// normal on z: laterals are +-x, +-y
};

inline int oppositeDir(int dir) { return dir ^ 1; }

// --- Triangle vs axis-aligned box overlap (Akenine-Moller SAT) ---

inline void findMinMax(float a, float b, float c, float& mn, float& mx)
{
	mn = mx = a;
	if (b < mn) mn = b;
	if (b > mx) mx = b;
	if (c < mn) mn = c;
	if (c > mx) mx = c;
}

bool planeBoxOverlap(const float* normal, const float* vert, const float* maxbox)
{
	float vmin[3], vmax[3];
	for (int q = 0; q < 3; ++q)
	{
		const float v = vert[q];
		if (normal[q] > 0.0f)
		{
			vmin[q] = -maxbox[q] - v;
			vmax[q] = maxbox[q] - v;
		}
		else
		{
			vmin[q] = maxbox[q] - v;
			vmax[q] = -maxbox[q] - v;
		}
	}
	if (dtVdot(normal, vmin) > 0.0f) return false;
	if (dtVdot(normal, vmax) >= 0.0f) return true;
	return false;
}

bool overlapTriBox(const float* boxcenter, const float* boxhalf,
				   const float* tv0, const float* tv1, const float* tv2)
{
	float v0[3], v1[3], v2[3];
	dtVsub(v0, tv0, boxcenter);
	dtVsub(v1, tv1, boxcenter);
	dtVsub(v2, tv2, boxcenter);

	float e0[3], e1[3], e2[3];
	dtVsub(e0, v1, v0);
	dtVsub(e1, v2, v1);
	dtVsub(e2, v0, v2);

	float mn, mx, p0, p1, p2, rad, fex, fey, fez;

	// 9 cross-axis tests.
	fex = fabsf(e0[0]); fey = fabsf(e0[1]); fez = fabsf(e0[2]);
	p0 = e0[2]*v0[1] - e0[1]*v0[2];
	p2 = e0[2]*v2[1] - e0[1]*v2[2];
	mn = p0 < p2 ? p0 : p2; mx = p0 < p2 ? p2 : p0;
	rad = fez*boxhalf[1] + fey*boxhalf[2];
	if (mn > rad || mx < -rad) return false;
	p0 = -e0[2]*v0[0] + e0[0]*v0[2];
	p2 = -e0[2]*v2[0] + e0[0]*v2[2];
	mn = p0 < p2 ? p0 : p2; mx = p0 < p2 ? p2 : p0;
	rad = fez*boxhalf[0] + fex*boxhalf[2];
	if (mn > rad || mx < -rad) return false;
	p1 = e0[1]*v1[0] - e0[0]*v1[1];
	p2 = e0[1]*v2[0] - e0[0]*v2[1];
	mn = p1 < p2 ? p1 : p2; mx = p1 < p2 ? p2 : p1;
	rad = fey*boxhalf[0] + fex*boxhalf[1];
	if (mn > rad || mx < -rad) return false;

	fex = fabsf(e1[0]); fey = fabsf(e1[1]); fez = fabsf(e1[2]);
	p0 = e1[2]*v0[1] - e1[1]*v0[2];
	p2 = e1[2]*v2[1] - e1[1]*v2[2];
	mn = p0 < p2 ? p0 : p2; mx = p0 < p2 ? p2 : p0;
	rad = fez*boxhalf[1] + fey*boxhalf[2];
	if (mn > rad || mx < -rad) return false;
	p0 = -e1[2]*v0[0] + e1[0]*v0[2];
	p2 = -e1[2]*v2[0] + e1[0]*v2[2];
	mn = p0 < p2 ? p0 : p2; mx = p0 < p2 ? p2 : p0;
	rad = fez*boxhalf[0] + fex*boxhalf[2];
	if (mn > rad || mx < -rad) return false;
	p0 = e1[1]*v0[0] - e1[0]*v0[1];
	p1 = e1[1]*v1[0] - e1[0]*v1[1];
	mn = p0 < p1 ? p0 : p1; mx = p0 < p1 ? p1 : p0;
	rad = fey*boxhalf[0] + fex*boxhalf[1];
	if (mn > rad || mx < -rad) return false;

	fex = fabsf(e2[0]); fey = fabsf(e2[1]); fez = fabsf(e2[2]);
	p0 = e2[2]*v0[1] - e2[1]*v0[2];
	p1 = e2[2]*v1[1] - e2[1]*v1[2];
	mn = p0 < p1 ? p0 : p1; mx = p0 < p1 ? p1 : p0;
	rad = fez*boxhalf[1] + fey*boxhalf[2];
	if (mn > rad || mx < -rad) return false;
	p0 = -e2[2]*v0[0] + e2[0]*v0[2];
	p1 = -e2[2]*v1[0] + e2[0]*v1[2];
	mn = p0 < p1 ? p0 : p1; mx = p0 < p1 ? p1 : p0;
	rad = fez*boxhalf[0] + fex*boxhalf[2];
	if (mn > rad || mx < -rad) return false;
	p1 = e2[1]*v1[0] - e2[0]*v1[1];
	p2 = e2[1]*v2[0] - e2[0]*v2[1];
	mn = p1 < p2 ? p1 : p2; mx = p1 < p2 ? p2 : p1;
	rad = fey*boxhalf[0] + fex*boxhalf[1];
	if (mn > rad || mx < -rad) return false;

	// 3 AABB tests.
	findMinMax(v0[0], v1[0], v2[0], mn, mx);
	if (mn > boxhalf[0] || mx < -boxhalf[0]) return false;
	findMinMax(v0[1], v1[1], v2[1], mn, mx);
	if (mn > boxhalf[1] || mx < -boxhalf[1]) return false;
	findMinMax(v0[2], v1[2], v2[2], mn, mx);
	if (mn > boxhalf[2] || mx < -boxhalf[2]) return false;

	// Plane test.
	float normal[3];
	dtVcross(normal, e0, e1);
	return planeBoxOverlap(normal, v0, boxhalf);
}

int compareKeys(const void* a, const void* b)
{
	const unsigned long long ka = *(const unsigned long long*)a;
	const unsigned long long kb = *(const unsigned long long*)b;
	if (ka < kb) return -1;
	if (ka > kb) return 1;
	return 0;
}

/// Zone-aware up direction for a world position.
void localUpAt(const dtSurfaceNavFilter* filter, const float* pos, float* up)
{
	const dtSurfaceNavGravityZone* zones = filter->getZones();
	const int n = filter->getZoneCount();
	for (int i = 0; i < n; ++i)
	{
		const dtSurfaceNavGravityZone& z = zones[i];
		if (pos[0] >= z.bmin[0] && pos[0] <= z.bmax[0] &&
			pos[1] >= z.bmin[1] && pos[1] <= z.bmax[1] &&
			pos[2] >= z.bmin[2] && pos[2] <= z.bmax[2])
		{
			dtVcopy(up, z.up);
			return;
		}
	}
	dtVcopy(up, filter->getDefaultUp());
}

} // namespace

dtSurfaceNav::dtSurfaceNav()
	: m_width(0), m_height(0), m_depth(0)
	, m_solid(0)
	, m_faces(0)
	, m_faceCount(0)
	, m_faceKeys(0)
{
}

dtSurfaceNav::~dtSurfaceNav()
{
	purge();
}

void dtSurfaceNav::purge()
{
	dtFree(m_solid); m_solid = 0;
	dtFree(m_faces); m_faces = 0;
	dtFree(m_faceKeys); m_faceKeys = 0;
	m_faceCount = 0;
	m_width = m_height = m_depth = 0;
}

bool dtSurfaceNav::solidAt(int x, int y, int z) const
{
	if (x < 0 || y < 0 || z < 0 || x >= m_width || y >= m_height || z >= m_depth)
		return false;
	const unsigned long long idx =
		((unsigned long long)z * m_height + y) * m_width + x;
	return (m_solid[idx >> 5] & (1u << (idx & 31))) != 0;
}

int dtSurfaceNav::faceIndexAt(int x, int y, int z, int dir) const
{
	if (!m_faceKeys || m_faceCount == 0)
		return -1;
	const unsigned long long key =
		((((unsigned long long)z * m_height + y) * m_width + x) * 6 + dir) << 20;
	// Binary search on the high (key) bits; low 20 bits carry the face index.
	int lo = 0, hi = m_faceCount - 1;
	while (lo <= hi)
	{
		const int mid = (lo + hi) / 2;
		const unsigned long long k = m_faceKeys[mid] & ~((1ull << 20) - 1);
		if (k == key)
			return (int)(m_faceKeys[mid] & ((1ull << 20) - 1));
		if (k < key)
			lo = mid + 1;
		else
			hi = mid - 1;
	}
	return -1;
}

dtStatus dtSurfaceNav::build(const dtSurfaceNavParams* params,
							 const float* verts, int nverts,
							 const int* tris, int ntris)
{
	if (!params || !verts || !tris || nverts <= 0 || ntris <= 0)
		return DT_FAILURE | DT_INVALID_PARAM;
	if (params->cellSize <= 0)
		return DT_FAILURE | DT_INVALID_PARAM;

	purge();
	m_params = *params;

	const float cs = params->cellSize;
	const int w = (int)ceilf((params->bmax[0] - params->bmin[0]) / cs);
	const int h = (int)ceilf((params->bmax[1] - params->bmin[1]) / cs);
	const int d = (int)ceilf((params->bmax[2] - params->bmin[2]) / cs);
	if (w <= 0 || h <= 0 || d <= 0)
		return DT_FAILURE | DT_INVALID_PARAM;
	if ((unsigned long long)w * h * d > DT_SURF_MAX_VOXELS ||
		w > 0xffff || h > 0xffff || d > 0xffff)
		return DT_FAILURE | DT_INVALID_PARAM;

	m_width = w; m_height = h; m_depth = d;

	// --- 1. Voxelize triangles into the solid bit grid. ---
	const unsigned long long nbits = (unsigned long long)w * h * d;
	const unsigned long long nwords = (nbits + 31) / 32;
	m_solid = (unsigned int*)dtAlloc((size_t)(nwords * sizeof(unsigned int)), DT_ALLOC_PERM);
	if (!m_solid)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	memset(m_solid, 0, (size_t)(nwords * sizeof(unsigned int)));

	const float halfBox[3] = { cs * 0.5001f, cs * 0.5001f, cs * 0.5001f };
	for (int t = 0; t < ntris; ++t)
	{
		const float* a = &verts[tris[t*3+0]*3];
		const float* b = &verts[tris[t*3+1]*3];
		const float* c = &verts[tris[t*3+2]*3];

		float tmin[3], tmax[3];
		dtVcopy(tmin, a); dtVcopy(tmax, a);
		dtVmin(tmin, b); dtVmax(tmax, b);
		dtVmin(tmin, c); dtVmax(tmax, c);

		int x0 = (int)floorf((tmin[0] - params->bmin[0]) / cs);
		int y0 = (int)floorf((tmin[1] - params->bmin[1]) / cs);
		int z0 = (int)floorf((tmin[2] - params->bmin[2]) / cs);
		int x1 = (int)floorf((tmax[0] - params->bmin[0]) / cs);
		int y1 = (int)floorf((tmax[1] - params->bmin[1]) / cs);
		int z1 = (int)floorf((tmax[2] - params->bmin[2]) / cs);
		if (x0 < 0) x0 = 0;
		if (y0 < 0) y0 = 0;
		if (z0 < 0) z0 = 0;
		if (x1 >= w) x1 = w - 1;
		if (y1 >= h) y1 = h - 1;
		if (z1 >= d) z1 = d - 1;

		for (int z = z0; z <= z1; ++z)
		{
			for (int y = y0; y <= y1; ++y)
			{
				for (int x = x0; x <= x1; ++x)
				{
					const float center[3] = {
						params->bmin[0] + (x + 0.5f) * cs,
						params->bmin[1] + (y + 0.5f) * cs,
						params->bmin[2] + (z + 0.5f) * cs,
					};
					if (overlapTriBox(center, halfBox, a, b, c))
					{
						const unsigned long long idx =
							((unsigned long long)z * h + y) * w + x;
						m_solid[idx >> 5] |= 1u << (idx & 31);
					}
				}
			}
		}
	}

	// --- 2. Extract exposed faces. ---
	int cap = 1024;
	m_faces = (dtSurfaceFace*)dtAlloc(sizeof(dtSurfaceFace) * cap, DT_ALLOC_PERM);
	if (!m_faces)
		return DT_FAILURE | DT_OUT_OF_MEMORY;

	for (int z = 0; z < d; ++z)
	{
		for (int y = 0; y < h; ++y)
		{
			for (int x = 0; x < w; ++x)
			{
				if (!solidAt(x, y, z))
					continue;
				for (int dir = 0; dir < 6; ++dir)
				{
					const int nx = x + DIR_VEC[dir][0];
					const int ny = y + DIR_VEC[dir][1];
					const int nz = z + DIR_VEC[dir][2];
					if (solidAt(nx, ny, nz))
						continue;

					if (m_faceCount == cap)
					{
						const int ncap = cap * 2;
						dtSurfaceFace* nf = (dtSurfaceFace*)dtAlloc(sizeof(dtSurfaceFace) * ncap, DT_ALLOC_PERM);
						if (!nf)
							return DT_FAILURE | DT_OUT_OF_MEMORY;
						memcpy(nf, m_faces, sizeof(dtSurfaceFace) * m_faceCount);
						dtFree(m_faces);
						m_faces = nf;
						cap = ncap;
					}
					// Face index must fit the 20 low bits of the lookup key.
					if (m_faceCount >= (1 << 20))
						return DT_FAILURE | DT_BUFFER_TOO_SMALL;

					dtSurfaceFace& f = m_faces[m_faceCount++];
					f.x = (unsigned short)x;
					f.y = (unsigned short)y;
					f.z = (unsigned short)z;
					f.dir = (unsigned char)dir;
					f.active = 1;
					f.region = -1;
					f.neis[0] = f.neis[1] = f.neis[2] = f.neis[3] = -1;
				}
			}
		}
	}

	if (m_faceCount == 0)
		return DT_SUCCESS;	// Empty but valid.

	// --- 3. Sorted key table for face lookup. ---
	m_faceKeys = (unsigned long long*)dtAlloc(sizeof(unsigned long long) * m_faceCount, DT_ALLOC_PERM);
	if (!m_faceKeys)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	for (int i = 0; i < m_faceCount; ++i)
	{
		const dtSurfaceFace& f = m_faces[i];
		const unsigned long long key =
			((((unsigned long long)f.z * m_height + f.y) * m_width + f.x) * 6 + f.dir);
		m_faceKeys[i] = (key << 20) | (unsigned long long)i;
	}
	qsort(m_faceKeys, m_faceCount, sizeof(unsigned long long), compareKeys);

	// --- 4. Adjacency. ---
	buildAdjacency();

	// --- 5. Region filtering. ---
	if (params->minRegionFaces > 1)
		filterRegions(params->minRegionFaces);

	return DT_SUCCESS;
}

void dtSurfaceNav::buildAdjacency()
{
	for (int i = 0; i < m_faceCount; ++i)
	{
		dtSurfaceFace& f = m_faces[i];
		const int axis = f.dir >> 1;
		const int* n = DIR_VEC[f.dir];

		for (int s = 0; s < 4; ++s)
		{
			const int tdir = LATERAL_DIRS[axis][s];
			const int* t = DIR_VEC[tdir];

			const int vx = f.x, vy = f.y, vz = f.z;

			// Concave transition: a wall voxel blocks the standing space in
			// direction t; step onto its face pointing back at us.
			if (solidAt(vx + n[0] + t[0], vy + n[1] + t[1], vz + n[2] + t[2]))
			{
				f.neis[s] = faceIndexAt(vx + n[0] + t[0], vy + n[1] + t[1], vz + n[2] + t[2],
										oppositeDir(tdir));
			}
			// Coplanar neighbour.
			else if (solidAt(vx + t[0], vy + t[1], vz + t[2]))
			{
				f.neis[s] = faceIndexAt(vx + t[0], vy + t[1], vz + t[2], f.dir);
			}
			// Convex transition: wrap around the edge onto our own voxel's
			// side face.
			else
			{
				f.neis[s] = faceIndexAt(vx, vy, vz, tdir);
			}
		}
	}
}

void dtSurfaceNav::filterRegions(int minRegionFaces)
{
	int* stack = (int*)dtAlloc(sizeof(int) * m_faceCount, DT_ALLOC_TEMP);
	int* component = (int*)dtAlloc(sizeof(int) * m_faceCount, DT_ALLOC_TEMP);
	if (!stack || !component)
	{
		dtFree(stack);
		dtFree(component);
		return;
	}

	int regionId = 0;
	for (int i = 0; i < m_faceCount; ++i)
	{
		if (m_faces[i].region != -1)
			continue;

		// Flood fill over coplanar neighbours (same face direction).
		int nstack = 0, ncomp = 0;
		stack[nstack++] = i;
		m_faces[i].region = regionId;
		while (nstack > 0)
		{
			const int cur = stack[--nstack];
			component[ncomp++] = cur;
			const dtSurfaceFace& f = m_faces[cur];
			for (int s = 0; s < 4; ++s)
			{
				const int nei = f.neis[s];
				if (nei < 0 || m_faces[nei].region != -1)
					continue;
				if (m_faces[nei].dir != f.dir)
					continue;	// Only grow across coplanar links.
				m_faces[nei].region = regionId;
				stack[nstack++] = nei;
			}
		}

		if (ncomp < minRegionFaces)
		{
			for (int c = 0; c < ncomp; ++c)
				m_faces[component[c]].active = 0;
		}
		++regionId;
	}

	// Detach culled faces from the graph.
	for (int i = 0; i < m_faceCount; ++i)
	{
		for (int s = 0; s < 4; ++s)
		{
			const int nei = m_faces[i].neis[s];
			if (nei >= 0 && !m_faces[nei].active)
				m_faces[i].neis[s] = -1;
		}
	}

	dtFree(stack);
	dtFree(component);
}

const dtSurfaceFace* dtSurfaceNav::getFace(int i) const
{
	if (i < 0 || i >= m_faceCount)
		return 0;
	return &m_faces[i];
}

void dtSurfaceNav::getFaceCenter(int i, float* center) const
{
	const dtSurfaceFace& f = m_faces[i];
	const float cs = m_params.cellSize;
	const int* n = DIR_VEC[f.dir];
	center[0] = m_params.bmin[0] + (f.x + 0.5f) * cs + n[0] * 0.5f * cs;
	center[1] = m_params.bmin[1] + (f.y + 0.5f) * cs + n[1] * 0.5f * cs;
	center[2] = m_params.bmin[2] + (f.z + 0.5f) * cs + n[2] * 0.5f * cs;
}

void dtSurfaceNav::getFaceNormal(int i, float* normal) const
{
	const int* n = DIR_VEC[m_faces[i].dir];
	normal[0] = (float)n[0];
	normal[1] = (float)n[1];
	normal[2] = (float)n[2];
}

bool dtSurfaceNav::passFilter(int i, const dtSurfaceNavFilter* filter) const
{
	if (i < 0 || i >= m_faceCount || !m_faces[i].active)
		return false;
	if (filter->isClimbing())
		return true;

	float center[3], normal[3], up[3];
	getFaceCenter(i, center);
	getFaceNormal(i, normal);
	localUpAt(filter, center, up);
	return dtVdot(normal, up) >= filter->getMaxSlopeCos();
}

int dtSurfaceNav::findNearestFace(const float* pos, float maxRadius,
								  const dtSurfaceNavFilter* filter) const
{
	int best = -1;
	float bestDistSqr = maxRadius > 0 ? maxRadius * maxRadius : -1.0f;
	for (int i = 0; i < m_faceCount; ++i)
	{
		if (!passFilter(i, filter))
			continue;
		float center[3];
		getFaceCenter(i, center);
		const float distSqr = dtVdistSqr(pos, center);
		if (best == -1 && bestDistSqr < 0)
		{
			best = i;
			bestDistSqr = distSqr;
		}
		else if (distSqr < bestDistSqr)
		{
			best = i;
			bestDistSqr = distSqr;
		}
	}
	return best;
}

dtStatus dtSurfaceNav::findPath(int startFace, int endFace,
								const dtSurfaceNavFilter* filter,
								int* path, int* pathCount, int maxPath) const
{
	if (!pathCount)
		return DT_FAILURE | DT_INVALID_PARAM;
	*pathCount = 0;
	if (!path || maxPath <= 0 || !filter)
		return DT_FAILURE | DT_INVALID_PARAM;
	if (!passFilter(startFace, filter) || !passFilter(endFace, filter))
		return DT_FAILURE | DT_INVALID_PARAM;

	if (startFace == endFace)
	{
		path[0] = startFace;
		*pathCount = 1;
		return DT_SUCCESS;
	}

	const int n = m_faceCount;
	float* cost = (float*)dtAlloc(sizeof(float) * n, DT_ALLOC_TEMP);
	int* parent = (int*)dtAlloc(sizeof(int) * n, DT_ALLOC_TEMP);
	unsigned char* state = (unsigned char*)dtAlloc(sizeof(unsigned char) * n, DT_ALLOC_TEMP);
	int* heap = (int*)dtAlloc(sizeof(int) * n, DT_ALLOC_TEMP);
	float* heapKey = (float*)dtAlloc(sizeof(float) * n, DT_ALLOC_TEMP);
	if (!cost || !parent || !state || !heap || !heapKey)
	{
		dtFree(cost); dtFree(parent); dtFree(state); dtFree(heap); dtFree(heapKey);
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	}
	memset(state, 0, (size_t)n);	// 0 = unseen, 1 = open, 2 = closed

	float endCenter[3];
	getFaceCenter(endFace, endCenter);

	int heapSize = 0;

	// Push helper is expressed inline (C++98, no lambdas).
#define HEAP_PUSH(idx, key) \
	do { \
		int hole = heapSize++; \
		while (hole > 0 && heapKey[(hole - 1) / 2] > (key)) \
		{ \
			heap[hole] = heap[(hole - 1) / 2]; \
			heapKey[hole] = heapKey[(hole - 1) / 2]; \
			hole = (hole - 1) / 2; \
		} \
		heap[hole] = (idx); \
		heapKey[hole] = (key); \
	} while (0)

	cost[startFace] = 0;
	parent[startFace] = -1;
	state[startFace] = 1;
	{
		float sc[3];
		getFaceCenter(startFace, sc);
		HEAP_PUSH(startFace, dtVdist(sc, endCenter));
	}

	bool found = false;
	while (heapSize > 0)
	{
		// Pop min.
		const int cur = heap[0];
		--heapSize;
		if (heapSize > 0)
		{
			const int moved = heap[heapSize];
			const float movedKey = heapKey[heapSize];
			int hole = 0;
			for (;;)
			{
				int child = hole * 2 + 1;
				if (child >= heapSize)
					break;
				if (child + 1 < heapSize && heapKey[child + 1] < heapKey[child])
					++child;
				if (heapKey[child] >= movedKey)
					break;
				heap[hole] = heap[child];
				heapKey[hole] = heapKey[child];
				hole = child;
			}
			heap[hole] = moved;
			heapKey[hole] = movedKey;
		}

		if (state[cur] == 2)
			continue;
		state[cur] = 2;

		if (cur == endFace)
		{
			found = true;
			break;
		}

		float curCenter[3];
		getFaceCenter(cur, curCenter);

		const dtSurfaceFace& f = m_faces[cur];
		for (int s = 0; s < 4; ++s)
		{
			const int nei = f.neis[s];
			if (nei < 0 || state[nei] == 2)
				continue;
			if (!passFilter(nei, filter))
				continue;

			float neiCenter[3];
			getFaceCenter(nei, neiCenter);
			const float newCost = cost[cur] + dtVdist(curCenter, neiCenter);
			if (state[nei] == 1 && newCost >= cost[nei])
				continue;

			cost[nei] = newCost;
			parent[nei] = cur;
			state[nei] = 1;
			HEAP_PUSH(nei, newCost + dtVdist(neiCenter, endCenter));
		}
	}
#undef HEAP_PUSH

	dtStatus status = DT_FAILURE;
	if (found)
	{
		// Count path length.
		int len = 0;
		for (int cur = endFace; cur != -1; cur = parent[cur])
			++len;
		// Write out (truncate from the start if needed, keeping the end).
		status = DT_SUCCESS;
		int writeCount = len;
		if (writeCount > maxPath)
		{
			writeCount = maxPath;
			status |= DT_BUFFER_TOO_SMALL;
		}
		// The parent chain runs end -> start. Skip the tail so the kept
		// entries are the first writeCount faces from the start (matching
		// Detour's partial-path convention).
		int cur = endFace;
		for (int s = 0; s < len - writeCount; ++s)
			cur = parent[cur];
		int idx = writeCount - 1;
		while (idx >= 0 && cur != -1)
		{
			path[idx--] = cur;
			cur = parent[cur];
		}
		*pathCount = writeCount;
	}

	dtFree(cost);
	dtFree(parent);
	dtFree(state);
	dtFree(heap);
	dtFree(heapKey);
	return status;
}

bool dtSurfaceNav::isSolid(const float* pos) const
{
	if (!m_solid)
		return false;
	const float cs = m_params.cellSize;
	const int x = (int)floorf((pos[0] - m_params.bmin[0]) / cs);
	const int y = (int)floorf((pos[1] - m_params.bmin[1]) / cs);
	const int z = (int)floorf((pos[2] - m_params.bmin[2]) / cs);
	return solidAt(x, y, z);
}

dtSurfaceNav* dtAllocSurfaceNav()
{
	void* mem = dtAlloc(sizeof(dtSurfaceNav), DT_ALLOC_PERM);
	if (!mem)
		return 0;
	return new(mem) dtSurfaceNav;
}

void dtFreeSurfaceNav(dtSurfaceNav* nav)
{
	if (!nav)
		return;
	nav->~dtSurfaceNav();
	dtFree(nav);
}
