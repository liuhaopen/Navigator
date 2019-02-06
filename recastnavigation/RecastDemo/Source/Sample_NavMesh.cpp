#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "SDL.h"
#include "SDL_opengl.h"
#include "imgui.h"
#include "InputGeom.h"
#include "Sample.h"
#include "Sample_NavMesh.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "RecastDump.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourDebugDraw.h"
#include "NavMeshTesterTool.h"
#include "NavMeshPruneTool.h"
#include "OffMeshConnectionTool.h"
#include "ConvexVolumeTool.h"
#include "CrowdTool.h"
#include "Recast.h"
#include "NavMeshLoader.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif


Sample_NavMesh::Sample_NavMesh() :
	m_keepInterResults(true),
	m_totalBuildTimeMs(0),
	m_triareas(0),
	m_solid(0),
	m_chf(0),
	m_cset(0),
	m_pmesh(0),
	m_dmesh(0),
	m_drawMode(DRAWMODE_NAVMESH)
{
	setTool(new NavMeshTesterTool);
}

Sample_NavMesh::~Sample_NavMesh()
{
	cleanup();
}

void Sample_NavMesh::cleanup()
{
	delete[] m_triareas;
	m_triareas = 0;
	rcFreeHeightField(m_solid);
	m_solid = 0;
	rcFreeCompactHeightfield(m_chf);
	m_chf = 0;
	rcFreeContourSet(m_cset);
	m_cset = 0;
	rcFreePolyMesh(m_pmesh);
	m_pmesh = 0;
	rcFreePolyMeshDetail(m_dmesh);
	m_dmesh = 0;
	dtFreeNavMesh(m_navMesh);
	m_navMesh = 0;
}

void Sample_NavMesh::handleSettings()
{
	Sample::handleCommonSettings();

	if (imguiCheck("Keep Itermediate Results", m_keepInterResults))
		m_keepInterResults = !m_keepInterResults;

	imguiSeparator();

	imguiIndent();
	imguiIndent();

	if (imguiButton("Save"))
	{
		Sample::saveAll("./unity_navmesh.bin", m_navMesh);
	}

	if (imguiButton("Load"))
	{
		dtFreeNavMesh(m_navMesh);
		m_navMesh = Sample::loadAll("unity_navmesh.bin");
		m_navQuery->init(m_navMesh, 2048);
	}

	imguiUnindent();
	imguiUnindent();

	char msg[64];
	snprintf(msg, 64, "Build Time: %.1fms", m_totalBuildTimeMs);
	imguiLabel(msg);

	imguiSeparator();
}

void Sample_NavMesh::handleTools()
{
	int type = !m_tool ? TOOL_NONE : m_tool->type();

	if (imguiCheck("Test Navmesh", type == TOOL_NAVMESH_TESTER))
	{
		setTool(new NavMeshTesterTool);
	}
	if (imguiCheck("Prune Navmesh", type == TOOL_NAVMESH_PRUNE))
	{
		setTool(new NavMeshPruneTool);
	}
	if (imguiCheck("Create Off-Mesh Connections", type == TOOL_OFFMESH_CONNECTION))
	{
		setTool(new OffMeshConnectionTool);
	}
	if (imguiCheck("Create Convex Volumes", type == TOOL_CONVEX_VOLUME))
	{
		setTool(new ConvexVolumeTool);
	}
	if (imguiCheck("Create Crowds", type == TOOL_CROWD))
	{
		setTool(new CrowdTool);
	}

	imguiSeparatorLine();

	imguiIndent();

	if (m_tool)
		m_tool->handleMenu();

	imguiUnindent();

}

void Sample_NavMesh::handleDebugMode()
{
	// Check which modes are valid.
	bool valid[MAX_DRAWMODE];
	for (int i = 0; i < MAX_DRAWMODE; ++i)
		valid[i] = false;

	if (m_geom)
	{
		valid[DRAWMODE_NAVMESH] = m_navMesh != 0;
		valid[DRAWMODE_NAVMESH_TRANS] = m_navMesh != 0;
		valid[DRAWMODE_NAVMESH_BVTREE] = m_navMesh != 0;
		valid[DRAWMODE_NAVMESH_NODES] = m_navQuery != 0;
		valid[DRAWMODE_NAVMESH_INVIS] = m_navMesh != 0;
		valid[DRAWMODE_MESH] = true;
		valid[DRAWMODE_VOXELS] = m_solid != 0;
		valid[DRAWMODE_VOXELS_WALKABLE] = m_solid != 0;
		valid[DRAWMODE_COMPACT] = m_chf != 0;
		valid[DRAWMODE_COMPACT_DISTANCE] = m_chf != 0;
		valid[DRAWMODE_COMPACT_REGIONS] = m_chf != 0;
		valid[DRAWMODE_REGION_CONNECTIONS] = m_cset != 0;
		valid[DRAWMODE_RAW_CONTOURS] = m_cset != 0;
		valid[DRAWMODE_BOTH_CONTOURS] = m_cset != 0;
		valid[DRAWMODE_CONTOURS] = m_cset != 0;
		valid[DRAWMODE_POLYMESH] = m_pmesh != 0;
		valid[DRAWMODE_POLYMESH_DETAIL] = m_dmesh != 0;
	}

	int unavail = 0;
	for (int i = 0; i < MAX_DRAWMODE; ++i)
		if (!valid[i]) unavail++;

	if (unavail == MAX_DRAWMODE)
		return;

	imguiLabel("Draw");
	if (imguiCheck("Input Mesh", m_drawMode == DRAWMODE_MESH, valid[DRAWMODE_MESH]))
		m_drawMode = DRAWMODE_MESH;
	if (imguiCheck("Navmesh", m_drawMode == DRAWMODE_NAVMESH, valid[DRAWMODE_NAVMESH]))
		m_drawMode = DRAWMODE_NAVMESH;
	if (imguiCheck("Navmesh Invis", m_drawMode == DRAWMODE_NAVMESH_INVIS, valid[DRAWMODE_NAVMESH_INVIS]))
		m_drawMode = DRAWMODE_NAVMESH_INVIS;
	if (imguiCheck("Navmesh Trans", m_drawMode == DRAWMODE_NAVMESH_TRANS, valid[DRAWMODE_NAVMESH_TRANS]))
		m_drawMode = DRAWMODE_NAVMESH_TRANS;
	if (imguiCheck("Navmesh BVTree", m_drawMode == DRAWMODE_NAVMESH_BVTREE, valid[DRAWMODE_NAVMESH_BVTREE]))
		m_drawMode = DRAWMODE_NAVMESH_BVTREE;
	if (imguiCheck("Navmesh Nodes", m_drawMode == DRAWMODE_NAVMESH_NODES, valid[DRAWMODE_NAVMESH_NODES]))
		m_drawMode = DRAWMODE_NAVMESH_NODES;
	if (imguiCheck("Voxels", m_drawMode == DRAWMODE_VOXELS, valid[DRAWMODE_VOXELS]))
		m_drawMode = DRAWMODE_VOXELS;
	if (imguiCheck("Walkable Voxels", m_drawMode == DRAWMODE_VOXELS_WALKABLE, valid[DRAWMODE_VOXELS_WALKABLE]))
		m_drawMode = DRAWMODE_VOXELS_WALKABLE;
	if (imguiCheck("Compact", m_drawMode == DRAWMODE_COMPACT, valid[DRAWMODE_COMPACT]))
		m_drawMode = DRAWMODE_COMPACT;
	if (imguiCheck("Compact Distance", m_drawMode == DRAWMODE_COMPACT_DISTANCE, valid[DRAWMODE_COMPACT_DISTANCE]))
		m_drawMode = DRAWMODE_COMPACT_DISTANCE;
	if (imguiCheck("Compact Regions", m_drawMode == DRAWMODE_COMPACT_REGIONS, valid[DRAWMODE_COMPACT_REGIONS]))
		m_drawMode = DRAWMODE_COMPACT_REGIONS;
	if (imguiCheck("Region Connections", m_drawMode == DRAWMODE_REGION_CONNECTIONS, valid[DRAWMODE_REGION_CONNECTIONS]))
		m_drawMode = DRAWMODE_REGION_CONNECTIONS;
	if (imguiCheck("Raw Contours", m_drawMode == DRAWMODE_RAW_CONTOURS, valid[DRAWMODE_RAW_CONTOURS]))
		m_drawMode = DRAWMODE_RAW_CONTOURS;
	if (imguiCheck("Both Contours", m_drawMode == DRAWMODE_BOTH_CONTOURS, valid[DRAWMODE_BOTH_CONTOURS]))
		m_drawMode = DRAWMODE_BOTH_CONTOURS;
	if (imguiCheck("Contours", m_drawMode == DRAWMODE_CONTOURS, valid[DRAWMODE_CONTOURS]))
		m_drawMode = DRAWMODE_CONTOURS;
	if (imguiCheck("Poly Mesh", m_drawMode == DRAWMODE_POLYMESH, valid[DRAWMODE_POLYMESH]))
		m_drawMode = DRAWMODE_POLYMESH;
	if (imguiCheck("Poly Mesh Detail", m_drawMode == DRAWMODE_POLYMESH_DETAIL, valid[DRAWMODE_POLYMESH_DETAIL]))
		m_drawMode = DRAWMODE_POLYMESH_DETAIL;

	if (unavail)
	{
		imguiValue("Tick 'Keep Itermediate Results'");
		imguiValue("to see more debug mode options.");
	}
}

void Sample_NavMesh::handleRender()
{
	if (!m_geom || !m_geom->getMesh())
		return;

	glEnable(GL_FOG);
	glDepthMask(GL_TRUE);

	const float texScale = 1.0f / (m_cellSize * 10.0f);

	if (m_drawMode != DRAWMODE_NAVMESH_TRANS)
	{
		// Draw mesh
		duDebugDrawTriMeshSlope(&m_dd, m_geom->getMesh()->getVerts(), m_geom->getMesh()->getVertCount(),
			m_geom->getMesh()->getTris(), m_geom->getMesh()->getNormals(), m_geom->getMesh()->getTriCount(),
			m_agentMaxSlope, texScale);
		m_geom->drawOffMeshConnections(&m_dd);
	}

	glDisable(GL_FOG);
	glDepthMask(GL_FALSE);

	// Draw bounds
	const float* bmin = m_geom->getNavMeshBoundsMin();
	const float* bmax = m_geom->getNavMeshBoundsMax();
	duDebugDrawBoxWire(&m_dd, bmin[0], bmin[1], bmin[2], bmax[0], bmax[1], bmax[2], duRGBA(255, 255, 255, 128), 1.0f);
	m_dd.begin(DU_DRAW_POINTS, 5.0f);
	m_dd.vertex(bmin[0], bmin[1], bmin[2], duRGBA(255, 255, 255, 128));
	m_dd.end();

	if (m_navMesh && m_navQuery &&
		(m_drawMode == DRAWMODE_NAVMESH ||
			m_drawMode == DRAWMODE_NAVMESH_TRANS ||
			m_drawMode == DRAWMODE_NAVMESH_BVTREE ||
			m_drawMode == DRAWMODE_NAVMESH_NODES ||
			m_drawMode == DRAWMODE_NAVMESH_INVIS))
	{
		if (m_drawMode != DRAWMODE_NAVMESH_INVIS)
			duDebugDrawNavMeshWithClosedList(&m_dd, *m_navMesh, *m_navQuery, m_navMeshDrawFlags);
		if (m_drawMode == DRAWMODE_NAVMESH_BVTREE)
			duDebugDrawNavMeshBVTree(&m_dd, *m_navMesh);
		if (m_drawMode == DRAWMODE_NAVMESH_NODES)
			duDebugDrawNavMeshNodes(&m_dd, *m_navQuery);
		duDebugDrawNavMeshPolysWithFlags(&m_dd, *m_navMesh, SAMPLE_POLYFLAGS_DISABLED, duRGBA(0, 0, 0, 128));
	}

	glDepthMask(GL_TRUE);

	if (m_chf && m_drawMode == DRAWMODE_COMPACT)
		duDebugDrawCompactHeightfieldSolid(&m_dd, *m_chf);

	if (m_chf && m_drawMode == DRAWMODE_COMPACT_DISTANCE)
		duDebugDrawCompactHeightfieldDistance(&m_dd, *m_chf);
	if (m_chf && m_drawMode == DRAWMODE_COMPACT_REGIONS)
		duDebugDrawCompactHeightfieldRegions(&m_dd, *m_chf);
	if (m_solid && m_drawMode == DRAWMODE_VOXELS)
	{
		glEnable(GL_FOG);
		duDebugDrawHeightfieldSolid(&m_dd, *m_solid);
		glDisable(GL_FOG);
	}
	if (m_solid && m_drawMode == DRAWMODE_VOXELS_WALKABLE)
	{
		glEnable(GL_FOG);
		duDebugDrawHeightfieldWalkable(&m_dd, *m_solid);
		glDisable(GL_FOG);
	}
	if (m_cset && m_drawMode == DRAWMODE_RAW_CONTOURS)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawRawContours(&m_dd, *m_cset);
		glDepthMask(GL_TRUE);
	}
	if (m_cset && m_drawMode == DRAWMODE_BOTH_CONTOURS)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawRawContours(&m_dd, *m_cset, 0.5f);
		duDebugDrawContours(&m_dd, *m_cset);
		glDepthMask(GL_TRUE);
	}
	if (m_cset && m_drawMode == DRAWMODE_CONTOURS)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawContours(&m_dd, *m_cset);
		glDepthMask(GL_TRUE);
	}
	if (m_chf && m_cset && m_drawMode == DRAWMODE_REGION_CONNECTIONS)
	{
		duDebugDrawCompactHeightfieldRegions(&m_dd, *m_chf);

		glDepthMask(GL_FALSE);
		duDebugDrawRegionConnections(&m_dd, *m_cset);
		glDepthMask(GL_TRUE);
	}
	if (m_pmesh && m_drawMode == DRAWMODE_POLYMESH)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawPolyMesh(&m_dd, *m_pmesh);
		glDepthMask(GL_TRUE);
	}
	if (m_dmesh && m_drawMode == DRAWMODE_POLYMESH_DETAIL)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawPolyMeshDetail(&m_dd, *m_dmesh);
		glDepthMask(GL_TRUE);
	}

	m_geom->drawConvexVolumes(&m_dd);

	if (m_tool)
		m_tool->handleRender();
	renderToolStates();

	glDepthMask(GL_TRUE);
}

void Sample_NavMesh::handleRenderOverlay(double* proj, double* model, int* view)
{
	if (m_tool)
		m_tool->handleRenderOverlay(proj, model, view);
	renderOverlayToolStates(proj, model, view);
}

void Sample_NavMesh::handleMeshChanged(class InputGeom* geom)
{
	Sample::handleMeshChanged(geom);

	dtFreeNavMesh(m_navMesh);
	m_navMesh = 0;

	if (m_tool)
	{
		m_tool->reset();
		m_tool->init(this);
	}
	resetToolStates();
	initToolStates(this);
}


bool Sample_NavMesh::handleBuild()
{
	if (!m_geom || !m_geom->getMesh())
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Input mesh is not specified.");
		return false;
	}

	cleanup();

	const float* bmin = m_geom->getNavMeshBoundsMin();
	const float* bmax = m_geom->getNavMeshBoundsMax();
	const float* verts = m_geom->getMesh()->getVerts();
	const int nverts = m_geom->getMesh()->getVertCount();
	const int* tris = m_geom->getMesh()->getTris();
	const int ntris = m_geom->getMesh()->getTriCount();

	//
	// Step 1. Initialize build config.
	//

	// Init build configuration from GUI
	memset(&m_cfg, 0, sizeof(m_cfg));
	m_cfg.cs = m_cellSize;
	m_cfg.ch = m_cellHeight;
	m_cfg.walkableSlopeAngle = m_agentMaxSlope;
	m_cfg.walkableHeight = (int)ceilf(m_agentHeight / m_cfg.ch);
	m_cfg.walkableClimb = (int)floorf(m_agentMaxClimb / m_cfg.ch);
	m_cfg.walkableRadius = (int)ceilf(m_agentRadius / m_cfg.cs);
	m_cfg.maxEdgeLen = (int)(m_edgeMaxLen / m_cellSize);
	m_cfg.maxSimplificationError = m_edgeMaxError;
	m_cfg.minRegionArea = (int)rcSqr(m_regionMinSize);		// Note: area = size*size
	m_cfg.mergeRegionArea = (int)rcSqr(m_regionMergeSize);	// Note: area = size*size
	m_cfg.maxVertsPerPoly = (int)m_vertsPerPoly;
	m_cfg.detailSampleDist = m_detailSampleDist < 0.9f ? 0 : m_cellSize * m_detailSampleDist;
	m_cfg.detailSampleMaxError = m_cellHeight * m_detailSampleMaxError;

	// Set the area where the navigation will be build.
	// Here the bounds of the input mesh are used, but the
	// area could be specified by an user defined box, etc.
	rcVcopy(m_cfg.bmin, bmin);
	rcVcopy(m_cfg.bmax, bmax);
	rcCalcGridSize(m_cfg.bmin, m_cfg.bmax, m_cfg.cs, &m_cfg.width, &m_cfg.height);

	// Reset build times gathering.
	m_ctx->resetTimers();

	// Start the build process.	
	m_ctx->startTimer(RC_TIMER_TOTAL);

	m_ctx->log(RC_LOG_PROGRESS, "Building navigation:");
	m_ctx->log(RC_LOG_PROGRESS, " - %d x %d cells", m_cfg.width, m_cfg.height);
	m_ctx->log(RC_LOG_PROGRESS, " - %.1fK verts, %.1fK tris", nverts / 1000.0f, ntris / 1000.0f);

	//
	// Step 2. Rasterize input polygon soup.
	//

	// Allocate voxel heightfield where we rasterize our input data to.
	if (false)
	{
		m_solid = rcAllocHeightfield();
		if (!m_solid)
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
			return false;
		}
		if (!rcCreateHeightfield(m_ctx, *m_solid, m_cfg.width, m_cfg.height, m_cfg.bmin, m_cfg.bmax, m_cfg.cs, m_cfg.ch))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
			return false;
		}

		// Allocate array that can hold triangle area types.
		// If you have multiple meshes you need to process, allocate
		// and array which can hold the max number of triangles you need to process.
		m_triareas = new unsigned char[ntris];
		if (!m_triareas)
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", ntris);
			return false;
		}

		// Find triangles which are walkable based on their slope and rasterize them.
		// If your input data is multiple meshes, you can transform them here, calculate
		// the are type for each of the meshes and rasterize them.
		memset(m_triareas, 0, ntris * sizeof(unsigned char));
		rcMarkWalkableTriangles(m_ctx, m_cfg.walkableSlopeAngle, verts, nverts, tris, ntris, m_triareas);
		if (!rcRasterizeTriangles(m_ctx, verts, nverts, tris, m_triareas, ntris, *m_solid, m_cfg.walkableClimb))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not rasterize triangles.");
			return false;
		}

		if (!m_keepInterResults)
		{
			delete[] m_triareas;
			m_triareas = 0;
		}

		//
		// Step 3. Filter walkables surfaces.
		//

		// Once all geoemtry is rasterized, we do initial pass of filtering to
		// remove unwanted overhangs caused by the conservative rasterization
		// as well as filter spans where the character cannot possibly stand.
		if (m_filterLowHangingObstacles)
			rcFilterLowHangingWalkableObstacles(m_ctx, m_cfg.walkableClimb, *m_solid);
		if (m_filterLedgeSpans)
			rcFilterLedgeSpans(m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, *m_solid);
		if (m_filterWalkableLowHeightSpans)
			rcFilterWalkableLowHeightSpans(m_ctx, m_cfg.walkableHeight, *m_solid);


		//
		// Step 4. Partition walkable surface to simple regions.
		//

		// Compact the heightfield so that it is faster to handle from now on.
		// This will result more cache coherent data as well as the neighbours
		// between walkable cells will be calculated.
		m_chf = rcAllocCompactHeightfield();
		if (!m_chf)
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
			return false;
		}
		if (!rcBuildCompactHeightfield(m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, *m_solid, *m_chf))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
			return false;
		}

		if (!m_keepInterResults)
		{
			rcFreeHeightField(m_solid);
			m_solid = 0;
		}

		// Erode the walkable area by agent radius.
		if (!rcErodeWalkableArea(m_ctx, m_cfg.walkableRadius, *m_chf))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
			return false;
		}

		// (Optional) Mark areas.
		const ConvexVolume* vols = m_geom->getConvexVolumes();
		for (int i = 0; i < m_geom->getConvexVolumeCount(); ++i)
			rcMarkConvexPolyArea(m_ctx, vols[i].verts, vols[i].nverts, vols[i].hmin, vols[i].hmax, (unsigned char)vols[i].area, *m_chf);


		// Partition the heightfield so that we can use simple algorithm later to triangulate the walkable areas.
		// There are 3 martitioning methods, each with some pros and cons:
		// 1) Watershed partitioning
		//   - the classic Recast partitioning
		//   - creates the nicest tessellation
		//   - usually slowest
		//   - partitions the heightfield into nice regions without holes or overlaps
		//   - the are some corner cases where this method creates produces holes and overlaps
		//      - holes may appear when a small obstacles is close to large open area (triangulation can handle this)
		//      - overlaps may occur if you have narrow spiral corridors (i.e stairs), this make triangulation to fail
		//   * generally the best choice if you precompute the nacmesh, use this if you have large open areas
		// 2) Monotone partioning
		//   - fastest
		//   - partitions the heightfield into regions without holes and overlaps (guaranteed)
		//   - creates long thin polygons, which sometimes causes paths with detours
		//   * use this if you want fast navmesh generation
		// 3) Layer partitoining
		//   - quite fast
		//   - partitions the heighfield into non-overlapping regions
		//   - relies on the triangulation code to cope with holes (thus slower than monotone partitioning)
		//   - produces better triangles than monotone partitioning
		//   - does not have the corner cases of watershed partitioning
		//   - can be slow and create a bit ugly tessellation (still better than monotone)
		//     if you have large open areas with small obstacles (not a problem if you use tiles)
		//   * good choice to use for tiled navmesh with medium and small sized tiles

		if (m_partitionType == SAMPLE_PARTITION_WATERSHED)
		{
			// Prepare for region partitioning, by calculating distance field along the walkable surface.
			if (!rcBuildDistanceField(m_ctx, *m_chf))
			{
				m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build distance field.");
				return false;
			}

			// Partition the walkable surface into simple regions without holes.
			if (!rcBuildRegions(m_ctx, *m_chf, 0, m_cfg.minRegionArea, m_cfg.mergeRegionArea))
			{
				m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build watershed regions.");
				return false;
			}
		}
		else if (m_partitionType == SAMPLE_PARTITION_MONOTONE)
		{
			// Partition the walkable surface into simple regions without holes.
			// Monotone partitioning does not need distancefield.
			if (!rcBuildRegionsMonotone(m_ctx, *m_chf, 0, m_cfg.minRegionArea, m_cfg.mergeRegionArea))
			{
				m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build monotone regions.");
				return false;
			}
		}
		else // SAMPLE_PARTITION_LAYERS
		{
			// Partition the walkable surface into simple regions without holes.
			if (!rcBuildLayerRegions(m_ctx, *m_chf, 0, m_cfg.minRegionArea))
			{
				m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build layer regions.");
				return false;
			}
		}

		//
		// Step 5. Trace and simplify region contours.
		//

		// Create contours.
		m_cset = rcAllocContourSet();
		if (!m_cset)
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
			return false;
		}
		if (!rcBuildContours(m_ctx, *m_chf, m_cfg.maxSimplificationError, m_cfg.maxEdgeLen, *m_cset))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
			return false;
		}
	}
	
	//// Create contours.
	//m_cset = rcAllocContourSet();
	//if (!m_cset)
	//{
	//	m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
	//	return false;
	//}
	//if (!buildContoursByMesh(m_ctx, m_geom, m_cfg.maxSimplificationError, m_cfg.maxEdgeLen, *m_cset))
	//{
	//	m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
	//	return false;
	//}
	//
	// Step 6. Build polygons mesh from contours.
	//

	// Build polygon navmesh from the contours.
	m_pmesh = rcAllocPolyMesh();
	if (!m_pmesh)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmesh'.");
		return false;
	}
	NavMeshLoader::FullPolyDataFromJson("navmesh.json", *m_pmesh);
	/*if (!rcBuildPolyMesh(m_ctx, *m_cset, m_cfg.maxVertsPerPoly, *m_pmesh))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours.");
		return false;
	}*/

	//
	// Step 7. Create detail mesh which allows to access approximate height on each polygon.
	//

	/*m_dmesh = rcAllocPolyMeshDetail();
	if (!m_dmesh)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmdtl'.");
		return false;
	}

	if (!rcBuildPolyMeshDetail(m_ctx, *m_pmesh, *m_chf, m_cfg.detailSampleDist, m_cfg.detailSampleMaxError, *m_dmesh))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build detail mesh.");
		return false;
	}*/

	if (!m_keepInterResults)
	{
		rcFreeCompactHeightfield(m_chf);
		m_chf = 0;
		rcFreeContourSet(m_cset);
		m_cset = 0;
	}

	// At this point the navigation mesh data is ready, you can access it from m_pmesh.
	// See duDebugDrawPolyMesh or dtCreateNavMeshData as examples how to access the data.

	//
	// (Optional) Step 8. Create Detour data from Recast poly mesh.
	//

	// The GUI may allow more max points per polygon than Detour can handle.
	// Only build the detour navmesh if we do not exceed the limit.
	if (m_cfg.maxVertsPerPoly <= DT_VERTS_PER_POLYGON)
	{
		unsigned char* navData = 0;
		int navDataSize = 0;

		// Update poly flags from areas.
		for (int i = 0; i < m_pmesh->npolys; ++i)
		{
			if (m_pmesh->areas[i] == RC_WALKABLE_AREA)
				m_pmesh->areas[i] = SAMPLE_POLYAREA_GROUND;

			if (m_pmesh->areas[i] == SAMPLE_POLYAREA_GROUND ||
				m_pmesh->areas[i] == SAMPLE_POLYAREA_GRASS ||
				m_pmesh->areas[i] == SAMPLE_POLYAREA_ROAD)
			{
				m_pmesh->flags[i] = SAMPLE_POLYFLAGS_WALK;
			}
			else if (m_pmesh->areas[i] == SAMPLE_POLYAREA_WATER)
			{
				m_pmesh->flags[i] = SAMPLE_POLYFLAGS_SWIM;
			}
			else if (m_pmesh->areas[i] == SAMPLE_POLYAREA_DOOR)
			{
				m_pmesh->flags[i] = SAMPLE_POLYFLAGS_WALK | SAMPLE_POLYFLAGS_DOOR;
			}
		}


		dtNavMeshCreateParams params;
		memset(&params, 0, sizeof(params));
		params.verts = m_pmesh->verts;
		params.vertCount = m_pmesh->nverts;
		params.polys = m_pmesh->polys;
		params.polyAreas = m_pmesh->areas;
		params.polyFlags = m_pmesh->flags;
		params.polyCount = m_pmesh->npolys;
		params.nvp = m_pmesh->nvp;
		/*params.detailMeshes = m_dmesh->meshes;
		params.detailVerts = m_dmesh->verts;
		params.detailVertsCount = m_dmesh->nverts;
		params.detailTris = m_dmesh->tris;
		params.detailTriCount = m_dmesh->ntris;*/
		params.offMeshConVerts = m_geom->getOffMeshConnectionVerts();
		params.offMeshConRad = m_geom->getOffMeshConnectionRads();
		params.offMeshConDir = m_geom->getOffMeshConnectionDirs();
		params.offMeshConAreas = m_geom->getOffMeshConnectionAreas();
		params.offMeshConFlags = m_geom->getOffMeshConnectionFlags();
		params.offMeshConUserID = m_geom->getOffMeshConnectionId();
		params.offMeshConCount = m_geom->getOffMeshConnectionCount();
		params.walkableHeight = m_agentHeight;
		params.walkableRadius = m_agentRadius;
		params.walkableClimb = m_agentMaxClimb;
		rcVcopy(params.bmin, m_pmesh->bmin);
		rcVcopy(params.bmax, m_pmesh->bmax);
		params.cs = m_cfg.cs;
		params.ch = m_cfg.ch;
		params.buildBvTree = true;

		if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
		{
			m_ctx->log(RC_LOG_ERROR, "Could not build Detour navmesh.");
			return false;
		}

		m_navMesh = dtAllocNavMesh();
		if (!m_navMesh)
		{
			dtFree(navData);
			m_ctx->log(RC_LOG_ERROR, "Could not create Detour navmesh");
			return false;
		}

		dtStatus status;

		status = m_navMesh->init(navData, navDataSize, DT_TILE_FREE_DATA);
		if (dtStatusFailed(status))
		{
			dtFree(navData);
			m_ctx->log(RC_LOG_ERROR, "Could not init Detour navmesh");
			return false;
		}

		status = m_navQuery->init(m_navMesh, 2048);
		if (dtStatusFailed(status))
		{
			m_ctx->log(RC_LOG_ERROR, "Could not init Detour navmesh query");
			return false;
		}
	}

	m_ctx->stopTimer(RC_TIMER_TOTAL);

	// Show performance stats.
	duLogBuildTimes(*m_ctx, m_ctx->getAccumulatedTime(RC_TIMER_TOTAL));
	m_ctx->log(RC_LOG_PROGRESS, ">> Polymesh: %d vertices  %d polygons", m_pmesh->nverts, m_pmesh->npolys);

	m_totalBuildTimeMs = m_ctx->getAccumulatedTime(RC_TIMER_TOTAL) / 1000.0f;

	if (m_tool)
		m_tool->init(this);
	initToolStates(this);

	return true;
}

bool Sample_NavMesh::rcBuildPolyMesh2(rcContext * ctx, class InputGeom* geom, const int nvp, rcPolyMesh & mesh)
{
	//rcAssert(ctx);

	//rcScopedTimer timer(ctx, RC_TIMER_BUILD_POLYMESH);

	//rcVcopy(mesh.bmin, m_cfg.bmin);
	//rcVcopy(mesh.bmax, m_cfg.bmax);
	//mesh.cs = m_cfg.cs;
	//mesh.ch = m_cfg.ch;
	//mesh.borderSize = m_cfg.borderSize;
	//mesh.maxEdgeError = 10;//cat todo

	//const rcMeshLoaderObj* cur_mesh_obj = geom->getMesh();
	//if (cur_mesh_obj == NULL)
	//	return false;
	//int maxVertices = cur_mesh_obj->getVertCount();
	//int maxVertices = 0;
	//int maxTris = 0;
	//int maxVertsPerCont = 0;
	//for (int i = 0; i < cset.nconts; ++i)
	//{
	//	// Skip null contours.
	//	if (cset.conts[i].nverts < 3) continue;
	//	maxVertices += cset.conts[i].nverts;
	//	maxTris += cset.conts[i].nverts - 2;
	//	maxVertsPerCont = rcMax(maxVertsPerCont, cset.conts[i].nverts);
	//}

	//if (maxVertices >= 0xfffe)
	//{
	//	ctx->log(RC_LOG_ERROR, "rcBuildPolyMesh: Too many vertices %d.", maxVertices);
	//	return false;
	//}

	//rcScopedDelete<unsigned char> vflags((unsigned char*)rcAlloc(sizeof(unsigned char)*maxVertices, RC_ALLOC_TEMP));
	//if (!vflags)
	//{
	//	ctx->log(RC_LOG_ERROR, "rcBuildPolyMesh: Out of memory 'vflags' (%d).", maxVertices);
	//	return false;
	//}
	//memset(vflags, 0, maxVertices);

	//mesh.verts = (unsigned short*)rcAlloc(sizeof(unsigned short)*maxVertices * 3, RC_ALLOC_PERM);
	//if (!mesh.verts)
	//{
	//	ctx->log(RC_LOG_ERROR, "rcBuildPolyMesh: Out of memory 'mesh.verts' (%d).", maxVertices);
	//	return false;
	//}
	//mesh.polys = (unsigned short*)rcAlloc(sizeof(unsigned short)*maxTris*nvp * 2, RC_ALLOC_PERM);
	//if (!mesh.polys)
	//{
	//	ctx->log(RC_LOG_ERROR, "rcBuildPolyMesh: Out of memory 'mesh.polys' (%d).", maxTris*nvp * 2);
	//	return false;
	//}
	//mesh.regs = (unsigned short*)rcAlloc(sizeof(unsigned short)*maxTris, RC_ALLOC_PERM);
	//if (!mesh.regs)
	//{
	//	ctx->log(RC_LOG_ERROR, "rcBuildPolyMesh: Out of memory 'mesh.regs' (%d).", maxTris);
	//	return false;
	//}
	//mesh.areas = (unsigned char*)rcAlloc(sizeof(unsigned char)*maxTris, RC_ALLOC_PERM);
	//if (!mesh.areas)
	//{
	//	ctx->log(RC_LOG_ERROR, "rcBuildPolyMesh: Out of memory 'mesh.areas' (%d).", maxTris);
	//	return false;
	//}

	//mesh.nverts = 0;
	//mesh.npolys = 0;
	//mesh.nvp = nvp;
	//mesh.maxpolys = maxTris;

	//memset(mesh.verts, 0, sizeof(unsigned short)*maxVertices * 3);
	//memset(mesh.polys, 0xff, sizeof(unsigned short)*maxTris*nvp * 2);
	//memset(mesh.regs, 0, sizeof(unsigned short)*maxTris);
	//memset(mesh.areas, 0, sizeof(unsigned char)*maxTris);

	//rcScopedDelete<int> nextVert((int*)rcAlloc(sizeof(int)*maxVertices, RC_ALLOC_TEMP));
	//if (!nextVert)
	//{
	//	ctx->log(RC_LOG_ERROR, "rcBuildPolyMesh: Out of memory 'nextVert' (%d).", maxVertices);
	//	return false;
	//}
	//memset(nextVert, 0, sizeof(int)*maxVertices);

	//rcScopedDelete<int> firstVert((int*)rcAlloc(sizeof(int)*VERTEX_BUCKET_COUNT, RC_ALLOC_TEMP));
	//if (!firstVert)
	//{
	//	ctx->log(RC_LOG_ERROR, "rcBuildPolyMesh: Out of memory 'firstVert' (%d).", VERTEX_BUCKET_COUNT);
	//	return false;
	//}
	//for (int i = 0; i < VERTEX_BUCKET_COUNT; ++i)
	//	firstVert[i] = -1;

	//rcScopedDelete<int> indices((int*)rcAlloc(sizeof(int)*maxVertsPerCont, RC_ALLOC_TEMP));
	//if (!indices)
	//{
	//	ctx->log(RC_LOG_ERROR, "rcBuildPolyMesh: Out of memory 'indices' (%d).", maxVertsPerCont);
	//	return false;
	//}
	//rcScopedDelete<int> tris((int*)rcAlloc(sizeof(int)*maxVertsPerCont * 3, RC_ALLOC_TEMP));
	//if (!tris)
	//{
	//	ctx->log(RC_LOG_ERROR, "rcBuildPolyMesh: Out of memory 'tris' (%d).", maxVertsPerCont * 3);
	//	return false;
	//}
	//rcScopedDelete<unsigned short> polys((unsigned short*)rcAlloc(sizeof(unsigned short)*(maxVertsPerCont + 1)*nvp, RC_ALLOC_TEMP));
	//if (!polys)
	//{
	//	ctx->log(RC_LOG_ERROR, "rcBuildPolyMesh: Out of memory 'polys' (%d).", maxVertsPerCont*nvp);
	//	return false;
	//}
	//unsigned short* tmpPoly = &polys[maxVertsPerCont*nvp];

	//for (int i = 0; i < cset.nconts; ++i)
	//{
	//	rcContour& cont = cset.conts[i];

	//	// Skip null contours.
	//	if (cont.nverts < 3)
	//		continue;

	//	// Triangulate contour
	//	for (int j = 0; j < cont.nverts; ++j)
	//		indices[j] = j;

	//	int ntris = triangulate(cont.nverts, cont.verts, &indices[0], &tris[0]);
	//	if (ntris <= 0)
	//	{
	//		// Bad triangulation, should not happen.
	//		/*			printf("\tconst float bmin[3] = {%ff,%ff,%ff};\n", cset.bmin[0], cset.bmin[1], cset.bmin[2]);
	//		printf("\tconst float cs = %ff;\n", cset.cs);
	//		printf("\tconst float ch = %ff;\n", cset.ch);
	//		printf("\tconst int verts[] = {\n");
	//		for (int k = 0; k < cont.nverts; ++k)
	//		{
	//		const int* v = &cont.verts[k*4];
	//		printf("\t\t%d,%d,%d,%d,\n", v[0], v[1], v[2], v[3]);
	//		}
	//		printf("\t};\n\tconst int nverts = sizeof(verts)/(sizeof(int)*4);\n");*/
	//		ctx->log(RC_LOG_WARNING, "rcBuildPolyMesh: Bad triangulation Contour %d.", i);
	//		ntris = -ntris;
	//	}

	//	// Add and merge vertices.
	//	for (int j = 0; j < cont.nverts; ++j)
	//	{
	//		const int* v = &cont.verts[j * 4];
	//		indices[j] = addVertex((unsigned short)v[0], (unsigned short)v[1], (unsigned short)v[2],
	//			mesh.verts, firstVert, nextVert, mesh.nverts);
	//		if (v[3] & RC_BORDER_VERTEX)
	//		{
	//			// This vertex should be removed.
	//			vflags[indices[j]] = 1;
	//		}
	//	}

	//	// Build initial polygons.
	//	int npolys = 0;
	//	memset(polys, 0xff, maxVertsPerCont*nvp * sizeof(unsigned short));
	//	for (int j = 0; j < ntris; ++j)
	//	{
	//		int* t = &tris[j * 3];
	//		if (t[0] != t[1] && t[0] != t[2] && t[1] != t[2])
	//		{
	//			polys[npolys*nvp + 0] = (unsigned short)indices[t[0]];
	//			polys[npolys*nvp + 1] = (unsigned short)indices[t[1]];
	//			polys[npolys*nvp + 2] = (unsigned short)indices[t[2]];
	//			npolys++;
	//		}
	//	}
	//	if (!npolys)
	//		continue;

	//	// Merge polygons.
	//	if (nvp > 3)
	//	{
	//		for (;;)
	//		{
	//			// Find best polygons to merge.
	//			int bestMergeVal = 0;
	//			int bestPa = 0, bestPb = 0, bestEa = 0, bestEb = 0;

	//			for (int j = 0; j < npolys - 1; ++j)
	//			{
	//				unsigned short* pj = &polys[j*nvp];
	//				for (int k = j + 1; k < npolys; ++k)
	//				{
	//					unsigned short* pk = &polys[k*nvp];
	//					int ea, eb;
	//					int v = getPolyMergeValue(pj, pk, mesh.verts, ea, eb, nvp);
	//					if (v > bestMergeVal)
	//					{
	//						bestMergeVal = v;
	//						bestPa = j;
	//						bestPb = k;
	//						bestEa = ea;
	//						bestEb = eb;
	//					}
	//				}
	//			}

	//			if (bestMergeVal > 0)
	//			{
	//				// Found best, merge.
	//				unsigned short* pa = &polys[bestPa*nvp];
	//				unsigned short* pb = &polys[bestPb*nvp];
	//				mergePolyVerts(pa, pb, bestEa, bestEb, tmpPoly, nvp);
	//				unsigned short* lastPoly = &polys[(npolys - 1)*nvp];
	//				if (pb != lastPoly)
	//					memcpy(pb, lastPoly, sizeof(unsigned short)*nvp);
	//				npolys--;
	//			}
	//			else
	//			{
	//				// Could not merge any polygons, stop.
	//				break;
	//			}
	//		}
	//	}

	//	// Store polygons.
	//	for (int j = 0; j < npolys; ++j)
	//	{
	//		unsigned short* p = &mesh.polys[mesh.npolys*nvp * 2];
	//		unsigned short* q = &polys[j*nvp];
	//		for (int k = 0; k < nvp; ++k)
	//			p[k] = q[k];
	//		mesh.regs[mesh.npolys] = cont.reg;
	//		mesh.areas[mesh.npolys] = cont.area;
	//		mesh.npolys++;
	//		if (mesh.npolys > maxTris)
	//		{
	//			ctx->log(RC_LOG_ERROR, "rcBuildPolyMesh: Too many polygons %d (max:%d).", mesh.npolys, maxTris);
	//			return false;
	//		}
	//	}
	//}


	//// Remove edge vertices.
	//for (int i = 0; i < mesh.nverts; ++i)
	//{
	//	if (vflags[i])
	//	{
	//		if (!canRemoveVertex(ctx, mesh, (unsigned short)i))
	//			continue;
	//		if (!removeVertex(ctx, mesh, (unsigned short)i, maxTris))
	//		{
	//			// Failed to remove vertex
	//			ctx->log(RC_LOG_ERROR, "rcBuildPolyMesh: Failed to remove edge vertex %d.", i);
	//			return false;
	//		}
	//		// Remove vertex
	//		// Note: mesh.nverts is already decremented inside removeVertex()!
	//		// Fixup vertex flags
	//		for (int j = i; j < mesh.nverts; ++j)
	//			vflags[j] = vflags[j + 1];
	//		--i;
	//	}
	//}

	//// Calculate adjacency.
	//if (!buildMeshAdjacency(mesh.polys, mesh.npolys, mesh.nverts, nvp))
	//{
	//	ctx->log(RC_LOG_ERROR, "rcBuildPolyMesh: Adjacency failed.");
	//	return false;
	//}

	//// Find portal edges
	//if (mesh.borderSize > 0)
	//{
	//	const int w = m_cfg.width;
	//	const int h = m_cfg.height;
	//	for (int i = 0; i < mesh.npolys; ++i)
	//	{
	//		unsigned short* p = &mesh.polys[i * 2 * nvp];
	//		for (int j = 0; j < nvp; ++j)
	//		{
	//			if (p[j] == RC_MESH_NULL_IDX) break;
	//			// Skip connected edges.
	//			if (p[nvp + j] != RC_MESH_NULL_IDX)
	//				continue;
	//			int nj = j + 1;
	//			if (nj >= nvp || p[nj] == RC_MESH_NULL_IDX) nj = 0;
	//			const unsigned short* va = &mesh.verts[p[j] * 3];
	//			const unsigned short* vb = &mesh.verts[p[nj] * 3];

	//			if ((int)va[0] == 0 && (int)vb[0] == 0)
	//				p[nvp + j] = 0x8000 | 0;
	//			else if ((int)va[2] == h && (int)vb[2] == h)
	//				p[nvp + j] = 0x8000 | 1;
	//			else if ((int)va[0] == w && (int)vb[0] == w)
	//				p[nvp + j] = 0x8000 | 2;
	//			else if ((int)va[2] == 0 && (int)vb[2] == 0)
	//				p[nvp + j] = 0x8000 | 3;
	//		}
	//	}
	//}

	//// Just allocate the mesh flags array. The user is resposible to fill it.
	//mesh.flags = (unsigned short*)rcAlloc(sizeof(unsigned short)*mesh.npolys, RC_ALLOC_PERM);
	//if (!mesh.flags)
	//{
	//	ctx->log(RC_LOG_ERROR, "rcBuildPolyMesh: Out of memory 'mesh.flags' (%d).", mesh.npolys);
	//	return false;
	//}
	//memset(mesh.flags, 0, sizeof(unsigned short) * mesh.npolys);

	//if (mesh.nverts > 0xffff)
	//{
	//	ctx->log(RC_LOG_ERROR, "rcBuildPolyMesh: The resulting mesh has too many vertices %d (max %d). Data can be corrupted.", mesh.nverts, 0xffff);
	//}
	//if (mesh.npolys > 0xffff)
	//{
	//	ctx->log(RC_LOG_ERROR, "rcBuildPolyMesh: The resulting mesh has too many polygons %d (max %d). Data can be corrupted.", mesh.npolys, 0xffff);
	//}

	return true;
}

bool Sample_NavMesh::buildContoursByMesh(rcContext * ctx, InputGeom * geom, const float maxError, const int maxEdgeLen, rcContourSet & cset, const int buildFlags)
{
	const int w = m_cfg.width;
	const int h = m_cfg.height;
	const int borderSize = m_cfg.borderSize;

	rcScopedTimer timer(ctx, RC_TIMER_BUILD_CONTOURS);

	rcVcopy(cset.bmin, m_cfg.bmin);
	rcVcopy(cset.bmax, m_cfg.bmax);
	if (borderSize > 0)
	{
		// If the heightfield was build with bordersize, remove the offset.
		const float pad = borderSize*m_cfg.cs;
		cset.bmin[0] += pad;
		cset.bmin[2] += pad;
		cset.bmax[0] -= pad;
		cset.bmax[2] -= pad;
	}
	cset.cs = m_cfg.cs;
	cset.ch = m_cfg.ch;
	cset.width = m_cfg.width - m_cfg.borderSize * 2;
	cset.height = m_cfg.height - m_cfg.borderSize * 2;
	cset.borderSize = m_cfg.borderSize;
	cset.maxError = maxError;

	//int maxContours = rcMax((int)chf.maxRegions, 8);
	int triCount = geom->getMesh()->getTriCount();
	cset.conts = (rcContour*)malloc(sizeof(rcContour)*triCount);
	if (!cset.conts)
		return false;
	cset.nconts = triCount;
	for (int i = 0; i < triCount; i++)
	{
		rcContour* cont = &cset.conts[i];
		//cont->
	}
	/*rcScopedDelete<unsigned char> flags((unsigned char*)malloc(sizeof(unsigned char)*chf.spanCount));
	if (!flags)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildContours: Out of memory 'flags' (%d).", chf.spanCount);
		return false;
	}*/

	//ctx->startTimer(RC_TIMER_BUILD_CONTOURS_TRACE);

	//// Mark boundaries.
	//for (int y = 0; y < h; ++y)
	//{
	//	for (int x = 0; x < w; ++x)
	//	{
	//		const rcCompactCell& c = chf.cells[x + y*w];
	//		for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
	//		{
	//			unsigned char res = 0;
	//			const rcCompactSpan& s = chf.spans[i];
	//			if (!chf.spans[i].reg || (chf.spans[i].reg & RC_BORDER_REG))
	//			{
	//				flags[i] = 0;
	//				continue;
	//			}
	//			for (int dir = 0; dir < 4; ++dir)
	//			{
	//				unsigned short r = 0;
	//				if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
	//				{
	//					const int ax = x + rcGetDirOffsetX(dir);
	//					const int ay = y + rcGetDirOffsetY(dir);
	//					const int ai = (int)chf.cells[ax + ay*w].index + rcGetCon(s, dir);
	//					r = chf.spans[ai].reg;
	//				}
	//				if (r == chf.spans[i].reg)
	//					res |= (1 << dir);
	//			}
	//			flags[i] = res ^ 0xf; // Inverse, mark non connected edges.
	//		}
	//	}
	//}

	//ctx->stopTimer(RC_TIMER_BUILD_CONTOURS_TRACE);

	//rcIntArray verts(256);
	//rcIntArray simplified(64);

	//for (int y = 0; y < h; ++y)
	//{
	//	for (int x = 0; x < w; ++x)
	//	{
	//		const rcCompactCell& c = chf.cells[x + y*w];
	//		for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
	//		{
	//			if (flags[i] == 0 || flags[i] == 0xf)
	//			{
	//				flags[i] = 0;
	//				continue;
	//			}
	//			const unsigned short reg = chf.spans[i].reg;
	//			if (!reg || (reg & RC_BORDER_REG))
	//				continue;
	//			const unsigned char area = chf.areas[i];

	//			verts.resize(0);
	//			simplified.resize(0);

	//			ctx->startTimer(RC_TIMER_BUILD_CONTOURS_TRACE);
	//			walkContour(x, y, i, chf, flags, verts);
	//			ctx->stopTimer(RC_TIMER_BUILD_CONTOURS_TRACE);

	//			ctx->startTimer(RC_TIMER_BUILD_CONTOURS_SIMPLIFY);
	//			simplifyContour(verts, simplified, maxError, maxEdgeLen, buildFlags);
	//			removeDegenerateSegments(simplified);
	//			ctx->stopTimer(RC_TIMER_BUILD_CONTOURS_SIMPLIFY);


	//			// Store region->contour remap info.
	//			// Create contour.
	//			if (simplified.size() / 4 >= 3)
	//			{
	//				if (cset.nconts >= maxContours)
	//				{
	//					// Allocate more contours.
	//					// This happens when a region has holes.
	//					const int oldMax = maxContours;
	//					maxContours *= 2;
	//					rcContour* newConts = (rcContour*)rcAlloc(sizeof(rcContour)*maxContours, RC_ALLOC_PERM);
	//					for (int j = 0; j < cset.nconts; ++j)
	//					{
	//						newConts[j] = cset.conts[j];
	//						// Reset source pointers to prevent data deletion.
	//						cset.conts[j].verts = 0;
	//						cset.conts[j].rverts = 0;
	//					}
	//					rcFree(cset.conts);
	//					cset.conts = newConts;

	//					ctx->log(RC_LOG_WARNING, "rcBuildContours: Expanding max contours from %d to %d.", oldMax, maxContours);
	//				}

	//				rcContour* cont = &cset.conts[cset.nconts++];

	//				cont->nverts = simplified.size() / 4;
	//				cont->verts = (int*)rcAlloc(sizeof(int)*cont->nverts * 4, RC_ALLOC_PERM);
	//				if (!cont->verts)
	//				{
	//					ctx->log(RC_LOG_ERROR, "rcBuildContours: Out of memory 'verts' (%d).", cont->nverts);
	//					return false;
	//				}
	//				memcpy(cont->verts, &simplified[0], sizeof(int)*cont->nverts * 4);
	//				if (borderSize > 0)
	//				{
	//					// If the heightfield was build with bordersize, remove the offset.
	//					for (int j = 0; j < cont->nverts; ++j)
	//					{
	//						int* v = &cont->verts[j * 4];
	//						v[0] -= borderSize;
	//						v[2] -= borderSize;
	//					}
	//				}

	//				cont->nrverts = verts.size() / 4;
	//				cont->rverts = (int*)rcAlloc(sizeof(int)*cont->nrverts * 4, RC_ALLOC_PERM);
	//				if (!cont->rverts)
	//				{
	//					ctx->log(RC_LOG_ERROR, "rcBuildContours: Out of memory 'rverts' (%d).", cont->nrverts);
	//					return false;
	//				}
	//				memcpy(cont->rverts, &verts[0], sizeof(int)*cont->nrverts * 4);
	//				if (borderSize > 0)
	//				{
	//					// If the heightfield was build with bordersize, remove the offset.
	//					for (int j = 0; j < cont->nrverts; ++j)
	//					{
	//						int* v = &cont->rverts[j * 4];
	//						v[0] -= borderSize;
	//						v[2] -= borderSize;
	//					}
	//				}

	//				cont->reg = reg;
	//				cont->area = area;
	//			}
	//		}
	//	}
	//}

	// Merge holes if needed.
	//if (cset.nconts > 0)
	//{
	//	// Calculate winding of all polygons.
	//	rcScopedDelete<char> winding((char*)rcAlloc(sizeof(char)*cset.nconts, RC_ALLOC_TEMP));
	//	if (!winding)
	//	{
	//		ctx->log(RC_LOG_ERROR, "rcBuildContours: Out of memory 'hole' (%d).", cset.nconts);
	//		return false;
	//	}
	//	int nholes = 0;
	//	for (int i = 0; i < cset.nconts; ++i)
	//	{
	//		rcContour& cont = cset.conts[i];
	//		// If the contour is wound backwards, it is a hole.
	//		winding[i] = calcAreaOfPolygon2D(cont.verts, cont.nverts) < 0 ? -1 : 1;
	//		if (winding[i] < 0)
	//			nholes++;
	//	}

	//	if (nholes > 0)
	//	{
	//		// Collect outline contour and holes contours per region.
	//		// We assume that there is one outline and multiple holes.
	//		const int nregions = chf.maxRegions + 1;
	//		rcScopedDelete<rcContourRegion> regions((rcContourRegion*)rcAlloc(sizeof(rcContourRegion)*nregions, RC_ALLOC_TEMP));
	//		if (!regions)
	//		{
	//			ctx->log(RC_LOG_ERROR, "rcBuildContours: Out of memory 'regions' (%d).", nregions);
	//			return false;
	//		}
	//		memset(regions, 0, sizeof(rcContourRegion)*nregions);

	//		rcScopedDelete<rcContourHole> holes((rcContourHole*)rcAlloc(sizeof(rcContourHole)*cset.nconts, RC_ALLOC_TEMP));
	//		if (!holes)
	//		{
	//			ctx->log(RC_LOG_ERROR, "rcBuildContours: Out of memory 'holes' (%d).", cset.nconts);
	//			return false;
	//		}
	//		memset(holes, 0, sizeof(rcContourHole)*cset.nconts);

	//		for (int i = 0; i < cset.nconts; ++i)
	//		{
	//			rcContour& cont = cset.conts[i];
	//			// Positively would contours are outlines, negative holes.
	//			if (winding[i] > 0)
	//			{
	//				if (regions[cont.reg].outline)
	//					ctx->log(RC_LOG_ERROR, "rcBuildContours: Multiple outlines for region %d.", cont.reg);
	//				regions[cont.reg].outline = &cont;
	//			}
	//			else
	//			{
	//				regions[cont.reg].nholes++;
	//			}
	//		}
	//		int index = 0;
	//		for (int i = 0; i < nregions; i++)
	//		{
	//			if (regions[i].nholes > 0)
	//			{
	//				regions[i].holes = &holes[index];
	//				index += regions[i].nholes;
	//				regions[i].nholes = 0;
	//			}
	//		}
	//		for (int i = 0; i < cset.nconts; ++i)
	//		{
	//			rcContour& cont = cset.conts[i];
	//			rcContourRegion& reg = regions[cont.reg];
	//			if (winding[i] < 0)
	//				reg.holes[reg.nholes++].contour = &cont;
	//		}

	//		// Finally merge each regions holes into the outline.
	//		for (int i = 0; i < nregions; i++)
	//		{
	//			rcContourRegion& reg = regions[i];
	//			if (!reg.nholes) continue;

	//			if (reg.outline)
	//			{
	//				mergeRegionHoles(ctx, reg);
	//			}
	//			else
	//			{
	//				// The region does not have an outline.
	//				// This can happen if the contour becaomes selfoverlapping because of
	//				// too aggressive simplification settings.
	//				ctx->log(RC_LOG_ERROR, "rcBuildContours: Bad outline for region %d, contour simplification is likely too aggressive.", i);
	//			}
	//		}
	//	}
	//}
	return true;
}


