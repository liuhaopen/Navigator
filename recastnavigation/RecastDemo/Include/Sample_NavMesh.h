#ifndef RECASTSAMPLENAVMESH_H
#define RECASTSAMPLENAVMESH_H

#include "Sample.h"
#include "DetourNavMesh.h"
#include "Recast.h"

class Sample_NavMesh : public Sample
{
protected:
	bool m_keepInterResults;
	float m_totalBuildTimeMs;

	unsigned char* m_triareas;
	rcHeightfield* m_solid;
	rcCompactHeightfield* m_chf;
	rcContourSet* m_cset;
	rcPolyMesh* m_pmesh;
	rcConfig m_cfg;
	rcPolyMeshDetail* m_dmesh;

	enum DrawMode
	{
		DRAWMODE_NAVMESH,
		DRAWMODE_NAVMESH_TRANS,
		DRAWMODE_NAVMESH_BVTREE,
		DRAWMODE_NAVMESH_NODES,
		DRAWMODE_NAVMESH_INVIS,
		DRAWMODE_MESH,
		DRAWMODE_VOXELS,
		DRAWMODE_VOXELS_WALKABLE,
		DRAWMODE_COMPACT,
		DRAWMODE_COMPACT_DISTANCE,
		DRAWMODE_COMPACT_REGIONS,
		DRAWMODE_REGION_CONNECTIONS,
		DRAWMODE_RAW_CONTOURS,
		DRAWMODE_BOTH_CONTOURS,
		DRAWMODE_CONTOURS,
		DRAWMODE_POLYMESH,
		DRAWMODE_POLYMESH_DETAIL,
		MAX_DRAWMODE
	};

	DrawMode m_drawMode;

	void cleanup();

public:
	Sample_NavMesh();
	virtual ~Sample_NavMesh();

	virtual void handleSettings();
	virtual void handleTools();
	virtual void handleDebugMode();

	virtual void handleRender();
	virtual void handleRenderOverlay(double* proj, double* model, int* view);
	virtual void handleMeshChanged(class InputGeom* geom);
	virtual bool handleBuild();

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	Sample_NavMesh(const Sample_NavMesh&);
	Sample_NavMesh& operator=(const Sample_NavMesh&);
	bool rcBuildPolyMesh2(rcContext* ctx, class InputGeom* geom, const int nvp, rcPolyMesh& mesh);
	bool buildContoursByMesh(rcContext* ctx, class InputGeom* geom,
		const float maxError, const int maxEdgeLen,
		rcContourSet& cset, const int buildFlags=-1);
};


#endif // RECASTSAMPLESOLOMESHSIMPLE_H
