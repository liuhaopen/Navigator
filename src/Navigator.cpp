#include "Navigator.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include "Recast.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourNavMeshBuilder.h"
#include <rapidjson/document.h>
using namespace rapidjson;

static const int NAVMESHSET_MAGIC = 'M' << 24 | 'S' << 16 | 'E' << 8 | 'T'; //'MSET';
static const int NAVMESHSET_VERSION = 1;
struct NavMeshSetHeader
{
	int magic;
	int version;
	int numTiles;
	dtNavMeshParams params;
};

struct NavMeshTileHeader
{
	dtTileRef tileRef;
	int dataSize;
};
Navigator::Navigator()
{
	m_polyPickExt[0] = 2;
	m_polyPickExt[1] = 4;
	m_polyPickExt[2] = 2;
}
Navigator::~Navigator()
{
	dtFreeNavMeshQuery(m_navQuery);
	dtFreeNavMesh(m_navMesh);
}

int Navigator::init(const char * path)
{
	m_navMesh = LoadNavMesh(path);
	if (m_navMesh != NULL)
	{
		m_navQuery = dtAllocNavMeshQuery();
		m_navQuery->init(m_navMesh, 2048);
	}
	return m_navMesh != NULL ? 1 : 0;
}
static float frand()
{
	return (float)rand() / (float)RAND_MAX;
}

int Navigator::findRandomPointAroundCircle(float * pos, float radius, float * outPos)
{
	dtPolyRef ref;
	dtPolyRef m_startRef = 0;
	dtQueryFilter m_filter;
	//m_filter.setIncludeFlags(0xffff);
	//m_filter.setExcludeFlags(0);
	m_navQuery->findNearestPoly(pos, m_polyPickExt, &m_filter, &m_startRef, 0);
	dtStatus status = m_navQuery->findRandomPointAroundCircle(m_startRef, pos, radius, &m_filter, frand, &ref, outPos);
	if (dtStatusSucceed(status))
	{
		return 1;
	}
	return 0;
}


int Navigator::ConvertJsonToNavBinFile(const char * jsonContent, const char * savePath)
{
	Document doc;
	doc.Parse(jsonContent);

	rcPolyMesh* m_pmesh = rcAllocPolyMesh();
	if (!m_pmesh)
	{
		return 0;
	}
	FullPolyDataFromJsonObj(doc, *m_pmesh);
	const int nvp = doc["nvp"].GetInt();
	dtNavMesh* navMesh = NULL;
	if (nvp <= DT_VERTS_PER_POLYGON)
	{
		unsigned char* navData = 0;
		int navDataSize = 0;

		dtNavMeshCreateParams params;
		memset(&params, 0, sizeof(params));
		params.verts = m_pmesh->verts;
		params.vertCount = m_pmesh->nverts;
		params.polys = m_pmesh->polys;
		params.polyAreas = m_pmesh->areas;
		params.polyFlags = m_pmesh->flags;
		params.polyCount = m_pmesh->npolys;
		params.nvp = m_pmesh->nvp;
		params.walkableHeight = doc["agentHeight"].GetFloat();
		params.walkableRadius = doc["agentRadius"].GetFloat();
		params.walkableClimb = doc["agentMaxClimb"].GetFloat();
		rcVcopy(params.bmin, m_pmesh->bmin);
		rcVcopy(params.bmax, m_pmesh->bmax);
		params.cs = m_pmesh->cs;
		params.ch = m_pmesh->ch;
		params.buildBvTree = true;

		if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
		{
			return 0;
		}

		navMesh = dtAllocNavMesh();
		if (!navMesh)
		{
			dtFree(navData);
			return 0;
		}

		dtStatus status;

		status = navMesh->init(navData, navDataSize, DT_TILE_FREE_DATA);
		if (dtStatusFailed(status))
		{
			dtFree(navData);
			return 0;
		}
		SaveNavMesh(savePath, navMesh);
	}
	return 1;
}

void Navigator::SaveNavMesh(const char * savePath, const dtNavMesh * mesh)
{
	if (!mesh) return;

	FILE* fp = fopen(savePath, "wb");
	if (!fp)
		return;

	// Store header.
	NavMeshSetHeader header;
	header.magic = NAVMESHSET_MAGIC;
	header.version = NAVMESHSET_VERSION;
	header.numTiles = 0;
	for (int i = 0; i < mesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;
		header.numTiles++;
	}
	memcpy(&header.params, mesh->getParams(), sizeof(dtNavMeshParams));
	fwrite(&header, sizeof(NavMeshSetHeader), 1, fp);

	// Store tiles.
	for (int i = 0; i < mesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;

		NavMeshTileHeader tileHeader;
		tileHeader.tileRef = mesh->getTileRef(tile);
		tileHeader.dataSize = tile->dataSize;
		fwrite(&tileHeader, sizeof(tileHeader), 1, fp);

		fwrite(tile->data, tile->dataSize, 1, fp);
	}

	fclose(fp);
}

dtNavMesh * Navigator::LoadNavMesh(const char * path)
{
	FILE* fp = fopen(path, "rb");
	if (!fp) return 0;

	// Read header.
	NavMeshSetHeader header;
	size_t readLen = fread(&header, sizeof(NavMeshSetHeader), 1, fp);
	if (readLen != 1)
	{
		fclose(fp);
		return 0;
	}
	if (header.magic != NAVMESHSET_MAGIC)
	{
		fclose(fp);
		return 0;
	}
	if (header.version != NAVMESHSET_VERSION)
	{
		fclose(fp);
		return 0;
	}

	dtNavMesh* mesh = dtAllocNavMesh();
	if (!mesh)
	{
		fclose(fp);
		return 0;
	}
	dtStatus status = mesh->init(&header.params);
	if (dtStatusFailed(status))
	{
		fclose(fp);
		return 0;
	}

	// Read tiles.
	for (int i = 0; i < header.numTiles; ++i)
	{
		NavMeshTileHeader tileHeader;
		readLen = fread(&tileHeader, sizeof(tileHeader), 1, fp);
		if (readLen != 1)
		{
			fclose(fp);
			return 0;
		}

		if (!tileHeader.tileRef || !tileHeader.dataSize)
			break;

		unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
		if (!data) break;
		memset(data, 0, tileHeader.dataSize);
		readLen = fread(data, tileHeader.dataSize, 1, fp);
		if (readLen != 1)
		{
			dtFree(data);
			fclose(fp);
			return 0;
		}

		mesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0);
	}
	fclose(fp);
	return mesh;
}

int Navigator::FullPolyDataFromJson(const char * path, rcPolyMesh & mesh)
{
	Document doc;
	ParseJson(path, doc);
	return FullPolyDataFromJsonObj(doc, mesh);
}

int Navigator::ParseJson(const char * path, rapidjson::Document & doc)
{
	FILE *fp = fopen(path, "r");
	if (!fp)
		return 0;
	fseek(fp, 0L, SEEK_END);
	int fileSize = ftell(fp);
	char* data = (char*)dtAlloc(fileSize, DT_ALLOC_PERM);
	if (!data)
		return 0;
	memset(data, 0, fileSize);
	fseek(fp, 0L, SEEK_SET);
	int is_read_ok = fread(data, fileSize, 1, fp);
	if (is_read_ok != 1)
	{
		dtFree(data);
		fclose(fp);
		return 0;
	}
	doc.Parse(data);
	dtFree(data);
	fclose(fp);
	if (doc.HasParseError())
	{
		return 0;
	}
	return 1;
}

int Navigator::FullPolyDataFromJsonObj(rapidjson::Document & doc, rcPolyMesh & mesh)
{
	mesh.maxEdgeError = 1.3;
	mesh.borderSize = 0;
	const int nvp = doc["nvp"].GetInt();
	const int cs = doc["cs"].GetInt();
	const int ch = doc["ch"].GetInt();
	mesh.nvp = nvp;
	mesh.cs = cs;
	mesh.ch = ch;
	rapidjson::Value& bmin_array = doc["bmin"];
	if (bmin_array.IsArray())
	{
		mesh.bmin[0] = bmin_array[0].GetFloat();
		mesh.bmin[1] = bmin_array[1].GetFloat();
		mesh.bmin[2] = bmin_array[2].GetFloat();
	}
	rapidjson::Value& bmax_array = doc["bmax"];
	if (bmax_array.IsArray())
	{
		mesh.bmax[0] = bmax_array[0].GetFloat();
		mesh.bmax[1] = bmax_array[1].GetFloat();
		mesh.bmax[2] = bmax_array[2].GetFloat();
	}
	rapidjson::Value& vertex_array = doc["v"];
	if (vertex_array.IsArray())
	{
		int maxVertices = vertex_array.Size();
		mesh.nverts = maxVertices;
		mesh.verts = (unsigned short*)malloc(sizeof(unsigned short)*maxVertices * 3);
		memset(mesh.verts, 0, sizeof(unsigned short) * maxVertices * 3);
		for (SizeType i = 0; i < vertex_array.Size(); i++)
		{
			rapidjson::Value& vertex = vertex_array[i];
			unsigned short* v = &mesh.verts[i * 3];
			v[0] = vertex[0].GetInt();
			v[1] = vertex[1].GetInt();
			v[2] = vertex[2].GetInt();
			//printf("a[%d] = %ud %ud %ud\n", i, v[0], v[1], v[2]);
		}
	}
	rapidjson::Value& polys_array = doc["p"];
	if (polys_array.IsArray())
	{
		int polyNum = polys_array.Size();
		mesh.npolys = polyNum;
		mesh.maxpolys = polyNum;
		int mallocSize = sizeof(unsigned short)*polyNum * 2 * nvp;
		mesh.polys = (unsigned short*)malloc(mallocSize);
		memset(mesh.polys, 0xffff, mallocSize);
		mesh.regs = (unsigned short*)malloc(sizeof(unsigned short)*polyNum);
		memset(mesh.regs, 1, sizeof(unsigned short)*polyNum);
		mesh.areas = (unsigned char*)malloc(sizeof(unsigned char)*polyNum);
		memset(mesh.areas, 1, sizeof(unsigned char)*polyNum);
		mesh.flags = (unsigned short*)malloc(sizeof(unsigned short)*polyNum);
		memset(mesh.flags, 1, sizeof(unsigned short)*polyNum);
		for (SizeType i = 0; i < polys_array.Size(); i++)
		{
			rapidjson::Value& poly = polys_array[i];
			assert(poly.IsArray() && poly.Size() == nvp * 2);
			unsigned short* p = &mesh.polys[i*nvp * 2];
			for (int j = 0; j < nvp * 2; j++)
			{
				p[j] = poly[j].GetInt();
			}
			//printf("a[%d] = %ud %ud %ud\n", i, v[0], v[1], v[2]);
		}
	}
	return 1;
}
//
//
//dtNavMesh * Navigator::load(const char * path)
//{
//	FILE* fp = fopen(path, "rb");
//	//printf("navigator load file pointer : %d\n", (int)fp);
//	if (!fp) return 0;
//
//	// Read header.
//	NavMeshSetHeader header;
//	size_t readLen = fread(&header, sizeof(NavMeshSetHeader), 1, fp);
//	if (readLen != 1)
//	{
//		fclose(fp);
//		return 0;
//	}
//	if (header.magic != NAVMESHSET_MAGIC)
//	{
//		fclose(fp);
//		return 0;
//	}
//	if (header.version != NAVMESHSET_VERSION)
//	{
//		fclose(fp);
//		return 0;
//	}
//
//	dtNavMesh* mesh = dtAllocNavMesh();
//	if (!mesh)
//	{
//		fclose(fp);
//		return 0;
//	}
//	dtStatus status = mesh->init(&header.params);
//	if (dtStatusFailed(status))
//	{
//		fclose(fp);
//		return 0;
//	}
//
//	// Read tiles.
//	for (int i = 0; i < header.numTiles; ++i)
//	{
//		NavMeshTileHeader tileHeader;
//		readLen = fread(&tileHeader, sizeof(tileHeader), 1, fp);
//		if (readLen != 1)
//		{
//			fclose(fp);
//			return 0;
//		}
//
//		if (!tileHeader.tileRef || !tileHeader.dataSize)
//			break;
//
//		unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
//		if (!data) break;
//		memset(data, 0, tileHeader.dataSize);
//		readLen = fread(data, tileHeader.dataSize, 1, fp);
//		if (readLen != 1)
//		{
//			dtFree(data);
//			fclose(fp);
//			return 0;
//		}
//
//		mesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0);
//	}
//
//	fclose(fp);
//
//	return mesh;
//}
//
//dtNavMesh * Navigator::loadFromObjFile(const char * path)
//{
//	FILE* fp = fopen(path, "rb");
//	//printf("navigator load file pointer : %d\n", (int)fp);
//	if (!fp) return 0;
//
//	// Read header.
//	NavMeshSetHeader header;
//	size_t readLen = fread(&header, sizeof(NavMeshSetHeader), 1, fp);
//	if (readLen != 1)
//	{
//		fclose(fp);
//		return 0;
//	}
//	if (header.magic != NAVMESHSET_MAGIC)
//	{
//		fclose(fp);
//		return 0;
//	}
//	if (header.version != NAVMESHSET_VERSION)
//	{
//		fclose(fp);
//		return 0;
//	}
//
//	dtNavMesh* mesh = dtAllocNavMesh();
//	if (!mesh)
//	{
//		fclose(fp);
//		return 0;
//	}
//	dtStatus status = mesh->init(&header.params);
//	if (dtStatusFailed(status))
//	{
//		fclose(fp);
//		return 0;
//	}
//
//	// Read tiles.
//	for (int i = 0; i < header.numTiles; ++i)
//	{
//		NavMeshTileHeader tileHeader;
//		readLen = fread(&tileHeader, sizeof(tileHeader), 1, fp);
//		if (readLen != 1)
//		{
//			fclose(fp);
//			return 0;
//		}
//
//		if (!tileHeader.tileRef || !tileHeader.dataSize)
//			break;
//
//		unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
//		if (!data) break;
//		memset(data, 0, tileHeader.dataSize);
//		readLen = fread(data, tileHeader.dataSize, 1, fp);
//		if (readLen != 1)
//		{
//			dtFree(data);
//			fclose(fp);
//			return 0;
//		}
//
//		mesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0);
//	}
//
//	fclose(fp);
//
//	return mesh;
//}
