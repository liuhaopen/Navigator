#include "NavMeshLoader.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include "Recast.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include <rapidjson/document.h>
//#include <rapidjson/istreamwrapper.h>
//#include <fstream>
using namespace rapidjson;
//using namespace std;

dtNavMesh * NavMeshLoader::LoadFromJson(const char * path)
{
	Document doc;
	ParseJson(path, doc);
	bool has_v = doc.HasMember("v");
	bool has_p = doc.HasMember("p");
	if (!has_v || !has_p)
	{
		return NULL;
	}
	rapidjson::Value& vertex_value = doc["v"];
	if (vertex_value.IsArray())
	{

	}
	return NULL;
}

int NavMeshLoader::FullPolyDataFromJson(const char * path, rcPolyMesh & mesh)
{
	Document doc;
	ParseJson(path, doc);
	/*bool has_v = doc.HasMember("v");
	bool has_p = doc.HasMember("p");
	if (!has_v || !has_p)
	{
		return 0;
	}*/
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
			unsigned short* v = &mesh.verts[i*3];
			v[0] = vertex[0].GetInt();
			v[1] = vertex[1].GetInt();
			v[2] = vertex[2].GetInt();
			printf("a[%d] = %ud %ud %ud\n", i, v[0], v[1], v[2]);
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
			unsigned short* p = &mesh.polys[i*nvp*2];
			for (int j = 0; j < nvp*2; j++)
			{
				p[j] = poly[j].GetInt();
			}
			//printf("a[%d] = %ud %ud %ud\n", i, v[0], v[1], v[2]);
		}
	}
	return 1;
}

int NavMeshLoader::ParseJson(const char * path, rapidjson::Document & doc)
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

//Document * NavMeshLoader::ParseJson(const char * path)
//{
//	FILE *fp = fopen(path, "r");
//	if (!fp)
//		return NULL;
//	fseek(fp, 0L, SEEK_END);
//	int fileSize = ftell(fp);
//	char* data = (char*)dtAlloc(fileSize, DT_ALLOC_PERM);
//	if (!data)
//		return NULL;
//	memset(data, 0, fileSize);
//	fseek(fp, 0L, SEEK_SET);
//	int is_read_ok = fread(data, fileSize, 1, fp);
//	if (is_read_ok != 1)
//	{
//		dtFree(data);
//		fclose(fp);
//		return NULL;
//	}
//
//	Document* doc = new Document();
//	doc->Parse(data);
//	dtFree(data);
//	fclose(fp);
//	if (doc->HasParseError())
//	{
//		delete doc;
//		return NULL;
//	}
//	return doc;
//}
