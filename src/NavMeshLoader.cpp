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
	bool has_v = doc.HasMember("v");
	bool has_p = doc.HasMember("p");
	if (!has_v || !has_p)
	{
		return 0;
	}
	rapidjson::Value& vertex_array = doc["v"];
	if (vertex_array.IsArray())
	{
		int maxVertices = vertex_array.Size();
		mesh.nverts = maxVertices;
		mesh.verts = (unsigned short*)malloc(sizeof(unsigned short)*maxVertices * 3);
		memset(mesh.verts, 0, sizeof(unsigned short)*maxVertices * 3);
		for (SizeType i = 0; i < vertex_array.Size(); i++)
		{
			rapidjson::Value& vertex = vertex_array[i];
			/*unsigned short* v = &mesh.verts[i];
			v[0] = vertex[0];
			v[1] = vertex[1];
			v[2] = vertex[2];*/
			printf("a[%f] = %f\n", i, vertex_array[i].GetFloat());
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
