#include <rapidjson/document.h>
//#include "Recast.h"

class NavMeshLoader
{
public:
	static class dtNavMesh* LoadFromJson(const char* path);
	static int FullPolyDataFromJson(const char* path, struct rcPolyMesh& mesh);

private:
	static int ParseJson(const char* path, rapidjson::Document& doc);
};