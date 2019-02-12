#ifndef __NAVIGATOR_H__
#define __NAVIGATOR_H__

//#ifdef NAVIGATOR_EXPORT
//	#define NAVIGATOR_API __declspec(dllexport)
//#else 
//	#define NAVIGATOR_API 
//#endif
#include <rapidjson/document.h>
class dtNavMesh;

class Navigator
{
public:
	Navigator();
	virtual ~Navigator();
	int init(const char* path);

	int findRandomPointAroundCircle(float* pos, float radius, float* outPos);
	static dtNavMesh* LoadNavMesh(const char* path);
	static void SaveNavMesh(const char* savePath, const dtNavMesh* mesh);
	static int ConvertJsonToNavBinFile(const char* jsonContent, const char* savePath);
	static int FullPolyDataFromJson(const char* path, struct rcPolyMesh& mesh);

protected:
	static int ParseJson(const char* path, rapidjson::Document& doc);
	static int FullPolyDataFromJsonObj(rapidjson::Document& doc, struct rcPolyMesh& mesh);

private:
	class dtNavMesh* m_navMesh;
	class dtNavMeshQuery* m_navQuery;
	float m_polyPickExt[3];

	//dtNavMesh* load(const char* path);
	//dtNavMesh* loadFromObjFile(const char* path);

};

#endif