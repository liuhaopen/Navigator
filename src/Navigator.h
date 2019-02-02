class Navigator
{
private:
	class dtNavMesh* m_navMesh;
	class dtNavMeshQuery* m_navQuery;
	float m_polyPickExt[3];

	dtNavMesh* load(const char* path);
	dtNavMesh* loadFromObjFile(const char* path);

public:
	Navigator();
	virtual ~Navigator();
	int init(const char* path);
	//findPath
	int findRandomPointAroundCircle(float* pos, float radius, float* outPos);
};