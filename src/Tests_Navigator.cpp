#include "catch.hpp"
#include "Navigator.h"

TEST_CASE("Navigator")
{
	SECTION("InitLoad")
	{
		Navigator navigator;
		int is_init_ok = navigator.init("navmesh.json");
		printf("is_init_ok : %d\n", is_init_ok);
		float curPos[3] = { 0,0,0 };
		float outPos[3] = { 0,0,0 };
		int is_finded = navigator.findRandomPointAroundCircle(curPos, 500, outPos);
		printf("is_finded random point : %d %f %f %f\n", is_finded, outPos[0], outPos[1], outPos[2]);
	}
}