#include "Navigator.h"

extern "C" __declspec(dllexport) int ConvertJsonToNavBinFile(const char* jsonContent, const char* savePath)
{
	return Navigator::ConvertJsonToNavBinFile(jsonContent, savePath);
}