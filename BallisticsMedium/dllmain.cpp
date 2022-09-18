// dllmain.cpp : 定义 DLL 应用程序的入口点。
#include "interface.h"
#define WIN32_LEAN_AND_MEAN
#include "Windows.h"

using namespace std;
using namespace a3_interface;

const string& version = "Ballistik Komputer v6.9"s;

// ReSharper disable CppInconsistentNaming

extern "C" {
__declspec(dllexport) void __stdcall RVExtension(char* output, int outputSize, const char* function);
__declspec(dllexport) int __stdcall RVExtensionArgs(char* output, int outputSize, const char* function,
                                                    const char** argv, int argc);
__declspec(dllexport) void __stdcall RVExtensionVersion(char* output, int outputSize);
}

void __stdcall RVExtension(char* output, int outputSize, const char* function)
{
}

// 0: success, 1: unknown, -1: invalid argument, -2: argument parse failed
int __stdcall RVExtensionArgs(char* output, int outputSize, const char* function, const char** argv, int argc)
{
	constexpr int args_count = 10;
	if (strcmp(function, "find_solution") != 0 || argc != args_count)
	{
		return -1;
	}

	vector<double> args(args_count);

	try
	{
		for (int i = 0; i < args_count; i++)
		{
			string arg(argv[i]);
			args[i] = stod(arg);
		}
	}
	catch (exception)
	{
		return -2;
	}

	if (const auto solution = find_solution(
		tuple(args[0], args[1]),
		vec3(args[2], args[3], args[4]),
		vec3(args[5], args[6], args[7]),
		args[8],
		args[9]
	))
	{
		auto& [horizonal, vertical, tof] = *solution;
		const string result = format("[{},{},{}]", horizonal, vertical, tof);
		strncpy_s(output, outputSize - 1, result.c_str(), _TRUNCATE);
		return 0;
	}

	return 1;
}


void __stdcall RVExtensionVersion(char* output, int outputSize)
{
	strncpy_s(output, outputSize - 1, version.c_str(), _TRUNCATE);
	init();
}

// int mian()
// {
// 	init();
// 	tuple<double, double> solution = find_solution(6000, vec3(0, 0, 0), vec3(0, 0, 0), 1600, -0.00005);
// 	return 0;
// }

BOOL WINAPI DllMain(
	HINSTANCE hinstDLL, // handle to DLL module
	DWORD fdwReason, // reason for calling function
	LPVOID lpvReserved) // reserved
{
	switch (fdwReason)
	{
	case DLL_PROCESS_ATTACH:
		break;
	case DLL_THREAD_ATTACH:
		break;
	case DLL_THREAD_DETACH:
		break;
	case DLL_PROCESS_DETACH:
		if (lpvReserved != nullptr)
		{
			break; // do not do cleanup if process termination scenario
		}
		cleanup();
		break;
	}
	return true;
}

// ReSharper restore CppInconsistentNaming
