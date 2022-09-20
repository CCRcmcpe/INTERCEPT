#define WIN32_LEAN_AND_MEAN
#include "Windows.h"
#include "solver.h"
#include <string>

using namespace std;

const string& VERSION = "Ballistik Komputer v6.9"s;

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

	double args[args_count] = {0};

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

	if (const auto solution = BallisticsSolver(
		Vector3d(0, args[0], args[1]),
		Vector3d(args[2], args[3], args[4]),
		Vector3d(args[5], args[6], args[7]),
		args[8],
		args[9]
	)(); solution != nullptr)
	{
		auto& [horizonal, vertical, tof] = *solution;
		const string result = format("[{},{},{}]", horizonal / DEGREE, vertical / DEGREE, tof);
		strncpy_s(output, static_cast<rsize_t>(outputSize) - 1, result.c_str(), _TRUNCATE);
		return 0;
	}

	return 1;
}


void __stdcall RVExtensionVersion(char* output, int outputSize)
{
	strncpy_s(output, static_cast<rsize_t>(outputSize) - 1, VERSION.c_str(), _TRUNCATE);
}

#ifdef _TEST
int main()
{
	constexpr double AP_TANK_INIT_SPEED = 1700;
	constexpr double HE_TANK_INIT_SPEED = 1410;

	constexpr double AP_TANK = -3.96e-005;
	constexpr double HE_TANK = -0.000275;
	constexpr double B_127x99 = -0.00085999997;
	constexpr double B_762x51 = -0.001;

	const auto solution = BallisticsSolver(
		Vector3d(0, 6000, 10),
		Vector3d(10, 0, 0),
		Vector3d(0, 0, 0),
		HE_TANK_INIT_SPEED,
		HE_TANK
	)();

	if (solution != nullptr)
	{
		cout << *solution << endl;
		return 0;
	}
	cout << "nosol" << endl;
}
#else
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
		// if (lpvReserved != nullptr)
		// {
		// 	break; // do not do cleanup if process termination scenario
		// }
		// cleanup();
		break;
	}
	return true;
}
#endif

// ReSharper restore CppInconsistentNaming
