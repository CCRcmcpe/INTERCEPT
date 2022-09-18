#include "interface.h"
#include "Python.h"

using namespace std;

namespace a3_interface
{
	PyObject* solver;

	shared_ptr<tuple<double, double, double>> find_solution(const tuple<double, double> target_range,
	                                                        const vec3 target_velocity,
	                                                        const vec3 target_acceleration, const double init_speed,
	                                                        const double air_friction)
	{
		if (!Py_IsInitialized())
		{
			throw exception("Python intepreter not initialized");
		}

		PyObject* args = PyTuple_New(5);

		{
			PyObject* target_range_t = PyTuple_New(2);

			{
				PyTuple_SetItem(target_range_t, 0, PyFloat_FromDouble(get<0>(target_range)));
				PyTuple_SetItem(target_range_t, 1, PyFloat_FromDouble(get<1>(target_range)));
			}

			PyTuple_SetItem(args, 0, target_range_t);
		}

		{
			PyObject* target_velocity_t = PyTuple_New(3);

			{
				PyTuple_SetItem(target_velocity_t, 0, PyFloat_FromDouble(get<0>(target_velocity)));
				PyTuple_SetItem(target_velocity_t, 1, PyFloat_FromDouble(get<1>(target_velocity)));
				PyTuple_SetItem(target_velocity_t, 2, PyFloat_FromDouble(get<2>(target_velocity)));
			}

			PyTuple_SetItem(args, 1, target_velocity_t);
		}

		{
			PyObject* target_acceleration_t = PyTuple_New(3);

			{
				PyTuple_SetItem(target_acceleration_t, 0, PyFloat_FromDouble(get<0>(target_acceleration)));
				PyTuple_SetItem(target_acceleration_t, 1, PyFloat_FromDouble(get<1>(target_acceleration)));
				PyTuple_SetItem(target_acceleration_t, 2, PyFloat_FromDouble(get<2>(target_acceleration)));
			}

			PyTuple_SetItem(args, 2, target_acceleration_t);
		}

		PyTuple_SetItem(args, 3, PyFloat_FromDouble(init_speed));
		PyTuple_SetItem(args, 4, PyFloat_FromDouble(air_friction));

		PyObject* result = PyObject_Call(solver, args, nullptr);

		if (result == nullptr)
		{
			PyErr_Print();
			throw exception("Python error encountered");
		}

		if (result == Py_None)
		{
			return nullptr;
		}

		return make_shared<tuple<double, double, double>>(
			PyFloat_AsDouble(PyTuple_GetItem(result, 0)),
			PyFloat_AsDouble(PyTuple_GetItem(result, 1)),
			PyFloat_AsDouble(PyTuple_GetItem(result, 2))
		);
	}

	void init()
	{
		if (Py_IsInitialized())
		{
			cleanup();
		}

		Py_SetPath(LR"(C:\Users\Rcmcpe\AppData\Local\Programs\Python\Python310\Lib)");
		Py_Initialize();

		//virtualenv activate py not working

		// PyObject* obj = Py_BuildValue("s", R"(.\venv\Scripts\activate_this.py)");
		// if (FILE* activate = _Py_fopen_obj(obj, "r"))
		// {
		// 	PyRun_SimpleFile(activate, R"(.\venv\Scripts\activate_this.py)");
		// }

		//fclose(activate); //raises exception

		//PyObject* sys_path = PySys_GetObject("path");
		//PyList_Append(sys_path, PyUnicode_FromString(R"(F:\Project\SolveBallistics)"));

		PyRun_SimpleString("import sys");
		PyRun_SimpleString(R"(sys.stderr = open('F:\\err.txt', 'w'))");
		PyRun_SimpleString(
			R"(sys.path = ['F:\\Project\\SolveBallistics', 'C:\\Users\\Rcmcpe\\AppData\\Local\\Programs\\Python\\Python310\\python310.zip', 'C:\\Users\\Rcmcpe\\AppData\\Local\\Programs\\Python\\Python310\\DLLs', 'C:\\Users\\Rcmcpe\\AppData\\Local\\Programs\\Python\\Python310\\lib', 'C:\\Users\\Rcmcpe\\AppData\\Local\\Programs\\Python\\Python310', 'F:\\Project\\SolveBallistics\\venv', 'F:\\Project\\SolveBallistics\\venv\\lib\\site-packages', 'C:\\Users\\Rcmcpe\\AppData\\Roaming\\Python\\Python310\\site-packages', 'C:\\Users\\Rcmcpe\\AppData\\Local\\Programs\\Python\\Python310\\lib\\site-packages'])");
		PyObject* interface = PyImport_ImportModule("arma_interface");
		if (!interface)
		{
			throw exception("Interface loading failed");
		}
		solver = PyObject_GetAttrString(interface, "solve");
		if (!solver)
		{
			throw exception("Solver is null");
		}
	}

	void cleanup()
	{
		if (Py_IsInitialized())
		{
			Py_FinalizeEx();
		}
	}
}
