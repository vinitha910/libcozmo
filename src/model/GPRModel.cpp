#include "model/GPRModel.hpp"
#include <Python.h>
#include <sstream> 
#include <iostream>

namespace libcozmo {
namespace model {

GPRModel::ScikitLearnModel::ScikitLearnModel(const std::string& model_path) {
	Py_Initialize();

	std::stringstream buf;
	buf << "from sklearn.gaussian_process import GaussianProcessRegressor" << std::endl
		<< "import cPickle as pickle" << std::endl
		<< "def load_model(filename):" << std::endl
		<< "	return pickle.load(open(filename, 'rb'))" << std::endl;

	PyObject* result = nullptr;

	// Compiled python function
	PyObject* pName = Py_CompileString(buf.str().c_str(), "", Py_file_input);
	std::cout << "Compiled function\n";

	// PyObject* pName = PyUnicode_DecodeFSDefault(buf.str().c_str());
	PyObject* pModule = PyImport_Import(pName);

	if (pModule != NULL) {
		PyObject* pFunc = PyObject_GetAttrString(pModule, "load_model");
		std::cout << "Created func\n";

		if (pFunc && PyCallable_Check(pFunc)) {
			// Filename argument
			PyObject* pArgs = PyUnicode_FromString(model_path.c_str());
			std::cout << "Filename argument\n";

			result = PyObject_CallObject(pFunc, pArgs);
			std::cout << "call object\n";
		}
	}
	
	if (result == NULL) {
		std::cout << "ERROR: model did not load correctly\n";
	}
	Py_Finalize();
}

}
}

using namespace libcozmo::model;

int main() {
	GPRModel::ScikitLearnModel model("/home/vinitha/workspaces/curiosity_ws/src/curiosity_project/python/models/novel_uninformed/angle_regressor_10.pkl");
}