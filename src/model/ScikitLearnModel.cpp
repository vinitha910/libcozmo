#include "model/ScikitLearnModel.hpp"
#include <sstream> 
#include <iostream>

namespace libcozmo {
namespace model {

ScikitLearnModel::ScikitLearnModel(const std::string& model_path) {
    // Py_Initialize();
    initialize(model_path);
}

ScikitLearnModel::~ScikitLearnModel() { 
    Py_DecRef(p_model);
    Py_DecRef(p_module);
    // Py_Finalize();
}

void ScikitLearnModel::initialize(const std::string& model_path) {
    std::stringstream buf;
    buf << "import _pickle as pickle" << std::endl
        << "def load_model(filename):" << std::endl
        << "    return pickle.load(open(filename, 'rb'), encoding='latin1')" << std::endl
        << "def inference(model, input):" << std::endl
        << "    return model.predict(input)[0]" << std::endl;

    // Compile python code
    PyObject* p_compiled_fn = 
        Py_CompileString(buf.str().c_str(), "" , Py_file_input);
    
    // Create module for python code
    if (p_compiled_fn != NULL) {    
        p_module = PyImport_ExecCodeModule("scikit_learn_module", p_compiled_fn);
        
        // Get load_model function
        if (p_module != NULL) {
            PyObject* p_load_model_fn = 
                PyObject_GetAttrString(p_module, "load_model");
        
            // Pack arguments and load model
            if (p_load_model_fn != NULL) {
                PyObject *p_file = Py_BuildValue("s", model_path.c_str());
                PyObject *p_args = PyTuple_Pack(1, p_file);
                p_model = PyObject_CallObject(p_load_model_fn, p_args);
                Py_DecRef(p_args);
                Py_DecRef(p_file);
            }

            Py_DecRef(p_load_model_fn);
        }
    }
    Py_DecRef(p_compiled_fn);
}

}
}

// using namespace libcozmo::model;

// int main() {
//     Py_Initialize();
// 	GPRModel::ScikitLearnModel model("/home/vinitha/workspaces/curiosity_ws/src/curiosity_project/python/models/novel_uninformed/angle_regressor_12.pkl");
//     model.distance_prediction({0,2,2});
//     model.distance_prediction({0,0,0});
//     model.distance_prediction({0,6,2});
//     Py_Finalize(); 
// }