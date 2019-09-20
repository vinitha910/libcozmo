#include "model/GPRModel.hpp"
#include <sstream> 
#include <iostream>

namespace libcozmo {
namespace model {

void GPRModel::inference(
    const Model::ModelInput& input_, Model::ModelOutput* output_) {
    const ModelInput input = 
        static_cast<const ModelInput&>(input_);
    PyObject* p_list = PyList_New(3);
    PyList_SetItem(p_list, 0, Py_BuildValue("f", input.speed));
    PyList_SetItem(p_list, 1, Py_BuildValue("f", input.edge_offset_ratio));
    PyList_SetItem(p_list, 2, Py_BuildValue("f", input.aspect_ratio));
    
    PyObject* p_input = PyList_New(1);
    PyList_SetItem(p_input, 0, p_list);

    PyObject* p_predict_fn = PyObject_GetAttrString(m_framework->p_module, "inference");
    PyObject* p_args = PyTuple_Pack(2, m_framework->p_model, p_input);
    PyObject* p_result = PyObject_CallObject(p_predict_fn, p_args);
    const double prediction = PyFloat_AsDouble(p_result);
    std::cout << prediction << std::endl;

    // // Index nested list
    // PyObject* outputs = PyList_GetItem(p_result, 0);
    // const double distance = PyFloat_AsDouble(PyList_GetItem(outputs, 0));
    // const double dtheta = PyFloat_AsDouble(PyList_GetItem(outputs, 1));

    // reminder, remember to test with updated model
    // ModelOutput* output = static_cast<ModelOutput*>(output_);
    // *output = ModelOutput(prediction, prediction);
}

}
}