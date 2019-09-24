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

    PyObject* p_inference_fn = 
        PyObject_GetAttrString(m_framework->get_module(), "inference");
    PyObject* p_args = PyTuple_Pack(2, m_framework->get_model(), p_input);
    PyObject* p_result = PyObject_CallObject(p_inference_fn, p_args);

    ModelOutput* output = static_cast<ModelOutput*>(output_);
    output->distance = PyFloat_AsDouble(PyList_GetItem(p_result, 0));
    output->dtheta = PyFloat_AsDouble(PyList_GetItem(p_result, 1));
}

}
}