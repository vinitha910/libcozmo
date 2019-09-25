#include "model/GPRModel.hpp"
#include <sstream> 
#include <iostream>
#include "aikido/statespace/SE2.hpp"
#include "utils/utils.hpp"

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

void GPRModel::predict_state(
    const Model::ModelInput& model_input_, 
    const aikido::statespace::StateSpace::State& in_,
    aikido::statespace::StateSpace::State* out_) {
    ModelOutput output;
    inference(model_input_, &output);
    const aikido::statespace::SE2::State in_state = 
        static_cast<const aikido::statespace::SE2::State&>(in_); 
    
    // Calculate new position
    const ModelInput model_input = 
        static_cast<const ModelInput&>(model_input_);
    const Eigen::Vector2d direction_norm = model_input.direction.normalized();
    const auto transform = in_state.getIsometry();
    const double x = 
        transform.translation().x() + output.distance * direction_norm.x();
    const double y = 
        transform.translation().y() + output.distance * direction_norm.y();

    // Calculate new rotation
    Eigen::Rotation2Dd rotation = Eigen::Rotation2Dd::Identity();
    rotation.fromRotationMatrix(transform.rotation());
    const double theta = 
        utils::angle_normalization(rotation.angle() + output.dtheta);

    // Set out state
    aikido::statespace::SE2::State* out_state = 
        static_cast<aikido::statespace::SE2::State*>(out_); 
    Eigen::Isometry2d t = Eigen::Isometry2d::Identity();
    const Eigen::Rotation2D<double> rot(theta);
    t.linear() = rot.toRotationMatrix();
    t.translation() = Eigen::Vector2d(x, y);
    out_state->setIsometry(t);
}

}
}