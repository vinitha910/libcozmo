#include "model/Model.hpp"
#include <Python.h>
#include <string>
#include <vector>

namespace libcozmo {
namespace model {

class GPRModel : public virtual Model {
 public:
 	class ScikitLearnModel : public Model::ModelType {
 	 public:
 	 	ScikitLearnModel(const std::string& model_path);
 	 	~ScikitLearnModel();

 	 	double distance_prediction(const std::vector<double>& input);
 	 	double angle_prediction();

 	 	PyObject* p_model;
 	 	PyObject* p_module;
 	};

 	bool load_model(Model::ModelType* model) override { return true; }
 	Model::ModelOutput* get_prediction(const Model::ModelInput& input) override { return nullptr; }
};

}
}