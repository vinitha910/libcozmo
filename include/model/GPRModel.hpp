#include "model/Model.hpp"
#include <string>

namespace libcozmo {
namespace model {

class GPRModel : public virtual Model {
 public:
 	class ScikitLearnModel : public Model::ModelType {
 	 public:
 	 	ScikitLearnModel(const std::string& model_path);
 	 	~ScikitLearnModel() = default;
 	};

 	bool load_model(Model::ModelType* model) override { return true; }
 	Model::ModelOutput* get_prediction(const Model::ModelInput& input) override { return nullptr; }
};

}
}