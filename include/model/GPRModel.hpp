#include "model/Model.hpp"
#include "model/ScikitLearnFramework.hpp"
#include "statespace/StateSpace.hpp"
#include <Python.h>
#include <memory>
#include <Eigen/Geometry>

namespace libcozmo {
namespace model {

class GPRModel : public virtual Model {
 public:

 	/// The input into the Guassian Process Regressor Model is the 
 	/// object-oriented action being executed by cozmo
 	class ModelInput : public Model::ModelInput {
 	 public:
 	 	explicit ModelInput(
 	 		const double& speed, 
 	 		const double& edge_offset_ratio, 
 	 		const double& aspect_ratio,
 	 		const Eigen::Vector2d& direction) : 
 	 		speed(speed), 
 	 		edge_offset_ratio(edge_offset_ratio), 
 	 		aspect_ratio(aspect_ratio),
 	 		direction(direction) {}

 	 	/// The speed in mm/s
 	 	const double speed;

 	 	/// Normalized distance from center of edge in range [-1, 1], where -1 
 	 	/// and 1 are the left and right corner of the object respectively
 	 	const double edge_offset_ratio;

 	 	/// Either the width or height in the aspect ratio of the object
 	 	const double aspect_ratio;

 	 	const Eigen::Vector2d direction;
 	};

 	/// The output of the Guassian Process Regressor Model is the delta state
 	/// (i.e. the distance the object moved and the change in orientation)
 	class ModelOutput : public Model::ModelOutput {
 	 public:
 		explicit ModelOutput(const double& distance, const double& dtheta) : 
 			distance(distance), dtheta(dtheta) {}

 		ModelOutput() : distance(0.0), dtheta(0.0) {}

 		/// The distance the object moved after applying an action 
 		double distance;

 		/// The change in orientation of the object after applying an action 
 		double dtheta;
 	};

 	GPRModel(
 		const std::shared_ptr<ScikitLearnFramework> framework,
 		const std::shared_ptr<aikido::statespace::StateSpace> statespace) :
 		m_framework(framework),
 		m_statespace(statespace) {}
 	
 	~GPRModel() = default;

 	void inference(
 		const Model::ModelInput& input, Model::ModelOutput* output) override;

 	void predict_state(
        const Model::ModelInput& input, 
	    const aikido::statespace::StateSpace::State& in_,
	    aikido::statespace::StateSpace::State* out_) override;

 private:
 	const std::shared_ptr<ScikitLearnFramework> m_framework;
 	const std::shared_ptr<aikido::statespace::StateSpace> m_statespace;
};

}
}