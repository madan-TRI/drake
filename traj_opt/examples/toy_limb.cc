#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/traj_opt/examples/example_base.h"

namespace drake {
namespace traj_opt {
namespace examples {
namespace toy_limb {

using multibody::MultibodyPlant;
using multibody::Parser;
using Eigen::Vector3d;

class ToyLimbExample : public TrajOptExample {
 public:
  ToyLimbExample() {
    // Set the camera viewpoint
    std::vector<double> p = {0.0, 1.0, -5.0};
    meshcat_->SetProperty("/Cameras/default/rotated/<object>", "position", p);
  }

 private:
  void CreatePlantModel(MultibodyPlant<double>* plant) const {
    // const std::string urdf_file_table =
    //     FindResourceOrThrow("drake/traj_opt/examples/models/symphony_table.sdf");
    const std::string urdf_file_limb =
        FindResourceOrThrow("drake/traj_opt/examples/models/three_link_arm.sdf");
    const std::string urdf_file_ball =
        FindResourceOrThrow("drake/traj_opt/examples/models/ball.sdf");
    // Parser(plant).AddAllModelsFromFile(urdf_file_table);
    Parser(plant).AddAllModelsFromFile(urdf_file_limb);
    Parser(plant).AddAllModelsFromFile(urdf_file_ball);

    // Define gravity (so we can turn the hand upside down)
    // if (FLAGS_upside_down) {
    // plant->mutable_gravity_field().set_gravity_vector(Vector3d(0, -9.81, 0.0));
    // }

    // plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("base_link"));
  }
};

}  // namespace toy_limb
}  // namespace examples
}  // namespace traj_opt
}  // namespace drake

int main() {
  drake::traj_opt::examples::toy_limb::ToyLimbExample example;
  const std::string yaml_file = "drake/traj_opt/examples/toy_limb.yaml";
  example.RunExample(yaml_file);

  return 0;
}
