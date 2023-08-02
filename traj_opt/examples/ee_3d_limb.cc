#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/traj_opt/examples/example_base.h"

namespace drake {
namespace traj_opt {
namespace examples {
namespace ee_3d_limb {

using Eigen::Vector3d;
using math::RigidTransformd;
using math::RollPitchYaw;
using multibody::ModelInstanceIndex;
using multibody::MultibodyPlant;
using multibody::Parser;

class IiwaLimbExample : public TrajOptExample {
 public:
  IiwaLimbExample() {
    // Set the camera viewpoint
    std::vector<double> p = {0.0, 1.0, -5.0};
    meshcat_->SetProperty("/Cameras/default/rotated/<object>", "position", p);
  }

 private:
  void CreatePlantModel(MultibodyPlant<double>* plant) const {
    // const std::string urdf_file_table =
    //     FindResourceOrThrow("drake/traj_opt/examples/models/symphony_table.sdf");
    const std::string urdf_file_limb =
        FindResourceOrThrow("drake/traj_opt/examples/models/two_link_limb.sdf");
    const std::string urdf_file_robot =
        FindResourceOrThrow("drake/traj_opt/examples/models/jaco_with_gripper.sdf");
    // Parser(plant).AddAllModelsFromFile(urdf_file_table);
    ModelInstanceIndex limb = Parser(plant).AddModelFromFile(urdf_file_limb, "limb");
    ModelInstanceIndex robot = Parser(plant).AddModelFromFile(urdf_file_robot, "j2s7s300_arm");

    RigidTransformd X_robot(RollPitchYaw<double>(0, 0, 0),
                           Vector3d(0, 0, 0));
    RigidTransformd X_limb(RollPitchYaw<double>(0, 0, 0),
                           Vector3d(0.47, 0.38, 0.77));
    plant->WeldFrames(plant->world_frame(),
                      plant->GetFrameByName("base", robot), X_robot);
    plant->WeldFrames(plant->world_frame(),
                      plant->GetFrameByName("limb_base", limb), X_limb);

    plant->disable_gravity(robot);

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
  drake::traj_opt::examples::ee_3d_limb::IiwaLimbExample example;
  const std::string yaml_file = "drake/traj_opt/examples/ee_3d_limb.yaml";
  example.RunExample(yaml_file);

  return 0;
}
