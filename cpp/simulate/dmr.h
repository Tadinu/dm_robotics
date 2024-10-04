#include <algorithm>
#include <memory>

#include "absl/container/btree_set.h"
#include "absl/types/span.h"
#include "dm_robotics/controllers/lsqp/cartesian_6d_to_joint_velocity_mapper.h"
#include "dm_robotics/mujoco/types.h"
#include "mujoco/mujoco.h"

namespace dmr = ::dm_robotics;

void demo_lsqp(mjModel* model, mjData* data) {
  // Initialize MuJoCo simulation. Assumes velocity controlled robot.
  // data->ctrl is an array of size 7 that corresponds to the commanded
  // velocities of the joints with IDs 7, 8, 9, 10, 12, 13, 14.

  // Create parameters.
  dmr::Cartesian6dToJointVelocityMapper::Parameters params;
  //
  // Set model parameters.
  params.model = model;
  params.joint_ids = absl::btree_set<int>{7, 8, 9, 10, 12, 13, 14};
  params.object_type = mjtObj::mjOBJ_SITE;
  params.object_name = "end_effector";
  params.integration_timestep = absl::Milliseconds(5);
  //
  // Enable joint position limit constraint. Limits are read automatically from
  // the model.
  params.enable_joint_position_limits = true;
  params.joint_position_limit_velocity_scale = 0.95;
  params.minimum_distance_from_joint_position_limit = 0.01;  // ~0.5deg.
  //
  // Enable joint velocity limits.
  params.enable_joint_velocity_limits = true;
  params.joint_velocity_magnitude_limits = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
  //
  // Enable joint acceleration limits.
  params.enable_joint_acceleration_limits = true;
  params.remove_joint_acceleration_limits_if_in_conflict = true;
  params.joint_acceleration_magnitude_limits = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
  //
  // Enable collision avoidance between the following geoms:
  //   * "gripper" and "base_link"
  //   * "base_link" and "floor"
  //   * "link1" and "floor"
  //   * "gripper" and "floor"
  //   * "link1" and "link4"
  //   * "link1" and "link5"
  //   * "link1" and "link6"
  //   * "link2" and "link4"
  //   * "link2" and "link5"
  //   * "link2" and "link6"
  // Note that collision avoidance will not be enabled for a pair of geoms if
  // they are attached to the same body or are attached to bodies that have a
  // parent-child relationship.
  params.enable_collision_avoidance = true;
  params.collision_avoidance_normal_velocity_scale = 0.5;
  params.minimum_distance_from_collisions = 0.01;
  params.collision_detection_distance = 0.3;
  params.collision_pairs = {
      dmr::CollisionPair(dmr::GeomGroup{"gripper"}, dmr::GeomGroup{"base_link"}),
      dmr::CollisionPair(dmr::GeomGroup{"base_link", "link1", "gripper"}, dmr::GeomGroup{"floor"}),
      dmr::CollisionPair(dmr::GeomGroup{"link1", "link2"}, dmr::GeomGroup{"link4", "link5", "link6"})};
  //
  // Numerical stability parameters.
  params.check_solution_validity = true;
  params.solution_tolerance = 1.0e-3;
  params.regularization_weight = 1e-2;
  params.enable_nullspace_control = true;
  params.return_error_on_nullspace_failure = false;
  params.nullspace_projection_slack = 1e-7;

  // Create mapper.
  dmr::Cartesian6dToJointVelocityMapper mapper(params);
#if 0
  // Compute joint velocities and apply them to the joint velocity actuator
  // commands at every step.
  while (true) {
    // The nullspace bias is often chosen to be a velocity towards the mid-range
    // of the joints, but can be chosen to be any vector of the same length as
    // the joint_ids parameter.
    const std::array<double, 3> nullspace_bias = GetNullspaceBias();
    const std::array<double, 6> target_velocity = GetEndEffectorTargetVelocity();
    absl::Span<const double> solution,
        mapper.ComputeJointVelocities(*data, target_velocity, nullspace_bias).value();
    std::copy(solution.begin(), solution.end(), data->ctrl);
    mj_step(model, data);
  }
#endif
}