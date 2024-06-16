/**
 * Author: Vamsi Kalagaturu
 *
 * Description: Library to handle basic utilities for the arm_actions package
 *
 * Copyright (c) [2023]
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
extern "C"
{
#include "kelo_motion_control/mediator.h"
}

#include "motion_spec_utils/utils.hpp"

void initialize_manipulator_state(int num_joints, int num_segments, ManipulatorState *rob)
{
  rob->nj = num_joints;
  rob->ns = num_segments;

  rob->q = new double[num_joints]{};
  rob->q_dot = new double[num_joints]{};
  rob->q_ddot = new double[num_joints]{};

  rob->s = new double *[num_segments];
  rob->s_dot = new double *[num_segments];
  rob->s_ddot = new double *[num_segments];

  for (size_t i = 0; i < num_segments; i++)
  {
    rob->s[i] = new double[7]{};  // *Assumption* - quaternion
    rob->s_dot[i] = new double[6]{};
    rob->s_ddot[i] = new double[6]{};
  }

  rob->tau_command = new double[num_joints]{};
  rob->tau_measured = new double[num_joints]{};
  rob->f_tool_command = new double[6]{};
  rob->f_tool_measured = new double[6]{};
}

void free_manipulator_state(ManipulatorState *rob)
{
  delete[] rob->q;
  delete[] rob->q_dot;
  delete[] rob->q_ddot;

  for (size_t i = 0; i < rob->ns; i++)
  {
    delete[] rob->s[i];
    delete[] rob->s_dot[i];
    delete[] rob->s_ddot[i];
  }

  delete[] rob->s;
  delete[] rob->s_dot;
  delete[] rob->s_ddot;

  delete[] rob->tau_command;
  delete[] rob->tau_measured;
  delete[] rob->f_tool_command;
  delete[] rob->f_tool_measured;
}

void initialize_mobile_base_state(MobileBaseState *base)
{
  base->pivot_angles = new double[4]{};
  base->wheel_encoder_values = new double[8]{};
  base->prev_wheel_encoder_values = new double[8]{};
  base->qd_wheel = new double[8]{};

  base->xd_platform = new double[3]{};  // vx, vy, va
  base->x_platform = new double[3]{};   // x, y, theta

  base->tau_command = new double[8]{};
}

void free_mobile_base_state(MobileBaseState *base)
{
  delete[] base->pivot_angles;
  delete[] base->wheel_encoder_values;
  delete[] base->prev_wheel_encoder_values;
  delete[] base->qd_wheel;

  delete[] base->xd_platform;
  delete[] base->x_platform;

  delete[] base->tau_command;
}

void free_manipulator(Manipulator<kinova_mediator> *rob)
{
  free_manipulator_state(rob->state);
  delete rob->mediator;
}

void free_mobile_base(MobileBase<Robile> *base)
{
  free_mobile_base_state(base->state);
  delete base->mediator->ethercat_config;
  delete[] base->mediator->kelo_base_config->index_to_EtherCAT;
  delete[] base->mediator->kelo_base_config->wheel_coordinates;
  delete[] base->mediator->kelo_base_config->pivot_angles_deviation;
}

void free_robot_data(Freddy *rob)
{
  free_manipulator(rob->kinova_left);
  free_manipulator(rob->kinova_right);
  free_mobile_base(rob->mobile_base);
}

void initialize_robot(std::string robot_urdf, char *interface, Freddy *freddy)
{
  // load the robot urdf
  if (!kdl_parser::treeFromFile(robot_urdf, freddy->tree))
  {
    std::cerr << "Failed to construct KDL tree" << std::endl;
    exit(1);
  }

  // *Assumption* - base_link is the root link
  std::string base_link = "base_link";

  // left arm
  if (!freddy->tree.getChain(base_link, freddy->kinova_left->tool_frame,
                             freddy->kinova_left->chain))
  {
    std::cerr << "Failed to get chain from KDL tree" << std::endl;
    exit(1);
  }

  // right arm
  if (!freddy->tree.getChain(base_link, freddy->kinova_right->tool_frame,
                             freddy->kinova_right->chain))
  {
    std::cerr << "Failed to get chain from KDL tree" << std::endl;
  }

  // initialize the states
  initialize_manipulator_state(freddy->kinova_left->chain.getNrOfJoints(),
                               freddy->kinova_left->chain.getNrOfSegments(),
                               freddy->kinova_left->state);

  initialize_manipulator_state(freddy->kinova_right->chain.getNrOfJoints(),
                               freddy->kinova_right->chain.getNrOfSegments(),
                               freddy->kinova_right->state);

  initialize_mobile_base_state(freddy->mobile_base->state);

  // initialize the mediators
  int r = 0;
  freddy->kinova_left->mediator->initialize(0, 0, 0.0);
  // r = freddy->kinova_left->mediator->set_control_mode(2);

  // if (r != 0)
  // {
  //   std::cerr << "Failed to set control mode for the left arm" << std::endl;
  //   exit(1);
  // }

  freddy->kinova_right->mediator->initialize(0, 1, 0.0);
  // r = freddy->kinova_right->mediator->set_control_mode(2);

  // if (r != 0)
  // {
  //   std::cerr << "Failed to set control mode for the right arm" << std::endl;
  //   exit(1);
  // }

  init_ecx_context(freddy->mobile_base->mediator->ethercat_config);

  int result = 0;
  char ifname[] = "eno1";
  establish_kelo_base_connection(freddy->mobile_base->mediator->kelo_base_config,
                                 freddy->mobile_base->mediator->ethercat_config, ifname, &result);

  if (result != 0)
  {
    std::cerr << "Failed to establish connection with the mobile base" << std::endl;
    exit(1);
  }

  std::cout << "Successfully initialized robot" << std::endl;
}

void initialize_robot_sim(std::string robot_urdf, Freddy *freddy)
{
  // load the robot urdf
  if (!kdl_parser::treeFromFile(robot_urdf, freddy->tree))
  {
    std::cerr << "Failed to construct KDL tree" << std::endl;
    exit(1);
  }

  // *Assumption* - base_link is the root link
  std::string base_link = "base_link";

  // left arm
  if (!freddy->tree.getChain(base_link, freddy->kinova_left->tool_frame,
                             freddy->kinova_left->chain))
  {
    std::cerr << "Failed to get chain from KDL tree" << std::endl;
    exit(1);
  }

  // right arm
  if (!freddy->tree.getChain(base_link, freddy->kinova_right->tool_frame,
                             freddy->kinova_right->chain))
  {
    std::cerr << "Failed to get chain from KDL tree" << std::endl;
  }

  // initialize the states
  initialize_manipulator_state(freddy->kinova_left->chain.getNrOfJoints(),
                               freddy->kinova_left->chain.getNrOfSegments(),
                               freddy->kinova_left->state);

  initialize_manipulator_state(freddy->kinova_right->chain.getNrOfJoints(),
                               freddy->kinova_right->chain.getNrOfSegments(),
                               freddy->kinova_right->state);

  initialize_mobile_base_state(freddy->mobile_base->state);

  std::cout << "Successfully initialized robot" << std::endl;
}

void initialize_robot_chain(std::string robot_urdf, std::string base_link, std::string tool_link,
                            KDL::Chain &robot_chain)
{
  KDL::Tree robot_tree;

  // load the robot urdf
  if (!kdl_parser::treeFromFile(robot_urdf, robot_tree))
  {
    std::cerr << "Failed to construct KDL tree" << std::endl;
  }

  // get the chain
  if (!robot_tree.getChain(base_link, tool_link, robot_chain))
  {
    std::cerr << "Failed to get chain from KDL tree" << std::endl;
  }

  std::cout << "Successfully initialized robot chain" << std::endl;
}

void getLinkIdFromChain(KDL::Chain &chain, std::string link_name, int &link_id)
{
  for (int i = 0; i < chain.getNrOfSegments(); i++)
  {
    if (chain.getSegment(i).getName() == link_name)
    {
      link_id = i;
      return;
    }
  }
}

void add(double *arr1, double *arr2, double *result, size_t size)
{
  for (size_t i = 0; i < size; i++)
  {
    result[i] = arr1[i] + arr2[i];
  }
}

void updateQandQdot(double *q_ddot, double dt, ManipulatorState *rob)
{
  for (size_t i = 0; i < rob->nj; i++)
  {
    rob->q_ddot[i] = q_ddot[i];
    rob->q_dot[i] += q_ddot[i] * dt;
    rob->q[i] += rob->q_dot[i] * dt;
  }
}

void getLinkId(Freddy *rob, std::string root_link, std::string tip_link, std::string link_name,
               int &link_id)
{
  KDL::Chain chain;
  if (!rob->tree.getChain(root_link, tip_link, chain))
  {
    std::cerr << "Failed to get chain from KDL tree" << std::endl;
  }

  getLinkIdFromChain(chain, link_name, link_id);
}

template <typename MediatorType>
void get_manipulator_data(Manipulator<MediatorType> *rob)
{
  KDL::JntArray q(rob->state->nj);
  KDL::JntArray q_dot(rob->state->nj);
  KDL::JntArray tau_measured(rob->state->nj);
  KDL::Wrench f_tool_measured;

  rob->mediator->get_robot_state(q, q_dot, tau_measured, f_tool_measured);

  for (size_t i = 0; i < rob->state->nj; i++)
  {
    rob->state->q[i] = q(i);
    rob->state->q_dot[i] = q_dot(i);
    rob->state->tau_measured[i] = tau_measured(i);
  }
}

void update_manipulator_state(ManipulatorState *state, std::string tool_frame, KDL::Tree *tree)
{
  // *Assumption* - computing values in the base_link frame
  // create a chain from the base_link to the tool_link
  KDL::Chain *chain = new KDL::Chain();
  if (!tree->getChain("base_link", tool_frame, *chain))
  {
    std::cerr << "Failed to get chain from KDL tree" << std::endl;
  }

  // update the s_dot values
  KDL::ChainFkSolverVel_recursive fk_solver_vel(*chain);
  std::vector<KDL::FrameVel> frame_vels(chain->getNrOfSegments());
  KDL::JntArrayVel q_dot_kdl(state->nj);
  for (size_t i = 0; i < state->nj; i++)
  {
    q_dot_kdl.q(i) = state->q[i];
    q_dot_kdl.qdot(i) = state->q_dot[i];
  }
  fk_solver_vel.JntToCart(q_dot_kdl, frame_vels);

  int additional_links = chain->getNrOfSegments() - state->nj;

  for (size_t i = 0; i < chain->getNrOfSegments(); i++)
  {
    int frame_vel_index = i;  // + additional_links;

    // update the s_dot values
    state->s_dot[i][0] = frame_vels[frame_vel_index].GetTwist().vel.x();
    state->s_dot[i][1] = frame_vels[frame_vel_index].GetTwist().vel.y();
    state->s_dot[i][2] = frame_vels[frame_vel_index].GetTwist().vel.z();
    state->s_dot[i][3] = frame_vels[frame_vel_index].GetTwist().rot.x();
    state->s_dot[i][4] = frame_vels[frame_vel_index].GetTwist().rot.y();
    state->s_dot[i][5] = frame_vels[frame_vel_index].GetTwist().rot.z();

    // update the s values
    state->s[i][0] = frame_vels[frame_vel_index].GetFrame().p.x();
    state->s[i][1] = frame_vels[frame_vel_index].GetFrame().p.y();
    state->s[i][2] = frame_vels[frame_vel_index].GetFrame().p.z();
    frame_vels[frame_vel_index].GetFrame().M.GetQuaternion(state->s[i][3], state->s[i][4],
                                                           state->s[i][5], state->s[i][6]);
  }
}

void cap_and_convert_torques(double *tau_command, int num_joints, KDL::JntArray &tau_jnt)
{
  // limit the torques to +/- 30 Nm
  double max_torque = 30.0;
  for (size_t i = 0; i < num_joints; i++)
  {
    if (tau_command[i] > max_torque)
    {
      tau_command[i] = max_torque;
    }
    else if (tau_command[i] < -max_torque)
    {
      tau_command[i] = -max_torque;
    }
  }

  // convert to JointArray
  for (size_t i = 0; i < num_joints; i++)
  {
    tau_jnt(i) = tau_command[i];
  }
}

int set_manipulator_torques(Freddy *rob, std::string root_link, KDL::JntArray *tau_command)
{
  // get robot from root link
  Manipulator<kinova_mediator> *arm =
      root_link == rob->kinova_left->base_frame ? rob->kinova_left : rob->kinova_right;

  // send the torques to the robot
  int r = arm->mediator->set_joint_torques(*tau_command);
  if (r != 0)
  {
    std::cerr << "Failed to set joint torques for the arm" << std::endl;
    return r;
  }

  return 0;
}

void getLinkSFromRob(std::string link_name, Freddy *rob, double *s)
{
  // find which robot the link belongs to

  if (link_name == "base_link")
  {
    for (size_t i = 0; i < 6; i++)
    {
      s[i] = 0.0;
    }
    s[6] = 1.0;  // *Assumption* - quaternion
    return;
  }

  int link_id = -1;
  bool is_in_left_chain, is_in_right_chain = false;
  findLinkInChain(link_name, &rob->kinova_left->chain, is_in_left_chain, link_id);
  if (!is_in_left_chain)
  {
    findLinkInChain(link_name, &rob->kinova_right->chain, is_in_right_chain, link_id);
  }

  if (!is_in_left_chain && !is_in_right_chain)
  {
    std::cerr << "[getLinkSFromRob] Link " << link_name << " not found in the robot chains"
              << std::endl;
    exit(1);
  }

  // Select appropriate state data based on which chain the link was found in
  const double *state = is_in_left_chain ? rob->kinova_left->state->s[link_id]
                                         : rob->kinova_right->state->s[link_id];

  // *Assumption* - quaternion
  for (size_t i = 0; i < 7; i++)
  {
    s[i] = state[i];
  }
}

void computeDistance(std::string *between_ents, std::string asb, Freddy *rob, double &distance)
{
  double *ent1 = new double[6]{};
  double *ent2 = new double[6]{};

  getLinkSFromRob(between_ents[0], rob, ent1);
  getLinkSFromRob(between_ents[1], rob, ent2);

  // *Assumption* - euclidean distance for linear components
  distance =
      sqrt(pow(ent1[0] - ent2[0], 2) + pow(ent1[1] - ent2[1], 2) + pow(ent1[2] - ent2[2], 2));
}

void computeDistance1D(std::string *between_ents, double *axis, std::string asb, Freddy *rob,
                       double &distance)
{
  double *ent1 = new double[6]{};
  double *ent2 = new double[6]{};

  getLinkSFromRob(between_ents[0], rob, ent1);
  getLinkSFromRob(between_ents[1], rob, ent2);

  // *Assumption* - axis-aligned vector for linear components
  for (size_t i = 0; i < 3; i++)
  {
    if (axis[i] == 1)
    {
      distance = abs(ent1[i] - ent2[i]);
      return;
    }
  }

  // if not
  std::cerr << "[computeDistance1D] Requested angular distance" << std::endl;
  raise(SIGINT);
}

void getLine(Freddy *rob, std::string *entities, size_t num_entities, double *direction)
{
  if (num_entities < 2)
  {
    std::cerr << "[get_line] Need at least 2 entities to find a line" << std::endl;
  }

  // get the position of the entities
  double **positions = new double *[num_entities];
  for (size_t i = 0; i < num_entities; i++)
  {
    double pose[7]{};
    getLinkSFromRob(entities[i], rob, pose);

    // get the position
    positions[i] = new double[3]{};
    for (size_t j = 0; j < 3; j++)
    {
      positions[i][j] = pose[j];
    }
  }

  // find the line passing through the entities
  for (size_t i = 0; i < 3; i++)
  {
    direction[i] = positions[1][i] - positions[0][i];
  }

  // normalize the direction vector
  findNormalizedVector(direction, direction);
}

void dotProduct(double *vec1, double *vec2, size_t size, double &result)
{
  result = 0.0;
  for (size_t i = 0; i < size; i++)
  {
    result += vec1[i] * vec2[i];
  }
}

void crossProduct(double *vec1, double *vec2, size_t size, double *result)
{
  result[0] = vec1[1] * vec2[2] - vec1[2] * vec2[1];
  result[1] = vec1[2] * vec2[0] - vec1[0] * vec2[2];
  result[2] = vec1[0] * vec2[1] - vec1[1] * vec2[0];
}

void getMagnitude(double *vec, size_t size, double &magnitude)
{
  magnitude = 0.0;
  for (size_t i = 0; i < size; i++)
  {
    magnitude += vec[i] * vec[i];
  }
  magnitude = sqrt(magnitude);
}

void getAngleBetweenLines(Freddy *rob, double *line1, double *line2, double *angle_about_vec,
                          double &angle)
{
  // based on the angle_about_vec, project the lines onto the plane perpendicular to the vector

  double *proj_line1 = new double[3]{};
  double *proj_line2 = new double[3]{};

  double angle_about_vec_angular[3]{};
  for (size_t i = 3; i < 6; i++)
  {
    angle_about_vec_angular[i - 3] = angle_about_vec[i];
  }

  double dot_product = 0.0;
  dotProduct(line1, angle_about_vec_angular, 3, dot_product);

  for (size_t i = 0; i < 3; i++)
  {
    proj_line1[i] = line1[i] - dot_product * angle_about_vec_angular[i];
    proj_line2[i] = line2[i] - dot_product * angle_about_vec_angular[i];
  }

  // find the angle between the projected lines
  double dot_product_proj = 0.0;
  dotProduct(proj_line1, proj_line2, 3, dot_product_proj);

  double cross_product[3]{};
  crossProduct(proj_line1, proj_line2, 3, cross_product);

  double cross_product_magnitude = 0.0;
  getMagnitude(cross_product, 3, cross_product_magnitude);

  angle = atan2(cross_product_magnitude, dot_product_proj);

  // free memory
  delete[] proj_line1;
  delete[] proj_line2;
}

void getLinkPosition(std::string link_name, std::string as_seen_by, std::string with_respect_to,
                     double *vec, Freddy *rob, double &out_position)
{
  double pose[7]{};
  getLinkSFromRob(link_name, rob, pose);

  // *Assumption* - wrt base_link
  if (with_respect_to != "base_link")
  {
    std::cerr << "[getLinkPosition] with-respect-to is not base_link but " << with_respect_to
              << ", which is not supported!" << std::endl;
    raise(SIGINT);
  }

  for (size_t i = 0; i < 3; i++)
  {
    if (vec[i] == 1)
    {
      out_position = pose[i];
      return;
    }
  }

  // if not
  std::cerr << "[getLinkPosition] Requested angular pose" << std::endl;
  raise(SIGINT);
}

void getLinkQuaternion(std::string link_name, std::string as_seen_by, std::string with_respect_to,
                       Freddy *rob, double *out_quaternion)
{
  double pose[7]{};
  getLinkSFromRob(link_name, rob, pose);

  // *Assumption* - wrt base_link
  if (with_respect_to != "base_link")
  {
    std::cerr << "[getLinkQuaternion] with-respect-to is not base_link but " << with_respect_to
              << ", which is not supported!" << std::endl;
    raise(SIGINT);
  }

  if (as_seen_by == "base_link")
  {
    for (size_t i = 0; i < 4; i++)
    {
      out_quaternion[i] = pose[i + 3];
    }
    return;
  }

  // Transformation to as_seen_by frame
  std::cerr << "[getLinkQuaternion] Transformation feature not implemented yet" << std::endl;
  raise(SIGINT);
}

void getLinkVelocity(std::string link_name, std::string as_seen_by, std::string with_respect_to,
                     double *vec, Freddy *rob, double &out_twist)
{
  // find which robot the link belongs to
  bool is_in_left_chain = false;
  int link_id = -1;
  findLinkInChain(link_name, &rob->kinova_left->chain, is_in_left_chain, link_id);

  bool is_in_right_chain = false;
  findLinkInChain(link_name, &rob->kinova_right->chain, is_in_right_chain, link_id);

  if (!is_in_left_chain && !is_in_right_chain)
  {
    std::cerr << "[getLinkVelocity] Link not found in the robot chains" << std::endl;
    exit(1);
  }

  // Select appropriate state data based on which chain the link was found in
  const double *state = is_in_left_chain ? rob->kinova_left->state->s_dot[link_id]
                                         : rob->kinova_right->state->s_dot[link_id];

  bool require_transform = false;
  if (as_seen_by == "base_link")
  {
    for (size_t i = 0; i < 6; ++i)
    {
      if (vec[i] != 0)
      {
        out_twist = state[i];
        return;
      }
    }
  }
  else
  {
    // Transformation of the twist is not yet implemented
    std::cerr << "Transformation feature not implemented yet" << std::endl;
    exit(1);
  }
}

void getLinkForce(std::string applied_by, std::string applied_to, std::string asb, double *vec,
                  Freddy *freddy, double &force)
{
  // TODO: implement this
  force = 0.0;
}

void findVector(std::string from_ent, std::string to_ent, Freddy *rob, double *vec)
{
  double *from_s = new double[7]{};
  double *to_s = new double[7]{};

  getLinkSFromRob(from_ent, rob, from_s);

  printf("from_s: %s: %f %f %f\n", from_ent.c_str(), from_s[0], from_s[1], from_s[2]);

  getLinkSFromRob(to_ent, rob, to_s);

  printf("to_s: %s: %f %f %f\n", to_ent.c_str(), to_s[0], to_s[1], to_s[2]);

  // TODO: dont use hardcoded values
  // *Assumption* - only linear components
  for (size_t i = 0; i < 3; i++)
  {
    vec[i] = to_s[i] - from_s[i];
  }
}

void findNormalizedVector(const double *vec, double *normalized_vec)
{
  double norm = 0.0;
  // TODO: dont use hardcoded values
  // *Assumption* - only linear components
  for (size_t i = 0; i < 3; i++)
  {
    norm += vec[i] * vec[i];
  }
  norm = sqrt(norm);

  for (size_t i = 0; i < 3; i++)
  {
    normalized_vec[i] = vec[i] / norm;
  }
}

void decomposeSignal(Freddy *rob, const std::string from_ent, const std::string to_ent,
                     std::string asb_ent, const double signal, double *vec)
{
  double *dir_vec = new double[3]{};
  findVector(from_ent, to_ent, rob, dir_vec);

  std::cout << "dir_vec_p: " << dir_vec[0] << " " << dir_vec[1] << " " << dir_vec[2] << std::endl;

  if (asb_ent != "base_link")
  {
    KDL::Chain chain;
    if (!rob->tree.getChain("base_link", asb_ent, chain))
    {
      std::cerr << "Failed to get chain from KDL tree" << std::endl;
      exit(1);
    }

    // check if there are any joints
    if (chain.getNrOfJoints() != 0)
    {
      std::cerr << "[decompose signal] Feature not implemented yet" << std::endl;
      exit(1);
    }

    KDL::Frame frame;
    KDL::ChainFkSolverPos_recursive fk_solver_pos(chain);

    KDL::JntArray q = KDL::JntArray(chain.getNrOfJoints());
    fk_solver_pos.JntToCart(q, frame);

    KDL::Vector vec1(dir_vec[0], dir_vec[1], dir_vec[2]);
    vec1 = frame * vec1;

    for (size_t i = 0; i < 3; i++)
    {
      dir_vec[i] = vec1[i];
    }
  }

  std::cout << "dir_vec: " << dir_vec[0] << " " << dir_vec[1] << " " << dir_vec[2] << std::endl;

  findNormalizedVector(dir_vec, dir_vec);

  for (size_t i = 0; i < 3; i++)
  {
    vec[i] = signal * dir_vec[i];
  }

  // TODO: update to only for linear components
  std::cerr << "[decomposeSignal] should only be for linear, update it" << std::endl;
  for (size_t i = 3; i < 6; i++)
  {
    vec[i] = 0.0;
  }
}

void get_robot_data(Freddy *freddy, double dt)
{
  get_manipulator_data(freddy->kinova_left);
  update_manipulator_state(freddy->kinova_left->state, freddy->kinova_left->tool_frame,
                           &freddy->tree);

  get_manipulator_data(freddy->kinova_right);
  update_manipulator_state(freddy->kinova_right->state, freddy->kinova_right->tool_frame,
                           &freddy->tree);

  send_and_receive_data(freddy->mobile_base->mediator->ethercat_config);
  get_kelo_base_state(
      freddy->mobile_base->mediator->kelo_base_config,
      freddy->mobile_base->mediator->ethercat_config, freddy->mobile_base->state->pivot_angles,
      freddy->mobile_base->state->wheel_encoder_values, freddy->mobile_base->state->qd_wheel);

  compute_kelo_platform_velocity(freddy);
  // ckpv(freddy, dt);
  // printf("[base velocities]: ");
  // print_array(freddy->mobile_base->state->xd_platform, 3);

  compute_kelo_platform_pose(freddy->mobile_base->state->xd_platform, dt,
                             freddy->mobile_base->state->x_platform);

  // std::cout << "odom: " << freddy->mobile_base->state->x_platform[0] << " "
  //           << freddy->mobile_base->state->x_platform[1] << " "
  //           << RAD2DEG(freddy->mobile_base->state->x_platform[2]) << std::endl;
}

void ckpv(Freddy *rob, double dt)
{
  // initialize the variables
  double vx, vy, va;
  vx = 0;
  vy = 0;
  va = 0;

  int nWheels = rob->mobile_base->mediator->kelo_base_config->nWheels;
  double r_w = rob->mobile_base->mediator->kelo_base_config->radius;
  double s_d_ratio = rob->mobile_base->mediator->kelo_base_config->castor_offset /
                     (rob->mobile_base->mediator->kelo_base_config->half_wheel_distance * 2);

  for (int i = 0; i < nWheels; i++)
  {
    double wl = (rob->mobile_base->state->wheel_encoder_values[2 * i] -
                 rob->mobile_base->state->prev_wheel_encoder_values[2 * i]) /
                dt;
    double wr = -(rob->mobile_base->state->wheel_encoder_values[2 * i + 1] -
                  rob->mobile_base->state->prev_wheel_encoder_values[2 * i + 1]) /
                dt;

    rob->mobile_base->state->prev_wheel_encoder_values[2 * i] =
        rob->mobile_base->state->wheel_encoder_values[2 * i];
    rob->mobile_base->state->prev_wheel_encoder_values[2 * i + 1] =
        rob->mobile_base->state->wheel_encoder_values[2 * i + 1];

    double theta =
        rob->mobile_base->state->pivot_angles[i];  // encoder_offset can be obtained from the yaml
                                                   // file or smartWheelDriver class

    vx -= r_w * ((wl + wr) * cos(theta));  // + 2 * s_d_ratio * (wl - wr) * sin(theta));
    vy -= r_w * ((wl + wr) * sin(theta));  // - 2 * s_d_ratio * (wl - wr) * cos(theta));

    double wangle =
        atan2(rob->mobile_base->mediator->kelo_base_config->wheel_coordinates[2 * i + 1],
              rob->mobile_base->mediator->kelo_base_config->wheel_coordinates[2 * i]);
    double d =
        sqrt(pow(rob->mobile_base->mediator->kelo_base_config->wheel_coordinates[2 * i], 2) +
             pow(rob->mobile_base->mediator->kelo_base_config->wheel_coordinates[2 * i + 1], 2));

    va += r_w *
          (2 * (wr - wl) * s_d_ratio * cos(theta - wangle) - (wr + wl) * sin(theta - wangle)) / d;

    // va += r_w * (wr + wl) * sin(theta - wangle) / d;
    // va += 4*swData->gyro_y;
  }
  // averaging the wheel velocity
  vx = vx / nWheels / 2;
  vy = vy / nWheels / 2;
  va = va / nWheels / 2;

  rob->mobile_base->state->xd_platform[0] = vx;
  rob->mobile_base->state->xd_platform[1] = vy;
  rob->mobile_base->state->xd_platform[2] = va;
}

void compute_kelo_platform_velocity(Freddy *rob)
{
  double caster_offsets[4]{};
  for (size_t i = 0; i < 4; i++)
  {
    caster_offsets[i] = rob->mobile_base->mediator->kelo_base_config->castor_offset;
  }

  double wheel_distances[4]{};
  for (size_t i = 0; i < 4; i++)
  {
    wheel_distances[i] = rob->mobile_base->mediator->kelo_base_config->half_wheel_distance * 2;
  }

  double wheel_diameters[8]{};
  for (size_t i = 0; i < 8; i++)
  {
    wheel_diameters[i] = rob->mobile_base->mediator->kelo_base_config->radius * 2;
  }

  double vel_dist_mat[rob->mobile_base->mediator->kelo_base_config->nWheels * 2 * 3]{};

  kelo_pltf_vel_dist_mat(rob->mobile_base->mediator->kelo_base_config->nWheels,
                         rob->mobile_base->mediator->kelo_base_config->wheel_coordinates,
                         caster_offsets, wheel_distances, wheel_diameters,
                         rob->mobile_base->state->pivot_angles, vel_dist_mat);

  double w_platform[3 * 3] = {
      // [1/N^2], [1/(N Nm)], [1/(Nm)^2]
      1.0, 0.0, 0.0,  // xx, xy, xm
      0.0, 1.0, 0.0,  // yx, yy, ym
      0.0, 0.0, 1.0   // mx, my, mm
  };
  double w_drive[4 * 4] = {
      // [1/N^2]
      1.0, 0.0, 0.0, 1.0,  // fl-xx, fl-xy, fl-yx, fl-yy
      1.0, 0.0, 0.0, 1.0,  // rl-xx, rl-xy, rl-yx, rl-yy
      1.0, 0.0, 0.0, 1.0,  // rr-xx, rr-xy, rr-yx, rr-yy
      1.0, 0.0, 0.0, 1.0   // fr-xx, fr-xy, fr-yx, fr-yy
  };

  kelo_pltf_slv_fwd_vel_comp(rob->mobile_base->mediator->kelo_base_config->nWheels, vel_dist_mat,
                             w_platform, w_drive, rob->mobile_base->state->qd_wheel,
                             rob->mobile_base->state->xd_platform);
}

void compute_kelo_platform_pose(double *xd_platform, double dt, double *x_platform)
{
  double dx, dy;

  if (fabs(xd_platform[2] > 0.001))
  {
    double vlin = sqrt(xd_platform[0] * xd_platform[0] + xd_platform[1] * xd_platform[1]);
    double direction = atan2(xd_platform[1], xd_platform[0]);
    double circleRadius = fabs(vlin / xd_platform[2]);
    double sign = 1;
    if (xd_platform[2] < 0)
      sign = -1;
    // displacement relative to direction of movement
    double dx_rel = circleRadius * sin(fabs(xd_platform[2]) * dt);
    double dy_rel = sign * circleRadius * (1 - cos(fabs(xd_platform[2]) * dt));

    // transform displacement to previous robot frame
    dx = dx_rel * cos(direction) - dy_rel * sin(direction);
    dy = dx_rel * sin(direction) + dy_rel * cos(direction);
  }
  else
  {
    dx = xd_platform[0] * dt;
    dy = xd_platform[1] * dt;
  }

  // transform displacement to odom frame
  x_platform[0] += dx * cos(x_platform[2]) - dy * sin(x_platform[2]);
  x_platform[1] += dx * sin(x_platform[2]) + dy * cos(x_platform[2]);
  x_platform[2] = norm(x_platform[2] + xd_platform[2] * (dt * 5));

  // // compute linear position components x, y and angular component theta
  // x_platform[0] += xd_platform[0] * dt * cos(x_platform[2]) - xd_platform[1] * dt *
  // sin(x_platform[2]); x_platform[1] += xd_platform[0] * dt * sin(x_platform[2]) + xd_platform[1]
  // * dt * cos(x_platform[2]); x_platform[2] += xd_platform[2] * 0.07;

  // // normalize the angle between -pi and pi
  // x_platform[2] = fmod(x_platform[2] + M_PI, 2 * M_PI);
  // if (x_platform[2] < 0)
  // {
  //   x_platform[2] += 2 * M_PI;
  // }
  // x_platform[2] -= M_PI;
}

void print_matrix2(int rows, int cols, const double *a)
{
  printf("[");
  for (int m_ = 0; m_ < rows; m_++)
  {
    printf("[");
    for (int n_ = 0; n_ < cols; n_++)
    {
      printf("%f", a[m_ + n_ * rows]);
      if (n_ != cols - 1)
        printf(", ");
    }
    printf("]");
    if (m_ != rows - 1)
      printf(",\n");
  }
  printf("]");
}

void print_robot_data(Freddy *rob)
{
  // std::cout << "Left arm state: " << std::endl;
  // std::cout << "-- q: ";
  // for (size_t i = 0; i < rob->kinova_left->state->nj; i++)
  // {
  //   std::cout << rob->kinova_left->state->q[i] << " ";
  // }
  // std::cout << std::endl;
  // std::cout << "-- tool pose: ";
  // for (size_t i = 0; i < 6; i++)
  // {
  //   std::cout << rob->kinova_left->state->s[rob->kinova_left->state->ns - 1][i] << " ";
  // }
  // std::cout << std::endl;
  // std::cout << "-- tool twist: ";
  // for (size_t i = 0; i < 6; i++)
  // {
  //   std::cout << rob->kinova_left->state->s_dot[rob->kinova_left->state->ns - 1][i]
  //             << " ";
  // }
  // std::cout << std::endl;
  // std::cout << "-- measured torque: ";
  // for (size_t i = 0; i < rob->kinova_left->state->nj; i++)
  // {
  //   std::cout << rob->kinova_left->state->tau_measured[i] << " ";
  // }
  // std::cout << std::endl;

  std::cout << "Right arm state: " << std::endl;
  std::cout << "-- q: ";
  for (size_t i = 0; i < rob->kinova_right->state->nj; i++)
  {
    std::cout << rob->kinova_right->state->q[i] << " ";
  }
  std::cout << std::endl;
  std::cout << "-- tool pose: ";
  for (size_t i = 0; i < 7; i++)
  {
    std::cout << rob->kinova_right->state->s[rob->kinova_right->state->ns - 1][i] << " ";
  }
  std::cout << std::endl;
  std::cout << "-- tool twist: ";
  for (size_t i = 0; i < 6; i++)
  {
    std::cout << rob->kinova_right->state->s_dot[rob->kinova_right->state->ns - 1][i] << " ";
  }
  std::cout << std::endl;

  // std::cout << "Right arm state: " << std::endl;
  // std::cout << "-- q: ";
  // for (size_t i = 0; i < rob->kinova_right->state->nj; i++)
  // {
  //   std::cout << rob->kinova_right->state->q[i] << " ";
  // }
  // std::cout << std::endl;
  // std::cout << "-- tool pose: ";
  // for (size_t i = 0; i < 6; i++)
  // {
  //   std::cout << rob->kinova_right->state->s[rob->kinova_right->state->ns - 1][i] << "
  //   ";
  // }
  // std::cout << std::endl;
  // std::cout << "-- tool twist: ";
  // for (size_t i = 0; i < 6; i++)
  // {
  //   std::cout << rob->kinova_right->state->s_dot[rob->kinova_right->state->ns - 1][i]
  //             << " ";
  // }
  // std::cout << std::endl;

  // std::cout << "Mobile base state: " << std::endl;
  // std::cout << "-- pivot angles: ";
  // for (size_t i = 0; i < rob->mobile_base->mediator->kelo_base_config->nWheels; i++)
  // {
  //   std::cout << rob->mobile_base->state->pivot_angles[i] << " ";
  // }
  // std::cout << std::endl;
}

void get_manipulator_data_sim(Manipulator<kinova_mediator> *rob, double *predicted_acc,
                              double time_step)
{
  // updated q and q_dot values
  for (size_t i = 0; i < rob->state->nj; i++)
  {
    rob->state->q_dot[i] += time_step * predicted_acc[i];
    rob->state->q[i] += time_step * rob->state->q_dot[i];
  }
}

void get_robot_data_sim(Freddy *freddy, double *kinova_left_predicted_acc,
                        double *kinova_right_predicted_acc, double time_step)
{
  // check if predicted acc is a nullptr
  if (kinova_left_predicted_acc != nullptr)
  {
    get_manipulator_data_sim(freddy->kinova_left, kinova_left_predicted_acc, time_step);
    update_manipulator_state(freddy->kinova_left->state, freddy->kinova_left->tool_frame,
                             &freddy->tree);
  }

  if (kinova_right_predicted_acc != nullptr)
  {
    get_manipulator_data_sim(freddy->kinova_right, kinova_right_predicted_acc, time_step);
    update_manipulator_state(freddy->kinova_right->state, freddy->kinova_right->tool_frame,
                             &freddy->tree);
  }
}

void computeQuaternionEqualityError(double *measured, double *ref, double *signal) 
{
  KDL::Rotation rot_measured = KDL::Rotation::Quaternion(measured[0], measured[1], measured[2], measured[3]);
  KDL::Rotation rot_ref = KDL::Rotation::Quaternion(ref[0], ref[1], ref[2], ref[3]);

  KDL::Vector diff = KDL::diff(rot_ref, rot_measured);
  signal[0] = diff.x();
  signal[1] = diff.y();
  signal[2] = diff.z();
}

void set_init_sim_data(Freddy *freddy)
{
  // left arm joint angles
  // 87.41 36.59 19.76 225.92 12.65 60.62 90.0
  freddy->kinova_left->state->q[0] = DEG2RAD(87.41);
  freddy->kinova_left->state->q[1] = DEG2RAD(36.59);
  freddy->kinova_left->state->q[2] = DEG2RAD(19.76);
  freddy->kinova_left->state->q[3] = DEG2RAD(225.92);
  freddy->kinova_left->state->q[4] = DEG2RAD(12.65);
  freddy->kinova_left->state->q[5] = DEG2RAD(60.62);
  freddy->kinova_left->state->q[6] = DEG2RAD(75.74);

  // right arm joint angles
  // 104.79 329.52 345.42 16.31 353.82 58.59 90.0
  freddy->kinova_right->state->q[0] = DEG2RAD(104.79);
  freddy->kinova_right->state->q[1] = DEG2RAD(329.52);
  freddy->kinova_right->state->q[2] = DEG2RAD(345.42);
  freddy->kinova_right->state->q[3] = DEG2RAD(16.31);
  freddy->kinova_right->state->q[4] = DEG2RAD(353.82);
  freddy->kinova_right->state->q[5] = DEG2RAD(58.59);
  freddy->kinova_right->state->q[6] = DEG2RAD(75.74);
}

void print_array(double *arr, int size)
{
  std::cout << "[ ";
  for (size_t i = 0; i < size; i++)
  {
    std::cout << arr[i] << " ";
  }
  std::cout << "]" << std::endl;
}

void init_2d_array(double **arr, int rows, int cols)
{
  for (size_t i = 0; i < rows; i++)
  {
    arr[i] = new double[cols]{};
  }
}

void get_new_folder_name(const char *dir_path, char *name)
{
  // get current time
  auto now = std::chrono::system_clock::now();
  auto now_c = std::chrono::system_clock::to_time_t(now);

  // convert to string
  std::stringstream ss;
  ss << std::put_time(std::localtime(&now_c), "%d_%m_%Y_%H_%M_%S");
  std::string time_str = ss.str();

  // create the folder name
  sprintf(name, "%s/%s", dir_path, time_str.c_str());
}

void write_odom_data_to_open_file(FILE *file, std::vector<std::array<double, 3>> &odom_data)
{
  // append the data to the file as comma separated values
  for (size_t i = 0; i < odom_data.size(); i++)
  {
    fprintf(file, "%f,%f,%f\n", odom_data[i][0], odom_data[i][1], odom_data[i][2]);
  }
}