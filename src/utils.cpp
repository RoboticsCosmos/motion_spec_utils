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
  // freddy->kinova_left->mediator->initialize(0, 0, 0.0);
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

void rne_solver(Freddy *rob, std::string root_link, std::string tip_link,
                double *root_acceleration, double **ext_wrench, double *constraint_tau)
{
  // create a chain from the root_link to the tip_link
  KDL::Chain chain;
  if (!rob->tree.getChain(root_link, tip_link, chain))
  {
    std::cerr << "Failed to get chain from KDL tree" << std::endl;
  }

  // get the corresponding robot state
  ManipulatorState *rob_state = nullptr;

  rob_state = root_link == rob->kinova_left->base_frame ? rob->kinova_left->state
                                                        : rob->kinova_right->state;

  if (rob_state == nullptr)
  {
    std::cerr << "Failed to find the robot state" << std::endl;
    exit(1);
  }

  // root acceleration
  KDL::Twist root_acc(
      KDL::Vector(root_acceleration[0], root_acceleration[1], root_acceleration[2]),
      KDL::Vector(root_acceleration[3], root_acceleration[4], root_acceleration[5]));

  KDL::ChainIdSolver_RNE rne(chain, root_acc.vel);

  // q, qd, qdd
  KDL::JntArray q = KDL::JntArray(rob_state->nj);
  KDL::JntArray qd = KDL::JntArray(rob_state->nj);

  for (size_t i = 0; i < rob_state->nj; i++)
  {
    q(i) = rob_state->q[i];
    qd(i) = rob_state->q_dot[i];
  }

  // ext wrench
  KDL::Wrenches f_ext;

  for (int i = 0; i < chain.getNrOfSegments(); i++)
  {
    KDL::Wrench wrench;
    for (int j = 0; j < 6; j++)
    {
      wrench(j) = ext_wrench[i][j];
    }
    f_ext.push_back(wrench);
  }

  // predicted accelerations
  KDL::JntArray qdd = KDL::JntArray(rob_state->nj);

  // constraint torques
  KDL::JntArray constraint_tau_jnt = KDL::JntArray(rob_state->nj);

  int r = rne.CartToJnt(q, qd, qdd, f_ext, constraint_tau_jnt);

  if (r < 0)
  {
    std::cerr << "Failed to solve the hybrid dynamics problem" << std::endl;
    std::cerr << "Error code: " << r << std::endl;
  }

  for (size_t i = 0; i < rob_state->nj; i++)
  {
    constraint_tau[i] = constraint_tau_jnt(i);
  }
}

void rne_solver_manipulator(Manipulator<kinova_mediator> *rob, double *root_acceleration,
                            double **ext_wrench, double *constraint_tau)
{
  // create a chain from the root_link to the tip_link
  KDL::Chain chain = rob->chain;

  // get the corresponding robot state
  ManipulatorState *rob_state = rob->state;

  // root acceleration
  KDL::Twist root_acc(
      KDL::Vector(root_acceleration[0], root_acceleration[1], root_acceleration[2]),
      KDL::Vector(root_acceleration[3], root_acceleration[4], root_acceleration[5]));

  KDL::ChainIdSolver_RNE rne(chain, root_acc.vel);

  // q, qd, qdd
  KDL::JntArray q = KDL::JntArray(rob_state->nj);
  KDL::JntArray qd = KDL::JntArray(rob_state->nj);

  for (size_t i = 0; i < rob_state->nj; i++)
  {
    q(i) = rob_state->q[i];
    qd(i) = rob_state->q_dot[i];
  }

  // ext wrench
  KDL::Wrenches f_ext;

  for (int i = 0; i < chain.getNrOfSegments(); i++)
  {
    KDL::Wrench wrench;
    for (int j = 0; j < 6; j++)
    {
      wrench(j) = ext_wrench[i][j];
    }
    f_ext.push_back(wrench);
  }

  // predicted accelerations
  KDL::JntArray qdd = KDL::JntArray(rob_state->nj);

  // constraint torques
  KDL::JntArray constraint_tau_jnt = KDL::JntArray(rob_state->nj);

  int r = rne.CartToJnt(q, qd, qdd, f_ext, constraint_tau_jnt);

  if (r < 0)
  {
    std::cerr << "Failed to solve the hybrid dynamics problem" << std::endl;
    std::cerr << "Error code: " << r << std::endl;
  }

  for (size_t i = 0; i < rob_state->nj; i++)
  {
    constraint_tau[i] = constraint_tau_jnt(i);
  }
}

void achd_solver_fext(Freddy *rob, std::string root_link, std::string tip_link,
                      double **ext_wrenches, double *constraint_tau)
{
  // create a chain from the root_link to the tip_link
  KDL::Chain *chain = new KDL::Chain();
  if (!rob->tree.getChain(root_link, tip_link, *chain))
  {
    std::cerr << "[achd fext] Failed to get chain from KDL tree" << std::endl;
  }

  // get the corresponding robot state
  ManipulatorState *rob_state = nullptr;

  rob_state = root_link == rob->kinova_left->base_frame ? rob->kinova_left->state
                                                        : rob->kinova_right->state;

  // root acceleration
  KDL::Twist root_acc(KDL::Vector(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.0, 0.0));

  int num_constraints = 6;

  KDL::ChainHdSolver_Vereshchagin_Fext vereshchagin_solver_fext(*chain, root_acc, num_constraints);

  // alpha - constraint forces
  KDL::Jacobian alpha_jac = KDL::Jacobian(num_constraints);

  // beta - accel energy
  KDL::JntArray beta_jnt = KDL::JntArray(num_constraints);

  // q, qd, qdd
  KDL::JntArray q = KDL::JntArray(rob_state->nj);
  KDL::JntArray qd = KDL::JntArray(rob_state->nj);

  for (size_t i = 0; i < rob_state->nj; i++)
  {
    q(i) = rob_state->q[i];
    qd(i) = rob_state->q_dot[i];
  }

  // ext wrench
  KDL::Wrenches f_ext;
  for (int i = 0; i < chain->getNrOfSegments(); i++)
  {
    KDL::Wrench wrench;
    for (int j = 0; j < 6; j++)
    {
      wrench(j) = ext_wrenches[i][j];
    }
    f_ext.push_back(wrench);
  }

  // feedforward torques
  KDL::JntArray ff_tau_jnt = KDL::JntArray(rob_state->nj);

  // predicted accelerations
  KDL::JntArray qdd = KDL::JntArray(rob_state->nj);

  // constraint torques
  KDL::JntArray constraint_tau_jnt = KDL::JntArray(rob_state->nj);

  auto start_time = std::chrono::high_resolution_clock::now();
  int r = vereshchagin_solver_fext.CartToJnt(q, qd, qdd, alpha_jac, beta_jnt, f_ext, ff_tau_jnt,
                                             constraint_tau_jnt);
  auto end_time = std::chrono::high_resolution_clock::now();

  if (r < 0)
  {
    std::cerr << "[achd fext] Failed to solve the hybrid dynamics problem" << std::endl;
    std::cerr << "[achd fext] Error code: " << r << std::endl;
  }

  for (size_t i = 0; i < rob_state->nj; i++)
  {
    constraint_tau[i] = constraint_tau_jnt(i);
  }

  std::chrono::duration<double> elapsed_time = end_time - start_time;
  // std::cout << "[achd fext] Time taken: " << elapsed_time.count() << "s" << std::endl;
}

void achd_solver(Freddy *rob, std::string root_link, std::string tip_link, int num_constraints,
                 double *root_acceleration, double **alpha, double *beta, double *tau_ff,
                 double *predicted_acc, double *constraint_tau)
{
  // create a chain from the root_link to the tip_link
  KDL::Chain chain;
  if (!rob->tree.getChain(root_link, tip_link, chain))
  {
    std::cerr << "[achd] Failed to get chain from KDL tree" << std::endl;
  }

  // joint inertias:
  const std::vector<double> joint_inertia{0.5580, 0.5580, 0.5580, 0.5580, 0.1389, 0.1389, 0.1389};

  // set joint inertias
  for (size_t i = 0; i < chain.getNrOfJoints(); i++)
  {
    chain.getSegment(i).setJoint().setJointInertia(joint_inertia[i]);
  }

  // get the corresponding robot state
  ManipulatorState *rob_state = nullptr;

  rob_state = root_link == rob->kinova_left->base_frame ? rob->kinova_left->state
                                                        : rob->kinova_right->state;

  if (rob_state == nullptr)
  {
    std::cerr << "Failed to find the robot state" << std::endl;
    exit(1);
  }

  // root acceleration
  KDL::Twist root_acc(
      KDL::Vector(-root_acceleration[0], -root_acceleration[1], -root_acceleration[2]),
      KDL::Vector(-root_acceleration[3], -root_acceleration[4], -root_acceleration[5]));

  KDL::ChainHdSolver_Vereshchagin vereshchagin_solver(chain, root_acc, num_constraints);

  // alpha - constraint forces
  KDL::Jacobian alpha_jac = KDL::Jacobian(num_constraints);

  for (int i = 0; i < num_constraints; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      alpha_jac(j, i) = alpha[i][j];
    }
  }

  // beta - accel energy
  KDL::JntArray beta_jnt = KDL::JntArray(num_constraints);

  for (int i = 0; i < num_constraints; i++)
  {
    beta_jnt(i) = beta[i];
  }

  // q, qd, qdd
  KDL::JntArray q(rob_state->nj);
  KDL::JntArray qd(rob_state->nj);

  for (size_t i = 0; i < rob_state->nj; i++)
  {
    q(i) = rob_state->q[i];
    qd(i) = rob_state->q_dot[i];
  }

  // ext wrench
  KDL::Wrenches f_ext;

  for (int i = 0; i < chain.getNrOfSegments(); i++)
  {
    KDL::Wrench wrench;
    f_ext.push_back(wrench);
  }

  // feedforward torques
  KDL::JntArray ff_tau_jnt = KDL::JntArray(rob_state->nj);

  for (size_t i = 0; i < rob_state->nj; i++)
  {
    ff_tau_jnt(i) = tau_ff[i];
  }

  // predicted accelerations
  KDL::JntArray qdd = KDL::JntArray(rob_state->nj);

  // constraint torques
  KDL::JntArray constraint_tau_jnt = KDL::JntArray(rob_state->nj);

  // print all the inputs
  // std::cout << "\nachd inputs: " << std::endl;
  // std::cout << "root acceleration: " << root_acc << std::endl;
  // std::cout << "alpha: \n" << alpha_jac << std::endl;
  // std::cout << "beta: " << beta_jnt << std::endl;
  // std::cout << "q: " << q << std::endl;
  // std::cout << "qd: " << qd << std::endl;
  // std::cout << "ff_tau: " << ff_tau_jnt << std::endl;

  int r = vereshchagin_solver.CartToJnt(q, qd, qdd, alpha_jac, beta_jnt, f_ext, ff_tau_jnt,
                                        constraint_tau_jnt);

  if (r < 0)
  {
    std::cerr << "[achd] Failed to solve the hybrid dynamics problem" << std::endl;
    std::cerr << "[achd] Error code: " << r << std::endl;
  }

  // vs_solver.get_constraint_torque(constraint_tau_jnt);

  // std::vector<KDL::Twist> twists(7);
  // vereshchagin_solver.getLinkCartesianAcceleration(twists);

  // KDL::Twist tool_twist = twists[6];

  // std::vector<KDL::Twist> twists(8);
  // vs_solver.get_transformed_link_acceleration(twists);

  // std::cout << "-- tool acc b: " << tool_twist << std::endl;

  // KDL::Chain chain_to_base;
  // if (!rob->tree.getChain(root_link, "base_link", chain_to_base))
  // {
  //   std::cerr << "Failed to get chain from KDL tree" << std::endl;
  // }

  // KDL::JntArray q1(0);

  // // fk solver
  // KDL::ChainFkSolverPos_recursive fk_solver(chain_to_base);
  // KDL::Frame frame;
  // fk_solver.JntToCart(q1, frame);

  // // transform the twist to the base_link frame
  // KDL::Twist tip_twist = frame.M.Inverse() * tool_twist;

  // std::cout << "-- tool acc: " << tip_twist << std::endl << std::endl;

  for (size_t i = 0; i < rob_state->nj; i++)
  {
    predicted_acc[i] = qdd(i);
    constraint_tau[i] = constraint_tau_jnt(i);
  }
}

void achd_solver_manipulator(Manipulator<kinova_mediator> *rob, int num_constraints,
                             double *root_acceleration, double **alpha, double *beta,
                             double *tau_ff, double *predicted_acc, double *constraint_tau)
{
  KDL::Chain chain = rob->chain;
  ManipulatorState *rob_state = rob->state;

  // root acceleration
  KDL::Twist root_acc(
      KDL::Vector(-root_acceleration[0], -root_acceleration[1], -root_acceleration[2]),
      KDL::Vector(-root_acceleration[3], -root_acceleration[4], -root_acceleration[5]));

  KDL::ChainHdSolver_Vereshchagin vereshchagin_solver(chain, root_acc, num_constraints);

  // alpha - constraint forces
  KDL::Jacobian alpha_jac = KDL::Jacobian(num_constraints);

  for (int i = 0; i < num_constraints; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      alpha_jac(j, i) = alpha[i][j];
    }
  }

  // beta - accel energy
  KDL::JntArray beta_jnt = KDL::JntArray(num_constraints);

  for (int i = 0; i < num_constraints; i++)
  {
    beta_jnt(i) = beta[i];
  }

  // q, qd, qdd
  KDL::JntArray q(rob_state->nj);
  KDL::JntArray qd(rob_state->nj);

  for (size_t i = 0; i < rob_state->nj; i++)
  {
    q(i) = rob_state->q[i];
    qd(i) = rob_state->q_dot[i];
  }

  // ext wrench
  KDL::Wrenches f_ext;

  for (int i = 0; i < chain.getNrOfSegments(); i++)
  {
    KDL::Wrench wrench;
    f_ext.push_back(wrench);
  }

  // feedforward torques
  KDL::JntArray ff_tau_jnt = KDL::JntArray(rob_state->nj);

  for (size_t i = 0; i < rob_state->nj; i++)
  {
    ff_tau_jnt(i) = tau_ff[i];
  }

  // predicted accelerations
  KDL::JntArray qdd = KDL::JntArray(rob_state->nj);

  // constraint torques
  KDL::JntArray constraint_tau_jnt = KDL::JntArray(rob_state->nj);

  // print all the inputs
  std::cout << "achd inputs: " << std::endl;
  std::cout << "root acceleration: " << root_acc << std::endl;
  std::cout << "alpha: \n" << alpha_jac << std::endl;
  std::cout << "beta: " << beta_jnt << std::endl;
  std::cout << "q: " << q << std::endl;
  std::cout << "qd: " << qd << std::endl;
  std::cout << "ff_tau: " << ff_tau_jnt << std::endl;

  int r = vereshchagin_solver.CartToJnt(q, qd, qdd, alpha_jac, beta_jnt, f_ext, ff_tau_jnt,
                                        constraint_tau_jnt);

  std::vector<KDL::Twist> twists(7);
  vereshchagin_solver.getLinkCartesianAcceleration(twists);

  std::string robot_urdf = "/home/batsy/rc/src/motion_spec_gen/urdf/freddy.urdf";

  KDL::Tree tree1;

  // load the robot urdf
  if (!kdl_parser::treeFromFile(robot_urdf, tree1))
  {
    std::cerr << "Failed to construct KDL tree" << std::endl;
    exit(1);
  }

  KDL::Chain chain1;
  if (!tree1.getChain("base_link", rob->base_frame, chain1))
  {
    std::cerr << "Failed to get chain from KDL tree" << std::endl;
  }

  KDL::JntArray q1(0);

  // fk solver
  KDL::ChainFkSolverPos_recursive fk_solver(chain1);
  KDL::Frame frame;
  fk_solver.JntToCart(q1, frame);

  // transform the twist to the base_link frame
  KDL::Twist tip_twist = twists[6];
  tip_twist = frame.M.Inverse() * tip_twist;

  // // print
  // std::cout << "-- tool pose: " << std::endl;
  // std::cout << frames[6].p.x() << " " << frames[6].p.y() << " " << frames[6].p.z() <<
  // std::endl;

  std::cout << "-- tool acceleration: " << tip_twist << std::endl;

  if (r < 0)
  {
    std::cerr << "[achd] Failed to solve the hybrid dynamics problem" << std::endl;
    std::cerr << "[achd] Error code: " << r << std::endl;
  }

  for (size_t i = 0; i < rob_state->nj; i++)
  {
    predicted_acc[i] = qdd(i);
    constraint_tau[i] = constraint_tau_jnt(i);
  }
}

void base_fd_solver(Freddy *rob, double *platform_forces, double *wheel_torques)
{
  const unsigned int N = 3;
  const unsigned int M = rob->mobile_base->mediator->kelo_base_config->nWheels * 2;

  TorqueControlState *torque_control_state = new TorqueControlState();
  init_torque_control_state(torque_control_state, N, M);

  set_platform_force(torque_control_state, platform_forces, N);

  set_weight_matrix(torque_control_state, N, M);

  compute_wheel_torques(rob->mobile_base->mediator->kelo_base_config, torque_control_state,
                        rob->mobile_base->state->pivot_angles, wheel_torques, N, M);

  free_torque_control_state(torque_control_state);
  delete torque_control_state;
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

void wrench_estimator(Freddy *rob, std::string root_link, std::string tip_link,
                      double *root_acceleration, double *tau, double *tool_wrench)
{
  // create a chain from the root_link to the tip_link
  KDL::Chain chain;
  if (!rob->tree.getChain(root_link, tip_link, chain))
  {
    std::cerr << "[achd] Failed to get chain from KDL tree" << std::endl;
  }

  // joint inertias:
  const std::vector<double> joint_inertia{0.5580, 0.5580, 0.5580, 0.5580, 0.1389, 0.1389, 0.1389};

  // set joint inertias
  for (size_t i = 0; i < chain.getNrOfJoints(); i++)
  {
    chain.getSegment(i).setJoint().setJointInertia(joint_inertia[i]);
  }

  // get the corresponding robot state
  ManipulatorState *rob_state = nullptr;

  rob_state = root_link == rob->kinova_left->base_frame ? rob->kinova_left->state
                                                        : rob->kinova_right->state;

  if (rob_state == nullptr)
  {
    std::cerr << "Failed to find the robot state" << std::endl;
    exit(1);
  }

  // root acceleration
  KDL::Twist root_acc(
      KDL::Vector(-root_acceleration[0], -root_acceleration[1], -root_acceleration[2]),
      KDL::Vector(root_acceleration[3], root_acceleration[4], root_acceleration[5]));

  KDL::ChainExternalWrenchEstimator ext_wrench_estimator(chain, root_acc.vel, 100.0, 30.0, 0.5);

  // q, qd, qdd
  KDL::JntArray q = KDL::JntArray(rob_state->nj);
  KDL::JntArray qd = KDL::JntArray(rob_state->nj);

  for (size_t i = 0; i < rob_state->nj; i++)
  {
    q(i) = rob_state->q[i];
    qd(i) = rob_state->q_dot[i];
  }

  KDL::JntArray jnt_tau = KDL::JntArray(rob_state->nj);

  for (size_t i = 0; i < rob_state->nj; i++)
  {
    jnt_tau(i) = tau[i];
  }

  // ext wrench
  KDL::Wrench f_ext;

  ext_wrench_estimator.JntToExtWrench(q, qd, jnt_tau, f_ext);

  for (size_t i = 0; i < 6; i++)
  {
    tool_wrench[i] = f_ext(i);
  }
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

void getLinkPosition(std::string link_name, std::string as_seen_by, std::string with_respect_to,
                     double *vec, Freddy *rob, double &out_position)
{
  double pose[7]{};
  getLinkSFromRob(link_name, rob, pose);

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

  if (as_seen_by == "base_link")
  {
    for (size_t i = 0; i < 4; i++)
    {
      out_quaternion[i] = pose[i + 3];
    }
    return;
  }

  // Transformation to as_seen_by frame
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

void findLinkInChain(std::string link_name, KDL::Chain *chain, bool &is_in_chain, int &link_id)
{
  is_in_chain = false;
  for (int i = 0; i < chain->getNrOfSegments(); i++)
  {
    if (chain->getSegment(i).getName() == link_name)
    {
      is_in_chain = true;
      link_id = i;
      return;
    }
  }
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

void transform_wrench(Freddy *rob, std::string from_ent, std::string to_ent, double *wrench,
                      double *transformed_wrench)
{
  KDL::Chain chain;
  if (!rob->tree.getChain(from_ent, to_ent, chain))
  {
    std::cerr << "Failed to get chain from KDL tree" << std::endl;
    return;
  }

  bool forward = true;
  if (from_ent.find("base_link") == std::string::npos &&
      to_ent.find("base_link") != std::string::npos)
  {
    forward = false;
  }

  KDL::Frame frame;
  KDL::ChainFkSolverPos_recursive fk_solver_pos(chain);

  KDL::JntArray q = KDL::JntArray(chain.getNrOfJoints());

  // update q values from the robot state
  bool is_in_left_chain, is_in_right_chain = false;

  std::string not_base_link = from_ent == "base_link" ? to_ent : from_ent;
  int link_id = -1;
  findLinkInChain(not_base_link, &rob->kinova_left->chain, is_in_left_chain, link_id);
  findLinkInChain(not_base_link, &rob->kinova_right->chain, is_in_right_chain, link_id);

  if (!is_in_left_chain && !is_in_right_chain)
  {
    std::cerr << "Link not found in the robot chains" << std::endl;
    return;
  }

  ManipulatorState *rob_state =
      is_in_left_chain ? rob->kinova_left->state : rob->kinova_right->state;

  if (chain.getNrOfJoints() != 0)
  {
    if (forward)
    {
      for (size_t i = 0; i < rob_state->nj; i++)
      {
        q(i) = rob_state->q[i];
      }
    }
    else
    {
      for (size_t i = 0; i < rob_state->nj; i++)
      {
        q(i) = rob_state->q[rob_state->nj - i - 1];
      }
    }
  }

  fk_solver_pos.JntToCart(q, frame);

  KDL::Wrench wrench_kdl;
  for (size_t i = 0; i < 6; i++)
  {
    wrench_kdl(i) = wrench[i];
  }

  KDL::Wrench transformed_wrench_kdl = frame.M * wrench_kdl;

  for (size_t i = 0; i < 6; i++)
  {
    transformed_wrench[i] = transformed_wrench_kdl(i);
  }
}

void transform_alpha(Manipulator<kinova_mediator> *rob, KDL::Tree *tree, std::string source_frame,
                     std::string target_frame, double **alpha, int nc, double **transformed_alpha)
{
  KDL::Chain chain;
  if (!tree->getChain(source_frame, target_frame, chain))
  {
    std::cerr << "Failed to get chain from KDL tree" << std::endl;
    return;
  }

  KDL::Frame frame;
  KDL::ChainFkSolverPos_recursive fk_solver_pos(chain);

  KDL::JntArray q = KDL::JntArray(chain.getNrOfJoints());

  ManipulatorState *rob_state = rob->state;

  bool forward = true;
  if (source_frame != "base_link" && target_frame == "base_link")
  {
    forward = false;
  }

  if (chain.getNrOfJoints() != 0)
  {
    if (forward)
    {
      for (size_t i = 0; i < rob_state->nj; i++)
      {
        q(i) = rob_state->q[i];
      }
    }
    else
    {
      for (size_t i = 0; i < rob_state->nj; i++)
      {
        q(i) = rob_state->q[rob_state->nj - i - 1];
      }
    }
  }

  fk_solver_pos.JntToCart(q, frame);

  KDL::Jacobian alpha_jac(nc);
  for (size_t i = 0; i < nc; i++)
  {
    for (size_t j = 0; j < 6; j++)
    {
      alpha_jac(j, i) = alpha[i][j];
    }
  }

  for (size_t i = 0; i < nc; i++)
  {
    alpha_jac.setColumn(i, frame.M * alpha_jac.getColumn(i));
  }

  for (size_t i = 0; i < nc; i++)
  {
    for (size_t j = 0; j < 6; j++)
    {
      transformed_alpha[i][j] = alpha_jac(j, i);
    }
  }
}

void transform_alpha_beta(Manipulator<kinova_mediator> *rob, KDL::Tree *tree,
                          std::string source_frame, std::string target_frame, double **alpha,
                          double *beta, int nc, double **transformed_alpha,
                          double *transformed_beta)
{
  KDL::Chain chain;
  if (!tree->getChain(source_frame, target_frame, chain))
  {
    std::cerr << "Failed to get chain from KDL tree" << std::endl;
    return;
  }

  KDL::Frame frame;
  KDL::ChainFkSolverPos_recursive fk_solver_pos(chain);

  KDL::JntArray q = KDL::JntArray(chain.getNrOfJoints());

  ManipulatorState *rob_state = rob->state;

  bool forward = true;
  if (source_frame != "base_link" && target_frame == "base_link")
  {
    forward = false;
  }

  if (chain.getNrOfJoints() != 0)
  {
    if (forward)
    {
      for (size_t i = 0; i < rob_state->nj; i++)
      {
        q(i) = rob_state->q[i];
      }
    }
    else
    {
      for (size_t i = 0; i < rob_state->nj; i++)
      {
        q(i) = rob_state->q[rob_state->nj - i - 1];
      }
    }
  }

  fk_solver_pos.JntToCart(q, frame);

  KDL::Jacobian alpha_jac(nc);
  for (size_t i = 0; i < nc; i++)
  {
    for (size_t j = 0; j < 6; j++)
    {
      alpha_jac(j, i) = alpha[i][j];
    }
  }

  for (size_t i = 0; i < nc; i++)
  {
    alpha_jac.setColumn(i, frame.M * alpha_jac.getColumn(i));
  }

  for (size_t i = 0; i < nc; i++)
  {
    for (size_t j = 0; j < 6; j++)
    {
      transformed_alpha[i][j] = alpha_jac(j, i);
    }
  }

  KDL::Twist beta_acc_twist;
  for (size_t i = 0; i < 6; i++)
  {
    beta_acc_twist(i) = beta[i];
  }

  KDL::Twist transformed_beta_acc_twist = frame.M * beta_acc_twist;

  for (size_t i = 0; i < 6; i++)
  {
    transformed_beta[i] = transformed_beta_acc_twist(i);
  }
}

void transform_alpha(Freddy *rob, std::string source_frame, std::string target_frame,
                     double **alpha, int nc, double **transformed_alpha)
{
  KDL::Chain chain;
  if (!rob->tree.getChain(source_frame, target_frame, chain))
  {
    std::cerr << "Failed to get chain from KDL tree" << std::endl;
    return;
  }

  KDL::Frame frame;
  KDL::ChainFkSolverPos_recursive fk_solver_pos(chain);

  KDL::JntArray q = KDL::JntArray(chain.getNrOfJoints());

  // update q values from the robot state
  bool is_in_left_chain, is_in_right_chain = false;

  std::string not_base_link = source_frame == "base_link" ? target_frame : source_frame;
  int link_id = -1;
  findLinkInChain(not_base_link, &rob->kinova_left->chain, is_in_left_chain, link_id);
  findLinkInChain(not_base_link, &rob->kinova_right->chain, is_in_right_chain, link_id);

  if (!is_in_left_chain && !is_in_right_chain)
  {
    std::cerr << "Link not found in the robot chains" << std::endl;
    return;
  }

  ManipulatorState *rob_state =
      is_in_left_chain ? rob->kinova_left->state : rob->kinova_right->state;

  bool forward = true;
  if (source_frame != "base_link" && target_frame == "base_link")
  {
    forward = false;
  }

  if (chain.getNrOfJoints() != 0)
  {
    if (forward)
    {
      for (size_t i = 0; i < rob_state->nj; i++)
      {
        q(i) = rob_state->q[i];
      }
    }
    else
    {
      for (size_t i = 0; i < rob_state->nj; i++)
      {
        q(i) = rob_state->q[rob_state->nj - i - 1];
      }
    }
  }

  fk_solver_pos.JntToCart(q, frame);

  KDL::Jacobian alpha_jac(nc);
  for (size_t i = 0; i < nc; i++)
  {
    for (size_t j = 0; j < 6; j++)
    {
      alpha_jac(j, i) = alpha[i][j];
    }
  }

  for (size_t i = 0; i < nc; i++)
  {
    alpha_jac.setColumn(i, frame.M * alpha_jac.getColumn(i));
  }

  for (size_t i = 0; i < nc; i++)
  {
    for (size_t j = 0; j < 6; j++)
    {
      transformed_alpha[i][j] = alpha_jac(j, i);
    }
  }
}

void transform_alpha_beta(Freddy *rob, std::string source_frame, std::string target_frame,
                          double **alpha, double *beta, int nc, double **transformed_alpha,
                          double *transformed_beta)
{
  KDL::Chain chain;
  if (!rob->tree.getChain(source_frame, target_frame, chain))
  {
    std::cerr << "Failed to get chain from KDL tree" << std::endl;
    return;
  }

  KDL::Frame frame;
  KDL::ChainFkSolverPos_recursive fk_solver_pos(chain);

  KDL::JntArray q = KDL::JntArray(chain.getNrOfJoints());

  // update q values from the robot state
  bool is_in_left_chain, is_in_right_chain = false;

  std::string not_base_link = source_frame == "base_link" ? target_frame : source_frame;
  int link_id = -1;
  findLinkInChain(not_base_link, &rob->kinova_left->chain, is_in_left_chain, link_id);
  findLinkInChain(not_base_link, &rob->kinova_right->chain, is_in_right_chain, link_id);

  if (!is_in_left_chain && !is_in_right_chain)
  {
    std::cerr << "Link not found in the robot chains" << std::endl;
    return;
  }

  ManipulatorState *rob_state =
      is_in_left_chain ? rob->kinova_left->state : rob->kinova_right->state;

  bool forward = true;
  if (source_frame != "base_link" && target_frame == "base_link")
  {
    forward = false;
  }

  if (chain.getNrOfJoints() != 0)
  {
    if (forward)
    {
      for (size_t i = 0; i < rob_state->nj; i++)
      {
        q(i) = rob_state->q[i];
      }
    }
    else
    {
      for (size_t i = 0; i < rob_state->nj; i++)
      {
        q(i) = rob_state->q[rob_state->nj - i - 1];
      }
    }
  }

  fk_solver_pos.JntToCart(q, frame);

  KDL::Jacobian alpha_jac(nc);
  for (size_t i = 0; i < nc; i++)
  {
    for (size_t j = 0; j < 6; j++)
    {
      alpha_jac(j, i) = alpha[i][j];
    }
  }

  for (size_t i = 0; i < nc; i++)
  {
    alpha_jac.setColumn(i, frame.M * alpha_jac.getColumn(i));
  }

  for (size_t i = 0; i < nc; i++)
  {
    for (size_t j = 0; j < 6; j++)
    {
      transformed_alpha[i][j] = alpha_jac(j, i);
    }
  }

  KDL::Twist beta_acc_twist;
  for (size_t i = 0; i < 6; i++)
  {
    beta_acc_twist(i) = beta[i];
  }

  KDL::Twist transformed_beta_acc_twist = frame.M * beta_acc_twist;

  for (size_t i = 0; i < 6; i++)
  {
    transformed_beta[i] = transformed_beta_acc_twist(i);
  }
}

void transformS(Freddy *rob, std::string source_frame, std::string target_frame, double *s,
                double *s_out)
{
  // construct KDL chain from from_ent to to_ent
  KDL::Chain chain;
  if (!rob->tree.getChain(source_frame, target_frame, chain))
  {
    std::cerr << "Failed to get chain from KDL tree" << std::endl;
    exit(1);
  }

  KDL::Frame frame;
  KDL::ChainFkSolverPos_recursive fk_solver_pos(chain);

  KDL::JntArray q = KDL::JntArray(chain.getNrOfJoints());

  // update q values from the robot state
  bool is_in_left_chain, is_in_right_chain = false;

  std::string not_base_link = source_frame == "base_link" ? target_frame : source_frame;
  int link_id = -1;
  findLinkInChain(not_base_link, &rob->kinova_left->chain, is_in_left_chain, link_id);

  if (!is_in_left_chain)
  {
    findLinkInChain(not_base_link, &rob->kinova_right->chain, is_in_right_chain, link_id);
  }

  if (!is_in_left_chain && !is_in_right_chain)
  {
    std::cerr << "[transformS] Link not found in the robot chains" << std::endl;
    exit(1);
  }

  ManipulatorState *rob_state =
      is_in_left_chain ? rob->kinova_left->state : rob->kinova_right->state;

  bool forward = true;
  if (source_frame != "base_link" && target_frame == "base_link")
  {
    forward = false;
  }

  if (chain.getNrOfJoints() != 0)
  {
    if (forward)
    {
      for (size_t i = 0; i < rob_state->nj; i++)
      {
        q(i) = rob_state->q[i];
      }
    }
    else
    {
      for (size_t i = 0; i < rob_state->nj; i++)
      {
        q(i) = rob_state->q[rob_state->nj - i - 1];
      }
    }
  }

  fk_solver_pos.JntToCart(q, frame);

  // construct the source frame from the s values
  KDL::Frame source_frame_kdl;
  source_frame_kdl.p = KDL::Vector(s[0], s[1], s[2]);
  source_frame_kdl.M = KDL::Rotation::Quaternion(s[3], s[4], s[5], s[6]);

  // transform the source frame to the target frame
  KDL::Frame target_frame_kdl = frame.Inverse() * source_frame_kdl;

  // convert the target frame to s values
  s_out[0] = target_frame_kdl.p.x();
  s_out[1] = target_frame_kdl.p.y();
  s_out[2] = target_frame_kdl.p.z();
  target_frame_kdl.M.GetQuaternion(s_out[3], s_out[4], s_out[5], s_out[6]);
}

void transformSdot(Freddy *rob, std::string source_frame, std::string target_frame, double *s_dot,
                   double *s_dot_out)
{
  // construct KDL chain from from_ent to to_ent
  KDL::Chain chain;
  if (!rob->tree.getChain(source_frame, target_frame, chain))
  {
    std::cerr << "Failed to get chain from KDL tree" << std::endl;
    exit(1);
  }

  KDL::Frame frame;
  KDL::ChainFkSolverPos_recursive fk_solver_pos(chain);

  KDL::JntArray q = KDL::JntArray(chain.getNrOfJoints());

  // update q values from the robot state
  bool is_in_left_chain, is_in_right_chain = false;

  std::string not_base_link = source_frame == "base_link" ? target_frame : source_frame;
  int link_id = -1;
  findLinkInChain(not_base_link, &rob->kinova_left->chain, is_in_left_chain, link_id);

  if (!is_in_left_chain)
  {
    findLinkInChain(not_base_link, &rob->kinova_right->chain, is_in_right_chain, link_id);
  }

  if (!is_in_left_chain && !is_in_right_chain)
  {
    std::cerr << "[transformS] Link not found in the robot chains" << std::endl;
    exit(1);
  }

  ManipulatorState *rob_state =
      is_in_left_chain ? rob->kinova_left->state : rob->kinova_right->state;

  bool forward = true;
  if (source_frame != "base_link" && target_frame == "base_link")
  {
    forward = false;
  }

  if (chain.getNrOfJoints() != 0)
  {
    if (forward)
    {
      for (size_t i = 0; i < rob_state->nj; i++)
      {
        q(i) = rob_state->q[i];
      }
    }
    else
    {
      for (size_t i = 0; i < rob_state->nj; i++)
      {
        q(i) = rob_state->q[rob_state->nj - i - 1];
      }
    }
  }

  fk_solver_pos.JntToCart(q, frame);

  // construct the twist from the s_dot values
  KDL::Twist source_twist;
  source_twist.vel = KDL::Vector(s_dot[0], s_dot[1], s_dot[2]);
  source_twist.rot = KDL::Vector(s_dot[3], s_dot[4], s_dot[5]);

  // transform the twist to the target frame
  KDL::Twist target_twist = frame.M.Inverse() * source_twist;

  // convert the target twist to s_dot values
  for (size_t i = 0; i < 6; i++)
  {
    s_dot_out[i] = target_twist(i);
  }
}

void transform_with_frame(double *source_frame, double *transform, double *transformed_frame)
{
  KDL::Frame source_frame_kdl;
  // TODO: maybe not the best way to handle this
  if (source_frame[6] == 0.0)
  {
    source_frame[6] = 1.0;
  }
  source_frame_kdl.p = KDL::Vector(source_frame[0], source_frame[1], source_frame[2]);
  source_frame_kdl.M = KDL::Rotation::Quaternion(source_frame[3], source_frame[4], source_frame[5],
                                                 source_frame[6]);

  KDL::Frame transform_kdl;
  transform_kdl.p = KDL::Vector(transform[0], transform[1], transform[2]);
  transform_kdl.M =
      KDL::Rotation::Quaternion(transform[3], transform[4], transform[5], transform[6]);

  KDL::Frame transformed_frame_kdl = transform_kdl * source_frame_kdl;

  transformed_frame[0] = transformed_frame_kdl.p.x();
  transformed_frame[1] = transformed_frame_kdl.p.y();
  transformed_frame[2] = transformed_frame_kdl.p.z();
  transformed_frame_kdl.M.GetQuaternion(transformed_frame[3], transformed_frame[4],
                                        transformed_frame[5], transformed_frame[6]);
}

void get_robot_data(Freddy *freddy, double dt)
{
  // get_manipulator_data(freddy->kinova_left);
  // update_manipulator_state(freddy->kinova_left->state, freddy->kinova_left->tool_frame,
  //                          &freddy->tree);

  get_manipulator_data(freddy->kinova_right);
  update_manipulator_state(freddy->kinova_right->state, freddy->kinova_right->tool_frame,
                           &freddy->tree);

  send_and_receive_data(freddy->mobile_base->mediator->ethercat_config);
  get_kelo_base_state(
      freddy->mobile_base->mediator->kelo_base_config,
      freddy->mobile_base->mediator->ethercat_config, freddy->mobile_base->state->pivot_angles,
      freddy->mobile_base->state->wheel_encoder_values, freddy->mobile_base->state->qd_wheel);

  compute_kelo_platform_velocity(freddy);
  compute_kelo_platform_pose(freddy->mobile_base->state->xd_platform, dt,
                             freddy->mobile_base->state->x_platform);
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
  x_platform[2] = norm(x_platform[2] + xd_platform[2] * dt);
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