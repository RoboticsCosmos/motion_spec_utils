/**
 * Author: Vamsi Kalagaturu
 *
 * Description: Library to perform solver operations for the arm_actions package
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
#include "kelo_motion_control/mediator.h"
#include "motion_spec_utils/solver_utils.hpp"

void rne_solver(Freddy *rob, std::string root_link, std::string tip_link,
                double *root_acceleration, double ext_wrenches[][6], double *constraint_tau)
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
      wrench(j) = ext_wrenches[i][j];
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
                      double ext_wrenches[][6], double *constraint_tau)
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

  delete chain;
}

void achd_solver(Freddy *rob, std::string root_link, std::string tip_link, int num_constraints,
                 double *root_acceleration, double alpha[][6], double *beta, double *tau_ff,
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

void base_fd_solver(Freddy *rob, double *platform_forces, double *wheel_torques)
{
  const unsigned int N = 3;
  const unsigned int M = rob->mobile_base->mediator->kelo_base_config->nWheels * 2;

  TorqueControlState *torque_control_state = new TorqueControlState();
  init_torque_control_state(torque_control_state, N, M);
  set_weight_matrix(torque_control_state, N, M);

  // *Assumption* - platform_forces is a 3D vector with forces in x, y, and moment about z
  set_platform_force(torque_control_state, platform_forces, N);

  compute_wheel_torques(rob->mobile_base->mediator->kelo_base_config, torque_control_state,
                        rob->mobile_base->state->pivot_angles, wheel_torques, N, M);

  free_torque_control_state(torque_control_state);
  delete torque_control_state;
}

void get_pivot_alignment_offsets(Freddy *robot, double *platform_force, double *lin_offsets,
                                 double *ang_offsets)
{
  Eigen::Rotation2Dd rot_ccw(M_PI / 2);
  Eigen::Rotation2Dd rot_cw(-M_PI / 2);

  Eigen::Vector2d lin_platform_force;
  lin_platform_force << platform_force[0], platform_force[1];
  lin_platform_force.normalize();

  // compute the direction vectors of the pivot links
  for (size_t i = 0; i < robot->mobile_base->mediator->kelo_base_config->nWheels; i++)
  {
    double pd_x = cos(robot->mobile_base->state->pivot_angles[i]);
    double pd_y = sin(robot->mobile_base->state->pivot_angles[i]);

    Eigen::Vector2d pivot_dir;
    pivot_dir << pd_x, pd_y;

    Eigen::Vector2d attachment;
    attachment << robot->mobile_base->mediator->kelo_base_config->wheel_coordinates[2 * i],
        robot->mobile_base->mediator->kelo_base_config->wheel_coordinates[2 * i + 1];

    // compute tangent vector of the attachment vector in cw or ccw based on platform force[2] -
    // moment if the moment is positive, the tangents are in ccw, otherwise in cw
    Eigen::Vector2d tangent = platform_force[2] > 0 ? rot_ccw * attachment : rot_cw * attachment;

    // get the angular offsets between the pivot direction and the tangent
    ang_offsets[i] = atan2(pivot_dir.x() * tangent.y() - pivot_dir.y() * tangent.x(),
                           pivot_dir.x() * tangent.x() + pivot_dir.y() * tangent.y());

    // get the linear offsets between the pivot direction and platform linear force
    lin_offsets[i] =
        atan2(pivot_dir.x() * lin_platform_force.y() - pivot_dir.y() * lin_platform_force.x(),
              pivot_dir.x() * lin_platform_force.x() + pivot_dir.y() * lin_platform_force.y());
  }
}

void base_fd_solver_with_alignment(Freddy *robot, double *platform_force, double *linear_offsets,
                                   double *angular_offsets, double *wheel_torques)
{
  assert(platform_force);
  assert(wheel_torques);

  size_t nWheels = robot->mobile_base->mediator->kelo_base_config->nWheels;
  assert(nWheels > 0);

  // transform the platform force by 90 degrees ccw
  Eigen::Rotation2Dd pf_correction_rot(M_PI / 2);
  Eigen::Vector2d lin_pf = Eigen::Vector2d(platform_force[0], platform_force[1]);
  lin_pf = pf_correction_rot * lin_pf;

  double pf[3] = {lin_pf.x(), lin_pf.y(), platform_force[2]};

  // compute the weights for the platform force
  double platform_weights[2];
  platform_weights[0] = abs(platform_force[2]) < 1e-6
                            ? 1.0
                            : sqrt(pow(platform_force[0], 2) + pow(platform_force[1], 2)) /
                                  (sqrt(pow(platform_force[0], 2) + pow(platform_force[1], 2) +
                                        pow(platform_force[2], 2)));

  platform_weights[1] = 1.0 - platform_weights[0];

  double lin_force_weight = lin_pf.norm() == 0.0 ? 0.0 : platform_weights[0];
  double moment_weight = pf[2] == 0.0 ? 0.0 : platform_weights[1];

  double alignment_taus[nWheels]{};
  for (size_t i = 0; i < nWheels; i++)
  {
    alignment_taus[i] = linear_offsets[i] * lin_force_weight + angular_offsets[i] * moment_weight;
  }

  double tau_wheel_ref[nWheels * 2]{};
  for (size_t i = 0; i < nWheels; i++)
  {
    tau_wheel_ref[2 * i] = alignment_taus[i];
    tau_wheel_ref[2 * i + 1] = -alignment_taus[i];
  }

  double tau_wheel_ref_limit = 10.0;

  for (size_t i = 0; i < nWheels * 2; i++)
  {
    if (tau_wheel_ref[i] > tau_wheel_ref_limit)
    {
      tau_wheel_ref[i] = tau_wheel_ref_limit;
    }
    else if (tau_wheel_ref[i] < -tau_wheel_ref_limit)
    {
      tau_wheel_ref[i] = -tau_wheel_ref_limit;
    }
  }

  // solver
  // initialize variables
  double caster_offsets[nWheels]{};
  for (size_t i = 0; i < nWheels; i++)
  {
    caster_offsets[i] = robot->mobile_base->mediator->kelo_base_config->castor_offset;
  }

  double wheel_distances[nWheels]{};
  for (size_t i = 0; i < nWheels; i++)
  {
    wheel_distances[i] = robot->mobile_base->mediator->kelo_base_config->half_wheel_distance * 2;
  }

  double wheel_diameters[8]{};
  for (size_t i = 0; i < nWheels * 2; i++)
  {
    wheel_diameters[i] = robot->mobile_base->mediator->kelo_base_config->radius * 2;
  }

  double w_drive[nWheels * nWheels] = {
      // [1/N^2]
      1.0, 0.0, 0.0, 1.0,  // fl-xx, fl-xy, fl-yx, fl-yy
      1.0, 0.0, 0.0, 1.0,  // rl-xx, rl-xy, rl-yx, rl-yy
      1.0, 0.0, 0.0, 1.0,  // rr-xx, rr-xy, rr-yx, rr-yy
      1.0, 0.0, 0.0, 1.0   // fr-xx, fr-xy, fr-yx, fr-yy
  };

  double force_dist_mat_whl[3 * 2 * nWheels];
  kelo_pltf_frc_comp_mat_whl(nWheels,
                             robot->mobile_base->mediator->kelo_base_config->wheel_coordinates,
                             caster_offsets, wheel_distances, wheel_diameters,
                             robot->mobile_base->state->pivot_angles, force_dist_mat_whl);

  // printf("\n");
  // printf("[");
  // for (int m_ = 0; m_ < 3; m_++) {
  //     printf("[");
  //     for (int n_ = 0; n_ < 8; n_++) {
  //         printf("%10f", force_dist_mat_whl[m_ + n_ * 3]);
  //         if (n_ != 8 - 1) printf(", ");
  //     }
  //     printf("]");
  //     if (m_ != 3 - 1) printf(",\n ");
  // }
  // printf("]");
  // printf("\n");

  kelo_pltf_slv_inv_frc_dist_cgls(nWheels, force_dist_mat_whl, w_drive, pf, tau_wheel_ref,
                                  wheel_torques);

  // double f_platform_out[3];

  // kelo_pltf_slv_fwd_frc_comp(4,
  //           force_dist_mat_whl,
  //           wheel_torques,
  //           f_platform_out);
}