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

#include "kdl_utils/solver_utils_old.hpp"

SolverUtils::SolverUtils() {}

SolverUtils::~SolverUtils() {}

void SolverUtils::populateAlphaUnitForces(
    const std::vector<std::vector<double>> &alpha_cols,
    KDL::Jacobian *alpha_unit_forces) {
  for (int i = 0; i < alpha_cols.size(); i++) {
    KDL::Vector unit_force_l(alpha_cols[i][0], alpha_cols[i][1],
                             alpha_cols[i][2]);
    KDL::Vector unit_force_a(alpha_cols[i][3], alpha_cols[i][4],
                             alpha_cols[i][5]);
    KDL::Twist unit_force(unit_force_l, unit_force_a);
    alpha_unit_forces->setColumn(i, unit_force);
  }
}

KDL::ChainHdSolver_Vereshchagin SolverUtils::initializeVereshchaginSolver(
    const KDL::Chain *robot_chain, int nc,
    const std::vector<std::vector<double>> &alpha_cols,
    KDL::Jacobian &alpha_unit_forces, std::vector<double> &beta_col,
    KDL::JntArray &beta_energy, KDL::JntArray &qd, KDL::JntArray &qdd,
    KDL::JntArray &ff_tau, KDL::JntArray &constraint_tau,
    KDL::Wrenches &f_ext) {
  // number of constraints
  int n_constraints = nc;

  // number of joints
  int n_joints = robot_chain->getNrOfJoints();

  // number of segments
  int n_segments = robot_chain->getNrOfSegments();

  // root acceleration - set gravity vector
  // direction is specified opposite for the solver
  KDL::Twist root_acc(KDL::Vector(0.0, 0.0, 9.81), KDL::Vector(0.0, 0.0, 0.0));

  // define unit constraint forces for EE
  alpha_unit_forces.resize(n_constraints);

  populateAlphaUnitForces(alpha_cols, &alpha_unit_forces);

  // beta - accel energy for EE
  beta_energy.resize(n_constraints);

  for (int i = 0; i < beta_col.size(); i++) {
    beta_energy(i) = beta_col[i];
  }

  // use vereshchagin solver
  KDL::ChainHdSolver_Vereshchagin vereshchagin_solver(*robot_chain, root_acc,
                                                      n_constraints);

  // set the solver parameters
  qd.resize(n_joints);              // input joint velocities
  qdd.resize(n_joints);             // input joint accelerations
  ff_tau.resize(n_joints);          // output feedforward torques
  constraint_tau.resize(n_joints);  // output constraint torques

  KDL::Vector f(0.0, 0.0, 0.0);
  KDL::Vector n(0.0, 0.0, 0.0);
  KDL::Wrench f_tool(f, n);
  f_ext.resize(n_segments);        // external forces at each segment
  f_ext[n_segments - 1] = f_tool;  // input external forces at EE

  return vereshchagin_solver;
}

std::tuple<std::array<double, 3>, std::array<double, 3>> SolverUtils::computeFK(
    KDL::Chain *robot_chain, KDL::JntArray &q, int seg_nr) {
  // create the solver
  KDL::ChainFkSolverPos_recursive fk_solver(*robot_chain);

  // create the frame that will contain the results
  KDL::Frame tool_tip_frame;

  // calculate forward kinematics
  int r = fk_solver.JntToCart(q, tool_tip_frame, seg_nr);
  if (r < 0) {
    std::cout << "Failed to compute forward kinematics with error: " << r
              << std::endl;
    return std::make_tuple(std::array<double, 3>{0, 0, 0},
                           std::array<double, 3>{0, 0, 0});
  }

  // extract position from frame
  double x = tool_tip_frame.p.x();
  double y = tool_tip_frame.p.y();
  double z = tool_tip_frame.p.z();

  // round the position to 4 decimal places
  x = round(x * 10000.0) / 10000.0;
  y = round(y * 10000.0) / 10000.0;
  z = round(z * 10000.0) / 10000.0;

  // convert the orientation to roll, pitch, yaw
  double roll, pitch, yaw;
  tool_tip_frame.M.GetRPY(roll, pitch, yaw);

  // convert the angles to degrees
  roll = roll * 180.0 / M_PI;
  pitch = pitch * 180.0 / M_PI;
  yaw = yaw * 180.0 / M_PI;

  // round the angles to 2 decimal places
  roll = round(roll * 100.0) / 100.0;
  pitch = round(pitch * 100.0) / 100.0;
  yaw = round(yaw * 100.0) / 100.0;

  // return the position and RPY values
  std::array<double, 3> position = {x, y, z};
  std::array<double, 3> rpy = {roll, pitch, yaw};
  return std::make_tuple(position, rpy);
}

KDL::Frame SolverUtils::computeFKFrame(KDL::Chain *robot_chain,
                                       KDL::JntArray &q, int seg_nr) {
  // create the solver
  KDL::ChainFkSolverPos_recursive fk_solver(*robot_chain);

  // create the frame that will contain the results
  KDL::Frame tool_tip_frame;

  // calculate forward kinematics
  int r = fk_solver.JntToCart(q, tool_tip_frame, seg_nr);
  if (r < 0) {
    std::cout << "Failed to compute forward kinematics with error: " << r
              << std::endl;
    return KDL::Frame();
  }

  return tool_tip_frame;
}

void SolverUtils::updateBetaEnergy(KDL::JntArray &beta_energy,
                                   const std::array<double, 6> &control_ae) {
  beta_energy(0) = control_ae[0];
  beta_energy(1) = control_ae[1];
  beta_energy(2) = 9.81 + control_ae[2];
  beta_energy(3) = control_ae[3];
  beta_energy(4) = control_ae[4];
  beta_energy(5) = control_ae[5];
}

void SolverUtils::updateQandQd(KDL::JntArray &q, KDL::JntArray &qd,
                               const KDL::JntArray *qdd, double dt) {
  // update the joint positions
  for (int i = 0; i < qdd->rows(); i++) {
    qd(i) = qd(i) + qdd->data(i) * dt;
    q(i) = q(i) + qd(i) * dt;
  }
}
