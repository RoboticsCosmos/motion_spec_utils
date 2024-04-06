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

#include "kdl_utils/utils.hpp"

#include "kdl/chainhdsolver_vereshchagin.hpp"
#include "kdl/kinfam_io.hpp"

void initialize_robot_state(int num_joints, int num_segments, Kinova *rob)
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
    rob->s[i] = new double[6]{};
    rob->s_dot[i] = new double[6]{};
    rob->s_ddot[i] = new double[6]{};
  }
}

void initialize_robot_state(int num_joints, int num_segments, double *init_q, Kinova *rob)
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
    rob->s[i] = new double[6]{};
    rob->s_dot[i] = new double[6]{};
    rob->s_ddot[i] = new double[6]{};
  }

  for (size_t i = 0; i < num_joints; i++)
  {
    rob->q[i] = init_q[i];
  }
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

// void computeForwardVelocityKinematics(std::string link_name, Kinova *rob, KDL::Chain
// *robot_chain,
//                                       double *out_twist)
// {
//   KDL::ChainFkSolverVel_recursive fk_solver(*robot_chain);

//   KDL::JntArrayVel q_dot(rob->nj);

//   for (size_t i = 0; i < rob->nj; i++)
//   {
//     q_dot.q(i) = rob->q[i];
//     q_dot.qdot(i) = rob->q_dot[i];
//   }

//   int seg_nr = -1;
//   getLinkIdFromChain(*robot_chain, link_name, seg_nr);

//   KDL::FrameVel frame_vel;

//   if (fk_solver.JntToCart(q_dot, frame_vel, seg_nr) < 0)
//   {
//     std::cerr << "Failed to compute forward velocity kinematics" << std::endl;
//   }

//   KDL::Twist twist = frame_vel.GetTwist();

//   out_twist[0] = twist.vel.x();
//   out_twist[1] = twist.vel.y();
//   out_twist[2] = twist.vel.z();
//   out_twist[3] = twist.rot.x();
//   out_twist[4] = twist.rot.y();
//   out_twist[5] = twist.rot.z();
// }

void computeForwardVelocityKinematics(std::string link_name,
                                      std::string as_seen_by,
                                      std::string with_respect_to,
                                      double *vec,
                                      Kinova* rob,
                                      KDL::Chain* robot_chain,
                                      double out_twist)
{
  int seg_nr = -1;
  getLinkIdFromChain(*robot_chain, link_name, seg_nr);

  int as_seen_by_id = -1;
  getLinkIdFromChain(*robot_chain, as_seen_by, as_seen_by_id);

  int with_respect_to_id = -1;
  getLinkIdFromChain(*robot_chain, with_respect_to, with_respect_to_id);

  for (size_t i = 0; i < 6; i++)
  {
    if (vec[i] != 0)
    {
      out_twist = rob->s_dot[seg_nr][i];
      break;
    }
  }
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

void updateQandQdot(double *q_ddot, double dt, Kinova *rob)
{
  for (size_t i = 0; i < rob->nj; i++)
  {
    rob->q_ddot[i] = q_ddot[i];
    rob->q_dot[i] += q_ddot[i] * dt;
    rob->q[i] += rob->q_dot[i] * dt;
  }
}

void achd_solver(Kinova *rob, KDL::Chain *chain, int num_constraints, double *root_acceleration,
                 double **alpha, double *beta, double **ext_wrench, double *tau_ff,
                 double *predicted_acc, double *constraint_tau)
{
  // root acceleration
  KDL::Twist root_acc(
      KDL::Vector(root_acceleration[0], root_acceleration[1], root_acceleration[2]),
      KDL::Vector(root_acceleration[3], root_acceleration[4], root_acceleration[5]));

  KDL::ChainHdSolver_Vereshchagin vereshchagin_solver(*chain, root_acc, num_constraints);

  // alpha - constraint forces
  KDL::Jacobian alpha_jac = KDL::Jacobian(num_constraints);

  for (int i = 0; i < num_constraints; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      alpha_jac(i, j) = alpha[i][j];
    }
  }

  // beta - accel energy
  KDL::JntArray beta_jnt = KDL::JntArray(num_constraints);

  for (int i = 0; i < num_constraints; i++)
  {
    beta_jnt(i) = beta[i];
  }

  // q, qd, qdd
  KDL::JntArray q = KDL::JntArray(rob->nj);
  KDL::JntArray qd = KDL::JntArray(rob->nj);

  for (size_t i = 0; i < rob->nj; i++)
  {
    q(i) = rob->q[i];
    qd(i) = rob->q_dot[i];
  }

  // ext wrench
  KDL::Wrenches f_ext;

  for (int i = 0; i < rob->ns; i++)
  {
    KDL::Wrench wrench;
    for (int j = 0; j < 6; j++)
    {
      wrench(j) = ext_wrench[i][j];
    }
    f_ext.push_back(wrench);
  }

  // feedforward torques
  KDL::JntArray ff_tau_jnt = KDL::JntArray(rob->nj);

  for (size_t i = 0; i < rob->nj; i++)
  {
    ff_tau_jnt(i) = tau_ff[i];
  }

  // predicted accelerations
  KDL::JntArray qdd = KDL::JntArray(rob->nj);

  // constraint torques
  KDL::JntArray constraint_tau_jnt = KDL::JntArray(rob->nj);

  int r = vereshchagin_solver.CartToJnt(q, qd, qdd, alpha_jac, beta_jnt, f_ext, ff_tau_jnt,
                                        constraint_tau_jnt);

  std::vector<KDL::Twist> twists(7);
  vereshchagin_solver.getLinkCartesianVelocity(twists);

  std::vector<KDL::Frame> frames(7);
  vereshchagin_solver.getLinkCartesianPose(frames);

  for (size_t j = 0; j < rob->ns; j++)
  {
    // update segment pose
    // convert KDL::Frame to double[6] - [x, y, z, rx, ry, rz]
    rob->s[j][0] = frames[j].p.x();
    rob->s[j][1] = frames[j].p.y();
    rob->s[j][2] = frames[j].p.z();
    frames[j].M.GetRPY(rob->s[j][3], rob->s[j][4], rob->s[j][5]);

    // update segment twist
    for (size_t i = 0; i < 6; i++)
    {
      rob->s_dot[j][i] = twists[j](i);
    }
  }

  if (r < 0)
  {
    std::cerr << "Failed to solve the hybrid dynamics problem" << std::endl;
    std::cerr << "Error code: " << r << std::endl;
  }

  for (size_t i = 0; i < rob->nj; i++)
  {
    predicted_acc[i] = qdd(i);
    constraint_tau[i] = constraint_tau_jnt(i);
  }
}