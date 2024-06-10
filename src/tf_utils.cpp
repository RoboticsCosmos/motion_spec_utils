/**
 * Author: Vamsi Kalagaturu
 *
 * Description: Library to perform frame transformations for the arm_actions
 * package
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

#include "motion_spec_utils/tf_utils.hpp"

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