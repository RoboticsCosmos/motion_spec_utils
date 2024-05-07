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
    rob->s[i] = new double[6]{};
    rob->s_dot[i] = new double[6]{};
    rob->s_ddot[i] = new double[6]{};
  }

  rob->tau_command = new double[num_joints]{};
  rob->tau_measured = new double[num_joints]{};
  rob->f_tool_command = new double[6]{};
  rob->f_tool_measured = new double[6]{};
}

void initialize_mobile_base_state(MobileBaseState *base)
{
  base->pivot_angles = new double[3]{};
  base->tau_command = new double[3]{};
}

// void initialize_robot_state(int num_joints, int num_segments, double *init_q,
//                             Manipulator *rob)
// {
//   rob->nj = num_joints;
//   rob->ns = num_segments;

//   rob->q = new double[num_joints]{};
//   rob->q_dot = new double[num_joints]{};
//   rob->q_ddot = new double[num_joints]{};

//   rob->s = new double *[num_segments];
//   rob->s_dot = new double *[num_segments];
//   rob->s_ddot = new double *[num_segments];

//   for (size_t i = 0; i < num_segments; i++)
//   {
//     rob->s[i] = new double[6]{};
//     rob->s_dot[i] = new double[6]{};
//     rob->s_ddot[i] = new double[6]{};
//   }

//   for (size_t i = 0; i < num_joints; i++)
//   {
//     rob->q[i] = init_q[i];
//   }

//   rob->tau_command = new double[num_joints]{};
//   rob->tau_measured = new double[num_joints]{};
//   rob->f_tool_command = new double[6]{};
//   rob->f_tool_measured = new double[6]{};
// }

void initialize_robot(std::string robot_urdf, Freddy *freddy)
{
  // load the robot urdf
  if (!kdl_parser::treeFromFile(robot_urdf, freddy->tree))
  {
    std::cerr << "Failed to construct KDL tree" << std::endl;
  }

  std::cout << "Successfully initialized robot tree" << std::endl;

  // get the chains
  if (!freddy->tree.getChain(freddy->kinova_left->base_frame,
                             freddy->kinova_left->tool_frame, freddy->kinova_left->chain))
  {
    std::cerr << "Failed to get chain from KDL tree" << std::endl;
  }

  if (!freddy->tree.getChain(freddy->kinova_right->base_frame,
                             freddy->kinova_right->tool_frame,
                             freddy->kinova_right->chain))
  {
    std::cerr << "Failed to get chain from KDL tree" << std::endl;
  }

  std::cout << "Successfully initialized robot chains" << std::endl;

  // initialize the states
  initialize_manipulator_state(freddy->kinova_left->chain.getNrOfJoints(),
                               freddy->kinova_left->chain.getNrOfSegments(),
                               freddy->kinova_left->state);

  initialize_manipulator_state(freddy->kinova_right->chain.getNrOfJoints(),
                               freddy->kinova_right->chain.getNrOfSegments(),
                               freddy->kinova_right->state);

  initialize_mobile_base_state(freddy->mobile_base->state);

  // initialize the mediators
  freddy->kinova_left->mediator->initialize(0, 0, 0.0);
  freddy->kinova_left->mediator->set_control_mode(2);

  freddy->kinova_right->mediator->initialize(0, 1, 0.0);
  freddy->kinova_right->mediator->set_control_mode(2);

  initialize_kelo_base(freddy->mobile_base->mediator->kelo_base_config,
                       freddy->mobile_base->mediator->ethercat_config);

  int result = 0;
  establish_kelo_base_connection(freddy->mobile_base->mediator->kelo_base_config,
                                 freddy->mobile_base->mediator->ethercat_config, "eno1",
                                 &result);

  if (result != 0)
  {
    std::cerr << "Failed to establish connection with the mobile base" << std::endl;
    exit(1);
  }

  std::cout << "Successfully initialized robot" << std::endl;
}

void initialize_robot_chain(std::string robot_urdf, std::string base_link,
                            std::string tool_link, KDL::Chain &robot_chain)
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

void rne_solver(ManipulatorState *rob, KDL::Chain *chain, double *root_acceleration,
                double **ext_wrench, double *constraint_tau)
{
  // root acceleration
  KDL::Twist root_acc(KDL::Vector(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.0, 0.0));

  KDL::ChainIdSolver_RNE rne(*chain, KDL::Vector(0, 0, 0));

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

  // predicted accelerations
  KDL::JntArray qdd = KDL::JntArray(rob->nj);

  // constraint torques
  KDL::JntArray constraint_tau_jnt = KDL::JntArray(rob->nj);

  int r = rne.CartToJnt(q, qd, qdd, f_ext, constraint_tau_jnt);

  if (r < 0)
  {
    std::cerr << "Failed to solve the hybrid dynamics problem" << std::endl;
    std::cerr << "Error code: " << r << std::endl;
  }

  for (size_t i = 0; i < rob->nj; i++)
  {
    constraint_tau[i] = constraint_tau_jnt(i);
  }
}

void achd_solver_fext(Freddy *rob, std::string root_link, std::string tip_link,
                      double **ext_wrench, double *constraint_tau)
{
  // create a chain from the root_link to the tip_link
  KDL::Chain *chain = new KDL::Chain();
  if (!rob->tree.getChain(root_link, tip_link, *chain))
  {
    std::cerr << "Failed to get chain from KDL tree" << std::endl;
  }

  // get the corresponding robot state
  ManipulatorState *rob_state;
  findManipulatorStateFromRootLink(root_link, rob, rob_state);

  // root acceleration
  KDL::Twist root_acc(KDL::Vector(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.0, 0.0));

  int num_constraints = 6;

  KDL::ChainHdSolver_Vereshchagin_Fext vereshchagin_solver_fext(*chain, root_acc,
                                                                num_constraints);

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

  for (int i = 0; i < rob_state->ns; i++)
  {
    KDL::Wrench wrench;
    for (int j = 0; j < 6; j++)
    {
      wrench(j) = ext_wrench[i][j];
    }
    f_ext.push_back(wrench);
  }

  // feedforward torques
  KDL::JntArray ff_tau_jnt = KDL::JntArray(rob_state->nj);

  // predicted accelerations
  KDL::JntArray qdd = KDL::JntArray(rob_state->nj);

  // constraint torques
  KDL::JntArray constraint_tau_jnt = KDL::JntArray(rob_state->nj);

  int r = vereshchagin_solver_fext.CartToJnt(q, qd, qdd, alpha_jac, beta_jnt, f_ext,
                                             ff_tau_jnt, constraint_tau_jnt);

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

void findManipulatorStateFromRootLink(std::string root_link, Freddy *rob, ManipulatorState *state)
{
  if (root_link == rob->kinova_left->base_frame)
  {
    state = rob->kinova_left->state;
  }
  else if (root_link == rob->kinova_right->base_frame)
  {
    state = rob->kinova_right->state;
  }
  else
  {
    std::cerr << "Root link not found in the robot chains" << std::endl;
  }
}

void achd_solver(Freddy *rob, std::string root_link, std::string tip_link,
                 int num_constraints, double *root_acceleration, double **alpha,
                 double *beta, double *tau_ff, double *predicted_acc,
                 double *constraint_tau)
{

  // create a chain from the root_link to the tip_link
  KDL::Chain *chain = new KDL::Chain();
  if (!rob->tree.getChain(root_link, tip_link, *chain))
  {
    std::cerr << "Failed to get chain from KDL tree" << std::endl;
  }

  // get the corresponding robot state
  ManipulatorState *rob_state;
  findManipulatorStateFromRootLink(root_link, rob, rob_state);

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
  KDL::JntArray q = KDL::JntArray(rob_state->nj);
  KDL::JntArray qd = KDL::JntArray(rob_state->nj);

  for (size_t i = 0; i < rob_state->nj; i++)
  {
    q(i) = rob_state->q[i];
    qd(i) = rob_state->q_dot[i];
  }

  // ext wrench
  KDL::Wrenches f_ext;

  for (int i = 0; i < rob_state->ns; i++)
  {
    KDL::Wrench wrench;
    for (int j = 0; j < 6; j++)
    {
      wrench(j) = 0.0;
    }
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

  int r = vereshchagin_solver.CartToJnt(q, qd, qdd, alpha_jac, beta_jnt, f_ext,
                                        ff_tau_jnt, constraint_tau_jnt);

  if (r < 0)
  {
    std::cerr << "Failed to solve the hybrid dynamics problem" << std::endl;
    std::cerr << "Error code: " << r << std::endl;
  }

  for (size_t i = 0; i < rob_state->nj; i++)
  {
    predicted_acc[i] = qdd(i);
    constraint_tau[i] = constraint_tau_jnt(i);
  }
}

template <typename MediatorType>
void get_manipulator_data(Manipulator<MediatorType> *rob, KDL::Tree *tree)
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

  for (size_t i = 0; i < 6; i++)
  {
    rob->state->f_tool_measured[i] = f_tool_measured(i);
  }

  // *Assumption* - computing values in the base_link frame
  // create a chain from the base_link to the tool_link
  KDL::Chain *chain = new KDL::Chain();
  if (!tree->getChain("base_link", rob->tool_frame, *chain))
  {
    std::cerr << "Failed to get chain from KDL tree" << std::endl;
  }

  // update the s_dot values
  KDL::ChainFkSolverVel_recursive fk_solver_vel(*chain);
  std::vector<KDL::FrameVel> frame_vels(rob->state->ns);
  KDL::JntArrayVel q_dot_kdl(rob->state->nj);
  for (size_t i = 0; i < rob->state->nj; i++)
  {
    q_dot_kdl.q(i) = rob->state->q[i];
    q_dot_kdl.qdot(i) = rob->state->q_dot[i];
  }
  fk_solver_vel.JntToCart(q_dot_kdl, frame_vels);

  for (size_t i = 0; i < rob->state->ns; i++)
  {
    // update the s_dot values
    rob->state->s_dot[i][0] = frame_vels[i].GetTwist().vel.x();
    rob->state->s_dot[i][1] = frame_vels[i].GetTwist().vel.y();
    rob->state->s_dot[i][2] = frame_vels[i].GetTwist().vel.z();
    rob->state->s_dot[i][3] = frame_vels[i].GetTwist().rot.x();
    rob->state->s_dot[i][4] = frame_vels[i].GetTwist().rot.y();
    rob->state->s_dot[i][5] = frame_vels[i].GetTwist().rot.z();

    // update the s values
    rob->state->s[i][0] = frame_vels[i].GetFrame().p.x();
    rob->state->s[i][1] = frame_vels[i].GetFrame().p.y();
    rob->state->s[i][2] = frame_vels[i].GetFrame().p.z();
    rob->state->s[i][3] = frame_vels[i].GetFrame().M.GetRot().x();
    rob->state->s[i][4] = frame_vels[i].GetFrame().M.GetRot().y();
    rob->state->s[i][5] = frame_vels[i].GetFrame().M.GetRot().z();
  }
}

void set_manipulator_torques(ManipulatorState *rob, kinova_mediator *mediator,
                             double *tau_command)
{
  KDL::JntArray tau_cmd(rob->nj);

  for (size_t i = 0; i < rob->nj; i++)
  {
    tau_cmd(i) = tau_command[i];
  }

  mediator->set_joint_torques(tau_cmd);
}

void getLinkSFromRob(std::string link_name, Freddy *rob, double *s)
{
  // find which robot the link belongs to
  int link_id = -1;
  bool is_in_left_chain, is_in_right_chain = false;
  findLinkInChain(link_name, &rob->kinova_left->chain, is_in_left_chain, link_id);
  findLinkInChain(link_name, &rob->kinova_right->chain, is_in_right_chain, link_id);

  if (!is_in_left_chain && !is_in_right_chain)
  {
    std::cerr << "Link not found in the robot chains" << std::endl;
    return;
  }

  // Select appropriate state data based on which chain the link was found in
  const double *state = is_in_left_chain ? rob->kinova_left->state->s[link_id]
                                         : rob->kinova_right->state->s[link_id];

  for (size_t i = 0; i < 6; i++)
  {
    s[i] = state[i];
  }
}

void computeDistance(std::string *between_ents, std::string asb, Freddy *rob,
                     double &distance)
{
  double *ent1 = new double[6]{};
  double *ent2 = new double[6]{};

  getLinkSFromRob(between_ents[0], rob, ent1);
  getLinkSFromRob(between_ents[1], rob, ent2);

  if (asb == "base_link")
  {
    distance = sqrt(pow(ent1[0] - ent2[0], 2) + pow(ent1[1] - ent2[1], 2) +
                    pow(ent1[2] - ent2[2], 2));
  }
  else
  {
    // construct a chain from the base_link to the asb link
    KDL::Chain chain;
    if (!rob->tree.getChain("base_link", asb, chain))
    {
      std::cerr << "Failed to get chain from KDL tree" << std::endl;
      return;
    }

    // get the transform from the base_link to the asb link
    KDL::Frame frame;
    KDL::ChainFkSolverPos_recursive fk_solver_pos(chain);

    if (chain.getNrOfJoints() == 0)
    {
      KDL::JntArray q = KDL::JntArray(chain.getNrOfJoints());

      fk_solver_pos.JntToCart(q, frame);

      KDL::Vector vec1(ent1[0], ent1[1], ent1[2]);
      KDL::Vector vec2(ent2[0], ent2[1], ent2[2]);

      vec1 = frame.M * vec1;
      vec2 = frame.M * vec2;

      distance = sqrt(pow(vec1.x() - vec2.x(), 2) + pow(vec1.y() - vec2.y(), 2) +
                      pow(vec1.z() - vec2.z(), 2));
    }
    else
    {
      std::cerr << "feature not implemented yet" << std::endl;
      exit(1);
    }
  }
}

void getLinkVelocity(std::string link_name, std::string as_seen_by,
                     std::string with_respect_to, double *vec, Freddy *rob,
                     double out_twist)
{
  // find which robot the link belongs to
  bool is_in_left_chain = false;
  int link_id = -1;
  findLinkInChain(link_name, &rob->kinova_left->chain, is_in_left_chain, link_id);

  bool is_in_right_chain = false;
  findLinkInChain(link_name, &rob->kinova_right->chain, is_in_right_chain, link_id);

  if (!is_in_left_chain && !is_in_right_chain)
  {
    std::cerr << "Link not found in the robot chains" << std::endl;
    return;
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

void getLinkForce(std::string applied_by, std::string applied_to, std::string asb,
                  double *vec, Freddy *freddy, double &force)
{
  // TODO: implement this
  force = 0.0;
}

void findLinkInChain(std::string link_name, KDL::Chain *chain, bool &is_in_chain,
                     int &link_id)
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
  int from_id, to_id = -1;
  bool is_in_left_chain, is_in_right_chain = false;
  double *from_s = new double[6]{};
  double *to_s = new double[6]{};

  getLinkSFromRob(from_ent, rob, from_s);
  getLinkSFromRob(to_ent, rob, to_s);

  for (size_t i = 0; i < sizeof(vec) / sizeof(vec[0]); i++)
  {
    vec[i] = to_s[i] - from_s[i];
  }
}

void findNormalizedVector(const double *vec, double *normalized_vec)
{
  double norm = 0.0;
  for (size_t i = 0; i < sizeof(vec) / sizeof(vec[0]); i++)
  {
    norm += vec[i] * vec[i];
  }
  norm = sqrt(norm);

  for (size_t i = 0; i < sizeof(vec) / sizeof(vec[0]); i++)
  {
    normalized_vec[i] = vec[i] / norm;
  }
}

void get_robot_data(Freddy *freddy)
{
  get_manipulator_data(freddy->kinova_left, &freddy->tree);
  get_manipulator_data(freddy->kinova_right, &freddy->tree);
  get_kelo_base_state(freddy->mobile_base->mediator->kelo_base_config,
                      freddy->mobile_base->mediator->ethercat_config,
                      freddy->mobile_base->state->pivot_angles);
}