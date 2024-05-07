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

#ifndef UTILS_HPP
#define UTILS_HPP

#include <ActuatorConfigClientRpc.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <KDetailedException.h>
#include <RouterClient.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>
#include <google/protobuf/util/json_util.h>

#include <array>
#include <iostream>
#include <string>
#include <vector>

#include "kdl/chain.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainfksolvervel_recursive.hpp"
#include "kdl/chainhdsolver_vereshchagin.hpp"
#include "kdl/chainhdsolver_vereshchagin_fext.hpp"
#include "kdl/chainidsolver_recursive_newton_euler.hpp"
#include "kdl/frames.hpp"
#include "kdl/jacobian.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/kinfam_io.hpp"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "kelo_motion_control/mediator.h"
#include "kinova_mediator/mediator.hpp"
#include "motion_spec_utils/robot_structs.hpp"

/**
 * @brief Initializes the robot state struct.
 * @param num_joints The number of joints.
 * @param num_segments The number of segments.
 * @param rob [out] The robot state struct.
 */
void initialize_manipulator_state(int num_joints, int num_segments,
                                  ManipulatorState *rob);

void initialize_mobile_base_state(MobileBaseState *base);

/**
 * @brief Initializes the robot state struct.
 * @param num_joints The number of joints.
 * @param num_segments The number of segments.
 * @param init_q The initial joint positions.
 * @param rob [out] The robot state struct.
 */
// void initialize_robot_state(int num_joints, int num_segments, double *init_q,
//                             Manipulator *rob);

void initialize_robot(std::string robot_urdf, Freddy *freddy);

/**
 * @brief Initializes the robot chain.
 * @param robot_urdf The path to the robot urdf.
 * @param base_link The name of the base link.
 * @param tool_link The name of the tool link.
 * @param robot_chain [out] The KDL chain representing the robot.
 */
void initialize_robot_chain(std::string robot_urdf, std::string base_link,
                            std::string tool_link, KDL::Chain &robot_chain);

/**
 * @brief Computes forward velocity kinematics.
 * @param link_name The name of the link.
 * @param as_seen_by The name of the link from which the twist is computed.
 * @param with_respect_to The name of the link with respect to which the twist is
 * computed.
 * @param vec The direction vector.
 * @param rob Robot state struct.
 * @param robot_chain The KDL chain representing the robot.
 * @param out_twist [out] The twist of the link.
 */
// void computeForwardVelocityKinematics(std::string link_name, std::string as_seen_by,
//                                       std::string with_respect_to, double *vec,
//                                       Manipulator *rob, KDL::Chain *robot_chain,
//                                       double out_twist);

/**
 * @brief Adds two arrays.
 * @param arr1 The first array.
 * @param arr2 The second array.
 * @param result [out] The result of the addition.
 * @param size The size of the arrays.
 */
extern void add(double *arr1, double *arr2, double *result, size_t size);

/**
 * @brief Updates the q and q_dot values of the robot state struct.
 * @param q_ddot The joint accelerations.
 * @param dt The time step.
 * @param rob [in/out] The robot state struct.
 */
void updateQandQdot(double *q_ddot, double dt, ManipulatorState *rob);

void findManipulatorStateFromRootLink(std::string root_link, Freddy *rob,
                                      ManipulatorState *state);

/**
 * @brief Solves the hybrid dynamics problem for a given robot chain.
 * @param rob The robot state struct.
 * @param robot_chain The KDL::Chain object representing the robot's kinematic chain.
 * @param num_constraints The number of constraints.
 * @param root_acceleration The root acceleration.
 * @param alpha The alpha matrix.
 * @param beta The beta vector.
 * @param ext_wrench The external wrenches.
 * @param tau_ff The feedforward torques.
 * @param predicted_acc [out] The output predicted joint accelerations.
 * @param constraint_tau [out] The output constraint torques.
 */
void achd_solver(Freddy *rob, std::string root_link, std::string tip_link,
                 int num_constraints, double *root_acceleration, double **alpha,
                 double *beta, double *tau_ff, double *predicted_acc,
                 double *constraint_tau);

void achd_solver_fext(Freddy *rob, std::string root_link, std::string tip_link,
                      double **ext_wrench, double *constraint_tau);

void rne_solver(ManipulatorState *rob, KDL::Chain *chain, double *root_acceleration,
                double **ext_wrench, double *constraint_tau);

void getLinkIdFromChain(KDL::Chain &chain, std::string link_name, int &link_id);

template <typename MediatorType>
void get_manipulator_data(Manipulator<MediatorType> *rob, KDL::Tree *tree);

void set_manipulator_torques(ManipulatorState *rob, kinova_mediator *mediator,
                             double *tau_command);

void getLinkSFromRob(std::string link_name, Freddy *rob, double *s);

void computeDistance(std::string *between_ents, std::string asb, Freddy *rob,
                     double &distance);

void getLinkForce(std::string applied_by, std::string applied_to, std::string asb,
                  double *vec, Freddy *rob, double &force);

void getLinkVelocity(std::string link_name, std::string as_seen_by,
                     std::string with_respect_to, double *vec, Freddy *rob,
                     double out_twist);

void findLinkInChain(std::string link_name, KDL::Chain *chain, bool &is_in_chain,
                     int &link_id);

void findVector(std::string from_ent, std::string to_ent, Freddy *rob, double *vec);

void findNormalizedVector(const double *vec, double *normalized_vec);

// void decomposeSignal(const std::string from_ent, const std::string to_ent,
//                      const double signal, Manipulator *rob, KDL::Chain *chain,
//                      double *vec);

void get_robot_data(Freddy *freddy);

#endif  // UTILS_HPP
