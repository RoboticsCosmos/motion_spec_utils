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

#define DEG2RAD(x) ((x) * M_PI / 180.0)
#define RAD2DEG(x) ((x) * 180.0 / M_PI)

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
#include <cstring>

#include <signal.h>

#include "kdl/chain.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainfksolvervel_recursive.hpp"
#include "kdl/chainhdsolver_vereshchagin.hpp"
#include "kdl/chainhdsolver_vereshchagin_fext.hpp"
#include "kdl/chainidsolver_recursive_newton_euler.hpp"
#include "kdl/chainexternalwrenchestimator.hpp"
#include "kdl/frames.hpp"
#include "kdl/jacobian.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/frames_io.hpp"
#include "kdl/kinfam_io.hpp"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "kinova_mediator/mediator.hpp"
#include "motion_spec_utils/robot_structs.hpp"

#include <kelo_kindyn/functions/kelo_platform.h>
#include <kelo_kindyn/functions/kelo_platform_solver.h>
#include <kelo_kindyn/functions/kelo_drive.h>
#include <kelo_kindyn/functions/kelo_wheel.h>

#include "motion_spec_utils/tf_utils.hpp"

/**
 * @brief Initializes the robot state struct.
 * @param num_joints The number of joints.
 * @param num_segments The number of segments.
 * @param rob [out] The robot state struct.
 */
void initialize_manipulator_state(int num_joints, int num_segments, ManipulatorState *rob);

void free_manipulator_state(ManipulatorState *rob);

void initialize_mobile_base_state(MobileBaseState *base);

void free_mobile_base_state(MobileBaseState *base);

void free_manipulator(Manipulator<kinova_mediator> *rob);

void free_mobile_base(MobileBase<Robile> *base);

void free_robot_data(Freddy *rob);

void ckpv(Freddy *rob, double dt);

/**
 * @brief Initializes the robot state struct.
 * @param num_joints The number of joints.
 * @param num_segments The number of segments.
 * @param init_q The initial joint positions.
 * @param rob [out] The robot state struct.
 */
// void initialize_robot_state(int num_joints, int num_segments, double *init_q,
//                             Manipulator *rob);

void initialize_robot(std::string robot_urdf, char *interface, Freddy *freddy);

void initialize_robot_sim(std::string robot_urdf, Freddy *freddy);

/**
 * @brief Initializes the robot chain.
 * @param robot_urdf The path to the robot urdf.
 * @param base_link The name of the base link.
 * @param tool_link The name of the tool link.
 * @param robot_chain [out] The KDL chain representing the robot.
 */
void initialize_robot_chain(std::string robot_urdf, std::string base_link, std::string tool_link,
                            KDL::Chain &robot_chain);

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

void getLinkIdFromChain(KDL::Chain &chain, std::string link_name, int &link_id);

template <typename MediatorType>
void get_manipulator_data(Manipulator<MediatorType> *rob);

void update_manipulator_state(ManipulatorState *state, std::string tool_frame, KDL::Tree *tree);

void cap_and_convert_manipulator_torques(double tau_command[], int num_joints, KDL::JntArray &tau_jnt);

int set_manipulator_torques(Freddy *rob, std::string root_link, KDL::JntArray *tau_command);

void set_mobile_base_torques(Freddy *rob, double *tau_command);

void getLinkSFromRob(std::string link_name, Freddy *rob, double *s);

void computeDistance(std::string *between_ents, std::string asb, Freddy *rob, double &distance);

void computeDistance1D(std::string *between_ents, double *axis, std::string asb, Freddy *rob,
                       double &distance);

void getMagnitude(double *vec, size_t size, double &magnitude);

void crossProduct(double *vec1, double*vec2, size_t size, double *result);

void dotProduct(double *vec1, double *vec2, size_t size, double &result);

void getLine(Freddy *rob, std::string *entities, size_t num_entities, double *direction);

void getAngleBetweenLines(Freddy *rob, double *line1, double *line2, double *angle_about_vec,
                          double &angle);

void getLinkForce(std::string applied_by, std::string applied_to, std::string asb, double *vec,
                  Freddy *rob, double &force);

void getLinkPosition(std::string link_name, std::string as_seen_by, std::string with_respect_to,
                     double *vec, Freddy *rob, double &out_position);

void getLinkQuaternion(std::string link_name, std::string as_seen_by, std::string with_respect_to,
                       Freddy *rob, double *out_quaternion);

void getLinkVelocity(std::string link_name, std::string as_seen_by, std::string with_respect_to,
                     double *vec, Freddy *rob, double &out_twist);

void getLinkId(Freddy *rob, std::string root_link, std::string tip_link, std::string link_name,
               int &link_id);

void findLinkInChain(std::string link_name, KDL::Chain *chain, bool &is_in_chain, int &link_id);

void findVector(std::string from_ent, std::string to_ent, Freddy *rob, double *vec);

void findNormalizedVector(const double *vec, double *normalized_vec);

void decomposeSignal(Freddy *rob, const std::string from_ent, const std::string to_ent,
                     std::string asb_ent, const double signal, double *vec);

void get_robot_data(Freddy *rob, double dt);

void print_robot_data(Freddy *rob);

void get_manipulator_data_sim(Manipulator<kinova_mediator> *rob, double *param_type,
                              double time_step);

void get_robot_data_sim(Freddy *freddy, double *kinova_left_predicted_acc,
                        double *kinova_right_predicted_acc, double time_step);

void set_init_sim_data(Freddy *freddy);

void compute_kelo_platform_velocity(Freddy *rob);

void compute_kelo_platform_pose(double *xd_platform, double dt, double *x_platform);

void computeQuaternionEqualityError(double *measured, double *ref, double *signal);

void print_array(double arr[], int size);

void init_2d_array(double **arr, int rows, int cols);

void get_new_folder_name(const char *dir_path, char *name);

void write_odom_data_to_open_file(FILE *file, std::vector<std::array<double, 3>> &odom_data);

void print_matrix2(int rows, int cols, const double *a);

#endif  // UTILS_HPP
