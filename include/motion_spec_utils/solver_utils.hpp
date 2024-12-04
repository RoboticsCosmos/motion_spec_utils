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
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef SOLVER_UTILS_HPP
#define SOLVER_UTILS_HPP

#include <array>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

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
#include "motion_spec_utils/robot_structs.hpp"

#include <kelo_kindyn/functions/kelo_platform.h>
#include <kelo_kindyn/functions/kelo_platform_solver.h>
#include <kelo_kindyn/functions/kelo_drive.h>
#include <kelo_kindyn/functions/kelo_wheel.h>

#include <unsupported/Eigen/MatrixFunctions>


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
void achd_solver(Freddy *rob, std::string root_link, std::string tip_link, int num_constraints,
                 double *root_acceleration, double alpha[][6], double *beta, double *tau_ff,
                 double *predicted_acc, double *constraint_tau);

void achd_solver_fext(Freddy *rob, std::string root_link, std::string tip_link,
                      double ext_wrenches[][6], double *constraint_tau);

void rne_solver(Freddy *rob, std::string root_link, std::string tip_link,
                double *root_acceleration, double ext_wrenches[][6], double *constraint_tau);

void base_fd_solver(Freddy *rob, double *platform_force, double *wheel_torques);

void wrench_estimator(Freddy *rob, std::string root_link, std::string tip_link,
                      double *root_acceleration, double *tau, double *tool_wrench);

void achd_solver_manipulator(Manipulator<kinova_mediator> *rob, int num_constraints,
                             double *root_acceleration, double **alpha, double *beta,
                             double *tau_ff, double *predicted_acc, double *constraint_tau);

void rne_solver_manipulator(Manipulator<kinova_mediator> *rob, double *root_acceleration,
                            double **ext_wrench, double *constraint_tau);

void get_pivot_alignment_offsets(Freddy *robot, double *platform_force, double *lin_offsets,
                                 double *ang_offsets);

void base_fd_solver_with_alignment(Freddy *robot, double *platform_force, double *linear_offsets,
                                   double *angular_offsets, double *wheel_torques);

void base_fd_solver_cgls(Freddy *robot, double *platform_force, double *alignment_taus,
                                    double *wheel_torques);

#endif  // SOLVER_UTILS_HPP