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
#include "kdl/chainfksolver.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainidsolver_vereshchagin.hpp"
#include "kdl/jacobian.hpp"
#include "kdl/jntarray.hpp"


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


#endif  // SOLVER_UTILS_HPP