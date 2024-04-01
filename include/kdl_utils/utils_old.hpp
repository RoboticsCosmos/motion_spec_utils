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

#include <array>
#include <iostream>
#include <string>
#include <vector>

#include "kdl/chain.hpp"
#include "kdl/frames.hpp"
#include "kdl/jacobian.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"

enum ENV { SIM, ROB };

class Utils {
 public:
  Utils();
  ~Utils();

  /**
   * @brief Prints the names of all links in a KDL::Tree.
   * @param tree The KDL::Tree object representing the robot's kinematic tree.
   */
  static void printLinkNames(KDL::Tree& tree);

  /**
   * @brief Get the names of all links in a KDL::Chain.
   * @param chain The KDL::Chain object representing a subset of the robot's
   * kinematic chain.
   * @return A vector containing the names of all links in the chain.
   */
  static std::vector<std::string> getLinkNamesFromChain(KDL::Chain& chain);

  /**
   * @brief Get the id of a link in a KDL::Chain.
   * @param chain The KDL::Chain object representing a subset of the robot's
   * kinematic chain.
   * @param link_name The name of the link.
   * @return The id of the link.
   */
  static int getLinkIdFromChain(KDL::Chain& chain,
                                const std::string& link_name);

  /**
   * @brief Check if the given link name is in the KDL::Chain.
   * @param chain The KDL::Chain object representing a subset of the robot's
   * kinematic chain.
   */
  static bool checkLinkInChain(KDL::Chain& chain, const std::string& link_name);

  /**
   * @brief Prints the names of all joints in a KDL::Chain.
   * @param chain The KDL::Chain object representing a subset of the robot's
   * kinematic chain.
   */
  static void printJointNames(KDL::Chain& chain);

  /**
   * @brief Prints the elements of a vector.
   * @tparam T The type of the vector elements.
   * @param vec The vector to be printed.
   */
  template <typename T>
  static void printVec(const T& vec);

  /**
   * @brief Prints the elements of a KDL::JntArray.
   * @tparam T The type of the KDL::JntArray.
   * @param jntArr The KDL::JntArray to be printed.
   */
  template <typename T>
  static void printJntArr(const T& jntArr);

  /**
   * @brief Initializes the robot using a URDF file and sets the initial joint
   * angles.
   * @param urdf_path The file path to the robot's URDF description.
   * @param robot_chain [out] The KDL chain representing the robot.
   * @param base_link The name of the robot's base link.
   * @param tool_link The name of the robot's tool link.
   * @return 0 on success, -1 on failure.
   */
  int initialize_robot_urdf(const std::string& urdf_path,
                            KDL::Chain& robot_chain,
                            const std::string& base_link,
                            const std::string& tool_link);

  /**
   * @brief Initializes the q with the initial joint angles.
   * @param initial_joint_angles A vector containing the initial joint angles.
   * @param q [out] The KDL joint array representing the robot's joint angles.
   * @param env a ENV enum value representing the environment the robot is in.
   * @return 0 on success, -1 on failure.
   */
  int init_q(
      KDL::Chain* robot_chain, KDL::JntArray& q,
      const std::vector<double>& initial_joint_angles = std::vector<double>(),
      ENV env = ENV::SIM);

  /**
   * @brief Computes the euclidean distance between two 3d points.
   * @param p1 The first point of type std::array<double, 3>.
   * @param p2 The second point of type std::array<double, 3>.
   * @return The euclidean distance between the two points.
   */
  static double computeEuclideanDistance(const std::array<double, 3>& current,
                                         const std::array<double, 3>& target);

  /**
   * @brief Calculates the error between two points.
   *
   * @param p1 The first point (x1, y1, z1).
   * @param p2 The second point (x2, y2, z2).
   * @return The error as a tuple (dx, dy, dz).
   */
  std::vector<double> calc_error(const std::array<double, 3>& p1,
                                 const std::array<double, 3>& p2);

  /**
   * @brief Calculates the error between two kdl vectors.
   * @param v1 The first vector.
   * @param v2 The second vector.
   * @return The error as a kdl vector.
   */
  KDL::Vector calc_error(const KDL::Vector& v1, const KDL::Vector& v2);
};

#endif  // UTILS_HPP
