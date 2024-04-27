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

#ifndef TF_UTILS_HPP
#define TF_UTILS_HPP

/**
 * @brief transformation utils to convert kdl frames between different
 * coordinate systems
 */

#include <iostream>

#include "kdl/chain.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"

/**
 * @brief enum for different coordinate systems
 */
enum class CoordinateSystem { BASE, EE };

class TfUtils {
 public:
  TfUtils();

  ~TfUtils();

  /**
   * @brief set the chain for the forward kinematics solver
   * @param chain A pointer to a KDL::Chain object representing the robot's
   * kinematic chain.
   */
  void setChain(KDL::Chain *chain);

  /**
   * @brief Converts a KDL::Frame from one frame to another.
   * @param source_frame The KDL::Frame to convert.
   * @param q A pointer to a KDL::JntArray object representing the robot's joint
   * positions.
   * @param source_cs enum for the source coordinate system
   * @param target_cs enum for the target coordinate system
   * @param segment_nr The segment number of the desired link. If not specified,
   * the last segment of the chain is used.
   */
  void transform(KDL::Frame &source_frame, KDL::JntArray *q,
                 CoordinateSystem source_cs, CoordinateSystem target_cs,
                 int segment_nr = -1);

  /**
   * @brief Converts a KDL::Twist from one frame to another.
   * @param source_twist The KDL::Twist to convert.
   * @param q A pointer to a KDL::JntArray object representing the robot's joint
   * positions.
   * @param source_cs enum for the source coordinate system
   * @param target_cs enum for the target coordinate system
   * @param segment_nr The segment number of the desired link. If not specified,
   * the last segment of the chain is used.
   */
  void transform(KDL::Twist &source_twist, KDL::JntArray *q,
                 CoordinateSystem source_cs, CoordinateSystem target_cs,
                 int segment_nr = -1);

  /**
   * @brief Transforms a KDL::Wrench from one frame to another.
   * @param source_wrench The KDL::Wrench to convert.
   * @param q A pointer to a KDL::JntArray object representing the robot's joint
   * positions.
   * @param source_cs enum for the source coordinate system
   * @param target_cs enum for the target coordinate system
   * @param segment_nr The segment number of the desired link. If not specified,
   * the last segment of the chain is used.
   */
  void transform(KDL::Wrench &source_wrench, KDL::JntArray *q,
                 CoordinateSystem source_cs, CoordinateSystem target_cs,
                 int segment_nr = -1);

  /**
   * @brief Converts a KDL::Jacobian from one frame to another.
   * @param source_jacobian The KDL::Jacobian to convert.
   * @param q A pointer to a KDL::JntArray object representing the robot's joint
   * positions.
   * @param source_cs enum for the source coordinate system
   * @param target_cs enum for the target coordinate system
   * @param segment_nr The segment number of the desired link. If not specified,
   * the last segment of the chain is used.
   */
  void transform(KDL::Jacobian &source_jacobian, KDL::JntArray *q,
                 CoordinateSystem source_cs, CoordinateSystem target_cs,
                 int segment_nr = -1);

  /**
   * @brief Converts a KDL::JntArray from EE frame to base frame.
   * @param source_jnt_array The KDL::JntArray to convert.
   * @param q A pointer to a KDL::JntArray object representing the robot's joint
   * positions.
   * @param source_cs enum for the source coordinate system
   * @param target_cs enum for the target coordinate system
   * @param segment_nr The segment number of the desired link. If not specified,
   * the last segment of the chain is used.
   */
  void transform(KDL::JntArray &source_jnt_array, KDL::JntArray *q,
                 CoordinateSystem source_cs, CoordinateSystem target_cs,
                 int segment_nr = -1);

 private:
  KDL::Chain *_chain;
  KDL::ChainFkSolverPos_recursive *_fksolver;
};

#endif  // TF_UTILS_HPP