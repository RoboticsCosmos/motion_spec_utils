/**
 * Author: Vamsi Kalagaturu
 *
 * Description: Library to perform math operations on KDL data types
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

#include "kdl_utils/math_utils.hpp"

MathUtils::MathUtils(double eq_tol) : _eq_tol(eq_tol){};

MathUtils::~MathUtils(){};

std::vector<bool> MathUtils::compare(KDL::Twist current_twist, double thresh,
                                     std::string comp_op) {
  // get the operator function
  std::function<bool(double, double)> op = comp_op_map.at(comp_op);

  std::vector<bool> result;

  // compare linear velocities
  for (int i = 0; i < 3; i++) {
    // if infinities are involved, just return true
    if (std::isinf(current_twist.vel[i]) || std::isinf(thresh)) {
      result.push_back(true);
      continue;
    }
    result.push_back(op(current_twist.vel[i], thresh));
  }

  // TODO: compare angular velocities
  for (int i = 0; i < 3; i++) {
    // if infinities are involved, just return true
    if (std::isinf(current_twist.rot[i]) || std::isinf(thresh)) {
      result.push_back(true);
      continue;
    }
    result.push_back(op(current_twist.rot[i], thresh));
  }

  return result;
}

std::vector<bool> MathUtils::compare(KDL::Twist current_twist,
                                     KDL::Twist target_twist,
                                     std::string comp_op) {
  // get the operator function
  std::function<bool(double, double)> op = comp_op_map.at(comp_op);

  std::vector<bool> result;

  // compare linear velocities
  for (int i = 0; i < 3; i++) {
    result.push_back(op(current_twist.vel[i], target_twist.vel[i]));
  }

  // TODO: compare angular velocities
  for (int i = 0; i < 3; i++) {
    result.push_back(op(current_twist.rot[i], target_twist.rot[i]));
  }

  return result;
}

std::vector<bool> MathUtils::compare(KDL::Twist current_twist,
                                     KDL::Twist target_twist, double thresh,
                                     std::string comp_op) {
  // get the operator function
  std::function<bool(double, double)> op = comp_op_map.at(comp_op);

  std::vector<bool> result;

  // compare linear velocities
  for (int i = 0; i < 3; i++) {
    if (std::isinf(target_twist.vel[i]) || std::isinf(thresh)) {
      result.push_back(true);
      continue;
    }
    result.push_back(op(current_twist.vel[i] + thresh, target_twist.vel[i]));
  }

  // TODO: compare angular velocities
  for (int i = 0; i < 3; i++) {
    if (std::isinf(target_twist.rot[i]) || std::isinf(thresh)) {
      result.push_back(true);
      continue;
    }
    result.push_back(op(current_twist.rot[i] + thresh, target_twist.rot[i]));
  }

  return result;
}

std::vector<bool> MathUtils::compare(KDL::Frame current_frame,
                                     KDL::Frame target_frame,
                                     std::string comp_op) {
  // get the operator function
  std::function<bool(double, double)> op = comp_op_map.at(comp_op);

  std::vector<bool> result;

  // compare position
  for (int i = 0; i < 3; i++) {
    result.push_back(op(current_frame.p[i], target_frame.p[i]));
  }

  // compare orientation
  // TOOD: figure out how to compare orientations

  return result;
}

KDL::Twist MathUtils::computeAverage(FixedSizeQueue<KDL::Twist, 10> *queue) {
  KDL::Twist sum = KDL::Twist::Zero();
  for (int i = 0; i < queue->size(); i++) {
    sum += queue->at(i);
  }
  return sum / queue->size();
}
