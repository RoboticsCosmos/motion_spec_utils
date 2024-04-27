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

#ifndef MATH_UTILS_HPP
#define MATH_UTILS_HPP

#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "kdl/frames.hpp"

#include "motion_spec_utils/fixed_size_queue.hpp"

class MathUtils
{
public:
  MathUtils(double eq_tol = 0.001);

  ~MathUtils();

  /**
   * @brief compare Twists
   * @param current_twist a KDL::Twist
   * @param double thresh a threshold value
   * @param comp_op comparison operator string
   * @return a vector of bools, one for each element of the twist
   */
  std::vector<bool> compare(KDL::Twist current_twist, double thresh,
                            std::string comp_op);

  /**
   * @brief compare Twists
   * @param current_twist a KDL::Twist
   * @param target_twist a KDL::Twist
   * @param comp_op comparison operator string
   * @return a vector of bools, one for each element of the twist
   */
  std::vector<bool> compare(KDL::Twist current_twist, KDL::Twist target_twist,
                            std::string comp_op);

  /**
   * @brief compare Twists
   * @param current_twist a KDL::Twist
   * @param target_twist a KDL::Twist
   * @param thresh a threshold value
   * @param comp_op comparison operator string
   * @return a vector of bools, one for each element of the twist
   */
  std::vector<bool> compare(KDL::Twist current_twist, KDL::Twist target_twist,
                            double thresh, std::string comp_op);

  /**
   * @brief compare Frames
   * @param current_frame a KDL::Frame
   * @param target_frame a KDL::Frame
   * @param comp_op comparison operator string
   * @return a vector of bools, one for each element of the frame
   */
  std::vector<bool> compare(KDL::Frame current_frame, KDL::Frame target_frame,
                            std::string comp_op);

  /**
   * @brief Computes average of a queue of KDL::Twists
   * @param queue a FixedSizeQueue queue of KDL::Twists
   * @return a KDL::Twist
   */
  KDL::Twist computeAverage(FixedSizeQueue<KDL::Twist, 10> *queue);

private:
  double _eq_tol;
  // Functions for different comparison operators

  /**
   * @brief checks if a < b
   * @param a
   * @param b
   */
  struct LessThan
  {
    template <typename T>
    bool operator()(T const &a, T const &b) const
    {
      return fabs(a) < fabs(b);
    }
  };

  /**
   * @brief checks if a > b
   * @param a
   * @param b
   */
  struct GreaterThan
  {
    template <typename T>
    bool operator()(T const &a, T const &b) const
    {
      return fabs(a) > fabs(b);
    }
  };

  /**
   * @brief checks if a == b (within a tolerance)
   * @param a
   * @param b
   */
  struct EqualTo
  {
    EqualTo(double eq_tol) : _eq_tol(eq_tol) {}
    template <typename T>
    bool operator()(T const &a, T const &b) const
    {
      return fabs(a - b) < _eq_tol;
    }

  private:
    double _eq_tol;
  };

  /**
   * @brief maps comparison operator strings to their corresponding functions
   * @param comp_op comparison operator string
   * @return function corresponding to comp_op
   */
  const std::map<std::string, std::function<bool(double, double)>> comp_op_map = {
      {"lt", LessThan()}, {"gt", GreaterThan()}, {"eq", EqualTo(_eq_tol)}};
};

#endif  // MATH_UTILS_HPP