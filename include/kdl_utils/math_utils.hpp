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
#include <cmath>

double eq_tol = 1e-4;

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
  template <typename T>
  bool operator()(T const &a, T const &b) const
  {
    return fabs(a - b) < eq_tol;
  }
};

/**
 * @brief maps comparison operator strings to their corresponding functions
 * @param comp_op comparison operator string
 * @return function corresponding to comp_op
 */
const std::map<std::string, std::function<bool(double, double)>> comp_op_map = {
    {"lt", LessThan()}, {"gt", GreaterThan()}, {"eq", EqualTo()}};

// funcs
/**
 * @brief Compares two doubles.
 * @param measured The measured value.
 * @param setpoint The setpoint value.
 * @param threshold The threshold value.
 * @param comp_op The comparison operator.
 * @param result [out] The result of the comparison.
 */
void compare(double measured, double setpoint, double threshold, std::string comp_op,
             bool &result);

#endif  // MATH_UTILS_HPP