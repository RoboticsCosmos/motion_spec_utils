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

#include "kdl/chain.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainfksolvervel_recursive.hpp"
#include "kdl/frames.hpp"
#include "kdl/jacobian.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/frames_io.hpp"
#include "kdl/kinfam_io.hpp"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "motion_spec_utils/robot_structs.hpp"

void transform_alpha(Freddy *rob, std::string source_frame, std::string target_frame,
                     double **alpha, int nc, double **transformed_alpha);

void transform_alpha(Manipulator<kinova_mediator> *rob, KDL::Tree *tree, std::string source_frame,
                     std::string target_frame, double **alpha, int nc, double **transformed_alpha);

void transform_alpha_beta(Manipulator<kinova_mediator> *rob, KDL::Tree *tree,
                          std::string source_frame, std::string target_frame, double **alpha,
                          double *beta, int nc, double **transformed_alpha,
                          double *transformed_beta);

void transform_alpha_beta(Freddy *rob, std::string source_frame, std::string target_frame,
                          double **alpha, double *beta, int nc, double **transformed_alpha,
                          double *transformed_beta);

void transform_wrench(Freddy *rob, std::string from_ent, std::string to_ent, double *wrench,
                      double *transformed_wrench);

void transform_wrench2(Freddy *rob, std::string from_ent, std::string to_ent, double *wrench,
                      double *transformed_wrench);

void transformS(Freddy *rob, std::string source_frame, std::string target_frame, double *s,
                double *s_out);

void transformSdot(Freddy *rob, std::string source_frame, std::string target_frame, double *s_dot,
                   double *s_dot_out);

void transform_with_frame(double *source_frame, double *transform, double *transformed_frame);

void findLinkInChain(std::string link_name, KDL::Chain *chain, bool &is_in_chain, int &link_id);

#endif  // TF_UTILS_HPP