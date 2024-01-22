/**
 * Author: Vamsi Kalagaturu
 * 
 * Description: Library to perform frame transformations for the arm_actions package
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

#include "kdl_utils/tf_utils.hpp"

TfUtils::TfUtils() : _fksolver(nullptr){}

TfUtils::~TfUtils() 
{
  if (_fksolver) {
    delete _fksolver;
  }
}

void TfUtils::setChain(KDL::Chain *chain)
{
  if (_fksolver) {
    delete _fksolver; // Clean up previous solver if it exists
  }

  // Allocate a new solver
  _fksolver = new KDL::ChainFkSolverPos_recursive(*chain);
}

void TfUtils::transform(KDL::Frame &source_frame, KDL::JntArray *q,
                           CoordinateSystem source_cs, CoordinateSystem target_cs, int segment_nr)
{
  KDL::Frame target_frame;
  switch (source_cs)
  {
    case CoordinateSystem::BASE:
      switch (target_cs)
      {
        case CoordinateSystem::BASE:
          break;
        case CoordinateSystem::EE:
          std::cout << "Converting from base to ee frame" << std::endl;
          _fksolver->JntToCart(*q, target_frame, segment_nr);
          source_frame = source_frame * target_frame.Inverse();
          break;
      }
      break;
    case CoordinateSystem::EE:
      switch (target_cs)
      {
        case CoordinateSystem::BASE:
          std::cout << "Converting from ee to base frame" << std::endl;
          _fksolver->JntToCart(*q, target_frame, segment_nr);
          source_frame = source_frame * target_frame;
          break;
        case CoordinateSystem::EE:
          break;
      }
      break;
  }
}

void TfUtils::transform(KDL::Twist &source_twist, KDL::JntArray *q,
                    CoordinateSystem source_cs, CoordinateSystem target_cs, int segment_nr)
{
  KDL::Frame target_frame;
  switch (source_cs)
  {
    case CoordinateSystem::BASE:
      switch (target_cs)
      {
        case CoordinateSystem::BASE:
          break;
        case CoordinateSystem::EE:
          std::cout << "Converting from base to ee frame" << std::endl;
          _fksolver->JntToCart(*q, target_frame, segment_nr);
          source_twist = target_frame.Inverse() * source_twist;
          break;
      }
      break;
    case CoordinateSystem::EE:
      switch (target_cs)
      {
        case CoordinateSystem::BASE:
          std::cout << "Converting from ee to base frame" << std::endl;
          _fksolver->JntToCart(*q, target_frame, segment_nr);
          source_twist = target_frame * source_twist;
          break;
        case CoordinateSystem::EE:
          break;
      }
      break;
  }
}

void TfUtils::transform(KDL::Wrench &source_wrench, KDL::JntArray *q,
                    CoordinateSystem source_cs, CoordinateSystem target_cs, int segment_nr)
{
  KDL::Frame target_frame;
  switch (source_cs)
  {
    case CoordinateSystem::BASE:
      switch (target_cs)
      {
        case CoordinateSystem::BASE:
          break;
        case CoordinateSystem::EE:
          std::cout << "Converting from base to ee frame" << std::endl;
          _fksolver->JntToCart(*q, target_frame, segment_nr);
          source_wrench = target_frame.Inverse() * source_wrench;
          break;
      }
      break;
    case CoordinateSystem::EE:
      switch (target_cs)
      {
        case CoordinateSystem::BASE:
          std::cout << "Converting from ee to base frame" << std::endl;
          _fksolver->JntToCart(*q, target_frame, segment_nr);
          source_wrench = target_frame * source_wrench;
          break;
        case CoordinateSystem::EE:
          break;
      }
      break;
  }
}

void TfUtils::transform(KDL::Jacobian &source_jacobian, KDL::JntArray *q,
                       CoordinateSystem source_cs, CoordinateSystem target_cs, int segment_nr)
{
  KDL::Frame target_frame;
  switch (source_cs)
  {
    case CoordinateSystem::BASE:
      switch (target_cs)
      {
        case CoordinateSystem::BASE:
          break;
        case CoordinateSystem::EE:
          std::cout << "Converting from base to ee frame" << std::endl;
          _fksolver->JntToCart(*q, target_frame, segment_nr);
          for (int i = 0; i < source_jacobian.columns(); i++)
          {
            source_jacobian.setColumn(i, target_frame.Inverse() * source_jacobian.getColumn(i));
          }
          break;
      }
      break;
    case CoordinateSystem::EE:
      switch (target_cs)
      {
        case CoordinateSystem::BASE:
          std::cout << "Converting from ee to base frame" << std::endl;
          _fksolver->JntToCart(*q, target_frame, segment_nr);
          for (int i = 0; i < source_jacobian.columns(); i++)
          {
            source_jacobian.setColumn(i, target_frame * source_jacobian.getColumn(i));
          }
          break;
        case CoordinateSystem::EE:
          break;
      }
      break;
  }
}

void TfUtils::transform(KDL::JntArray &source_jnt_array, KDL::JntArray *q,
                       CoordinateSystem source_cs, CoordinateSystem target_cs, int segment_nr)
{
  KDL::Frame target_frame;
  KDL::Vector source_jnt_array_vector;
  switch (source_cs)
  {
    case CoordinateSystem::BASE:
      switch (target_cs)
      {
        case CoordinateSystem::BASE:
          break;
        case CoordinateSystem::EE:
          std::cout << "Converting from base to ee frame" << std::endl;
          _fksolver->JntToCart(*q, target_frame, segment_nr);
          // multiply the source_jnt_array with the inverse of the target_frame
          // convert the jnt_array to a vector
          
          for (int i = 0; i < source_jnt_array.rows(); i++)
          {
            source_jnt_array_vector(i) = source_jnt_array(i);
          }
          // multiply the vector with the inverse of the target_frame
          source_jnt_array_vector = target_frame.Inverse() * source_jnt_array_vector;
          // convert the vector back to a jnt_array
          for (int i = 0; i < source_jnt_array.rows(); i++)
          {
            source_jnt_array(i) = source_jnt_array_vector(i);
          }
          break;
      }
      break;
    case CoordinateSystem::EE:
      switch (target_cs)
      {
        case CoordinateSystem::BASE:
          std::cout << "Converting from ee to base frame" << std::endl;
          _fksolver->JntToCart(*q, target_frame, segment_nr);
          // multiply the source_jnt_array with the target_frame
          // convert the jnt_array to a vector
          for (int i = 0; i < source_jnt_array.rows(); i++)
          {
            source_jnt_array_vector(i) = source_jnt_array(i);
          }
          // multiply the vector with the target_frame
          source_jnt_array_vector = target_frame * source_jnt_array_vector;
          // convert the vector back to a jnt_array
          for (int i = 0; i < source_jnt_array.rows(); i++)
          {
            source_jnt_array(i) = source_jnt_array_vector(i);
          }
        case CoordinateSystem::EE:
          break;
      }
      break;
  }
}