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

#include "kdl_utils/utils_old.hpp"

Utils::Utils() {}

Utils::~Utils() {}

void Utils::printLinkNames(KDL::Tree& tree) {
  std::vector<std::string> link_names;

  // get all segments
  KDL::SegmentMap segments = tree.getSegments();

  // iterate through all segments
  for (auto segment : segments) {
    // get the name of the segment
    std::string name = segment.first;

    // add the name to the vector
    link_names.push_back(name);
  }

  // print the link names
  std::cout << "Link names: " << std::endl;
  for (int i = 0; i < link_names.size(); i++) {
    std::cout << link_names[i] << std::endl;
  }
  std::cout << std::endl;
}

std::vector<std::string> Utils::getLinkNamesFromChain(KDL::Chain& chain) {
  std::vector<std::string> link_names;
  for (int i = 0; i < chain.getNrOfSegments(); i++) {
    link_names.push_back(chain.getSegment(i).getName());
  }

  return link_names;
}

int Utils::getLinkIdFromChain(KDL::Chain& chain, const std::string& link_name) {
  for (int i = 0; i < chain.getNrOfSegments(); i++) {
    if (chain.getSegment(i).getName() == link_name) {
      return i;
    }
  }

  return -1;
}

bool Utils::checkLinkInChain(KDL::Chain& chain, const std::string& link_name) {
  std::vector<std::string> link_names = getLinkNamesFromChain(chain);

  for (int i = 0; i < link_names.size(); i++) {
    if (link_names[i] == link_name) {
      return true;
    }
  }

  return false;
}

void Utils::printJointNames(KDL::Chain& chain) {
  std::vector<std::string> joint_names;
  for (int i = 0; i < chain.getNrOfSegments(); i++) {
    joint_names.push_back(chain.getSegment(i).getJoint().getName());
  }

  std::cout << "Joint names: " << std::endl;
  for (int i = 0; i < joint_names.size(); i++) {
    std::cout << joint_names[i] << std::endl;
  }
  std::cout << std::endl;
}

template <typename T>
void Utils::printVec(const T& vec) {
  std::cout << "[";
  for (int i = 0; i < vec.size(); ++i) {
    std::cout << vec[i] << ", ";
    if (i != vec.size() - 1) {
      std::cout << ", ";
    }
  }
  std::cout << "]" << std::endl;
}

template <typename T>
void Utils::printJntArr(const T& jntArr) {
  std::cout << "[";
  for (int i = 0; i < jntArr.rows(); ++i) {
    std::cout << jntArr(i);
    if (i != jntArr.rows() - 1) {
      std::cout << ", ";
    }
  }
  std::cout << "]" << std::endl;
}

int Utils::initialize_robot_urdf(const std::string& urdf_path,
                                 KDL::Chain& robot_chain,
                                 const std::string& base_link,
                                 const std::string& tool_link) {
  KDL::Tree robot_tree;

  // load the robot URDF into the KDL tree
  if (!kdl_parser::treeFromFile(urdf_path, robot_tree)) {
    std::cout << "Failed to construct KDL tree" << std::endl;
    return -1;
  }

  // create the KDL chain
  if (!robot_tree.getChain(base_link, tool_link, robot_chain)) {
    std::cout << "Failed to get KDL chain" << std::endl;
    return -1;
  }

  std::cout << "Successfully initialized robot urdf" << std::endl;
  return 0;
}

int Utils::init_q(KDL::Chain* robot_chain, KDL::JntArray& q,
                  const std::vector<double>& initial_joint_angles, ENV env) {
  if (env == ENV::SIM) {
    // set the initial joint angles
    q.resize(robot_chain->getNrOfJoints());
    for (int i = 0; i < robot_chain->getNrOfJoints(); i++) {
      q(i) = initial_joint_angles[i];
    }
  }

  std::cout << "Successfully initialized joint angles" << std::endl;
  return 0;
}

double Utils::computeEuclideanDistance(const std::array<double, 3>& current,
                                       const std::array<double, 3>& target) {
  double x_diff = target[0] - current[0];
  double y_diff = target[1] - current[1];
  double z_diff = target[2] - current[2];

  return sqrt(pow(x_diff, 2) + pow(y_diff, 2) + pow(z_diff, 2));
}

std::vector<double> Utils::calc_error(const std::array<double, 3>& p1,
                                      const std::array<double, 3>& p2) {
  double dx = p2[0] - p1[0];
  double dy = p2[1] - p1[1];
  double dz = p2[2] - p1[2];

  return {dx, dy, dz};
}

KDL::Vector Utils::calc_error(const KDL::Vector& v1, const KDL::Vector& v2) {
  return v2 - v1;
}
