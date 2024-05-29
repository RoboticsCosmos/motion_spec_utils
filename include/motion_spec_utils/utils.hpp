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

#define DEG2RAD(x) ((x) * M_PI / 180.0)
#define RAD2DEG(x) ((x) * 180.0 / M_PI)

#include <ActuatorConfigClientRpc.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <KDetailedException.h>
#include <RouterClient.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>
#include <google/protobuf/util/json_util.h>

#include <array>
#include <iostream>
#include <string>
#include <vector>

#include "kdl/chain.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainfksolvervel_recursive.hpp"
#include "kdl/chainhdsolver_vereshchagin.hpp"
#include "kdl/chainhdsolver_vereshchagin_fext.hpp"
#include "kdl/chainidsolver_recursive_newton_euler.hpp"
#include "kdl/frames.hpp"
#include "kdl/jacobian.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/kinfam_io.hpp"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "kinova_mediator/mediator.hpp"
#include "motion_spec_utils/robot_structs.hpp"
#include <cstring>

struct LogControlData
{
  double measured_value;
  double reference_value;

  double control_singal;

  void populate(double measured_value, double reference_value, double control_signal)
  {
    this->measured_value = measured_value;
    this->reference_value = reference_value;
    this->control_singal = control_signal;
  }
};

struct LogControlDataVector
{
  const char *control_variable;
  std::vector<LogControlData> control_data;

  const char *log_dir;
  char *filename;
  FILE *file;

  // constructor
  LogControlDataVector(const char *control_variable, const char *log_dir)
  {
    this->control_variable = control_variable;
    this->log_dir = log_dir;

    // create the filename
    sprintf(this->filename, "%s/control_log_%s.csv", log_dir, control_variable);

    // create the file
    this->file = fopen(this->filename, "w");
    if (this->file == NULL)
    {
      std::cerr << "Error opening file: " << this->filename << std::endl;
      exit(1);
    }

    // write the header
    fprintf(file, "Reference Value,Measured Value,Control Signal\n");
  }

  // destructor
  ~LogControlDataVector()
  {
    fclose(this->file);
  }

  void addControlData(double measured_value, double reference_value, double control_signal)
  {
    LogControlData data;
    data.populate(measured_value, reference_value, control_signal);
    this->control_data.push_back(data);

    if (this->control_data.size() >= 100)
    {
      writeToOpenFile();
    }
  }

  void writeToOpenFile()
  {
    for (size_t i = 0; i < this->control_data.size(); i++)
    {
      fprintf(file, "%f,%f,%f\n", this->control_data[i].reference_value,
              this->control_data[i].measured_value, this->control_data[i].control_singal);
    }
    // clear the data
    this->control_data.clear();
  }
};

struct LogManipulatorData
{
  // kinova info
  // double q[7];
  // double q_dot[7];
  double f_tool_measured[6]{};
  double tool_pose[6]{};
  double tool_twist[6]{};
  // double tool_acc_twist[6];

  // elbow
  double elbow_pose[6]{};
  double elbow_twist[6]{};

  // achd info
  double beta[6]{};
  double tau_command[7]{};
  double f_tool_command[6]{};
  // double q_ddot[7]{};

  // add methods to populate the data
  void populateManipulatorData(Manipulator<kinova_mediator> *rob)
  {
    // std::memcpy(this->q, rob->state->q, sizeof(this->q));
    // std::memcpy(this->q_dot, rob->state->q_dot, sizeof(this->q_dot));

    std::memcpy(this->f_tool_measured, rob->state->f_tool_measured, sizeof(this->f_tool_measured));
    std::memcpy(this->tool_pose, rob->state->s[rob->state->ns - 1], sizeof(this->tool_pose));
    std::memcpy(this->tool_twist, rob->state->s_dot[rob->state->ns - 1], sizeof(this->tool_twist));

    // elbow
    std::memcpy(this->elbow_pose, rob->state->s[rob->state->ns - 4], sizeof(this->elbow_pose));
    std::memcpy(this->elbow_twist, rob->state->s_dot[rob->state->ns - 4],
                sizeof(this->elbow_twist));
  }

  void populateAchdData(double *beta, double *tau_command, double *f_tool_command, double *q_ddot)
  {
    std::memcpy(this->tau_command, tau_command, sizeof(this->tau_command));

    if (beta != nullptr)
    {
      std::memcpy(this->beta, beta, sizeof(this->beta));
    }

    // if (q_ddot != nullptr)
    // {
    //   std::memcpy(this->q_ddot, q_ddot, sizeof(this->q_ddot));
    // }

    if (f_tool_command != nullptr)
    {
      std::memcpy(this->f_tool_command, f_tool_command, sizeof(this->f_tool_command));
    }
  }

  void populate(Manipulator<kinova_mediator> *rob, double *beta, double *tau_command,
                double *f_tool_command, double *q_ddot)
  {
    populateManipulatorData(rob);
    populateAchdData(beta, tau_command, f_tool_command, q_ddot);
  }
};



struct LogDataMobileBase
{
  // mobile base info
  double pivot_angles[4];
  double platform_force[3];
  double tau_command[8];

  double platform_pose[3];
  double platform_twist[3];

  // add methods to populate the data
  void populateMobileBaseData(RobileBase *rob)
  {
    std::memcpy(this->pivot_angles, rob->state->pivot_angles, sizeof(this->pivot_angles));
  }

  void setPlatformData(double *platform_pose, double *platform_twist)
  {
    std::memcpy(this->platform_pose, platform_pose, sizeof(this->platform_pose));
    std::memcpy(this->platform_twist, platform_twist, sizeof(this->platform_twist));
  }

  void populateSolverData(double *platform_force, double *tau_command)
  {
    std::memcpy(this->platform_force, platform_force, sizeof(this->platform_force));
    std::memcpy(this->tau_command, tau_command, sizeof(this->tau_command));
  }
};

/**
 * @brief Initializes the robot state struct.
 * @param num_joints The number of joints.
 * @param num_segments The number of segments.
 * @param rob [out] The robot state struct.
 */
void initialize_manipulator_state(int num_joints, int num_segments, ManipulatorState *rob);

void free_manipulator_state(ManipulatorState *rob);

void initialize_mobile_base_state(MobileBaseState *base);

void free_mobile_base_state(MobileBaseState *base);

void free_manipulator(Manipulator<kinova_mediator> *rob);

void free_mobile_base(MobileBase<Robile> *base);

void free_robot_data(Freddy *rob);

/**
 * @brief Initializes the robot state struct.
 * @param num_joints The number of joints.
 * @param num_segments The number of segments.
 * @param init_q The initial joint positions.
 * @param rob [out] The robot state struct.
 */
// void initialize_robot_state(int num_joints, int num_segments, double *init_q,
//                             Manipulator *rob);

void initialize_robot(std::string robot_urdf, char *interface, Freddy *freddy);

void initialize_robot_sim(std::string robot_urdf, Freddy *freddy);

/**
 * @brief Initializes the robot chain.
 * @param robot_urdf The path to the robot urdf.
 * @param base_link The name of the base link.
 * @param tool_link The name of the tool link.
 * @param robot_chain [out] The KDL chain representing the robot.
 */
void initialize_robot_chain(std::string robot_urdf, std::string base_link, std::string tool_link,
                            KDL::Chain &robot_chain);

/**
 * @brief Computes forward velocity kinematics.
 * @param link_name The name of the link.
 * @param as_seen_by The name of the link from which the twist is computed.
 * @param with_respect_to The name of the link with respect to which the twist is
 * computed.
 * @param vec The direction vector.
 * @param rob Robot state struct.
 * @param robot_chain The KDL chain representing the robot.
 * @param out_twist [out] The twist of the link.
 */
// void computeForwardVelocityKinematics(std::string link_name, std::string as_seen_by,
//                                       std::string with_respect_to, double *vec,
//                                       Manipulator *rob, KDL::Chain *robot_chain,
//                                       double out_twist);

/**
 * @brief Adds two arrays.
 * @param arr1 The first array.
 * @param arr2 The second array.
 * @param result [out] The result of the addition.
 * @param size The size of the arrays.
 */
extern void add(double *arr1, double *arr2, double *result, size_t size);

/**
 * @brief Updates the q and q_dot values of the robot state struct.
 * @param q_ddot The joint accelerations.
 * @param dt The time step.
 * @param rob [in/out] The robot state struct.
 */
void updateQandQdot(double *q_ddot, double dt, ManipulatorState *rob);

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
                 double *root_acceleration, double **alpha, double *beta, double *tau_ff,
                 double *predicted_acc, double *constraint_tau);

void achd_solver_fext(Freddy *rob, std::string root_link, std::string tip_link,
                      double **ext_wrenches, double *constraint_tau);

void rne_solver(Freddy *rob, std::string root_link, std::string tip_link,
                double *root_acceleration, double **ext_wrench, double *constraint_tau);

void base_fd_solver(Freddy *rob, double *platform_force, double *wheel_torques);

void getLinkIdFromChain(KDL::Chain &chain, std::string link_name, int &link_id);

template <typename MediatorType>
void get_manipulator_data(Manipulator<MediatorType> *rob);

void update_manipulator_state(ManipulatorState *state, std::string tool_frame, KDL::Tree *tree);

void cap_and_convert_torques(double *tau_command, int num_joints, KDL::JntArray &tau_jnt);

int set_manipulator_torques(Freddy *rob, std::string root_link, KDL::JntArray *tau_command);

void set_mobile_base_torques(Freddy *rob, double *tau_command);

void getLinkSFromRob(std::string link_name, Freddy *rob, double *s);

void computeDistance(std::string *between_ents, std::string asb, Freddy *rob, double &distance);

void computeDistance1D(std::string *between_ents, double *axis, std::string asb, Freddy *rob,
                       double &distance);

void getLinkForce(std::string applied_by, std::string applied_to, std::string asb, double *vec,
                  Freddy *rob, double &force);

void getLinkPosition(std::string link_name, std::string as_seen_by, std::string with_respect_to,
                     double *vec, Freddy *rob, double &out_position);

void getLinkQuaternion(std::string link_name, std::string as_seen_by, std::string with_respect_to,
                       Freddy *rob, double *out_quaternion);

void getLinkVelocity(std::string link_name, std::string as_seen_by, std::string with_respect_to,
                     double *vec, Freddy *rob, double &out_twist);

void getLinkId(Freddy *rob, std::string root_link, std::string tip_link, std::string link_name, int &link_id);

void findLinkInChain(std::string link_name, KDL::Chain *chain, bool &is_in_chain, int &link_id);

void findVector(std::string from_ent, std::string to_ent, Freddy *rob, double *vec);

void findNormalizedVector(const double *vec, double *normalized_vec);

void decomposeSignal(Freddy *rob, const std::string from_ent, const std::string to_ent,
                     std::string asb_ent, const double signal, double *vec);

void get_robot_data(Freddy *rob);

void print_robot_data(Freddy *rob);

void get_manipulator_data_sim(Manipulator<kinova_mediator> *rob, double *param_type,
                              double time_step);

void get_robot_data_sim(Freddy *freddy, double *kinova_left_predicted_acc,
                        double *kinova_right_predicted_acc, double time_step);

void set_init_sim_data(Freddy *freddy);

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

void transformS(Freddy *rob, std::string source_frame, std::string target_frame, double *s,
                double *s_out);

void achd_solver_manipulator(Manipulator<kinova_mediator> *rob, int num_constraints,
                             double *root_acceleration, double **alpha, double *beta,
                             double *tau_ff, double *predicted_acc, double *constraint_tau);

void rne_solver_manipulator(Manipulator<kinova_mediator> *rob, double *root_acceleration,
                            double **ext_wrench, double *constraint_tau);

void print_array(double *arr, int size);

void init_2d_array(double **arr, int rows, int cols);

void plot_control_log_data(std::vector<LogControlDataVector> &log_data);

void get_new_folder_name(const char *dir_path, char *name);

void write_control_log_to_open_file(FILE *file, LogControlDataVector &log_data);

void appendArrayToStream(std::stringstream &ss, double *arr, size_t size);

struct LogManipulatorDataVector
{
  std::vector<LogManipulatorData> log_data;

  const char *log_dir;
  char *filename;
  FILE *file;

  // constructor
  LogManipulatorDataVector(const char *log_dir)
  {
    this->log_dir = log_dir;

    // create the filename
    sprintf(this->filename, "%s/manipulator_log.csv", log_dir);

    // create the file
    this->file = fopen(this->filename, "w");
    if (this->file == NULL)
    {
      std::cerr << "Error opening file: " << this->filename << std::endl;
      exit(1);
    }

    // write the header
    fprintf(file,
            "Tool Pose X,Tool Pose Y,Tool Pose Z,Tool Pose R,Tool Pose P,Tool Pose Y,"
            "Tool Twist X,Tool Twist Y,Tool Twist Z,Tool Twist R,Tool Twist P,Tool Twist Y,"
            "Elbow Pose X,Elbow Pose Y,Elbow Pose Z,Elbow Pose R,Elbow Pose P,Elbow Pose Y,"
            "Elbow Twist X,Elbow Twist Y,Elbow Twist Z,Elbow Twist R,Elbow Twist P,Elbow Twist Y,"
            "F Tool Measured X,F Tool Measured Y,F Tool Measured Z,F Tool Measured R,F Tool "
            "Measured P,F Tool Measured Y,"
            "Beta X,Beta Y,Beta Z,Beta R,Beta P,Beta Y,"
            "Tau Command 1,Tau Command 2,Tau Command 3,Tau Command 4,Tau Command 5,Tau Command "
            "6,Tau Command 7,"
            "F Tool Command X,F Tool Command Y,F Tool Command Z,F Tool Command R,F Tool Command "
            "P,F Tool Command Y\n");
  }

  // destructor
  ~LogManipulatorDataVector()
  {
    fclose(this->file);
  }

  void addManipulatorData(Manipulator<kinova_mediator> *rob, double *beta, double *tau_command,
                          double *f_tool_command, double *q_ddot)
  {
    LogManipulatorData data;
    data.populate(rob, beta, tau_command, f_tool_command, q_ddot);
    this->log_data.push_back(data);

    if (this->log_data.size() >= 100)
    {
      writeToOpenFile();
    }
  }

  void writeToOpenFile()
  {
    for (size_t i = 0; i < this->log_data.size(); i++)
    {
      // append all the data to a string
      std::stringstream ss;
      appendArrayToStream(ss, this->log_data[i].tool_pose, 6);
      appendArrayToStream(ss, this->log_data[i].tool_twist, 6);
      appendArrayToStream(ss, this->log_data[i].elbow_pose, 6);
      appendArrayToStream(ss, this->log_data[i].elbow_twist, 6);
      appendArrayToStream(ss, this->log_data[i].f_tool_measured, 6);
      appendArrayToStream(ss, this->log_data[i].beta, 6);
      appendArrayToStream(ss, this->log_data[i].tau_command, 7);
      appendArrayToStream(ss, this->log_data[i].f_tool_command, 6);

      // write the string to the file
      fprintf(file, "%s\n", ss.str().c_str());
    }
    // clear the data
    this->log_data.clear();
  }
};

#endif  // UTILS_HPP
