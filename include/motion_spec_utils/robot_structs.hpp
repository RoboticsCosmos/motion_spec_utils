#ifndef ROB_STRUCTS_HPP
#define ROB_STRUCTS_HPP

#include "kinova_mediator/mediator.hpp"
#include "kelo_motion_control/mediator.h"

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>

struct ManipulatorState
{
  int nj;
  int ns;
  double *q;
  double *q_dot;
  double *q_ddot;
  double **s;
  double **s_dot;
  double **s_ddot;
  double *tau_command;
  double *tau_measured;
  double *f_tool_command;
  double *f_tool_measured;
};

template <typename MediatorType>
struct Manipulator
{
  ManipulatorState *state;
  MediatorType *mediator;

  std::string base_frame;
  std::string tool_frame;

  KDL::Chain chain = KDL::Chain();
};

typedef Manipulator<kinova_mediator> KinovaManipulator;

struct MobileBaseState
{
  double *pivot_angles;
  double *wheel_encoder_values;
  double *prev_wheel_encoder_values;
  double *qd_wheel;

  double *xd_platform;
  double *x_platform;

  double *tau_command;
};

template <typename MediatorType>
struct MobileBase
{
  MobileBaseState *state;
  MediatorType *mediator;
};

struct Robile
{
  EthercatConfig *ethercat_config;
  KeloBaseConfig *kelo_base_config;
};

typedef MobileBase<Robile> RobileBase;

struct Freddy
{
  KinovaManipulator *kinova_left;
  KinovaManipulator *kinova_right;
  RobileBase *mobile_base;

  KDL::Tree tree = KDL::Tree();
};

#endif  // ROB_STRUCTS_HPP