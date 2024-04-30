#ifndef ROB_STRUCTS_HPP
#define ROB_STRUCTS_HPP

struct Manipulator
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

struct MobileBase
{
  double *pivot_angles;
  double *tau_command;
};

struct Freddy
{
  Manipulator *arm1;
  Manipulator *arm2;
  MobileBase *robile;
};

#endif  // ROB_STRUCTS_HPP