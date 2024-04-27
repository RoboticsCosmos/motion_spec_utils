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
};

struct MobileBase
{
  int ndrives;
  double *wheel_diameter;
  double *tau_command;
};

struct Freddy
{
  Manipulator *arm1;
  Manipulator *arm2;
  MobileBase *robile;
};

#endif  // ROB_STRUCTS_HPP