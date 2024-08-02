#ifndef LOG_STRUCTS_HPP
#define LOG_STRUCTS_HPP

#include <vector>
#include <iostream>
#include <cstring>

#include "motion_spec_utils/robot_structs.hpp"

void appendArrayToStream(std::stringstream &ss, double *arr, size_t size)
{
  for (size_t i = 0; i < size; i++)
  {
    ss << arr[i] << ",";
  }
}

struct LogControlData
{
  double measured_value = 0.0;
  double reference_value = 0.0;
  double error = 0.0;
  double control_singal = 0.0;

  double p, i, d, error_sum = 0.0;
  double stiffness, damping = 0.0;

  void populate(double measured_value, double reference_value, double error, double control_signal)
  {
    this->measured_value = measured_value;
    this->reference_value = reference_value;
    this->error = error;
    this->control_singal = control_signal;
  }

  void populate_pid_ctrl(double p, double i, double d, double error_sum)
  {
    this->p = p;
    this->i = i;
    this->d = d;
    this->error_sum = error_sum;
  }

  void populate_impedance_ctrl(double stiffness, double damping)
  {
    this->stiffness = stiffness;
    this->damping = damping;
  }
};

struct LogControlDataVector
{
  std::string control_variable;
  std::string controller_type;
  std::vector<LogControlData> control_data;

  std::string log_dir;
  std::string filename;
  FILE *file;

  int write_frequency = 0;

  // constructor
  LogControlDataVector(std::string control_variable, std::string controller_type,
                       std::string log_dir, int write_frequency = 50)
  {
    this->control_variable = control_variable;
    this->controller_type = controller_type;
    this->log_dir = log_dir;
    this->write_frequency = write_frequency;

    // create the filename
    this->filename = log_dir + "/control_log_" + control_variable + "_" + controller_type + ".csv";

    // create the file
    this->file = fopen(this->filename.c_str(), "w");
    if (this->file == NULL)
    {
      std::cerr << "Error opening file: " << this->filename << std::endl;
      exit(1);
    }

    // write the header
    if (controller_type == "pid")
    {
      fprintf(file, "measured_value,reference_value,error,control_signal,p,i,d,error_sum\n");
    }
    else if (controller_type == "impedance")
    {
      fprintf(file, "measured_value,reference_value,control_signal,stiffness,damping\n");
    }
  }

  // destructor
  ~LogControlDataVector()
  {
    fclose(this->file);
  }

  void addControlDataPID(double measured_value, double reference_value, double error,
                         double control_signal, double p, double i, double d, double error_sum)
  {
    LogControlData data;
    data.populate(measured_value, reference_value, error, control_signal);
    data.populate_pid_ctrl(p, i, d, error_sum);
    this->control_data.push_back(data);

    if (this->control_data.size() >= this->write_frequency)
    {
      writeToOpenFile();
    }
  }

  void addControlDataImpedance(double measured_value, double reference_value, double error,
                               double control_signal, double stiffness, double damping)
  {
    LogControlData data;
    data.populate(measured_value, reference_value, error, control_signal);
    data.populate_impedance_ctrl(stiffness, damping);
    this->control_data.push_back(data);

    if (this->control_data.size() >= 50)
    {
      writeToOpenFile();
    }
  }

  void writeToOpenFile()
  {
    for (size_t i = 0; i < this->control_data.size(); i++)
    {
      // append all the data to a string
      std::stringstream ss;
      ss << this->control_data[i].reference_value << "," << this->control_data[i].measured_value
         << "," << this->control_data[i].error << "," << this->control_data[i].control_singal
         << ",";

      if (this->controller_type == "pid")
      {
        ss << this->control_data[i].p << "," << this->control_data[i].i << ","
           << this->control_data[i].d << "," << this->control_data[i].error_sum;
      }
      else if (this->controller_type == "impedance")
      {
        ss << this->control_data[i].stiffness << "," << this->control_data[i].damping;
      }

      // write the string to the file
      fprintf(file, "%s\n", ss.str().c_str());
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
  double tool_pose[7]{};
  double tool_twist[6]{};
  // double tool_acc_twist[6];

  // elbow
  double elbow_pose[7]{};

  // arm_base
  double arm_base_pose[7]{};

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

    // arm_base
    std::memcpy(this->arm_base_pose, rob->state->s[rob->state->ns - 8],
                sizeof(this->arm_base_pose));
  }

  void populateAchdData(double *beta, double *tau_command, double *f_tool_command)
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
                double *f_tool_command)
  {
    populateManipulatorData(rob);
    populateAchdData(beta, tau_command, f_tool_command);
  }
};

struct LogManipulatorDataVector
{
  std::string arm_name;
  std::vector<LogManipulatorData> log_data;

  std::string log_dir;
  std::string filename;
  FILE *file;
  int write_frequency = 0;

  // constructor
  LogManipulatorDataVector(std::string arm_name, std::string log_dir, int write_frequncy = 50)
  {
    this->log_dir = log_dir;
    this->arm_name = arm_name;
    this->write_frequency = write_frequncy;

    // create the filename
    this->filename = log_dir + "/" + arm_name + "_log.csv";

    // create the file
    this->file = fopen(this->filename.c_str(), "w");
    if (this->file == NULL)
    {
      std::cerr << "Error opening file: " << this->filename << std::endl;
      exit(1);
    }

    // write the header
    fprintf(file,
            "ee_s_x,ee_s_y,ee_s_z,ee_s_qx,ee_s_qy,ee_s_qz,ee_s_qw,"
            "ee_twist_x,ee_twist_y,ee_twist_z,ee_twist_qx,ee_twist_qy,ee_twist_qz,"
            "elbow_s_x,elbow_s_y,elbow_s_z,elbow_s_qx,elbow_s_qy,elbow_s_qz,elbow_s_qw,"
            "arm_base_s_x,arm_base_s_y,arm_base_s_z,arm_base_s_qx,arm_base_s_qy,arm_base_s_qz,arm_"
            "base_s_qw,"
            "ee_f_e_x, ee_f_e_y, ee_f_e_z, ee_f_e_qx, ee_f_e_qy, ee_f_e_qz,"
            "ee_beta_x,ee_beta_y,ee_beta_z,ee_beta_qx,ee_beta_qy,ee_beta_qz,"
            "tau_c_1,tau_c_2,tau_c_3,tau_c_4,tau_c_5,tau_c_6,tau_c_7,"
            "ee_f_c_x,ee_f_c_y,ee_f_c_z,ee_f_c_qx,ee_f_c_qy,ee_f_c_qz\n");
  }

  // destructor
  ~LogManipulatorDataVector()
  {
    fclose(this->file);
  }

  void addManipulatorData(Manipulator<kinova_mediator> *rob, double *beta, double *tau_command,
                          double *f_tool_command)
  {
    LogManipulatorData data;
    data.populate(rob, beta, tau_command, f_tool_command);
    this->log_data.push_back(data);

    if (this->log_data.size() >= this->write_frequency)
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
      appendArrayToStream(ss, this->log_data[i].tool_pose, 7);
      appendArrayToStream(ss, this->log_data[i].tool_twist, 6);
      appendArrayToStream(ss, this->log_data[i].elbow_pose, 7);
      appendArrayToStream(ss, this->log_data[i].arm_base_pose, 7);
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

struct LogManipulatorVoltageCurrentData
{
  double base_voltage;
  double base_current;

  double actuator_voltage[7];
  double actuator_current[7];

  void populate(double base_voltage, double base_current, double *actuator_voltage,
                double *actuator_current)
  {
    this->base_voltage = base_voltage;
    this->base_current = base_current;

    std::memcpy(this->actuator_voltage, actuator_voltage, sizeof(this->actuator_voltage));
    std::memcpy(this->actuator_current, actuator_current, sizeof(this->actuator_current));
  }
};

struct LogManipulatorVoltageCurrentDataVector
{
  std::string arm_name;
  std::vector<LogManipulatorVoltageCurrentData> log_data;

  std::string log_dir;
  std::string filename;
  FILE *file;

  // constructor
  LogManipulatorVoltageCurrentDataVector(std::string arm_name, std::string log_dir)
  {
    this->log_dir = log_dir;
    this->arm_name = arm_name;

    // create the filename
    this->filename = log_dir + "/" + arm_name + "_voltage_current_log.csv";

    // create the file
    this->file = fopen(this->filename.c_str(), "w");
    if (this->file == NULL)
    {
      std::cerr << "Error opening file: " << this->filename << std::endl;
      exit(1);
    }

    // write the header
    fprintf(file,
            "base_voltage,base_current,"
            "actuator_1_voltage,actuator_2_voltage,actuator_3_voltage,actuator_4_voltage,"
            "actuator_5_voltage,actuator_6_voltage,actuator_7_voltage,"
            "actuator_1_current,actuator_2_current,actuator_3_current,actuator_4_current,"
            "actuator_5_current,actuator_6_current,actuator_7_current\n");
  }

  // destructor
  ~LogManipulatorVoltageCurrentDataVector()
  {
    fclose(this->file);
  }

  void addVoltageCurrentData(double base_voltage, double base_current, double *actuator_voltage,
                             double *actuator_current)
  {
    LogManipulatorVoltageCurrentData data;
    data.populate(base_voltage, base_current, actuator_voltage, actuator_current);
    this->log_data.push_back(data);

    if (this->log_data.size() >= 50)
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
      ss << this->log_data[i].base_voltage << "," << this->log_data[i].base_current << ",";
      appendArrayToStream(ss, this->log_data[i].actuator_voltage, 7);
      appendArrayToStream(ss, this->log_data[i].actuator_current, 7);

      // write the string to the file
      fprintf(file, "%s\n", ss.str().c_str());
    }
    // clear the data
    this->log_data.clear();
  }
};

struct LogMobileBaseData
{
  // mobile base info
  double pivot_angles[4];
  double platform_force[3];
  double tau_command[8];

  double x_platform[3];
  double xd_platform[3];

  // add methods to populate the data
  void populateMobileBaseData(RobileBase *rob)
  {
    std::memcpy(this->pivot_angles, rob->state->pivot_angles, sizeof(this->pivot_angles));
  }

  void setPlatformData(double *x_platform, double *xd_platform)
  {
    std::memcpy(this->x_platform, x_platform, sizeof(this->x_platform));
    std::memcpy(this->xd_platform, xd_platform, sizeof(this->xd_platform));
  }

  void populateSolverData(double *platform_force, double *tau_command)
  {
    if (platform_force != nullptr) std::memcpy(this->platform_force, platform_force, sizeof(this->platform_force));
    if (tau_command != nullptr) std::memcpy(this->tau_command, tau_command, sizeof(this->tau_command));
  }
};

struct LogMobileBaseDataVector
{
  std::vector<LogMobileBaseData> log_data;

  std::string log_dir;
  std::string filename;
  FILE *file;
  int write_frequency = 0;

  // constructor
  LogMobileBaseDataVector(std::string log_dir, int write_freuqency = 50)
  {
    this->log_dir = log_dir;
    this->write_frequency = write_freuqency;

    // create the filename
    this->filename = log_dir + "/mobile_base_log.csv";

    // create the file
    this->file = fopen(this->filename.c_str(), "w");
    if (this->file == NULL)
    {
      std::cerr << "Error opening file: " << this->filename << std::endl;
      exit(1);
    }

    // write the header
    fprintf(
        file,
        "pivot_1,pivot_2,pivot_3,pivot_4,platform_force_x,platform_force_y,platform_force_z,"
        "tau_c_1,tau_c_2,tau_c_3,tau_c_4,tau_c_5,tau_c_6,tau_c_7,tau_c_8,"
        "x_platform_x,x_platform_y,x_platform_qz,xd_platform_x,xd_platform_y,xd_platform_qz\n");
  }

  // destructor
  ~LogMobileBaseDataVector()
  {
    fclose(this->file);
  }

  void addMobileBaseData(RobileBase *rob, double *x_platform, double *xd_platform, double *platform_force,
                         double *tau_command)
  {
    LogMobileBaseData data;
    data.populateMobileBaseData(rob);
    data.setPlatformData(x_platform, xd_platform);
    data.populateSolverData(platform_force, tau_command);
    this->log_data.push_back(data);

    if (this->log_data.size() >= this->write_frequency)
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
      appendArrayToStream(ss, this->log_data[i].pivot_angles, 4);
      appendArrayToStream(ss, this->log_data[i].platform_force, 3);
      appendArrayToStream(ss, this->log_data[i].tau_command, 8);
      appendArrayToStream(ss, this->log_data[i].x_platform, 3);
      appendArrayToStream(ss, this->log_data[i].xd_platform, 3);

      // write the string to the file
      fprintf(file, "%s\n", ss.str().c_str());
    }
    // clear the data
    this->log_data.clear();
  }
};

struct LogMobileBaseVoltageCurrentData
{
  double bus_voltages[4];

  double actuator_voltage[8];
  double actuator_current[8];

  void populate(double *bus_voltages, double *actuator_voltage, double *actuator_current)
  {
    std::memcpy(this->bus_voltages, bus_voltages, sizeof(this->bus_voltages));
    std::memcpy(this->actuator_voltage, actuator_voltage, sizeof(this->actuator_voltage));
    std::memcpy(this->actuator_current, actuator_current, sizeof(this->actuator_current));
  }
};

struct LogMobileBaseVoltageCurrentDataVector
{
  std::vector<LogMobileBaseVoltageCurrentData> log_data;

  std::string log_dir;
  std::string filename;
  FILE *file;

  // constructor
  LogMobileBaseVoltageCurrentDataVector(std::string log_dir)
  {
    this->log_dir = log_dir;

    // create the filename
    this->filename = log_dir + "/mobile_base_voltage_current_log.csv";

    // create the file
    this->file = fopen(this->filename.c_str(), "w");
    if (this->file == NULL)
    {
      std::cerr << "Error opening file: " << this->filename << std::endl;
      exit(1);
    }

    // write the header
    fprintf(file,
            "bus_voltage_1,bus_voltage_2,bus_voltage_3,bus_voltage_4,"
            "actuator_1_voltage,actuator_2_voltage,actuator_3_voltage,actuator_4_voltage,"
            "actuator_5_voltage,actuator_6_voltage,actuator_7_voltage,actuator_8_voltage,"
            "actuator_1_current,actuator_2_current,actuator_3_current,actuator_4_current,"
            "actuator_5_current,actuator_6_current,actuator_7_current,actuator_8_current\n");
  }

  // destructor
  ~LogMobileBaseVoltageCurrentDataVector()
  {
    fclose(this->file);
  }

  void addVoltageCurrentData(double *bus_voltages, double *actuator_voltage,
                             double *actuator_current)
  {
    LogMobileBaseVoltageCurrentData data;
    data.populate(bus_voltages, actuator_voltage, actuator_current);
    this->log_data.push_back(data);

    if (this->log_data.size() >= 50)
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
      appendArrayToStream(ss, this->log_data[i].bus_voltages, 4);
      appendArrayToStream(ss, this->log_data[i].actuator_voltage, 8);
      appendArrayToStream(ss, this->log_data[i].actuator_current, 8);

      // write the string to the file
      fprintf(file, "%s\n", ss.str().c_str());
    }
    // clear the data
    this->log_data.clear();
  }
};

#endif  // LOG_STRUCTS_HPP