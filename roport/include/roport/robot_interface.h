#ifndef ROPORT_ROBOT_INTERFACE_H
#define ROPORT_ROBOT_INTERFACE_H

#include <array>
#include <atomic>
#include <functional>
#include <mutex>
#include <string>

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>

#include "roport/webots_interface.h"


using namespace hardware_interface;

namespace roport
{
  class Duration {
  public:
    /**
     * Creates a new Duration instance with zero milliseconds.
     */
    Duration() noexcept;

    /**
     * Creates a new Duration instance from the given number of milliseconds.
     *
     * @param[in] milliseconds Number of milliseconds.
     */
    explicit Duration(uint64_t milliseconds) noexcept;

    /**
     * Creates a new Duration instance from an std::chrono::duration.
     *
     * @param[in] duration Duration.
     */
    Duration(std::chrono::duration<uint64_t, std::milli> duration) noexcept;

    /**
     * Creates a copy of a Duration instance.
     */
    Duration(const Duration&) = default;

    /**
     * Assigns the contents of one Duration to another.
     *
     * @return Result of the operation.
     */
    Duration& operator=(const Duration&) = default;

    /**
     * Returns the stored duration as an std::chrono::duration.
     *
     * @return Duration as std::chrono::duration.
     */
    operator std::chrono::duration<uint64_t, std::milli>() const noexcept;

    /**
     * Returns the stored duration in \f$[s]\f$.
     *
     * @return Duration in \f$[s]\f$.
     */
    double toSec() const noexcept;

    /**
     * Returns the stored duration in \f$[ms]\f$.
     *
     * @return Duration in \f$[ms]\f$.
     */
    uint64_t toMSec() const noexcept;

    /**
     * @name Arithmetic operators
     * @{
     */

    /**
     * Performs addition.
     *
     * @param[in] rhs Right-hand side of the operation.
     *
     * @return Result of the operation.
     */
    Duration operator+(const Duration& rhs) const noexcept;
    /**
     * Performs addition.
     *
     * @param[in] rhs Right-hand side of the operation.
     *
     * @return This duration.
     */
    Duration& operator+=(const Duration& rhs) noexcept;

    /**
     * Performs subtraction.
     *
     * @param[in] rhs Right-hand side of the operation.
     *
     * @return Result of the operation.
     */
    Duration operator-(const Duration& rhs) const noexcept;
    /**
     * Performs subtraction.
     *
     * @param[in] rhs Right-hand side of the operation.
     *
     * @return This duration.
     */
    Duration& operator-=(const Duration& rhs) noexcept;

    /**
     * Performs multiplication.
     *
     * @param[in] rhs Right-hand side of the operation.
     *
     * @return Result of the operation.
     */
    Duration operator*(uint64_t rhs) const noexcept;
    /**
     * Performs multiplication.
     *
     * @param[in] rhs Right-hand side of the operation.
     *
     * @return This duration.
     */
    Duration& operator*=(uint64_t rhs) noexcept;

    /**
     * Performs division.
     *
     * @param[in] rhs Right-hand side of the operation.
     *
     * @return Result of the operation.
     */
    uint64_t operator/(const Duration& rhs) const noexcept;
    /**
     * Performs division.
     *
     * @param[in] rhs Right-hand side of the operation.
     *
     * @return Result of the operation.
     */
    Duration operator/(uint64_t rhs) const noexcept;
    /**
     * Performs division.
     *
     * @param[in] rhs Right-hand side of the operation.
     *
     * @return This duration.
     */
    Duration& operator/=(uint64_t rhs) noexcept;

    /**
     * Performs the modulo operation.
     *
     * @param[in] rhs Right-hand side of the operation.
     *
     * @return Result of the operation.
     */
    Duration operator%(const Duration& rhs) const noexcept;
    /**
     * Performs the modulo operation.
     *
     * @param[in] rhs Right-hand side of the operation.
     *
     * @return Result of the operation.
     */
    Duration operator%(uint64_t rhs) const noexcept;
    /**
     * Performs the modulo operation.
     *
     * @param[in] rhs Right-hand side of the operation.
     *
     * @return This duration.
     */
    Duration& operator%=(const Duration& rhs) noexcept;
    /**
     * Performs the modulo operation.
     *
     * @param[in] rhs Right-hand side of the operation.
     *
     * @return This duration.
     */
    Duration& operator%=(uint64_t rhs) noexcept;

    /**
     * @}
     */

    /**
     * @name Comparison operators
     * @{
     */

    /**
     * Compares two durations for equality.
     *
     * @param[in] rhs Right-hand side of the comparison.
     *
     * @return True if the duration are equal, false otherwise.
     */
    bool operator==(const Duration& rhs) const noexcept;
    /**
     * Compares two durations for inequality.
     *
     * @param[in] rhs Right-hand side of the comparison.
     *
     * @return True if the duration are not equal, false otherwise.
     */
    bool operator!=(const Duration& rhs) const noexcept;

    /**
     * Compares two durations.
     *
     * @param[in] rhs Right-hand side of the comparison.
     *
     * @return True if this duration is shorter than rhs, false otherwise.
     */
    bool operator<(const Duration& rhs) const noexcept;
    /**
     * Compares two durations.
     *
     * @param[in] rhs Right-hand side of the comparison.
     *
     * @return True if this duration is shorter than or equal to rhs, false otherwise.
     */
    bool operator<=(const Duration& rhs) const noexcept;

    /**
     * Compares two durations.
     *
     * @param[in] rhs Right-hand side of the comparison.
     *
     * @return True if this duration is longer than rhs, false otherwise.
     */
    bool operator>(const Duration& rhs) const noexcept;
    /**
     * Compares two durations.
     *
     * @param[in] rhs Right-hand side of the comparison.
     *
     * @return True if this duration is longer than or equal to rhs, false otherwise.
     */
    bool operator>=(const Duration& rhs) const noexcept;

    /**
     * @}
     */

  private:
    std::chrono::duration<uint64_t, std::milli> duration_;
  };

  /**
   * Describes the robot state.
   */
  struct RobotState {
    /**
     * \f$^{O}T_{EE}\f$
     * Measured end effector pose in base frame.
     * Pose is represented as a 4x4 matrix in column-major format.
     */
    std::array<double, 16> O_T_EE{};  // NOLINT(readability-identifier-naming)

    /**
     * \f${^OT_{EE}}_{d}\f$
     * Last desired end effector pose of motion generation in base frame.
     * Pose is represented as a 4x4 matrix in column-major format.
     */
    std::array<double, 16> O_T_EE_d{};  // NOLINT(readability-identifier-naming)

    /**
     * \f$^{F}T_{EE}\f$
     * End effector frame pose in flange frame.
     * Pose is represented as a 4x4 matrix in column-major format.
     *
     * @see F_T_NE
     * @see NE_T_EE
     * @see Robot for an explanation of the NE and EE frames.
     */
    std::array<double, 16> F_T_EE{};  // NOLINT(readability-identifier-naming)

    /**
     * \f$^{F}T_{NE}\f$
     * Nominal end effector frame pose in flange frame.
     * Pose is represented as a 4x4 matrix in column-major format.
     *
     * @see F_T_EE
     * @see NE_T_EE
     * @see Robot for an explanation of the NE and EE frames.
     */
    std::array<double, 16> F_T_NE{};  // NOLINT(readability-identifier-naming)

    /**
     * \f$^{NE}T_{EE}\f$
     * End effector frame pose in nominal end effector frame.
     * Pose is represented as a 4x4 matrix in column-major format.
     *
     * @see Robot::setEE to change this frame.
     * @see F_T_EE
     * @see F_T_NE
     * @see Robot for an explanation of the NE and EE frames.
     */
    std::array<double, 16> NE_T_EE{};  // NOLINT(readability-identifier-naming)

    /**
     * \f$^{EE}T_{K}\f$
     * Stiffness frame pose in end effector frame.
     * Pose is represented as a 4x4 matrix in column-major format.
     *
     * See also @ref k-frame "K frame".
     */
    std::array<double, 16> EE_T_K{};  // NOLINT(readability-identifier-naming)

    /**
     * \f$m_{EE}\f$
     * Configured mass of the end effector.
     */
    double m_ee{};

    /**
     * \f$I_{EE}\f$
     * Configured rotational inertia matrix of the end effector load with respect to center of mass.
     */
    std::array<double, 9> I_ee{};  // NOLINT(readability-identifier-naming)

    /**
     * \f$^{F}x_{C_{EE}}\f$
     * Configured center of mass of the end effector load with respect to flange frame.
     */
    std::array<double, 3> F_x_Cee{};  // NOLINT(readability-identifier-naming)

    /**
     * \f$m_{load}\f$
     * Configured mass of the external load.
     */
    double m_load{};

    /**
     * \f$I_{load}\f$
     * Configured rotational inertia matrix of the external load with respect to center of mass.
     */
    std::array<double, 9> I_load{};  // NOLINT(readability-identifier-naming)

    /**
     * \f$^{F}x_{C_{load}}\f$
     * Configured center of mass of the external load with respect to flange frame.
     */
    std::array<double, 3> F_x_Cload{};  // NOLINT(readability-identifier-naming)

    /**
     * \f$m_{total}\f$
     * Sum of the mass of the end effector and the external load.
     */
    double m_total{};

    /**
     * \f$I_{total}\f$
     * Combined rotational inertia matrix of the end effector load and the external load with respect
     * to the center of mass.
     */
    std::array<double, 9> I_total{};  // NOLINT(readability-identifier-naming)

    /**
     * \f$^{F}x_{C_{total}}\f$
     * Combined center of mass of the end effector load and the external load with respect to flange
     * frame.
     */
    std::array<double, 3> F_x_Ctotal{};  // NOLINT(readability-identifier-naming)

    /**
     * Elbow configuration.
     *
     * The values of the array are:
     *  - [0] Position of the 3rd joint in [rad].
     *  - [1] Sign of the 4th joint. Can be +1 or -1.
     */
    std::array<double, 2> elbow{};

    /**
     * Desired elbow configuration.
     *
     * The values of the array are:
     *  - [0] Position of the 3rd joint in [rad].
     *  - [1] Sign of the 4th joint. Can be +1 or -1.
     */
    std::array<double, 2> elbow_d{};

    /**
     * Commanded elbow configuration.
     *
     * The values of the array are:
     *  - [0] Position of the 3rd joint in [rad].
     *  - [1] Sign of the 4th joint. Can be +1 or -1.
     */
    std::array<double, 2> elbow_c{};

    /**
     * Commanded elbow velocity.
     *
     * The values of the array are:
     *  - [0] Velocity of the 3rd joint in [rad/s].
     *  - [1] Sign of the 4th joint. Can be +1 or -1.
     */
    std::array<double, 2> delbow_c{};

    /**
     * Commanded elbow acceleration.
     *
     * The values of the array are:
     *  - [0] Acceleration of the 3rd joint in [rad/s^2].
     *  - [1] Sign of the 4th joint. Can be +1 or -1.
     */
    std::array<double, 2> ddelbow_c{};

    /**
     * \f$\tau_{J}\f$
     * Measured link-side joint torque sensor signals. Unit: \f$[Nm]\f$
     */
    std::vector<double> tau_J{};  // NOLINT(readability-identifier-naming)

    /**
     * \f${\tau_J}_d\f$
     * Desired link-side joint torque sensor signals without gravity. Unit: \f$[Nm]\f$
     */
    std::array<double, 7> tau_J_d{};  // NOLINT(readability-identifier-naming)

    /**
     * \f$\dot{\tau_{J}}\f$
     * Derivative of measured link-side joint torque sensor signals. Unit: \f$[\frac{Nm}{s}]\f$
     */
    std::array<double, 7> dtau_J{};  // NOLINT(readability-identifier-naming)

    /**
     * \f$q\f$
     * Measured joint position. Unit: \f$[rad]\f$
     */
    std::array<double, 7> q{};

    /**
     * \f$q_d\f$
     * Desired joint position. Unit: \f$[rad]\f$
     */
    std::vector<double> q_d{};

    /**
     * \f$\dot{q}\f$
     * Measured joint velocity. Unit: \f$[\frac{rad}{s}]\f$
     */
    std::array<double, 7> dq{};

    /**
     * \f$\dot{q}_d\f$
     * Desired joint velocity. Unit: \f$[\frac{rad}{s}]\f$
     */
    std::vector<double> dq_d{};

    /**
     * \f$\dot{q}_d\f$
     * Desired joint acceleration. Unit: \f$[\frac{rad}{s^2}]\f$
     */
    std::array<double, 7> ddq_d{};

    /**
     * Indicates which contact level is activated in which joint. After contact disappears, value
     * turns to zero.
     *
     * @see Robot::setCollisionBehavior for setting sensitivity values.
     */
    std::array<double, 7> joint_contact{};

    /**
     * Indicates which contact level is activated in which Cartesian dimension \f$(x,y,z,R,P,Y)\f$.
     * After contact disappears, the value turns to zero.
     *
     * @see Robot::setCollisionBehavior for setting sensitivity values.
     */
    std::array<double, 6> cartesian_contact{};

    /**
     * Indicates which contact level is activated in which joint. After contact disappears, the value
     * stays the same until a reset command is sent.
     *
     * @see Robot::setCollisionBehavior for setting sensitivity values.
     * @see Robot::automaticErrorRecovery for performing a reset after a collision.
     */
    std::array<double, 7> joint_collision{};

    /**
     * Indicates which contact level is activated in which Cartesian dimension \f$(x,y,z,R,P,Y)\f$.
     * After contact disappears, the value stays the same until a reset command is sent.
     *
     * @see Robot::setCollisionBehavior for setting sensitivity values.
     * @see Robot::automaticErrorRecovery for performing a reset after a collision.
     */
    std::array<double, 6> cartesian_collision{};

    /**
     * \f$\hat{\tau}_{\text{ext}}\f$
     * External torque, filtered. Unit: \f$[Nm]\f$.
     */
    std::array<double, 7> tau_ext_hat_filtered{};

    /**
     * \f$^OF_{K,\text{ext}}\f$
     * Estimated external wrench (force, torque) acting on stiffness frame, expressed
     * relative to the base frame. See also @ref k-frame "K frame".
     * Unit: \f$[N,N,N,Nm,Nm,Nm]\f$.
     */
    std::array<double, 6> O_F_ext_hat_K{};  // NOLINT(readability-identifier-naming)

    /**
     * \f$^{K}F_{K,\text{ext}}\f$
     * Estimated external wrench (force, torque) acting on stiffness frame,
     * expressed relative to the stiffness frame. See also @ref k-frame "K frame".
     * Unit: \f$[N,N,N,Nm,Nm,Nm]\f$.
     */
    std::array<double, 6> K_F_ext_hat_K{};  // NOLINT(readability-identifier-naming)

    /**
     * \f${^OdP_{EE}}_{d}\f$
     * Desired end effector twist in base frame.
     * Unit: \f$[\frac{m}{s},\frac{m}{s},\frac{m}{s},\frac{rad}{s},\frac{rad}{s},\frac{rad}{s}]\f$.
     */
    std::array<double, 6> O_dP_EE_d{};  // NOLINT(readability-identifier-naming)

    /**
     * \f${^OT_{EE}}_{c}\f$
     * Last commanded end effector pose of motion generation in base frame.
     * Pose is represented as a 4x4 matrix in column-major format.
     */
    std::array<double, 16> O_T_EE_c{};  // NOLINT(readability-identifier-naming)

    /**
     * \f${^OdP_{EE}}_{c}\f$
     * Last commanded end effector twist in base frame.
     * Unit: \f$[\frac{m}{s},\frac{m}{s},\frac{m}{s},\frac{rad}{s},\frac{rad}{s},\frac{rad}{s}]\f$.
     */
    std::array<double, 6> O_dP_EE_c{};  // NOLINT(readability-identifier-naming)

    /**
     * \f${^OddP_{EE}}_{c}\f$
     * Last commanded end effector acceleration in base frame.
     * Unit:
     * \f$[\frac{m}{s^2},\frac{m}{s^2},\frac{m}{s^2},\frac{rad}{s^2},\frac{rad}{s^2},\frac{rad}{s^2}]\f$.
     */
    std::array<double, 6> O_ddP_EE_c{};  // NOLINT(readability-identifier-naming)

    /**
     * \f$\theta\f$
     * Motor position. Unit: \f$[rad]\f$
     */
    std::array<double, 7> theta{};

    /**
     * \f$\dot{\theta}\f$
     * Motor velocity. Unit: \f$[rad]\f$
     */
    std::array<double, 7> dtheta{};

    /**
     * Percentage of the last 100 control commands that were successfully received by the robot.
     *
     * Shows a value of zero if no control or motion generator loop is currently running.
     *
     * Range: \f$[0, 1]\f$.
     */
    double control_command_success_rate{};

    /**
    * Strictly monotonically increasing timestamp since robot start.
    *
    * Inside of control loops @ref callback-docs "time_step" parameter of Robot::control can be used
    * instead.
    */
    Duration time{};

  };

  /**
   * Helper type for control and motion generation loops.
   *
   * Used to determine whether to terminate a loop after the control callback has returned.
   *
   * @see @ref callback-docs "Documentation on callbacks"
   */
  struct Finishable {
    /**
     * Determines whether to finish a currently running motion.
     */
    bool motion_finished = false;
  };

  /**
   * Stores joint-level torque commands without gravity and friction.
   */
  class Torques : public Finishable {
  public:
    /**
     * Creates a new Torques instance.
     *
     * @param[in] torques Desired joint-level torques without gravity and friction in [Nm].
     */
    Torques(const std::vector<double>& torques) noexcept;

    /**
     * Creates a new Torques instance.
     *
     * @param[in] torques Desired joint-level torques without gravity and friction in [Nm].
     *
     * @throw std::invalid_argument if the given initializer list has an invalid number of arguments.
     */
    Torques(std::initializer_list<double> torques);

    /**
     * Desired torques in [Nm].
     */
    std::vector<double> tau_J{};  // NOLINT(readability-identifier-naming)
  };

/**
 * Stores values for joint position motion generation.
 */
  class JointPositions : public Finishable {
  public:
    /**
     * Creates a new JointPositions instance.
     *
     * @param[in] joint_positions Desired joint angles in [rad].
     */
    JointPositions(const std::vector<double>& joint_positions) noexcept;

    /**
     * Creates a new JointPositions instance.
     *
     * @param[in] joint_positions Desired joint angles in [rad].
     *
     * @throw std::invalid_argument if the given initializer list has an invalid number of arguments.
     */
    JointPositions(std::initializer_list<double> joint_positions);

    /**
     * Desired joint angles in [rad].
     */
    std::vector<double> q{};
  };

/**
 * Stores values for joint velocity motion generation.
 */
  class JointVelocities : public Finishable {
  public:
    /**
     * Creates a new JointVelocities instance.
     *
     * @param[in] joint_velocities Desired joint velocities in [rad/s].
     *
     */
    JointVelocities(const std::vector<double>& joint_velocities) noexcept;

    /**
     * Creates a new JointVelocities instance.
     *
     * @param[in] joint_velocities Desired joint velocities in [rad/s].
     *
     * @throw std::invalid_argument if the given initializer list has an invalid number of arguments.
     */
    JointVelocities(std::initializer_list<double> joint_velocities);

    /**
     * Desired joint velocities in [rad/s].
     */
    std::vector<double> dq{};
  };

  class RobotInterface: public hardware_interface::RobotHW
  {
  public:
    RobotInterface() = delete;

    RobotInterface(std::vector<std::string> joint_names,
                   const urdf::Model& urdf_model,
                   const std::vector<double>& position,
                   const std::vector<double>& velocity,
                   const std::vector<double>& effort,
                   const std::vector<double>& position_cmd,
                   const std::vector<double>& velocity_cmd,
                   const std::vector<double>& effort_cmd
    );

    ~RobotInterface() override;
    void init();
    void update(const roport::RobotState& robot_state);
    void control(const std::function<bool(const ros::Time&, const ros::Duration&)>& ros_cb);
    //void enforceLimits(const ros::Duration& period);
    //void reset();

  protected:
    std::vector<std::string> joint_names_;

    std::vector<double> q_;
    std::vector<double> dq_;
    std::vector<double> tau_J_;

    // Interfaces
    hardware_interface::JointStateInterface joint_state_interface_{};
    hardware_interface::PositionJointInterface position_joint_interface_{};
    hardware_interface::VelocityJointInterface velocity_joint_interface_{};
    hardware_interface::EffortJointInterface effort_joint_interface_{};

  private:
    using Callback = std::function<bool(const roport::RobotState&, roport::Duration)>;
    std::function<void(Callback)> run_function_;
    std::function<bool()> get_limit_rate_;

    roport::JointPositions position_joint_cmd_;
    roport::JointVelocities velocity_joint_cmd_;
    roport::Torques effort_joint_cmd_;

  };

}

#endif //ROPORT_ROBOT_INTERFACE_H
