/**
 * @file mycobot_hardware.hpp
 * @brief ros2_control SystemInterface for Laifual L70I-E-100-BF joint modules
 *        over EtherCAT (CiA 402 CSP — Cyclic Synchronous Position).
 *
 * Architecture summary
 * --------------------
 *   - One ros2_control "system" plugin instance manages all EtherCAT slaves.
 *   - Each URDF joint maps to one slave (slave_index parameter).
 *   - Drive runs in CiA 402 mode 8 (CSP). Plugin streams target_position
 *     every cycle; drive's internal PID closes the loop.
 *   - PDO mapping (must match the drive's accepted layout exactly):
 *
 *       RxPDO 0x1602 — 12 bytes, master → drive
 *         uint16  controlword         (0x6040)
 *         int32   target_position     (0x607A)
 *         uint16  touch_probe_func    (0x60B8)
 *         uint32  physical_output     (0x60FE:01)   bit 0 = brake relay (DO1)
 *
 *       TxPDO 0x1A02 — 26 bytes, drive → master
 *         uint16  statusword          (0x6041)
 *         int32   position_actual     (0x6064)
 *         int16   torque_actual       (0x6077)
 *         int32   following_error     (0x60F4)
 *         uint16  touch_probe_stat    (0x60B9)
 *         int32   touch_probe_pos1    (0x60BA)
 *         int32   touch_probe_pos2    (0x60BC)
 *         uint32  physical_inputs     (0x60FD)
 *
 *   - Free-run sync (no DC SYNC0). Cycle ~5 ms.
 *   - PDO exchange is performed in write() (after commands are staged into
 *     the output buffer). read() copies fresh state from the input buffer
 *     produced by the previous cycle's exchange.
 */

#pragma once

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <vector>
#include <string>
#include <cstdint>

extern "C" {
#include "soem/soem.h"
}

namespace mycobot_hardware {

/* ------------------------------------------------------------------ */
/* PDO structures — byte-for-byte layout, must be packed              */
/* ------------------------------------------------------------------ */
typedef struct __attribute__((packed)) {
  uint16_t controlword;       /* 0x6040 */
  int32_t  target_position;   /* 0x607A — 32-bit (CSP mode) */
  uint16_t touch_probe_func;  /* 0x60B8 */
  uint32_t physical_output;   /* 0x60FE:01 — bit0 = brake */
} csp_rxpdo_t;  /* 12 bytes */

typedef struct __attribute__((packed)) {
  uint16_t statusword;        /* 0x6041 */
  int32_t  position_actual;   /* 0x6064 */
  int16_t  torque_actual;     /* 0x6077 */
  int32_t  following_error;   /* 0x60F4 */
  uint16_t touch_probe_stat;  /* 0x60B9 */
  int32_t  touch_probe_pos1;  /* 0x60BA */
  int32_t  touch_probe_pos2;  /* 0x60BC */
  uint32_t physical_inputs;   /* 0x60FD */
} csp_txpdo_t;  /* 26 bytes */

/* ------------------------------------------------------------------ */
/* CiA 402 controlword / statusword bit definitions                   */
/* ------------------------------------------------------------------ */
constexpr uint16_t CW_SWITCH_ON         = 1u << 0;
constexpr uint16_t CW_ENABLE_VOLTAGE    = 1u << 1;
constexpr uint16_t CW_QUICK_STOP        = 1u << 2;
constexpr uint16_t CW_ENABLE_OPERATION  = 1u << 3;
constexpr uint16_t CW_FAULT_RESET       = 1u << 7;

constexpr uint16_t SW_READY_TO_SWITCH_ON = 0x0021;
constexpr uint16_t SW_SWITCHED_ON        = 0x0023;
constexpr uint16_t SW_OPERATION_ENABLED  = 0x0027;
constexpr uint16_t SW_FAULT              = 0x0008;
constexpr uint16_t SW_STATE_MASK         = 0x006F;

/* ------------------------------------------------------------------ */
/* Plugin class                                                       */
/* ------------------------------------------------------------------ */
class MyCobotHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /* ros2_control storage (units: SI — radians, rad/s) */
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;

  /* per-joint hardware mapping (parsed from URDF) */
  std::vector<int>     joint_to_slave_;     /* SOEM slave index, 1-based */
  std::vector<double>  counts_per_rad_;     /* encoder counts per radian (output side) */
  std::vector<int32_t> last_position_counts_; /* for velocity diff */
  std::vector<int32_t> initial_position_counts_; /* startup offset */

  /* plugin-level config (parsed from URDF <hardware> block) */
  std::string ifname_;        /* e.g. "eth0", "enp3s0" */
  size_t      n_joints_{0};

  /* SOEM state */
  ecx_contextt context_;
  uint8_t      io_map_[4096]{};
  uint8_t      group_{0};
  bool         soem_running_{false};

  /* convenience pointers into the IO map (per slave; index = SOEM slave_idx) */
  std::vector<csp_rxpdo_t *> rxpdo_;  /* size = max_slave + 1, [0] unused */
  std::vector<csp_txpdo_t *> txpdo_;

  rclcpp::Logger logger_{rclcpp::get_logger("MyCobotHardware")};

  /* ----------------- helpers ----------------- */
  bool init_soem();                          /* ec_init + config_init + map */
  bool reach_safe_op();
  bool reach_operational();
  bool enable_drive(int slave_idx);          /* CiA 402 state machine to OP_ENABLED */
  void disable_drive(int slave_idx);
  void close_soem();
  int  fieldbus_roundtrip();                 /* one PDO exchange, returns wkc */

  static const char * cia402_state_str(uint16_t sw);

  /* PO2SOconfig — applied during transition PRE_OP → SAFE_OP.
     Must be a free function (or static) for SOEM's C function-pointer hook. */
  static int slave_csp_config(ecx_contextt * context, uint16_t slave);
};

}  // namespace mycobot_hardware
