/**
 * @file mycobot_hardware.cpp
 * @brief Implementation of MyCobotHardware ros2_control SystemInterface.
 *        Drives Laifual L70I-E-100-BF over EtherCAT in CiA 402 CSP mode.
 *
 * Most of the logic mirrors the proven simple_ng.c reference, with three
 * deviations for CSP (vs CSV in the reference):
 *
 *   1. Object 0x6060 (modes_of_operation) is set to 8 (CSP), not 9 (CSV).
 *   2. RxPDO maps 0x607A (Target Position) instead of 0x60FF (Target Velocity).
 *   3. Before enabling operation, target_position is initialised to the
 *      current position_actual to prevent a step-jump when the drive enables.
 *
 * Everything else (SyncManagers, brake mask, watchdog, free-run, state
 * machine sequencing) is identical to the proven configuration.
 */

#include "mycobot_hardware/mycobot_hardware.hpp"

#include "pluginlib/class_list_macros.hpp"

#include <cmath>
#include <cstring>
#include <limits>

namespace mycobot_hardware {

/* ================================================================== */
/* Static helpers                                                     */
/* ================================================================== */
const char * MyCobotHardware::cia402_state_str(uint16_t sw)
{
  uint16_t m = sw & SW_STATE_MASK;
  if ((m & 0x004F) == 0x0040) return "NOT_READY_TO_SWITCH_ON";
  if ((m & 0x006F) == 0x0060) return "SWITCH_ON_DISABLED";
  if (m == SW_READY_TO_SWITCH_ON) return "READY_TO_SWITCH_ON";
  if (m == SW_SWITCHED_ON)        return "SWITCHED_ON";
  if (m == SW_OPERATION_ENABLED)  return "OPERATION_ENABLED";
  if (m == 0x0007)                return "QUICK_STOP_ACTIVE";
  if ((m & 0x004F) == 0x000F)     return "FAULT_REACTION_ACTIVE";
  if (sw & SW_FAULT)              return "FAULT";
  return "UNKNOWN";
}

/* ================================================================== */
/* PO2SOconfig — runs during PRE_OP → SAFE_OP for each slave           */
/* This is the CSP equivalent of laifual_csv_config() from simple_ng.c. */
/* ================================================================== */
int MyCobotHardware::slave_csp_config(ecx_contextt * context, uint16_t slave)
{
  int wc;
  int sz;

  /* Step 1: set modes_of_operation = 8 (CSP) */
  int8_t mode = 8;
  wc = ecx_SDOwrite(context, slave, 0x6060, 0x00, FALSE,
                    sizeof(mode), &mode, EC_TIMEOUTSAFE);
  if (wc <= 0) {
    return -1;
  }
  osal_usleep(50000);

  int8_t mode_rb = 0;
  sz = sizeof(mode_rb);
  ecx_SDOread(context, slave, 0x6060, 0x00, FALSE, &sz, &mode_rb, EC_TIMEOUTSAFE);
  if (mode_rb != 8) {
    return -1;  /* drive did not accept CSP */
  }

  /* Step 2: interpolation period 10 ms (10^-3 s base) */
  int8_t interp_val = 10;
  int8_t interp_idx = -3;
  ecx_SDOwrite(context, slave, 0x60C2, 0x01, FALSE,
               sizeof(interp_val), &interp_val, EC_TIMEOUTSAFE);
  ecx_SDOwrite(context, slave, 0x60C2, 0x02, FALSE,
               sizeof(interp_idx), &interp_idx, EC_TIMEOUTSAFE);

  /* Step 3: TxPDO 0x1A02 — 26 bytes (same as CSV reference) */
  uint32_t tx_objs[] = {
    0x60410010,  /* Statusword         16 */
    0x60640020,  /* Position Actual    32 */
    0x60770010,  /* Torque Actual      16 */
    0x60F40020,  /* Following Error    32 */
    0x60B90010,  /* Touch Probe Stat   16 */
    0x60BA0020,  /* Touch Probe Pos1   32 */
    0x60BC0020,  /* Touch Probe Pos2   32 */
    0x60FD0020   /* Physical Inputs    32 */
  };
  const int tx_count = sizeof(tx_objs) / sizeof(tx_objs[0]);
  uint8_t zero = 0;

  ecx_SDOwrite(context, slave, 0x1A02, 0x00, FALSE,
               sizeof(zero), &zero, EC_TIMEOUTSAFE);
  for (int i = 0; i < tx_count; ++i) {
    wc = ecx_SDOwrite(context, slave, 0x1A02, i + 1, FALSE,
                      sizeof(tx_objs[i]), &tx_objs[i], EC_TIMEOUTSAFE);
    if (wc <= 0) return -1;
  }
  uint8_t tx_n = static_cast<uint8_t>(tx_count);
  ecx_SDOwrite(context, slave, 0x1A02, 0x00, FALSE,
               sizeof(tx_n), &tx_n, EC_TIMEOUTSAFE);

  /* Step 4: RxPDO 0x1602 — 12 bytes (CSP variant: 0x607A target position) */
  uint32_t rx_objs[] = {
    0x60400010,  /* Controlword         16 */
    0x607A0020,  /* Target Position     32  *** CSP key change vs CSV *** */
    0x60B80010,  /* Touch Probe Func    16 */
    0x60FE0120   /* Physical Output:01  32 */
  };
  const int rx_count = sizeof(rx_objs) / sizeof(rx_objs[0]);

  ecx_SDOwrite(context, slave, 0x1602, 0x00, FALSE,
               sizeof(zero), &zero, EC_TIMEOUTSAFE);
  for (int i = 0; i < rx_count; ++i) {
    wc = ecx_SDOwrite(context, slave, 0x1602, i + 1, FALSE,
                      sizeof(rx_objs[i]), &rx_objs[i], EC_TIMEOUTSAFE);
    if (wc <= 0) return -1;
  }
  uint8_t rx_n = static_cast<uint8_t>(rx_count);
  ecx_SDOwrite(context, slave, 0x1602, 0x00, FALSE,
               sizeof(rx_n), &rx_n, EC_TIMEOUTSAFE);

  /* Step 4b: enable physical_output bit 0 (brake relay) */
  uint32_t output_mask = 0x00000001;
  ecx_SDOwrite(context, slave, 0x60FE, 0x02, FALSE,
               sizeof(output_mask), &output_mask, EC_TIMEOUTSAFE);

  /* Step 5: assign SyncManagers */
  uint16_t rx_map = 0x1602;
  uint16_t tx_map = 0x1A02;
  uint8_t  one    = 1;

  ecx_SDOwrite(context, slave, 0x1C12, 0x00, FALSE,
               sizeof(zero), &zero, EC_TIMEOUTSAFE);
  ecx_SDOwrite(context, slave, 0x1C12, 0x01, FALSE,
               sizeof(rx_map), &rx_map, EC_TIMEOUTSAFE);
  ecx_SDOwrite(context, slave, 0x1C12, 0x00, FALSE,
               sizeof(one), &one, EC_TIMEOUTSAFE);

  ecx_SDOwrite(context, slave, 0x1C13, 0x00, FALSE,
               sizeof(zero), &zero, EC_TIMEOUTSAFE);
  ecx_SDOwrite(context, slave, 0x1C13, 0x01, FALSE,
               sizeof(tx_map), &tx_map, EC_TIMEOUTSAFE);
  ecx_SDOwrite(context, slave, 0x1C13, 0x00, FALSE,
               sizeof(one), &one, EC_TIMEOUTSAFE);

  /* Step 6: free-run sync mode (no DC SYNC0 needed) */
  uint16_t sync_free = 0;
  ecx_SDOwrite(context, slave, 0x1C32, 0x01, FALSE,
               sizeof(sync_free), &sync_free, EC_TIMEOUTSAFE);
  ecx_SDOwrite(context, slave, 0x1C33, 0x01, FALSE,
               sizeof(sync_free), &sync_free, EC_TIMEOUTSAFE);

  return 1;
}

/* ================================================================== */
/* on_init — parse URDF, allocate vectors                              */
/* ================================================================== */
hardware_interface::CallbackReturn MyCobotHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  /* --- plugin-level params from <hardware> block --- */
  auto it = info_.hardware_parameters.find("ifname");
  if (it == info_.hardware_parameters.end()) {
    RCLCPP_ERROR(logger_, "Missing required <param name=\"ifname\"> "
                          "in URDF <hardware> block (e.g. \"eth0\").");
    return hardware_interface::CallbackReturn::ERROR;
  }
  ifname_ = it->second;

  /* --- per-joint params from <joint> blocks --- */
  n_joints_ = info_.joints.size();
  hw_positions_.assign(n_joints_, 0.0);
  hw_velocities_.assign(n_joints_, 0.0);
  hw_commands_.assign(n_joints_, std::numeric_limits<double>::quiet_NaN());
  joint_to_slave_.assign(n_joints_, 0);
  counts_per_rad_.assign(n_joints_, 0.0);
  last_position_counts_.assign(n_joints_, 0);
  initial_position_counts_.assign(n_joints_, 0);

  int max_slave = 0;
  for (size_t i = 0; i < n_joints_; ++i) {
    const auto & j = info_.joints[i];

    auto it_s = j.parameters.find("slave_index");
    auto it_c = j.parameters.find("counts_per_rad");
    if (it_s == j.parameters.end() || it_c == j.parameters.end()) {
      RCLCPP_ERROR(logger_,
        "Joint '%s' is missing required <param>: slave_index AND counts_per_rad",
        j.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    joint_to_slave_[i] = std::stoi(it_s->second);
    counts_per_rad_[i] = std::stod(it_c->second);
    if (joint_to_slave_[i] > max_slave) max_slave = joint_to_slave_[i];

    /* sanity check: only "position" command and state interfaces expected */
    bool has_pos_cmd = false;
    for (const auto & ci : j.command_interfaces) {
      if (ci.name == hardware_interface::HW_IF_POSITION) has_pos_cmd = true;
    }
    if (!has_pos_cmd) {
      RCLCPP_ERROR(logger_,
        "Joint '%s' must declare a 'position' command interface.", j.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  /* size pointer arrays for slaves (1-based indexing matches SOEM) */
  rxpdo_.assign(max_slave + 1, nullptr);
  txpdo_.assign(max_slave + 1, nullptr);

  RCLCPP_INFO(logger_,
    "MyCobotHardware initialised: ifname='%s', %zu joint(s), max slave index %d.",
    ifname_.c_str(), n_joints_, max_slave);

  return hardware_interface::CallbackReturn::SUCCESS;
}

/* ================================================================== */
/* Interface export                                                    */
/* ================================================================== */
std::vector<hardware_interface::StateInterface>
MyCobotHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> ifaces;
  for (size_t i = 0; i < n_joints_; ++i) {
    ifaces.emplace_back(info_.joints[i].name,
                        hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    ifaces.emplace_back(info_.joints[i].name,
                        hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return ifaces;
}

std::vector<hardware_interface::CommandInterface>
MyCobotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ifaces;
  for (size_t i = 0; i < n_joints_; ++i) {
    ifaces.emplace_back(info_.joints[i].name,
                        hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }
  return ifaces;
}

/* ================================================================== */
/* SOEM bring-up                                                       */
/* ================================================================== */
bool MyCobotHardware::init_soem()
{
  std::memset(&context_, 0, sizeof(context_));
  std::memset(io_map_, 0, sizeof(io_map_));

  RCLCPP_INFO(logger_, "ec_init on '%s'...", ifname_.c_str());
  if (!ecx_init(&context_, ifname_.c_str())) {
    RCLCPP_ERROR(logger_,
      "ec_init failed on '%s'. Check interface name and that the binary has "
      "CAP_NET_RAW (or is running as root).", ifname_.c_str());
    return false;
  }

  if (ecx_config_init(&context_) <= 0) {
    RCLCPP_ERROR(logger_, "No EtherCAT slaves found on '%s'.", ifname_.c_str());
    ecx_close(&context_);
    return false;
  }
  RCLCPP_INFO(logger_, "Found %d slave(s).", context_.slavecount);

  /* attach our PO2SOconfig hook to every slave we'll command */
  for (size_t i = 0; i < n_joints_; ++i) {
    int s = joint_to_slave_[i];
    if (s < 1 || s > context_.slavecount) {
      RCLCPP_ERROR(logger_,
        "Joint '%s' references slave_index=%d but the bus has only %d slaves.",
        info_.joints[i].name.c_str(), s, context_.slavecount);
      ecx_close(&context_);
      return false;
    }
    context_.slavelist[s].PO2SOconfig = &MyCobotHardware::slave_csp_config;
  }

  ecx_config_map_group(&context_, io_map_, group_);

  /* Disable SM watchdog timeout (reg 0x0422) — same as reference. */
  for (size_t i = 0; i < n_joints_; ++i) {
    int s = joint_to_slave_[i];
    uint16_t wd_off = 0;
    ecx_FPWR(&context_.port, context_.slavelist[s].configadr,
             0x0422, sizeof(wd_off), &wd_off, EC_TIMEOUTRET);
  }

  /* Stash convenience pointers to each slave's PDO buffers */
  for (size_t i = 0; i < n_joints_; ++i) {
    int s = joint_to_slave_[i];
    rxpdo_[s] = reinterpret_cast<csp_rxpdo_t *>(context_.slavelist[s].outputs);
    txpdo_[s] = reinterpret_cast<csp_txpdo_t *>(context_.slavelist[s].inputs);
    if (!rxpdo_[s] || !txpdo_[s]) {
      RCLCPP_ERROR(logger_, "Slave %d has null PDO buffers after mapping.", s);
      ecx_close(&context_);
      return false;
    }
  }

  return true;
}

bool MyCobotHardware::reach_safe_op()
{
  context_.slavelist[0].state = EC_STATE_SAFE_OP;
  ecx_writestate(&context_, 0);

  for (int i = 0; i < 20; ++i) {
    ecx_statecheck(&context_, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
    ecx_readstate(&context_);
    if (context_.slavelist[0].state >= EC_STATE_SAFE_OP) return true;
    /* clear errors if any slave is in ERROR */
    for (int s = 1; s <= context_.slavecount; ++s) {
      if (context_.slavelist[s].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR) ||
          context_.slavelist[s].state == (EC_STATE_PRE_OP  + EC_STATE_ERROR)) {
        context_.slavelist[s].state =
          (context_.slavelist[s].state & 0x0F) + EC_STATE_ACK;
        ecx_writestate(&context_, s);
        osal_usleep(100000);
      }
    }
  }
  return context_.slavelist[0].state >= EC_STATE_SAFE_OP;
}

bool MyCobotHardware::reach_operational()
{
  /* Stream a few "safe" PDOs first so SM3 has data before going to OP. */
  for (size_t i = 0; i < n_joints_; ++i) {
    int s = joint_to_slave_[i];
    rxpdo_[s]->controlword       = CW_QUICK_STOP | CW_ENABLE_VOLTAGE;
    rxpdo_[s]->target_position   = 0;  /* fixed up below from actual */
    rxpdo_[s]->touch_probe_func  = 0;
    rxpdo_[s]->physical_output   = 0;  /* brake engaged */
  }
  for (int i = 0; i < 5; ++i) fieldbus_roundtrip();

  /* Snap target_position to actual_position to prevent jump on enable */
  for (size_t i = 0; i < n_joints_; ++i) {
    int s = joint_to_slave_[i];
    rxpdo_[s]->target_position = txpdo_[s]->position_actual;
  }
  for (int i = 0; i < 5; ++i) fieldbus_roundtrip();

  /* Request OPERATIONAL */
  context_.slavelist[0].state = EC_STATE_OPERATIONAL;
  ecx_writestate(&context_, 0);

  for (int i = 0; i < 40; ++i) {
    fieldbus_roundtrip();
    ecx_statecheck(&context_, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE / 10);
    ecx_readstate(&context_);
    bool all_op = true;
    for (size_t j = 0; j < n_joints_; ++j) {
      if (context_.slavelist[joint_to_slave_[j]].state != EC_STATE_OPERATIONAL) {
        all_op = false;
        break;
      }
    }
    if (all_op) return true;
  }
  return false;
}

int MyCobotHardware::fieldbus_roundtrip()
{
  ecx_send_processdata(&context_);
  return ecx_receive_processdata(&context_, 2000);
}

bool MyCobotHardware::enable_drive(int s)
{
  uint16_t sw = txpdo_[s]->statusword;

  /* Fault reset if needed */
  if (sw & SW_FAULT) {
    rxpdo_[s]->controlword = CW_FAULT_RESET;
    for (int t = 0; t < 100; ++t) {
      fieldbus_roundtrip();
      sw = txpdo_[s]->statusword;
      if (!(sw & SW_FAULT)) break;
      osal_usleep(5000);
    }
    if (sw & SW_FAULT) {
      RCLCPP_ERROR(logger_, "Slave %d: fault would not clear (sw=0x%04X)", s, sw);
      return false;
    }
  }

  /* Shutdown → READY_TO_SWITCH_ON  (CW = 0x0006) */
  rxpdo_[s]->controlword = CW_QUICK_STOP | CW_ENABLE_VOLTAGE;
  for (int t = 0; t < 200; ++t) {
    fieldbus_roundtrip();
    sw = txpdo_[s]->statusword;
    if ((sw & SW_STATE_MASK) == SW_READY_TO_SWITCH_ON) break;
    osal_usleep(5000);
  }
  if ((sw & SW_STATE_MASK) != SW_READY_TO_SWITCH_ON) {
    RCLCPP_ERROR(logger_, "Slave %d: failed Shutdown (sw=0x%04X / %s)",
                 s, sw, cia402_state_str(sw));
    return false;
  }

  /* Switch On → SWITCHED_ON  (CW = 0x0007) */
  rxpdo_[s]->controlword = CW_QUICK_STOP | CW_ENABLE_VOLTAGE | CW_SWITCH_ON;
  for (int t = 0; t < 200; ++t) {
    fieldbus_roundtrip();
    sw = txpdo_[s]->statusword;
    if ((sw & SW_STATE_MASK) == SW_SWITCHED_ON) break;
    osal_usleep(5000);
  }
  if ((sw & SW_STATE_MASK) != SW_SWITCHED_ON) {
    RCLCPP_ERROR(logger_, "Slave %d: failed Switch On (sw=0x%04X / %s)",
                 s, sw, cia402_state_str(sw));
    return false;
  }

  /* Refresh target_position again right before enabling operation */
  rxpdo_[s]->target_position = txpdo_[s]->position_actual;

  /* Enable Operation → OPERATION_ENABLED  (CW = 0x000F) */
  rxpdo_[s]->controlword = CW_QUICK_STOP | CW_ENABLE_VOLTAGE
                         | CW_SWITCH_ON  | CW_ENABLE_OPERATION;
  for (int t = 0; t < 200; ++t) {
    fieldbus_roundtrip();
    sw = txpdo_[s]->statusword;
    if ((sw & SW_STATE_MASK) == SW_OPERATION_ENABLED) break;
    osal_usleep(5000);
  }
  if ((sw & SW_STATE_MASK) != SW_OPERATION_ENABLED) {
    RCLCPP_ERROR(logger_, "Slave %d: failed Enable Operation (sw=0x%04X / %s)",
                 s, sw, cia402_state_str(sw));
    return false;
  }

  /* Release brake (DO1 high) and stream for ~100 ms so the brake actually opens */
  for (int t = 0; t < 20; ++t) {
    rxpdo_[s]->physical_output = 0x00000001;
    fieldbus_roundtrip();
    osal_usleep(5000);
  }

  RCLCPP_INFO(logger_, "Slave %d: OPERATION_ENABLED, brake released. pos_actual=%d",
              s, txpdo_[s]->position_actual);
  return true;
}

void MyCobotHardware::disable_drive(int s)
{
  if (!txpdo_[s] || !rxpdo_[s]) return;

  /* Hold position, keep enabled, engage brake, then quick stop. */
  rxpdo_[s]->target_position = txpdo_[s]->position_actual;
  rxpdo_[s]->physical_output = 0;     /* brake engaged */
  rxpdo_[s]->controlword     = CW_QUICK_STOP | CW_ENABLE_VOLTAGE;
  for (int t = 0; t < 20; ++t) {
    fieldbus_roundtrip();
    osal_usleep(5000);
  }
}

void MyCobotHardware::close_soem()
{
  if (!soem_running_) return;
  context_.slavelist[0].state = EC_STATE_INIT;
  ecx_writestate(&context_, 0);
  ecx_close(&context_);
  soem_running_ = false;
  RCLCPP_INFO(logger_, "EtherCAT socket closed.");
}

/* ================================================================== */
/* on_activate — bring drives to OPERATION_ENABLED                     */
/* ================================================================== */
hardware_interface::CallbackReturn MyCobotHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Activating MyCobotHardware...");

  if (!init_soem()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  soem_running_ = true;

  if (!reach_safe_op()) {
    RCLCPP_ERROR(logger_, "Failed to reach SAFE_OP.");
    close_soem();
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!reach_operational()) {
    RCLCPP_ERROR(logger_, "Failed to reach OPERATIONAL.");
    close_soem();
    return hardware_interface::CallbackReturn::ERROR;
  }

  /* CiA 402 enable per-drive */
  for (size_t i = 0; i < n_joints_; ++i) {
    int s = joint_to_slave_[i];
    if (!enable_drive(s)) {
      RCLCPP_ERROR(logger_, "Drive enable failed for joint '%s' (slave %d).",
                   info_.joints[i].name.c_str(), s);
      close_soem();
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  /* Seed hw_positions_ and hw_commands_ from the live PDO data so that
     the very first controller update doesn't try to track NaN. */
  for (size_t i = 0; i < n_joints_; ++i) {
    int s = joint_to_slave_[i];
    int32_t cnt = txpdo_[s]->position_actual;
    initial_position_counts_[i] = cnt;
    last_position_counts_[i] = cnt;
    hw_positions_[i]  = 0.0;
    hw_velocities_[i] = 0.0;
    hw_commands_[i]   = 0.0;   /* hold-position on activation */
  }

  RCLCPP_INFO(logger_, "MyCobotHardware ACTIVE — %zu drive(s) operating.", n_joints_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

/* ================================================================== */
/* on_deactivate                                                       */
/* ================================================================== */
hardware_interface::CallbackReturn MyCobotHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Deactivating MyCobotHardware...");

  for (size_t i = 0; i < n_joints_; ++i) {
    disable_drive(joint_to_slave_[i]);
  }
  close_soem();
  return hardware_interface::CallbackReturn::SUCCESS;
}

/* ================================================================== */
/* read — copy fresh feedback from the IO map                          */
/* (PDO exchange occurs in write(); this just reads the latest buffer) */
/* ================================================================== */
hardware_interface::return_type MyCobotHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const double dt = period.seconds() > 1e-6 ? period.seconds() : 0.005;

  for (size_t i = 0; i < n_joints_; ++i) {
    int s = joint_to_slave_[i];
    int32_t cnt = txpdo_[s]->position_actual;
    int32_t rel_cnt = cnt - initial_position_counts_[i];

    double pos_rad = static_cast<double>(rel_cnt) / counts_per_rad_[i];
    hw_velocities_[i] =
      (static_cast<double>(cnt - last_position_counts_[i]) / counts_per_rad_[i]) / dt;
    hw_positions_[i]  = pos_rad;
    last_position_counts_[i] = cnt;
  }
  return hardware_interface::return_type::OK;
}

/* ================================================================== */
/* write — stage commands and perform the PDO exchange                 */
/* ================================================================== */
hardware_interface::return_type MyCobotHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < n_joints_; ++i) {
    int s = joint_to_slave_[i];

    /* Defensive: if controller hasn't published yet, hold position. */
    double cmd = hw_commands_[i];
    if (!std::isfinite(cmd)) cmd = hw_positions_[i];

    int32_t cnt = static_cast<int32_t>(std::lround(cmd * counts_per_rad_[i])) + initial_position_counts_[i];

    uint16_t sw = txpdo_[s]->statusword;
    
    if (sw & SW_FAULT) {
      // Drive faulted (e.g. following error from a step jump). 
      // ENGAGE brake and stop commanding movement.
      rxpdo_[s]->target_position  = txpdo_[s]->position_actual;
      rxpdo_[s]->controlword      = CW_QUICK_STOP | CW_ENABLE_VOLTAGE; 
      rxpdo_[s]->physical_output  = 0x00000000;   /* brake ENGAGED */
    } else {
      rxpdo_[s]->target_position  = cnt;
      rxpdo_[s]->controlword      = CW_QUICK_STOP | CW_ENABLE_VOLTAGE
                                  | CW_SWITCH_ON  | CW_ENABLE_OPERATION;
      rxpdo_[s]->physical_output  = 0x00000001;   /* brake released */
    }
    rxpdo_[s]->touch_probe_func = 0;
  }

  int wkc = fieldbus_roundtrip();
  if (wkc <= 0) {
    static rclcpp::Clock throttle_clock(RCL_STEADY_TIME);
    RCLCPP_WARN_THROTTLE(logger_, throttle_clock, 1000,
                         "PDO roundtrip wkc=%d (link issue?).", wkc);
    /* don't return ERROR — one missed frame should not bring the system down */
  }
  return hardware_interface::return_type::OK;
}

}  // namespace mycobot_hardware

PLUGINLIB_EXPORT_CLASS(mycobot_hardware::MyCobotHardware,
                       hardware_interface::SystemInterface)
