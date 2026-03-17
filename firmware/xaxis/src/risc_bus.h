#ifndef RISC_BUS_H
#define RISC_BUS_H

// COMMAND BUS INDICES (PC -> Robot)
#define CMD_STATE_REQUEST     0
#define CMD_TOOL_ROT_TARGET   1
#define CMD_TOOL_SWING_TARGET 2
#define CMD_X_LEAD_TARGET     3
#define CMD_ZL_TARGET         4
#define CMD_ZE_TARGET         5
#define CMD_WHEEL_FL_VEL      6
#define CMD_WHEEL_FE_VEL      7
#define CMD_WHEEL_RL_VEL      8
#define CMD_WHEEL_RE_VEL      9
#define CMD_GRIP_OPEN         10
#define CMD_EXTRACTOR_ON      11
#define CMD_CONVEYOR_ON       12
#define CMD_ADHESIVE_ON       13
#define CMD_TRIGGER_HOMING    14

// STATUS BUS INDICES (Robot -> PC)
#define STAT_HW_ID            0
#define STAT_POS_ALPHA        1
#define STAT_POS_BETA         2
#define GRIPPER_DETECT        3
#define STAT_PROX_SENSOR      4
#define STAT_TASK_COMPLETE    5
#define STAT_LIMIT_MIN_HIT    6
#define STAT_LIMIT_MAX_HIT    7

#endif