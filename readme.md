[TOC]

[中文文档](Doc/ch/readme.md)

RoboRTS infantry control firmware

### Software Environment

- Toolchain/IDE : MDK-ARM V5
- cube version:  STM32CubeMX 4.22.1
- package version: STM32Cube FW_F4 V1.16.0
- FreeRTOS version: 9.0.0
- CMSIS-RTOS version: 1.02

### Programme Specification

- Names of user-defined variables and function should follow Unix/Linux style.
- Due to real time control task executes in Application/AppCtrl, it is prohibited to use any blocking operation.
- When blocking is operated in other tasks, time-out period should be set rationally. Be careful of using mutex or other blocking operation in different priority level and frequency. For example, *detect_task* has lower frequency and *info_get_task*  has higher frequency.

### Attention

- All new document should be written in UTF-8 format in case Chinese characters gibberish generated.
- Document *sys_config.h* includes all configuration parameters of the whole infantry system and user can change those parameters as needed.

### Module Off-line Description

When one of the modules is off-line, user can find out which module is out of work based on indicated light and buzzer in board.

In this project, error warns depend on the priority of off-line modules. 

Corresponding states of each module off-line are shown as following while the number corresponds the number of times of the red light flashes. 

1. Remote controller off-line
2. Gimbal motor off-line
3. Trigger motor or single gyroscope off-line
4. Chassis motor off-line
5. Single gyroscope sensor of chassis off-line
6. Referee system or PC serial port off-line and the red light keeps on

### Documentation

- Quick start and program guide [document](Doc/tutorial/readme.md)
- Communication protocol [document](Doc/protocol/readme.md)

