/****************************************** ATTENTION ****************************************
 *
 * 1. In order to support Chinese characters and comments, please use the UTF-8 format
 *    for all newly created files
 * 2. sys_config.h file contains the configuration parameters of the entire infantry system, 
 *    you can change the corresponding parameters as required
 * 3. Steps for board-level migration of different chips:
 *  3.1 Configure related pins and peripherals in Cube
 *  3.2 Modify the macro definition and related configuration of hardware abstraction 
 *      object in sys_config.h
 *  3.3 Related to the communication part of such as: can, uart, flash and so on due to 
 *      the different number of chips and board interface uncertain need to manually 
 *      add and change in the bsp.
 * 
*********************************************************************************************/


/********************************* Gimbal calibration method ********************************
*
* Note: The calibration operation needs gimbal in disability mode
* 
* Method 1: (Recommended)
* 1. Use tx1/2 or PC host computer to send the gimbal start and end calibration instructions, 
*    the command id 0x00A5 corresponding to the data
* 2. After the host computer receives the correct calibration data (0x0015) returned, 
*    the gimbal calibration is successful
* Method 2:
* 1. Connect j-link to the development board and the computer
* 2. Into debug mode, find this variable cali_param
* 3. Handle the gimbal to the midpoint, then set the variable cali_param.gim_cali_data[0].cali_cmd to 1
*
*********************************************************************************************/


/***********************************注意事项************************************
 *
 * 1、为了支持中文字符和注释，所有新建文件请使用UTF-8格式
 * 2、sys_config.h文件包含了整个步兵车系统的配置参数，可以按照需求更改相应参数
 * 3、进行不同芯片的板级移植时的步骤：
 *  3.1、在cube中配置相关引脚和外设；
 *  3.2、修改sys_config.h中硬件抽象对象的宏定义和相关配置
 *  3.3、涉及到通信部分的如：can、uart、flash等由于不同的芯片和板子接口数量不定，
 *       需要在bsp中进行手动添加和改动。
 * 
*******************************************************************************/


/**********************************云台校准方法*********************************
*
* 注意：校准需要云台在失能模式下进行
* 
* 方法一：（推荐）
* 1、使用 tx1/2 或 pc 端上位机依次发送云台开始和结束校准指令，指令为命令码 0x00A5 对应的数据
* 2、上位机接收到返回的正确的校准数据（0x0015）后，云台校准成功
* 方法二：
* 1、将 jlink 连接开发板和电脑
* 2、进入 debug 模式，找到 cali_param 这个变量
* 3、用手将云台扶正，云台在正中间时将变量cali_param.gim_cali_data[0].cali_cmd设为1即可
*
*******************************************************************************/
