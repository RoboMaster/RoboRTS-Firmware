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
* 1. First, connect j-link to the development board and the computer
* 2. Turn the remote control levers to gimbal disabled position and infantry to power on
* 3. Into debug mode, find this variable cali_param
* 4. Handle the gimbal to the midpoint, then set the variable cali_param.Gimbal.cali_cmd to 1
*
*********************************************************************************************/

/******************************** Module Off-line Description *******************************
*
* When one of the modules is off-line, user can find out which module is out of work based on 
*    indicated light and buzzer in board.
*
* Corresponding states of each module off-line are shown as following while the number corresponds 
* the number of times of the red light flashes. 
*   

1. Remote controller off-line

*   2. Gimbal motor off-line
*   
3. Trigger motor or single gyroscope off-line
*   
4. Chassis motor off-line
*   
5. Single gyroscope sensor of chassis off-line

*   6. Referee system or PC serial port off-line and the red light keeps on

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
* 1、首先将jlink连接开发板和电脑
* 2、将遥控器拨杆拨到云台失能档位，步兵上电
* 3、进入debug模式，找到cali_param这个变量
* 4、用手将云台扶正，云台在正中间时将变量cali_param.Gimbal.cali_cmd设为1即可
*
*******************************************************************************/

/********************************模块离线检测说明*******************************
*
* 1、当车辆的某个模块离线时，可以根据开发板指示灯和蜂鸣器的不同状态判断哪个模块
*    出现了问题，并进行错误定位
*
* 注意：
* 按照优先级进行错误指示，例如遥控器优先级高于云台电机，如果同时发生离线，
* 先指示当前离线设备是遥控器
* 
* 模块离线对应的状态如下，数字对应红灯每次连续闪的次数，按照优先级排序：
* 1、遥控器离线
* 2、云台电机离线，包括pitch和yaw轴
* 3、拨弹电机离线
* 4、底盘电机存在离线
* 5、底盘单轴陀螺仪传感器离线
* 6、裁判系统或者pc端串口没有连接，此时红灯常亮
*
********************************************************************************/