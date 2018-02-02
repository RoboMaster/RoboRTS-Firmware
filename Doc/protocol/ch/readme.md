## 协议说明

### 协议数据

协议数据按照通信方向可以分为两大类：

#### 一、底层发送给上层的数据：

1 反馈信息：包含各个机构传感器反馈信息、底层计算出来的一些反馈信息；

2 底层状态信息：包含底层设备运行状态、底层对上层数据的一些响应等；

3 转发数据：包含裁判系统的全部信息、服务器端的自定义信息；

#### 二、底层接收的上层数据：

1 控制信息：上层对底层 3 个执行机构的控制信息；

2 配置信息：上层对机器人如轮距、轴距、云台位置等结构配置信息，以及上层程序运行状态，底层响应级别等；

3 转发数据：需要底层转发给裁判系统，并最终在客户端上显示出来的用户自定义信息；

## 工作模式说明

*备注：*

黑体部分为自动模式下能用到的各模块工作模式，PC 端数据发送控制信息时，需要注意模式的初始值。

### 云台

```c
typedef enum
{
  GIMBAL_RELAX         = 0,
  GIMBAL_INIT          = 1,
  GIMBAL_NO_ARTI_INPUT = 2,
  GIMBAL_FOLLOW_ZGYRO  = 3,
  GIMBAL_TRACK_ARMOR   = 4,
  GIMBAL_PATROL_MODE   = 5,
  GIMBAL_SHOOT_BUFF    = 6,
  GIMBAL_POSITION_MODE = 7,
} gimbal_mode_e;
```

| 云台模式                     | 对应功能                        |
| ------------------------ | --------------------------- |
| GIMBAL_RELAX             | 云台断电                        |
| GIMBAL_INIT              | 云台由断电状态回中                   |
| GIMBAL_NO_ARTI_INPUT     | 无手动控制信息输入模式                 |
| GIMBAL_FOLLOW_ZGYRO      | 云台跟随底盘模式                    |
| GIMBAL_TRACK_ARMOR       | 云台追踪装甲，icra 不使用             |
| **GIMBAL_PATROL_MODE**   | 巡逻模式，云台 yaw 周期运动，pitch 不受控制 |
| GIMBAL_SHOOT_BUFF        | 打大符模式，icra 不使用              |
| **GIMBAL_POSITION_MODE** | 云台位置模式，上层控制角度两轴角度           |

### 底盘

```c
typedef enum
{
  CHASSIS_RELAX          = 0,
  CHASSIS_STOP           = 1,
  MANUAL_SEPARATE_GIMBAL = 2,
  MANUAL_FOLLOW_GIMBAL   = 3,
  DODGE_MODE             = 4,
  AUTO_SEPARATE_GIMBAL   = 5,
  AUTO_FOLLOW_GIMBAL     = 6,
} chassis_mode_e;
```

| 底盘模式                     | 对应功能                 |
| ------------------------ | -------------------- |
| CHASSIS_RELAX            | 底盘断电                 |
| CHASSIS_STOP             | 底盘停止/刹车              |
| MANUAL_SEPARATE_GIMBAL   | 手动模式底盘云台分离           |
| MANUAL_FOLLOW_GIMBAL     | 手动模式底盘跟随云台           |
| **DODGE_MODE**           | 底盘躲避模式，底盘固定旋转，平移不受控制 |
| **AUTO_SEPARATE_GIMBAL** | 底盘和云台分离模式，旋转、平移受上层控制 |
| **AUTO_FOLLOW_GIMBAL**   | 底盘跟随云台，平移受上层控制       |

### 发射机构

```c
typedef enum
{
  SHOT_DISABLE       = 0,
  REMOTE_CTRL_SHOT   = 1,
  KEYBOARD_CTRL_SHOT = 2,
  SEMIAUTO_CTRL_SHOT = 3,
  AUTO_CTRL_SHOT     = 4,
} shoot_mode_e;
```

| 发射机构模式             | 对应功能                 |
| ------------------ | -------------------- |
| SHOT_DISABLE       | 发射机构断电               |
| REMOTE_CTRL_SHOT   | 遥控器控制发射机构            |
| KEYBOARD_CTRL_SHOT | 键盘控制发射机构             |
| SEMIAUTO_CTRL_SHOT | 单发、连发上层控制            |
| **AUTO_CTRL_SHOT** | 摩擦轮开关、速度、单发、连发全部上层控制 |



## 协议数据说明

一帧协议数据分为 4 个部分，分别是帧头数据、命令码 ID、数据、帧尾校验数据。

### 1、帧头数据  

``` c
/** 
  * @brief  frame header structure definition
  */
typedef __packed struct
{
  uint8_t  sof;
  uint16_t data_length;
  uint8_t  seq;
  uint8_t  crc8;
} frame_header_t;
```


| 帧头数据        | 占用字节 | 详细描述                               |
| :---------- | :--- | :--------------------------------- |
| sof         | 1    | 数据的域ID，主控和裁判系统域：0xA5；主控和PC上层域：0xA0 |
| data_length | 2    | 每帧内数据data的长度                       |
| seq         | 1    | 包序号，在0xA0域中保留                      |
| crc8        | 1    | 帧头的crc校验结果                         |

*备注：*

数据的域 ID 主要分为上层域 0xA0 和底层域 0xA5。

0xA5 域为裁判系统和机器人主控的通信域 ID ，主要是裁判系统的比赛信息数据和用户自定义数据上传。

0xA0 域的数据为上层 PC 和主控的通信数据，主要包括 PC 对底层的控制数据和底层的反馈数据，以及主控转发给上层的裁判系统数据。

### 2、命令码

| 命令码   | 占用字节 |
| :---- | :--- |
| cmdid | 2    |

一个命令码对应一帧包含具体信息的数据，下面是现有所有数据对应的命令码。

*备注：*由于机器人主控把裁判系统数据都转发给了上层 PC，所以 0xA0 中包含了 0xA5 域中的所有信息。

``` c
typedef enum
{
  GAME_INFO_ID        = 0x0001,
  REAL_BLOOD_DATA_ID  = 0x0002,
  REAL_SHOOT_DATA_ID  = 0x0003,
  REAL_FIELD_DATA_ID  = 0x0005,
  GAME_RESULT_ID      = 0x0006,
  GAIN_BUFF_ID        = 0x0007,
  
  CHASSIS_DATA_ID     = 0x0010,
  GIMBAL_DATA_ID      = 0x0011,
  SHOOT_TASK_DATA_ID  = 0x0012,
  INFANTRY_ERR_ID     = 0x0013,
  CONFIG_RESPONSE_ID  = 0x0014,
  CALI_RESPONSE_ID    = 0x0015,
  REMOTE_CTRL_INFO_ID = 0x0016,
  BOTTOM_VERSION_ID   = 0x0017,
  
  CHASSIS_CTRL_ID     = 0x00A0,
  GIMBAL_CTRL_ID      = 0x00A1,
  SHOOT_CTRL_ID       = 0x00A2,
  ERROR_LEVEL_ID      = 0x00A3,
  INFANTRY_STRUCT_ID  = 0x00A4,
  CALI_GIMBAL_ID      = 0x00A5,
  
  STU_CUSTOM_DATA_ID  = 0x0100,
  ROBOT_TO_CLIENT_ID  = 0x0101,
  CLIENT_TO_ROBOT_ID  = 0x0102,
} command_id_e;
```


命令码对应的数据传输方向和具体功能如下：

| 命令码    | 传输方向    | 功能介绍             | 频率             |
| :----- | :------ | :--------------- | :------------- |
| 0x0001 | 主控-->PC | 比赛时机器人状态         | 裁判系统10Hz       |
| 0x0002 | 主控-->PC | 实时伤害数据           | 受到攻击时发送        |
| 0x0003 | 主控-->PC | 实时射击数据           | 裁判系统           |
| 0x0005 | 主控-->PC | 场地交互数据           | 检测到 IC 卡发送     |
| 0x0006 | 主控-->PC | 比赛结果数据           | 比赛结束时发送        |
| 0x0007 | 主控-->PC | 获得 buff 数据       | 裁判系统           |
|        |         |                  |                |
| 0x0010 | 主控-->PC | 机器人底盘相关信息        | 50Hz定频         |
| 0x0011 | 主控-->PC | 机器人云台相关信息        | 50Hz定频         |
| 0x0012 | 主控-->PC | 机器人射击任务信息        | 50Hz定频         |
| 0x0013 | 主控-->PC | 机器人底层故障信息        | 50Hz定频         |
| 0x0014 | 主控-->PC | 机器人结构配置状态反馈      | 50Hz定频         |
| 0x0015 | 主控-->PC | 机器人云台校准反馈        | 接收到有效校准信息后发送一次 |
| 0x0016 | 主控-->PC | 解析后的遥控器信息        | 50Hz定频         |
| 0x0017 | 主控-->PC | 底层软件版本信息         | 1Hz定频          |
|        |         |                  |                |
| 0x00A0 | PC-->主控 | 云台控制信息           | 50Hz定频         |
| 0x00A1 | PC-->主控 | 底盘控制信息           | 50Hz定频         |
| 0x00A2 | PC-->主控 | 发射机构控制信息         | 50Hz定频         |
| 0x00A3 | PC-->主控 | 电脑端运行错误警告级别      | 有错误时发送         |
| 0x00A4 | PC-->主控 | 机器人结构配置信息        | 一般在上电前一段时间     |
| 0x00A5 | PC-->主控 | 云台相关校准信息         | 需要校准云台时发送      |
|        |         |                  |                |
| 0x0100 | PC-->主控 | PC 需要转发到客户端显示的数据 | 定频（Hz）         |
| 0x0101 | PC-->主控 | PC 需要转发到客户端的数据   | 定频（Hz）         |
| 0x0102 | 主控-->PC | 客户端给 PC 的数据      | 有数据时转发         |

*注意事项：*

命令码 0x0100/0x0101 为 PC 发送给主控，并由主控转发到裁判系统最终显示在客户端界面上的数据，命令码 0x0102 为客户端用户的操作信息经过服务器和裁判系统后，由主控转发给 PC 的数据。

### 3、数据  

为命令码 ID 对应的数据结构，数据长度即这个结构体的大小。

| 数据   | 占用字节        |
| :--- | :---------- |
| data | data_length |

#### 第一类

##### 0x0001 比赛进程 

对应数据结构 game_info_t，比赛进程信息

```c
typedef __packed struct
{
  uint16_t   stage_remain_time;
  uint8_t    game_process;
  /* current race stage
     0 not start
     1 preparation stage
     2 self-check stage
     3 5 seconds count down
     4 fighting stage
     5 result computing stage */
  uint8_t    reserved;
  uint16_t   remain_hp;
  uint16_t   max_hp;
  position_t position;
} game_robot_state_t;
```

| 数据                | 说明            |
| :---------------- | ------------- |
| stage_remain_time | 当前阶段剩余时间，单位 s |
| game_process      | 当前比赛阶段        |
|                   | 0: 未开始比赛      |
|                   | 1: 准备阶段       |
|                   | 2: 自检阶段       |
|                   | 3: 5s 倒计时     |
|                   | 4: 对战中        |
|                   | 5: 比赛结算中      |
| reserved          | 保留位           |
| remain_hp         | 机器人当前血量       |
| max_hp            | 机器人满血量        |
| position          | 位置、角度信息       |

*备注：*

位置、角度控制信息包含在 position_t 结构体中：

```c
typedef __packed struct
{
  uint8_t valid_flag;
  float x;
  float y;
  float z;
  float yaw;
} position_t;
```

| 数据         | 说明           |
| ---------- | ------------ |
| valid_flag | 位置、角度信息有效标志位 |
|            | 0: 无效        |
|            | 1: 有效        |
| x          | 位置 X 坐标值     |
| y          | 位置 Y 坐标值     |
| z          | 位置 Z 坐标值     |
| yaw        | 枪口朝向角度值      |

##### 0x0002 伤害数据 

对应数据结构 robot_hurt_data_t，伤害数据

```c
typedef __packed struct
{
  uint8_t armor_type:4;
 /* 0-3bits: the attacked armor id:
    0x00: 0 front
    0x01：1 left
    0x02：2 behind
    0x03：3 right
    others reserved*/
  uint8_t hurt_type:4;
 /* 4-7bits: blood volume change type
    0x00: armor attacked
    0x01：module offline
    0x02: bullet over speed
    0x03: bullet over frequency */
} robot_hurt_data_t;
```

| 数据                      | 说明                          |
| ----------------------- | --------------------------- |
| armor_type（受到攻击的装甲 ID ） | 0-3bits: 若变化类型为装甲伤害时，标识装甲ID |
|                         | 0x00: 0 号装甲面 （前）            |
|                         | 0x01: 1 号装甲面 （左）            |
|                         | 0x02: 2 号装甲面 （后）            |
|                         | 0x03: 3 号装甲面 （右）            |
|                         | 其他保留                        |
| hurt_type（扣血类型）         | 4-7bits: 血量变化类型             |
|                         | 0x0: 装甲伤害（受到攻击）             |
|                         | 0x1: 模块离线                   |
|                         | 0x2: 弹丸超速                   |
|                         | 0x3: 弹丸超频                   |

##### 0x0003 实时射击 

对应数据结构 real_shoot_data_t，实时射击信息

```c
typedef __packed struct
{
  uint8_t reserved;
  uint8_t bullet_freq;
  float   bullet_speed;
  float   reserved;
} real_shoot_data_t;
```

| 数据           | 说明   |
| ------------ | ---- |
| reserved     | 保留   |
| bullet_freq  | 弹丸射频 |
| bullet_speed | 弹丸射速 |
| reserved     | 保留   |

##### 0x0005 场地交互

对应数据结构 rfid_detect_t，场地交互数据

```c
typedef __packed struct
{
  uint8_t card_type;
  uint8_t card_idx;
} rfid_detect_t;
```

| 数据        | 说明             |
| --------- | -------------- |
| card_type | 卡类型            |
|           | 0: 攻击加成卡       |
|           | 1: 防御加成卡       |
| card_idx  | 卡索引号，可用于区分不同区域 |

##### 0x0006 比赛结果 

对应数据结构 game_result_t，比赛胜负数据

```c
typedef __packed struct
{
  uint8_t winner;
} game_result_t;
```

| 数据     | 说明     |
| ------ | ------ |
| winner | 比赛结果   |
|        | 0: 平局  |
|        | 1: 红方胜 |
|        | 2: 蓝方胜 |

##### 0x0007 获取buff 

对应数据结构 get_buff_t，获取到的Buff数据

```c
typedef __packed struct
{
  uint8_t buff_type;
  uint8_t buff_addition;
} get_buff_t;
```

| 数据            | 说明      |
| ------------- | ------- |
| buff_type     | Buff类型  |
|               | 0: 攻击加成 |
|               | 1: 防御加成 |
| buff_addition | 加成百分比   |

#### 第二类

##### 0x0010 底盘信息

对应数据结构 chassis_info_t，底盘状态信息

```c
typedef __packed struct
{
  uint8_t ctrl_mode;      /* chassis control mode */
  float   gyro_palstance; /* chassis palstance(degree/s) from gyroscope */
  float   gyro_angle;     /* chassis angle(degree) relative to ground from gyroscope */
  float   ecd_palstance;  /* chassis palstance(degree/s) from chassis motor encoder calculated */
  float   ecd_calc_angle; /* chassis angle(degree) relative to ground from chassis motor encoder calculated */
  int16_t x_speed;        /* chassis x-axis move speed(mm/s) from chassis motor encoder calculated */
  int16_t y_speed;        /* chassis y-axis move speed(mm/s) from chassis motor encoder calculated */
  int32_t x_position;     /* chassis x-axis position(mm) relative to the starting point */
  int32_t y_position;     /* chassis y-axis position(mm) relative to the starting point */
} chassis_info_t;
```

| 数据             | 说明                      |
| -------------- | ----------------------- |
| ctrl_mode      | 底盘的控制模式                 |
| gyro_palstance | 单轴模块测量的底盘角速度(degree/s)  |
| gyro_angle     | 单轴模块测量的底盘角度(degree)     |
| ecd_palstance  | 底盘编码器计算的底盘角速度(degree/s) |
| ecd_calc_angle | 底盘编码器计算的底盘角度(degree)    |
| x_speed        | 底盘 x 轴运动速度(mm/s)        |
| y_speed        | 底盘 y 轴运动速度(mm/s)        |
| x_position     | 底盘 x 轴相对于起点的坐标位置(mm)    |
| y_position     | 底盘 y 轴相对于起点的坐标位置(mm)    |

*备注：*

> 所有位置信息都遵循右手坐标系，包含底盘和云台的所有位置相关数据，右手系的 x 轴为前向，y 轴为左向。

##### 0x0011 云台信息

对应数据结构 gimbal_info_t，云台状态信息

```c
typedef __packed struct
{
  uint8_t ctrl_mode;          /* gimbal control mode */
  float   pit_relative_angle; /* pitch angle(degree) relative to the gimbal center */
  float   yaw_relative_angle; /* yaw angle(degree) relative to the gimbal center */
  float   pit_absolute_angle; /* pitch angle(degree) relative to ground */
  float   yaw_absolute_angle; /* yaw angle(degree) relative to ground */
  float   pit_palstance;      /* pitch axis palstance(degree/s) */
  float   yaw_palstance;      /* yaw axis palstance(degree/s) */
} gimbal_info_t;
```

| 数据                 | 说明                        |
| ------------------ | ------------------------- |
| ctrl_mode          | 云台的控制模式                   |
| pit_relative_angle | pitch 轴相对于云台中点的角度(degree) |
| yaw_relative_angle | yaw 轴相对于云台中点的角度(degree)   |
| pit_absolute_angle | pitch 轴相对于地面的角度(degree)   |
| yaw_absolute_angle | yaw 轴相对于地面的角度(degree)     |
| pit_palstance      | pitch 轴角速度(degree/s)      |
| yaw_palstance      | yaw 轴角速度(degree/s)        |

##### 0x0012 发射机构

对应数据结构 shoot_info_t，发射机构状态信息

```c
typedef __packed struct
{
  int16_t remain_bullets;  /* the member of remain bullets */
  int16_t shot_bullets;    /* the member of bullets that have been shot */
  uint8_t fric_wheel_run; /* friction run or not */
} shoot_info_t;
```

| 数据             | 说明       |
| -------------- | -------- |
| remain_bullets | 剩余弹量(个)  |
| shot_bullets   | 已发射数量(个) |
| fric_wheel_run | 摩擦轮是否启动  |

##### 0x0013 底层错误

对应数据结构 infantry_err_t，底层步兵错误信息

```c
typedef __packed struct
{
  bottom_err_e err_sta;                 /* bottom error state */
  bottom_err_e err[ERROR_LIST_LENGTH];  /* device error list */
} infantry_err_t;
```

| 数据                     | 说明          |
| ---------------------- | ----------- |
| err_sta                | 底层设备全局状态    |
| err[ERROR_LIST_LENGTH] | 所有设备、机构工作状态 |

*备注：*

底层错误信息的枚举类型 bottom_err_e 如下，如果相应设备出现错误，状态位被置为`ERROR_EXIST`

```c
typedef enum
{
  DEVICE_NORMAL = 0,
  ERROR_EXIST   = 1,
  UNKNOWN_STATE = 2,
} bottom_err_e;
```

底层错误信息的所有分类包含在 err_id_e 中，主要分 3 类。第一类是 `设备_OFFLINE` 相关的设备离线；第二类是机构运行故障，目前只有卡弹这一项；第三类是 `_CONFIG_ERR` 软件相关的配置出现错误，如配置的云台安装位置超出了底盘的物理范围等。

```c
typedef enum
{
  BOTTOM_DEVICE        = 0,
  GIMBAL_GYRO_OFFLINE  = 1,
  CHASSIS_GYRO_OFFLINE = 2,
  CHASSIS_M1_OFFLINE   = 3,
  CHASSIS_M2_OFFLINE   = 4,
  CHASSIS_M3_OFFLINE   = 5,
  CHASSIS_M4_OFFLINE   = 6,
  REMOTE_CTRL_OFFLINE  = 7,
  JUDGE_SYS_OFFLINE    = 8,
  GIMBAL_YAW_OFFLINE   = 9,
  GIMBAL_PIT_OFFLINE   = 10,
  TRIGGER_MOTO_OFFLINE = 11,
  BULLET_JAM           = 12,
  CHASSIS_CONFIG_ERR   = 13,
  GIMBAL_CONFIG_ERR    = 14,
  ERROR_LIST_LENGTH    = 15,
} err_id_e;
```

##### 0x0014 结构配置反馈

对应数据结构 config_response_t，底层结构配置反馈信息

```c
typedef __packed struct
{
  struct_config_e chassis_config;
  struct_config_e gimbal_config;
} config_response_t;
```

| 数据             | 说明          |
| -------------- | ----------- |
| chassis_config | 底盘现在使用的结构配置 |
| gimbal_config  | 云台现在使用的结构配置 |

*备注：*

枚举类型 struct_config_e 的定义如下：

```c
typedef enum
{
  NO_CONFIG      = 0,
  DEFAULT_CONFIG = 1,
  CUSTOM_CONFIG  = 3,
} struct_config_e;
```

如果需要配置底盘或者云台的相关信息，需要将配置状态设置为 `CUSTOM_CONFIG` ，然后填充所有相关的数据，不能只配置底盘的某个数据或者云台的某个数据。如果配置状态为 `DEFAULT_CONFIG` ，或者从不发送这帧数据，则底层会使用默认配置。

##### 0x0015 云台校准反馈

对应数据结构 cali_response_t，云台校准反馈信息

```c
typedef __packed struct
{
  uint8_t type;
  int16_t yaw_offset;
  int16_t pitch_offset;
} cali_response_t;
```

| 数据           | 说明                      |
| ------------ | ----------------------- |
| type         | 是否配置成功（1: 成功 0: 失败）     |
| yaw_offset   | yaw 轴中点编码器值（0 ~ 8191）   |
| pitch_offset | pitch 轴中点编码器值（0 ~ 8191） |

##### 0x0016 遥控器信息 

对应数据结构 rc_info_t，遥控器信息

```c
typedef __packed struct
{
  /* rocker channel information */
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t ch4;
  /* left and right lever information */
  uint8_t sw1;
  uint8_t sw2;
  /* mouse movement and button information */
  __packed struct
  {
    int16_t x;
    int16_t y;
    int16_t z;
  
    uint8_t l;
    uint8_t r;
  } mouse;
  /* keyboard key information */
  __packed union
  {
    uint16_t key_code;
    __packed struct 
    {
      uint16_t W:1;
      uint16_t S:1;
      uint16_t A:1;
      uint16_t D:1;
      uint16_t SHIFT:1;
      uint16_t CTRL:1;
      uint16_t Q:1;
      uint16_t E:1;
      uint16_t R:1;
      uint16_t F:1;
      uint16_t G:1;
      uint16_t Z:1;
      uint16_t X:1;
      uint16_t C:1;
      uint16_t V:1;
      uint16_t B:1;
    } bit;
  } kb;
} rc_info_t;
```

| 数据        | 说明                                       |
| --------- | ---------------------------------------- |
| ch1 ~ ch4 | 遥控器4个遥杆通道的数据，数据范围(-660 ~ 660)            |
| sw1 ~ sw2 | 遥控器两个拨杆数据(上:1 中:3 下:2)                   |
| mouse     | 鼠标数据，鼠标x、y移动速度，左右键键值                     |
| key_code  | 键盘的按键值，可用的按键由共用体 kb 中的 bit 和 key_code 对应 |

##### 0x0017 底层版本

对应数据结构 version_info_t，底层软件版本信息

```c
typedef __packed struct
{
  uint8_t num[4];
} version_info_t;
```

| 数据     | 说明    |
| ------ | ----- |
| num[4] | 存放版本号 |

#### 第三类

##### 0x00A0 底盘控制 

对应数据结构 chassis_ctrl_t，底盘控制信息

```C
typedef __packed struct
{
  uint8_t          ctrl_mode; /* chassis control mode */
  int16_t          x_speed;   /* x-axis move speed(mm/s) of chassis */
  int16_t          y_speed;   /* y-axis move speed(mm/s) of chassis */
  chassis_rotate_t w_info;    /* rotation control of chassis */
} chassis_ctrl_t;
```

| 数据        | 说明                  |
| --------- | ------------------- |
| ctrl_mode | 控制底盘的工作模式           |
| x_speed   | 控制底盘 x 轴的移动速度(mm/s) |
| y_speed   | 控制底盘 y 轴的移动速度(mm/s) |
| w_info    | 控制底盘的旋转             |

*备注：*

底盘的旋转控制信息包含在 chassis_rotate_t 结构体中：

```c
typedef __packed struct
{
  int16_t x_offset;   /* offset(mm) relative to the x-axis of the chassis center */
  int16_t y_offset;   /* offset(mm) relative to the y-axis of the chassis center */
  float   w_speed;    /* rotation speed(degree/s) of chassis */
} chassis_rotate_t;
```

其中包含底盘旋转中心位置和底盘旋转速度，旋转中心位置为相对于底盘几何中心的位置坐标，x、y 轴分别对应 x_offset 和 y_offset 单位为(mm)，旋转速度单位是(degree/s)。

##### 0x00A1 云台控制 

对应数据结构 gimbal_ctrl_t，云台控制信息

```c
typedef __packed struct
{
  uint8_t ctrl_mode;    /* gimbal control mode */
  float   pit_ref;      /* gimbal pitch reference angle(degree) */
  float   yaw_ref;      /* gimbal yaw reference angle(degree) */
  uint8_t visual_valid; /* visual information valid or not */
} gimbal_ctrl_t;
```

| 数据           | 说明                        |
| ------------ | ------------------------- |
| ctrl_mode    | 控制云台的工作模式                 |
| pit_ref      | pitch 轴相对于中点的目标角度         |
| yaw_ref      | yaw 轴相对于中点的目标角度           |
| visual_valid | 视觉信息有效位，用来判断此时的云台控制数据是否可信 |

##### 0x00A2 发射机构控制 

对应数据结构 shoot_ctrl_t，发射机构控制信息

```C
typedef __packed struct
{
  uint8_t shoot_cmd;      /* single shoot command */
  uint8_t c_shoot_cmd;    /* continuous shoot command */
  uint8_t fric_wheel_run; /* friction run or not */
  uint8_t fric_wheel_spd; /* fricrion wheel speed */
} shoot_ctrl_t;
```

| 数据             | 说明                     |
| -------------- | ---------------------- |
| shoot_cmd      | 单发命令                   |
| c_shoot_cmd    | 连发命令                   |
| fric_wheel_run | 开关摩擦轮，0为关，1为开          |
| fric_wheel_spd | 摩擦轮转速，1000 ~ 2000的速度区间 |

##### 0x00A3 全局错误

对应数据结构 global_err_level_t，整个系统的运行警告级别

```c
typedef __packed struct
{
  err_level_e err_level;  /* the error level is included in err_level_e enumeration */
} global_err_level_t;
```

| 数据        | 说明                      |
| --------- | ----------------------- |
| err_level | 主要参见 err_level_e 类型中的数据 |

*备注：*

整个系统的运行警告级别包含在 err_level_e 枚举类型中：

```c
typedef enum
{
  GLOBAL_NORMAL        = 0,
  SOFTWARE_WARNING     = 1,
  SOFTWARE_ERROR       = 2,
  SOFTWARE_FATAL_ERROR = 3,
  SHOOT_ERROR          = 4,
  CHASSIS_ERROR        = 5,
  GIMBAL_ERROR         = 6,
} err_level_e;
```

上面的信息可以理解为按照优先级或者紧急程度排序，数字越大代表优先级、紧急程度越高。

*备注：*

> 系统的错误级别由上层发送给底层，如果出现多种类型错误，发送时会选择最高优先级的发送。错误级别数据可以分为两类：一类是上层软件的运行情况，第二类是底层硬件出现的错误。用户可以自定义出现这些不同级别的错误时的处理，目前的处理方式为，软件层的状态除了 `SOFTWARE_FATAL_ERROR`，其他的情况底层都不会做出响应，底层收到出现 `SOFTWARE_FATAL_ERROR` 的信息后，会切断云台和底盘的输出。第二类硬件的错误，主要由底层数据 err_id_e 中包含的数据错误列表分类得出，如果是哪个机构出现了问题，则它自己以及比它优先级低的设备都会切断输出。

##### 0x00A4 结构配置 

对应数据结构 infantry_structure_t，步兵结构配置信息

```c
typedef __packed struct
{
  struct_config_e  chassis_config;  /* chassis structure config state */
  uint16_t         wheel_perimeter; /* the perimeter(mm) of wheel */
  uint16_t         wheel_track;     /* wheel track distance(mm) */
  uint16_t         wheel_base;      /* wheelbase distance(mm) */
  struct_config_e  gimbal_config;   /* gimbal structure config state */
  int16_t          gimbal_x_offset; /* gimbal offset(mm) relative to the x-axis of the chassis center */
  int16_t          gimbal_y_offset; /* gimbal offset(mm) relative to the y-axis of the chassis center */
} infantry_structure_t;
```

| 数据              | 说明                     |
| --------------- | ---------------------- |
| chassis_config  | 底盘结构配置状态               |
| wheel_perimeter | 底盘的轮子周长(mm)            |
| wheel_track     | 底盘的轮距(mm)              |
| wheel_base      | 底盘的轴距(mm)              |
| gimbal_config   | 云台结构配置状态               |
| gimbal_x_offset | 云台安装位置距离底盘中心 x 轴距离(mm) |
| gimbal_y_offset | 云台安装位置距离底盘中心 y 轴距离(mm) |

##### 0x00A5 云台校准

对应数据结构 cali_cmd_t，云台校准指令信息

```c
typedef __packed struct
{
  uint8_t type;
  /* 0x01: calibrate gimbal center start
     0x02: calibrate gimbal center end
     0x03: calibrate camera start
     0x04: calibrate camera end
     other: invalid */
} cali_cmd_t;
```

| 数据   | 说明             |
| ---- | -------------- |
| type | 校准类型           |
|      | 0x01: 开始校准云台中点 |
|      | 0x02: 结束校准云台中点 |
|      | 0x03: 开始校准相机   |
|      | 0x04: 结束校准相机   |

#### 第四类

##### 0x0100 客户端显示

对应数据结构 client_show_data_t，自定义数据

```c
typedef __packed struct
{
  float data1;
  float data2;
  float data3;
} client_show_data_t;
```

| 数据    | 说明     |
| ----- | ------ |
| data1 | 自定义数据1 |
| data2 | 自定义数据2 |
| data3 | 自定义数据3 |

##### 0x0101 转发到服务器

对应数据结构 user_to_server_t，透传上行数据

```c
typedef __packed struct
{
  uint8_t data[64];
} user_to_server_t;
```

| 数据       | 说明          |
| -------- | ----------- |
| data[64] | 自定义数据，最大为64 |

##### 0x0102 转发到决策 PC

对应数据结构 server_to_user_t，透传下行数据

```c
typedef __packed struct
{
  uint8_t data[32];
} server_to_user_t;
```

| 数据       | 说明          |
| -------- | ----------- |
| data[32] | 自定义数据，最大为32 |

### 4、帧尾数据

每帧数据的crc16校验结果存储在这个位置。

| 帧尾数据  | 占用字节 |
| :---- | :--- |
| crc16 | 2    |



## 数据发送和接收

### 1、数据发送

使用如下函数打包要发送的数据：

```c
/**
  * @brief     pack data to bottom device
  * @param[in] cmd_id:  command id of data
  * @param[in] *p_data: pointer to the data to be sent
  * @param[in] len:     the data length
  * @usage     data_pack_handle(CHASSIS_CTRL_ID, &chassis_control_data, sizeof(chassis_ctrl_t))
  */

void data_pack_handle(uint16_t cmd_id, uint8_t *p_data, uint16_t len)
{
  memset(computer_tx_buf, 0, COMPUTER_FRAME_BUFLEN);
  frame_header_t *p_header = (frame_header_t*)computer_tx_buf;
  
  p_header->sof          = UP_REG_ID;
  p_header->data_length  = len;
  
  memcpy(&computer_tx_buf[HEADER_LEN], (uint8_t*)&cmd_id, CMD_LEN);
  append_crc8_check_sum(computer_tx_buf, HEADER_LEN);
  
  memcpy(&computer_tx_buf[HEADER_LEN + CMD_LEN], p_data, len);
  append_crc16_check_sum(computer_tx_buf, HEADER_LEN + CMD_LEN + len + CRC_LEN);

}
```

### 2、数据接收

使用下面的方式解决粘包的问题

```c
void read_and_unpack_thread(void *argu)
{
  uint8_t byte = 0;
  int32_t read_len;
  int32_t buff_read_index;
  
  uint16_t      data_len;
  unpack_step_e unpack_step;
  int32_t       index;
  uint8_t       protocol_packet[PROTOCAL_FRAME_MAX_SIZE];

  while (1)
  {
    read_len = uart_recv(uart_fd, computer_rx_buf, UART_BUFF_SIZE);
    buff_read_index = 0;
    
    while (read_len--)
    {
      byte = computer_rx_buf[buff_read_index++];
      
      switch(unpack_step)
      {
        case STEP_HEADER_SOF:
        {
          if(byte == UP_REG_ID)
          {
            unpack_step = STEP_LENGTH_LOW;
            protocol_packet[index++] = byte;
          }
          else
          {
            index = 0;
          }
        }break;
        
        case STEP_LENGTH_LOW:
        {
          data_len = byte;
          protocol_packet[index++] = byte;
          unpack_step = STEP_LENGTH_HIGH;
        }break;
        
        case STEP_LENGTH_HIGH:
        {
          data_len |= (byte << 8);
          protocol_packet[index++] = byte;

          if(data_len < (PROTOCAL_FRAME_MAX_SIZE - HEADER_LEN - CRC_LEN))
          {
            unpack_step = STEP_FRAME_SEQ;
          }
          else
          {
            unpack_step = STEP_HEADER_SOF;
            index = 0;
          }
        }break;
      
        case STEP_FRAME_SEQ:
        {
          protocol_packet[index++] = byte;
          unpack_step = STEP_HEADER_CRC8;
        }break;

        case STEP_HEADER_CRC8:
        {
          protocol_packet[index++] = byte;

          if (index == HEADER_LEN)
          {
            if ( verify_crc8_check_sum(protocol_packet, HEADER_LEN) )
            {
              unpack_step = STEP_DATA_CRC16;
            }
            else
            {
              unpack_step = STEP_HEADER_SOF;
              index = 0;
            }
          }
        }break;  

        case STEP_DATA_CRC16:
        {
          if (index < (HEADER_LEN + CMD_LEN + data_len + CRC_LEN))
          {
             protocol_packet[index++] = byte;  
          }
          if (index >= (HEADER_LEN + CMD_LEN + data_len + CRC_LEN))
          {
            unpack_step = STEP_HEADER_SOF;
            index = 0;

            if ( verify_crc16_check_sum(protocol_packet, HEADER_LEN + CMD_LEN + data_len + CRC_LEN) )
            {
              data_handle(protocol_packet);
            }
          }
        }break;

        default:
        {
          unpack_step = STEP_HEADER_SOF;
          index = 0;
        }break;
      }
    }
  }
}

void data_handle(uint8_t *p_frame)
{
  frame_header_t *p_header = (frame_header_t*)p_frame;
  memcpy(p_header, p_frame, HEADER_LEN);

  uint16_t data_length = p_header->data_length;
  uint16_t cmd_id      = *(uint16_t *)(p_frame + HEADER_LEN);
  uint8_t *data_addr   = p_frame + HEADER_LEN + CMD_LEN;
  
  switch (cmd_id)
  {
    case GAME_INFO_ID:
      memcpy(&game_information, data_addr, data_length);
    break;
    
    //............
    //............

  }
}
```



## 协议版本

当前版本 v1.3