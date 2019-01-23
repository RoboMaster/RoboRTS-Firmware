## 协议说明

### 协议数据

协议数据按照通信方向可以分为两大类：

#### 一、底层发送给上层的数据：

1 反馈信息：包含各个机构传感器反馈信息、底层计算出来的一些反馈信息；

2 底层状态信息：包含底层设备运行状态、底层对上层数据的一些响应等；

3 转发数据：包含裁判系统的全部信息、服务器端的自定义信息；

#### 二、底层接收的上层数据：

1 控制信息：上层对底层 3 个执行机构的控制信息；

## 协议数据说明

一帧协议数据分为 4 个部分，分别是帧头数据、命令码ID、数据、帧尾校验数据。

### 1、帧头数据  

``` c
/** 
  * @brief  frame header structure definition
  */
/* This Struct Is Used To Describe A Package Header */
typedef struct
{
  uint8_t sof; /*!< Identify Of A Package */
  union {
    struct
    {
      uint16_t data_len : 10; /*!< Data Length, Include Header And Crc */
      uint16_t version : 6;   /*!< Protocol Version */
    };
    uint16_t ver_data_len;
  };
  union {
    struct
    {
      uint8_t session : 5;   /*!< Need(0~1) Or Not Need(2~63) Ack */
      uint8_t pack_type : 1; /*!< Ack Package Or Normal Package */
      uint8_t res : 2;       /*!< Reserve */
    };
    uint8_t S_A_R_c;
  };
  uint8_t sender;   /*!< Sender Module Information */
  uint8_t reciver;  /*!< Receiver Module Information */
  uint16_t res1;    /*!< Reserve 1 */
  uint16_t seq_num; /*!< Sequence Number */
  uint16_t crc_16;  /*!< CRC16 */
  uint8_t pdata[];
} protocol_pack_desc_t;
```


| 帧头数据      | 占用字节 | 详细描述                            |
| :------------| :-------| :--------------------------------- |
| sof          | 1       | 数据的域ID                          |
| ver_data_len | 2       | 每帧内数据的长度和协议版本号          |
| session      | 1       | 包序号，在0xA0域中保留               |
| sender       | 1       |发送者地址                          |
| reciver      | 1       |发送者地址                          |
| res          | 2       |保留位                              |
| seq          | 2       |包序号                              |
| crc16        | 2       | 帧头的crc校验结果                   |


### 2、命令码

| 命令码   | 占用字节 |
| :---- | :--- |
| cmdid | 2    |

一个命令码对应一帧包含具体信息的数据，下面是现有所有数据对应的命令码。


命令码对应的数据传输方向和具体功能如下：

| 命令码    | 传输方向    | 功能介绍             | 频率             |
| :----- | :------ | :--------------- | :------------- |
| 0x0001 | 主控-->PC | 比赛时机器人状态         | 裁判系统10Hz       |
| 0x0002 | 主控-->PC | 实时伤害数据           | 受到攻击时发送        |
| 0x0003 | 主控-->PC | 实时射击数据           | 裁判系统           |
| 0x0004 | 主控-->PC | 实时功率、热量数据        | ICRA不使用，不发送    |
| 0x0005 | 主控-->PC | 场地 RFID 数据       | 检测到 IC 卡发送     |
| 0x0006 | 主控-->PC | 比赛结果数据           | 比赛结束时发送        |
| 0x0007 | 主控-->PC | 获得 buff 数据       | 裁判系统           |
| 0x0008 | 主控-->PC | 场地 UWB 数据        | 裁判系统           |
|        |           |                  |                |
| 0x0204 | 主控-->PC | 机器人底盘相关信息        | 100Hz定频         |
| 0x0304 | 主控-->PC | 机器人云台相关信息        | 100Hz定频         |
| 0x0402 | 主控-->PC | UWB相关信息        | UWB更新频率         |
|        |           |                  |                |
| 0x0206 | PC-->主控 | 设置底盘速度        |          |
| 0x0208 | PC-->主控 | 设置底盘速度(有加速度)  |      |
| 0x0309 | PC-->主控 | 控制摩擦轮转速         | 开启摩擦轮使用         |
| 0x030A | PC-->主控 | 控制射击     |         |
| 0x0403 | PC-->主控 | 云台相关校准信息         | 需要校准云台时发送      |

### 3、数据  

为命令码 ID 对应的数据结构，数据长度即这个结构体的大小。

| 数据   | 占用字节        |
| :--- | :---------- |
| data | data_length |

#### 第一类

##### 0x0001 比赛进程 

对应数据结构 game_robot_state_t，比赛进程信息

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
  uint8_t bullet_type;
  uint8_t bullet_freq;
  float   bullet_speed;
} real_shoot_data_t;
```

| 数据           | 说明            |
| ------------ | ------------- |
| bullet_type  | 弹丸类型          |
|              | 0x01: 17mm 弹丸 |
|              | 0x02: 42mm 弹丸 |
| bullet_freq  | 弹丸射频          |
| bullet_speed | 弹丸射速          |

##### 0x0004 实时功率

对应数据结构 real_power_data_t，实时功率、热量信息

```c
typedef __packed struct
{
  float chassis_volt;
  float chassis_current;
  float chassis_power;
  float chassis_pwr_buf;
  uint16_t shooter1_heat;
  uint16_t shooter2_heat;
} real_power_data_t;
```

ICRA 比赛中不使用这些数据，不做详细介绍。

##### 0x0005 场地交互

对应数据结构 field_rfid_t，场地交互数据

```c
typedef __packed struct
{
  uint8_t card_type;
  uint8_t card_idx;
} field_rfid_t;
```

| 数据        | 说明              |
| --------- | --------------- |
| card_type | 卡类型             |
|           | 11: ICRA 大符卡    |
|           | 其他: 非 ICRA 比赛用卡 |
| card_idx  | 卡索引号，ICRA中无效    |

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
  uint16_t buff_musk;
} get_buff_t;
```

| 数据        | 说明                          |
| --------- | --------------------------- |
| buff_musk | 全场buff信息，0 ~ 15位bit的数据，1为有效 |
|           | bit13: ICRA 己方获得buff        |
|           | bit14: ICRA 敌方获得buff        |

##### 0x0008 机器人位置

对应数据结构 robot_position_t，机器人的位置、角度信息

```c
typedef __packed struct
{
  float x;
  float y;
  float z;
  float yaw;
} robot_position_t;
```

| 数据   | 说明       |
| ---- | -------- |
| x    | 位置 X 坐标值 |
| y    | 位置 Y 坐标值 |
| z    | 位置 Z 坐标值 |
| yaw  | 枪口朝向角度值  |

#### 第二类

控制信息与推送信息：
详细指令查看application/infantry_cmd.h

```c

struct cmd_chassis_info
{
  int16_t gyro_angle;
  int16_t gyro_palstance;
  int32_t position_x_mm;
  int32_t position_y_mm;
  int16_t angle_deg;
  int16_t v_x_mm;
  int16_t v_y_mm;
};

struct cmd_gimbal_info
{
  uint8_t   mode;
  /* unit: degree */
  int16_t pitch_ecd_angle;
  int16_t yaw_ecd_angle;
  int16_t pitch_gyro_angle;
  int16_t yaw_gyro_angle;
  /* uint: degree/s */
  int16_t yaw_rate;
  int16_t pitch_rate;
};

struct cmd_gimbal_angle
{
  union{
    uint8_t flag;
    struct{
        uint8_t yaw_mode:1;  // 0 code angle
        uint8_t pitch_mode:1;
    }bit;
  } ctrl;
  int16_t pitch;
  int16_t yaw;
};

struct cmd_chassis_speed
{
  int16_t vx; // forward/back
  int16_t vy; // left/right
  int16_t vw; // anticlockwise/clockwise
  int16_t rotate_x_offset;
  int16_t rotate_y_offset;
};

struct cmd_chassis_spd_acc
{
  int16_t   vx; 
  int16_t   vy;
  int16_t   vw; 

  int16_t   ax; 
  int16_t   ay; 
  int16_t   wz; 
  
  int16_t rotate_x_offset;
  int16_t rotate_y_offset;
};

struct cmd_firction_speed
{
  uint16_t left;
  uint16_t right;
};

struct cmd_shoot_num
{
  uint8_t  shoot_cmd;
  uint32_t shoot_add_num;
  uint16_t shoot_freq;
};

```

## 数据发送和接收

### 1、数据发送

使用如下API配置协议和发送的数据：

```c
int32_t protocol_send_cmd_config(uint16_t cmd,
                                 uint8_t resend_times,
                                 uint16_t resend_timeout,
                                 uint8_t ack_enable,
                                 ack_handle_fn_t ack_callback,
                                 no_ack_handle_fn_t no_ack_callback);

int32_t protocol_rcv_cmd_register(uint16_t cmd, rcv_handle_fn_t rcv_callback);
int32_t protocol_rcv_cmd_unregister(uint16_t cmd);
int32_t protocol_send_cmd_unregister(uint16_t cmd);
uint32_t protocol_send_flush(void);
uint32_t protocol_unpack_flush(void);
uint32_t protocol_send_list_add_callback_reg(void_fn_t fn);

int32_t protocol_can_interface_register(char *interface_name,
                                        uint16_t rcv_buf_size,
                                        uint8_t boardcast_output_enable,
                                        uint8_t can_port,
                                        uint32_t can_tx_id,
                                        uint32_t can_rx_id,
                                        int (*can_send_fn)(uint32_t std_id, uint8_t *p_data, uint32_t len));
int32_t protocol_uart_interface_register(char *interface_name,
                                        uint16_t rcv_buf_size,
                                        uint8_t boardcast_output_enable,
                                        uint8_t com_port,
                                        int (*com_send_fn)(uint8_t *p_data, uint32_t len));

int32_t protocol_set_route(uint8_t tar_add, const char *name);
uint32_t protocol_can_rcv_data(uint8_t can_port, uint32_t rcv_id, void *p_data, uint32_t data_len);
uint32_t protocol_uart_rcv_data(uint8_t com_port, void *p_data, uint32_t data_len);

```

### 2、数据接收

使用下面的方式解决裁判系统粘包的问题

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
