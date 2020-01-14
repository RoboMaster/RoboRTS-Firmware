## protocol instruction

### protocol data

protocol data could separate to 2 category depend on transmition direction:

#### 1.lower layer send to upper layer：

1 Feedback information: including feedback information of various agencies' sensors and some feedback information calculated by the bottom layer;

2 Underlying state information: including the running state of the underlying device, some response from the underlying layer to the upper layer data, etc.;

3 Forward data: contains all the information of the referee system, customized information on the server side;

#### 2. lower layer received from upper layer:

1 Control information: upper layer control information on the bottom three actuators;

## protocol data instruction

One frame of protocol data is divided into four parts, frame header data, command code ID, data, and frame end check data.

### 1、frame header data

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


| frame header data| Size in Byte | description                            					|
| :----------------| :------------| :------------------------------------------------------ |
| sof          	   | 1            | Domain ID of the data                                   |
| ver_data_len 	   | 2            | Length of data in each frame and protocol version number|
| session          | 1            | Packet sequence number, reserved in the 0xA0 field      |
| sender           | 1            | sender address                                          |
| reciver          | 1            | receiver address                                        |
| res              | 2            | reserved bit                                            |
| seq              | 2            | packet sequence number                                  |
| crc16            | 2            | Frame header CRC  check result                          |


### 2、Command code

| command code | size in byte |
| :----------- | :----------- |
| cmdid 	   | 2            |

A command code corresponds to one frame of data containing specific information, and the following is a command code corresponding to all existing data.


The data transmission direction and specific functions corresponding to the command code are as follows:

| command code    | transmition direction    | function            						| frequency             					|
| :-------------- | :----------------------- | :--------------------------------------- | :---------------------------------------- |
| 0x0001          | main control-->PC        | robot state in competition               | referee system 10Hz       				|
| 0x0002          | main control-->PC        | real-time damage data           		    | when taking damage        				|
| 0x0003          | main control-->PC        | real-time shooting data                  | referee system           					|
| 0x0004          | main control-->PC        | real-time power consumption, heat data   | won't use in ICRA    						|
| 0x0005          | main control-->PC        | field RFID data    					    | when IC card sending detected 			|
| 0x0006          | main control-->PC        | competition result data           		| at the end of game        				|
| 0x0007          | main control-->PC        | get buff data      						| referee system           					|
| 0x0008          | main control-->PC        | field UWB data        					| referee system           					|
|                 |                          |                  						|                							|
| 0x0204          | main control-->PC        | robot chassis information        		| 100Hz				         				|
| 0x0304          | main control-->PC        | robot gimbal information        			| 100Hz         							|
| 0x0402          | main control-->PC        | UWB information        					| UWB refresh frequency         			|
|                 |                          |                  						|                							|
| 0x0206          | PC-->main control        | set chassis speed        				|          									|
| 0x0208          | PC-->main control        | set chassis speed (with acceleration)  	|      										|
| 0x0309          | PC-->main control        | control the speed of friction wheel      | at the beginning of using friction wheel  |
| 0x030A          | PC-->main control        | control the shooting    					|         									|
| 0x0403          | PC-->main control        | gimbal calibration information         	| when gimbal calibration needed	        |

### 3、Data  

For the data structure corresponding to the command code ID, the data length is the size of this structure.

| data   | size in byte |
| :----- | :----------- |
| data   | data_length  |

#### Type 1

referee system user data
for more details please check referee system manual

#### Type 2

control and push information
for more details please check application/infantry_cmd.h

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

## Data sending and receiving

### 1、data sending

use API below to configure protocol and the sending data:

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
                                        uint8_t broadcast_output_enable,
                                        uint8_t can_port,
                                        uint32_t can_tx_id,
                                        uint32_t can_rx_id,
                                        int (*can_send_fn)(uint32_t std_id, uint8_t *p_data, uint32_t len));
int32_t protocol_uart_interface_register(char *interface_name,
                                        uint16_t rcv_buf_size,
                                        uint8_t broadcast_output_enable,
                                        uint8_t com_port,
                                        int (*com_send_fn)(uint8_t *p_data, uint32_t len));

int32_t protocol_set_route(uint8_t tar_add, const char *name);
uint32_t protocol_can_rcv_data(uint8_t can_port, uint32_t rcv_id, void *p_data, uint32_t data_len);
uint32_t protocol_uart_rcv_data(uint8_t com_port, void *p_data, uint32_t data_len);

```

### 2、Data receiving

use the method below to solve referee system sticky problem

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
