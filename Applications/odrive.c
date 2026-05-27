#include "odrive.h"

OdriveAxisSetState_t odrive_set_axis0;
OdriveAxisSetState_t odrive_set_axis1;

State_t odrive_axis0_state;
State_t odrive_axis1_state;

void odrive_set_speed(float speed) // TODO: check
{
//   if(speed < 0)
//     return;
  odrive_set_axis0.input_vel = speed; 
  //odrive_set_axis1.input_vel = speed; // TODO: 检查电机的旋转方向

  odrv_write_msg(AXIS_0, MSG_SET_INPUT_VEL);  //MSG_SET_INPUT_VEL = 0x00D,设置速度
  //odrv_write_msg(AXIS_1, MSG_SET_INPUT_VEL);
}

uint8_t odrv_write_msg(Axis_t axis, Odrive_Commond cmd)
{
  extern CAN_HandleTypeDef hcan1;

  CAN_TxHeaderTypeDef header;
  uint32_t send_mail_box;
  OdriveAxisSetState_t *odrive_set;//轴信息

  uint8_t data[8] = {0};
  float_to_uint8_t pack;   // 共同体
  uint8_t tmp_word[4];
	
  header.IDE = CAN_ID_STD;
  if(axis == AXIS_0)	
  {
    header.StdId = AXIS0_NODE_ID  + cmd;
    odrive_set = &odrive_set_axis0;
  }
  else if(axis == AXIS_1)
  {
    header.StdId = AXIS1_NODE_ID  + cmd;
    odrive_set = &odrive_set_axis1;
  }
  else
  {
    return 0;    // 0 —> Error
  }   

  switch(cmd)
  {
    case MSG_ODRIVE_ESTOP:
        /* TODO: Implement */
        break;
    case MSG_GET_MOTOR_ERROR:
        header.RTR = CAN_RTR_REMOTE;
        header.DLC = 0;
        break;
    case MSG_GET_ENCODER_ERROR:
        header.RTR = CAN_RTR_REMOTE;
        header.DLC = 0;
        break;
    case MSG_GET_SENSORLESS_ERROR:
        /* TODO: Implement */
        break;
    case MSG_SET_AXIS_NODE_ID:
        /* TODO: Implement */
        break;
    case MSG_SET_AXIS_REQUESTED_STATE:
        memcpy(data, &(odrive_set->requested_state), 4);
        header.RTR = CAN_RTR_DATA;
        header.DLC = 4;
        break;
    case MSG_SET_AXIS_STARTUP_CONFIG:
        /* TODO: Implement */
        break;
    case MSG_GET_ENCODER_ESTIMATES:
        header.RTR = CAN_RTR_REMOTE;
        header.DLC = 0;
        break;
    case MSG_GET_ENCODER_COUNT:
        header.RTR = CAN_RTR_REMOTE;  //数据帧的优先级大于远程帧，当有多个设备向一个ID同时发送数据时，用远程帧可以有效地避免总线冲突，即ID收到消息后会先回发数据帧，再处理远程帧
        header.DLC = 0;
        break;
    case MSG_SET_CONTROLLER_MODES:
        data[0] = odrive_set->control_mode;
        data[4] = odrive_set->input_mode;
        header.RTR = CAN_RTR_DATA;
        header.DLC = 8;
        break;
    case MSG_SET_INPUT_POS:
				memcpy(data, &(odrive_set->input_pos), 4);//4个字节 32位 ，input_pos也是32位，复制到data数组中  uint8_t data[8]   会将数组前四位补齐
				/*
				void *memcpy(void *destin, void *source, unsigned n);
				以source指向的地址为起点，将连续的n个字节数据，复制到以destin指向的地址为起点的内存中。
				函数有三个参数，第一个是目标地址，第二个是源地址，第三个是数据长度。
				数据长度（第三个参数）的单位是字节（1byte = 8bit）。
				*/
				//odrive_set->vel_ff  vel_ff有16位 两个字节
        data[4] = odrive_set->vel_ff & 0x00FF;//一个数字与0xFF进行与操作后结果还是原值
				//packetlength & 0x00FF中的&是先把&两边的值转换成二进制形式，然后在进行按位运算，其中按位计算，&两边操作数对应位上全为1时，结果的该位值为1。否则该位值为0。
        data[5] = odrive_set->vel_ff >> 8;
        data[6] = odrive_set->current_ff & 0x00FF;
        data[7] = odrive_set->current_ff >> 8;
        header.RTR = CAN_RTR_DATA;
        header.DLC = 8;
        break;
    case MSG_SET_INPUT_VEL:  // 0x00D,设置速度        input_vel  torque_vel均为32位    float value[2];   float有4个字节 32位   
				//pack是联合体  最后发的pack.raw   uint8_t raw[8];     32位的value会转换成8位的raw 将raw[8]每一位都填满
        pack.value[0] = odrive_set->input_vel;  // odrive_set_axis0.input_vel;单位为turn/s      
        pack.value[1] = odrive_set->torque_vel; // odrive_set_axis0.torque_vel;
        header.RTR = CAN_RTR_DATA;
        header.DLC = 0x08;
        break;
    case MSG_SET_INPUT_CURRENT:
        memcpy(data, &(odrive_set->input_current), 4);
        header.RTR = CAN_RTR_DATA;
        header.DLC = 4;
        break;
    case MSG_SET_VEL_LIMIT:
        memcpy(data, &(odrive_set->vel_limit), 4);
        header.RTR = CAN_RTR_DATA;
        header.DLC = 4;
        break;
    case MSG_START_ANTICOGGING:
        header.RTR = CAN_RTR_REMOTE;
        header.DLC = 0;
        break;
    case MSG_SET_TRAJ_VEL_LIMIT://梯形模式速度
        memcpy(data, &(odrive_set->traj_vel_limit), 4);
        header.RTR = CAN_RTR_DATA;
        header.DLC = 4;
        break;
    case MSG_SET_TRAJ_ACCEL_LIMITS:
        memcpy(data, &(odrive_set->traj_accel_limit), 4);
        memcpy(tmp_word, &(odrive_set->traj_decel_limit), 4);
        data[4] = tmp_word[0];
        data[5] = tmp_word[1];
        data[6] = tmp_word[2];
        data[7] = tmp_word[3];
        header.RTR = CAN_RTR_DATA;
        header.DLC = 4;
        break;
    case MSG_SET_TRAJ_A_PER_CSS:
        memcpy(data, &(odrive_set->traj_a_per_css), 4);
        header.RTR = CAN_RTR_DATA;
        header.DLC = 4;
        break;
    case MSG_GET_IQ:
        /* TODO: Implement */
        break;
    case MSG_GET_SENSORLESS_ESTIMATES:
        /* TODO: Implement */
        break;
    case MSG_RESET_ODRIVE:
				header.RTR = CAN_RTR_REMOTE;//remote为遥控帧 请求获取数据
        header.DLC = 0;
        break;
    case MSG_GET_VBUS_VOLTAGE:
        header.RTR = CAN_RTR_REMOTE;
        header.DLC = 0;
        break;
    case MSG_CLEAR_ERRORS:
        header.RTR = CAN_RTR_REMOTE;
        header.DLC = 0;
        break;
    case MSG_CO_HEARTBEAT_CMD:
        /* TODO: Implement */
        break;
    default:
        break;
  }
   if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)  //若当前有空闲の发送邮箱数,则发送数据
   {
    HAL_CAN_AddTxMessage(&hcan1, &header, pack.raw, &send_mail_box); //CAN发送函数 参数三发的是数据段 根据can通信的定义 数据段有8个Byte(字节)，每个字节有8位 即uint32_t
     return 1;
   }
  return 0;
} 

