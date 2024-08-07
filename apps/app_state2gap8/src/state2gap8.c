/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2023 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * App layer application that communicates with the GAP8 on an AI deck.
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "cpx.h"
#include "cpx_internal_router.h"

#include "FreeRTOS.h"
#include "task.h"

#include "stabilizer_types.h"

#define DEBUG_MODULE "APP"
#include "debug.h"

// Callback that is called when a CPX packet arrives
static void cpxPacketCallback(const CPXPacket_t* cpxRx);

// CPX packet pointer
static CPXPacket_t txStatePacket;

// State to be sent to the aideck gap8. This is the global pointer in estimator_kalman.c
extern state_t *taskEstimatorState_p;

void appMain() {
  DEBUG_PRINT("Hello! I am the state2gap8 app\n");

  // Register a callback for CPX packets.
  // Packets sent to destination=CPX_T_STM32 and function=CPX_F_APP will arrive here
  cpxRegisterAppMessageHandler(cpxPacketCallback);

  //uint8_t counter = 0;
  while(1) {
    vTaskDelay(M2T(2000));

    //cpxInitRoute(CPX_T_STM32, CPX_T_GAP8, CPX_F_STATE, &txPacket.route);
    //txPacket.data[0] = counter;
    //txPacket.dataLength = 1;

    //cpxSendPacketBlocking(&txPacket);
    //DEBUG_PRINT("Sent packet to GAP8 (%u)\n", counter);
    //counter++;
  }
}

static void cpxPacketCallback(const CPXPacket_t* cpxRx) {
    /** 
   * Send the state to the aideck gap8 for the gap8 to embed in the image
   */
  cpxInitRoute(CPX_T_STM32, CPX_T_GAP8, CPX_F_APP, &txStatePacket.route);
  // Offset used to copy values to the data array
  uint8_t offset = 0;

  // request_id
  memcpy(txStatePacket.data+offset, cpxRx->data, sizeof(uint8_t));
  offset += sizeof(uint8_t);

  // attitude.roll
  memcpy(txStatePacket.data+offset, &taskEstimatorState_p->attitude.roll, sizeof(float));
  offset += sizeof(float);

  // attitude.pitch
  memcpy(txStatePacket.data+offset, &taskEstimatorState_p->attitude.pitch, sizeof(float));
  offset += sizeof(float);

  // attitude.yaw
  memcpy(txStatePacket.data+offset, &taskEstimatorState_p->attitude.yaw, sizeof(float));
  offset += sizeof(float);

  // attitudeQuaternion.x
  memcpy(txStatePacket.data+offset, &taskEstimatorState_p->attitudeQuaternion.x, sizeof(float));
  offset += sizeof(float);

  // attitudeQuaternion.y
  memcpy(txStatePacket.data+offset, &taskEstimatorState_p->attitudeQuaternion.y, sizeof(float));
  offset += sizeof(float);

  // attitudeQuaternion.z
  memcpy(txStatePacket.data+offset, &taskEstimatorState_p->attitudeQuaternion.z, sizeof(float));
  offset += sizeof(float);

  // attitudeQuaternion.w
  memcpy(txStatePacket.data+offset, &taskEstimatorState_p->attitudeQuaternion.w, sizeof(float));
  offset += sizeof(float);

  // position.timestamp
  memcpy(txStatePacket.data+offset, &taskEstimatorState_p->position.timestamp, sizeof(uint32_t));
  offset += sizeof(uint32_t);

  // position.x
  memcpy(txStatePacket.data+offset, &taskEstimatorState_p->position.x, sizeof(float));
  offset += sizeof(float);

  // position.y
  memcpy(txStatePacket.data+offset, &taskEstimatorState_p->position.y, sizeof(float));
  offset += sizeof(float);

  // position.z
  memcpy(txStatePacket.data+offset, &taskEstimatorState_p->position.z, sizeof(float));
  offset += sizeof(float);

  // velocity.timestamp
  memcpy(txStatePacket.data+offset, &taskEstimatorState_p->velocity.timestamp, sizeof(uint32_t));
  offset += sizeof(uint32_t);

  // velocity.x
  memcpy(txStatePacket.data+offset, &taskEstimatorState_p->velocity.x, sizeof(float));
  offset += sizeof(float);

  // velocity.y
  memcpy(txStatePacket.data+offset, &taskEstimatorState_p->velocity.y, sizeof(float));
  offset += sizeof(float);

  // velocity.z
  memcpy(txStatePacket.data+offset, &taskEstimatorState_p->velocity.z, sizeof(float));
  offset += sizeof(float);

  // acc.timestamp
  memcpy(txStatePacket.data+offset, &taskEstimatorState_p->acc.timestamp, sizeof(uint32_t));
  offset += sizeof(uint32_t);

  // acc.x
  memcpy(txStatePacket.data+offset, &taskEstimatorState_p->acc.x, sizeof(float));
  offset += sizeof(float);

  // acc.y
  memcpy(txStatePacket.data+offset, &taskEstimatorState_p->acc.y, sizeof(float));
  offset += sizeof(float);

  // acc.z
  memcpy(txStatePacket.data+offset, &taskEstimatorState_p->acc.z, sizeof(float));
  offset += sizeof(float);

  // Send the packet
  txStatePacket.dataLength = offset;
  cpxSendPacketBlocking(&txStatePacket);
}
