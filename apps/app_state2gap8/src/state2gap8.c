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
  uint8_t request_id = cpxRx->data[0];
  //DEBUG_PRINT("Got request from GAP8 (request_id: %u)\n", request_id);

  cpxInitRoute(CPX_T_STM32, CPX_T_GAP8, CPX_F_APP, &txStatePacket.route);
  txStatePacket.data[0] = request_id;
  memcpy(txStatePacket.data+1, taskEstimatorState_p, sizeof(state_t));
  txStatePacket.dataLength = 1+sizeof(state_t);
  cpxSendPacketBlocking(&txStatePacket);
}
