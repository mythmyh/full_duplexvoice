/*
 * audio_player.h
 *
 *  Created on: Jun 9, 2020
 *      Author: admin
 */

#ifndef INC_AUDIO_PLAYER_H_
#define INC_AUDIO_PLAYER_H_

#include "main.h"
void Audio_Player_Init3(I2C_HandleTypeDef*);
void Audio_Player_Init(I2C_HandleTypeDef*);
void Audio_Player_Start(void);
void Audio_Player_Pause(void);
void Audio_Player_Resume(void);
void Audio_Player_Stop(void);
void Audio_Player_Start2(void);

#endif /* INC_AUDIO_PLAYER_H_ */
