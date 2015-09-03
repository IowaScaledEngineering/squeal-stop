/*************************************************************************
Title:    Flange Squeal Stopper
Authors:  Michael Petersen <railfan@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2015 Nathan D. Holmes & Michael D. Petersen

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

/*
  Gates the signal from a block detector (such as the CKT-IRSENSE 
  proximity sensor) and only allows it to trigger flange squeal 
  sounds from a CKT-SQUEAL sound player when the programmed DCC decoder
  address is set to a speed larger than a programmable threshold.
  Currently, it can only listen to a single, programmable DCC address
  making it suitable for small layouts with one train active at any time.

  This sketch uses the NmraDcc library by Alex Shepherd.
  https://github.com/mrrwa
*/

#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <NmraDcc.h>

#define BACKLIGHT_PIN        10
#define DETECTOR_INPUT_PIN_1   11  // D11 terminal on ARD-DCCSHIELD
#define DETECTOR_INPUT_PIN_2   12  // D12 terminal on ARD-DCCSHIELD
#define DETECTOR_INPUT_PIN_3   3   // JP4 on ARD-DCCSHIELD
#define SOUND_OUTPUT_PIN_1     13  // On the SF Redboard, this pin has an LED.  Therefore, it cannot be an input since the pullup cannot overcome the load.
#define SOUND_OUTPUT_PIN_2     A5  // SCL
#define SOUND_OUTPUT_PIN_3     A4  // SDA

#define EE_START   0x100

#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

#define STATE_NORMAL  0
#define STATE_SETUP   1
#define STATE_CONFIG  2
#define STATE_SAVE    3

#define KEYDELAY_DEFAULT   250
#define CONFIG_TIMEOUT     10000
#define LONGPRESS          5000

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

NmraDcc  Dcc ;
DCC_MSG  Packet ;

uint16_t listenAddr[16] = {0};
uint8_t listenAddrIndex = 0;
uint8_t currentSpeed = 0;
uint8_t speedThreshold = 3;

uint8_t backlightState = 1;

uint8_t state = STATE_NORMAL;
uint8_t updateDisplay = 0;

char str[17];

unsigned long previousMillis = 0;
unsigned long pressTime = 0;
uint16_t keyDelay = KEYDELAY_DEFAULT;
uint8_t previousButton = btnNONE;

//#define NOTIFY_DCC_MSG
#ifdef  NOTIFY_DCC_MSG
void notifyDccMsg( DCC_MSG * Msg)
{
	Serial.print("notifyDccMsg: ") ;
	for(uint8_t i = 0; i < Msg->Size; i++)
	{
		Serial.print(Msg->Data[i], HEX);
		Serial.write(' ');
	}
	Serial.println();
}
#endif

void notifyDccSpeed(uint16_t Addr, uint8_t Speed, uint8_t ForwardDir, uint8_t MaxSpeed)
{
	uint16_t realAddr = Addr & 0x3FFF;
	if(realAddr == listenAddr[listenAddrIndex])
	{
		currentSpeed = Speed;
	}
}

uint8_t F0_state = 1;
uint8_t F0_toggleCount = 0;

void notifyDccFunc(uint16_t Addr, FN_GROUP FuncGrp, uint8_t FuncState)
{
/*	uint16_t realAddr = Addr & 0x3FFF;*/
/*	if(FN_0_4 == FuncGrp)*/
/*	{*/
/*		if((!F0_state) && (FuncState & 0x10))*/
/*		{*/
/*			// F0 off, but packet says to turn on*/
/*			F0_toggleCount++;*/
/*			F0_state = 1;*/
/*		}*/
/*		else if(F0_state && ~(FuncState | 0xEF))*/
/*		{*/
/*			// F0 on, but packet says to turn off*/
/*			F0_toggleCount++;*/
/*			F0_state = 0;*/
/*		}*/
/*		*/
/*		if(F0_toggleCount > 10)*/
/*			listenAddr = Addr & 0x3FFF;*/
/*	}*/
		
		// Function 0
/*		Serial.print(" DCC Func Addr ");*/
/*		Serial.print(realAddr, DEC);*/
/*		Serial.print(" FuncState ");*/
/*		Serial.println(FuncState, HEX);*/
/*	if((0 == FuncNum) && (millis() < CONFIG_TIMEOUT))*/
/*	{*/
/*		listenAddr = Addr & 0x3FFF;*/
/*	}*/
}

uint8_t oldButton = btnNONE;

int lcdReadButtons()
{
	int adc = analogRead(0);
	uint8_t button = btnNONE;

/*	sprintf(str, "%4d", adc);*/
/*	lcd.setCursor(8,1);*/
/*	lcd.print(str);*/

	if (adc > 1000) button=btnNONE;
	else if (adc < 50)   button=btnRIGHT;
	else if (adc < 195)  button=btnUP;
	else if (adc < 380)  button=btnDOWN;
	else if (adc < 555)  button=btnLEFT;
	else if (adc < 790)
	{
		// SELECT
		// Only look for toggles
		if(oldButton != btnSELECT)
		{
			oldButton = btnSELECT;
			pressTime = millis();
			return btnSELECT;
		}
		else
		{
			// Check for long press
			if((millis() - pressTime) > LONGPRESS)
			{
				pressTime = millis();
				state = STATE_SETUP;
			}
			return btnNONE;
		}
	}
	
	oldButton = button;
	return button;
}

void setup()
{
	int i;

	Serial.begin(115200);

	lcd.begin(16, 2);

	pinMode(DETECTOR_INPUT_PIN_1, INPUT);      // Detector input
	pinMode(DETECTOR_INPUT_PIN_2, INPUT);
	pinMode(DETECTOR_INPUT_PIN_3, INPUT);
	digitalWrite(DETECTOR_INPUT_PIN_1, HIGH);  // Enable pull-up
	digitalWrite(DETECTOR_INPUT_PIN_2, HIGH);
	digitalWrite(DETECTOR_INPUT_PIN_3, HIGH);
	digitalWrite(SOUND_OUTPUT_PIN_1, HIGH);  // Preset sound module trigger high (it's an active low trigger)
	digitalWrite(SOUND_OUTPUT_PIN_2, HIGH);
	digitalWrite(SOUND_OUTPUT_PIN_3, HIGH);
	pinMode(SOUND_OUTPUT_PIN_1, OUTPUT);     // Sound module output
	pinMode(SOUND_OUTPUT_PIN_2, OUTPUT);
	pinMode(SOUND_OUTPUT_PIN_3, OUTPUT);

	pinMode(BACKLIGHT_PIN, OUTPUT);     // Backlight

	Dcc.pin(0, 2, 1);

	// Call the main DCC Init function to enable the DCC Receiver
//	Dcc.init( MAN_ID_DIY, 10, FLAGS_OUTPUT_ADDRESS_MODE | FLAGS_DCC_ACCESSORY_DECODER, 0 );
	Dcc.init( MAN_ID_DIY, 10, FLAGS_OUTPUT_ADDRESS_MODE, 0 );

	// Cheat
/*	listenAddr[0] = 250;*/
/*	listenAddr[1] = 303;*/
/*	listenAddr[2] = 600;*/
/*	listenAddr[3] = 850;*/
/*	listenAddr[4] = 413;*/
/*	listenAddr[5] = 401;*/
/*	listenAddr[6] = 601;*/
/*	listenAddr[7] = 602;*/
/*	listenAddr[8] = 325;*/
/*	listenAddr[9] = 457;*/
/*	listenAddr[10] = 473;*/
/*	listenAddr[11] = 300;*/
/*	listenAddr[12] = 309;*/
/*	listenAddr[13] = 436;*/
/*	listenAddr[14] = 9500;*/
/*	listenAddr[15] = 513;*/
/*	*/
/*	for(i=0; i<(sizeof(listenAddr)/sizeof(listenAddr[0])); i++)*/
/*	{*/
/*		EEPROM.update(EE_START + 2*i + 1, listenAddr[i] >> 8);*/
/*		EEPROM.update(EE_START + 2*i + 0, listenAddr[i] & 0xFF);*/
/*	}*/

	// Load address table from EEPROM
	for(i=0; i<(sizeof(listenAddr)/sizeof(listenAddr[0])); i++)
	{
		listenAddr[i] = ((uint16_t)EEPROM.read(EE_START + 2*i + 1) << 8) | EEPROM.read(EE_START + 2*i + 0);
		if(listenAddr[i] > 9999)
			listenAddr[i] = 0;
	}
}

void loop()
{
	int i;
	uint8_t buttonAvailable = 0;
	uint8_t button;
	
	Dcc.process();

	unsigned long currentMillis;

	digitalWrite(BACKLIGHT_PIN, backlightState);  // Enable

	currentMillis = millis();
	if((currentMillis - previousMillis) > keyDelay)
	{
		previousMillis = currentMillis;

		button = lcdReadButtons();

		if((btnNONE != button) && (button == previousButton))
		{
			if(keyDelay >= 10)
				keyDelay -= 10;
			else
				keyDelay = 0;
		}
		else
		{
			keyDelay = KEYDELAY_DEFAULT;
		}

		previousButton = button;
		buttonAvailable = 1;
	}

	switch(state)
	{
		case STATE_NORMAL:
			if(buttonAvailable)
			{
				buttonAvailable = 0;
				switch(button)
				{
					case btnRIGHT:
						if(((sizeof(listenAddr)/sizeof(listenAddr[0])) - 1) == listenAddrIndex)
							listenAddrIndex = 0;
						else
							listenAddrIndex++;
						currentSpeed = 0;
						break;
					case btnLEFT:
						if(0 == listenAddrIndex)
							listenAddrIndex = ((sizeof(listenAddr)/sizeof(listenAddr[0])) - 1);
						else
							listenAddrIndex--;
						currentSpeed = 0;
						break;
					case btnUP:
						if(speedThreshold < 128)
							speedThreshold++;
						break;
					case btnDOWN:
						if(speedThreshold)
							speedThreshold--;
						break;
					case btnSELECT:
						// Toggle backlight
						backlightState ^= 1;
						break;
					case btnNONE:
						break;
				}
			}

			if(currentSpeed > speedThreshold)
			{
				// Allow the detector to trigger flange squeal
				if(digitalRead(DETECTOR_INPUT_PIN_1))
				{
					digitalWrite(SOUND_OUTPUT_PIN_1, HIGH);
					lcd.setCursor(13,1);
					lcd.print("-");
				}
				else
				{
					digitalWrite(SOUND_OUTPUT_PIN_1, LOW);
					lcd.setCursor(13,1);
					lcd.print("+");
				}

				if(digitalRead(DETECTOR_INPUT_PIN_2))
				{
					digitalWrite(SOUND_OUTPUT_PIN_2, HIGH);
					lcd.setCursor(14,1);
					lcd.print("-");
				}
				else
				{
					digitalWrite(SOUND_OUTPUT_PIN_2, LOW);
					lcd.setCursor(14,1);
					lcd.print("+");
				}

				if(digitalRead(DETECTOR_INPUT_PIN_3))
				{
					digitalWrite(SOUND_OUTPUT_PIN_3, HIGH);
					lcd.setCursor(15,1);
					lcd.print("-");
				}
				else
				{
					digitalWrite(SOUND_OUTPUT_PIN_3, LOW);
					lcd.setCursor(15,1);
					lcd.print("+");
				}
			}
			else
			{
				// Block the flange squeal
				digitalWrite(SOUND_OUTPUT_PIN_1, HIGH);
				lcd.setCursor(13,1);
				lcd.print("-");
				digitalWrite(SOUND_OUTPUT_PIN_2, HIGH);
				lcd.setCursor(14,1);
				lcd.print("-");
				digitalWrite(SOUND_OUTPUT_PIN_3, HIGH);
				lcd.setCursor(15,1);
				lcd.print("-");
			}

			sprintf(str, "ADR:%4d", listenAddr[listenAddrIndex]);
			lcd.setCursor(0,0);
			lcd.print(str);

			sprintf(str, "THR:%3d", speedThreshold);
			lcd.setCursor(9,0);
			lcd.print(str);

			sprintf(str, "SPD:%3d", currentSpeed);
			lcd.setCursor(0,1);
			lcd.print(str);
			break;
		case STATE_SETUP:
			backlightState = 1;
			listenAddrIndex = 0;
			lcd.clear();
			updateDisplay = 1;
			state = STATE_CONFIG;
			break;
		case STATE_CONFIG:
			if(buttonAvailable)
			{
				buttonAvailable = 0;
				switch(button)
				{
					case btnRIGHT:
						if(((sizeof(listenAddr)/sizeof(listenAddr[0])) - 1) == listenAddrIndex)
							listenAddrIndex = 0;
						else
							listenAddrIndex++;
						updateDisplay = 1;
						break;
					case btnLEFT:
						if(0 == listenAddrIndex)
							listenAddrIndex = ((sizeof(listenAddr)/sizeof(listenAddr[0])) - 1);
						else
							listenAddrIndex--;
						updateDisplay = 1;
						break;
					case btnUP:
						if(listenAddr[listenAddrIndex] < 9999)
							listenAddr[listenAddrIndex]++;
						updateDisplay = 1;
						break;
					case btnDOWN:
						if(listenAddr[listenAddrIndex] > 0)
							listenAddr[listenAddrIndex]--;
						updateDisplay = 1;
						break;
					case btnSELECT:
						// Finish
						state = STATE_SAVE;
						break;
					case btnNONE:
						break;
				}
			}

			if(updateDisplay)
			{
				updateDisplay = 0;
				sprintf(str, "Config Addr %2d", listenAddrIndex+1);
				lcd.setCursor(0,0);
				lcd.print(str);
				sprintf(str, "%4d", listenAddr[listenAddrIndex]);
				lcd.setCursor(6,1);
				lcd.print(str);
			}

			break;
		case STATE_SAVE:
			lcd.clear();
			lcd.setCursor(0,0);
			lcd.print("Saving...");
			// Write address table to EEPROM
			for(i=0; i<(sizeof(listenAddr)/sizeof(listenAddr[0])); i++)
			{
				EEPROM.update(EE_START + 2*i + 1, listenAddr[i] >> 8);
				EEPROM.update(EE_START + 2*i + 0, listenAddr[i] & 0xFF);
			}
			delay(1000);
			lcd.clear();
			state = STATE_NORMAL;
			break;
	}
}

