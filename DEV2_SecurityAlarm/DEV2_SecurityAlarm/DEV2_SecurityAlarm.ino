/*
 * DEV2_SecurityAlarm.ino
 *
 * Created: 7/3/2018 12:34:26 PM
 * Author: tyutchenko_ai
 * �������� ������������ �� FreeRTOS
 * ����������� � ������ ����� ��������
 * �������� ��� ����
 */ 

#define DEBUG_ON

#include <Arduino_FreeRTOS.h>
#include <timers.h>																					//������� FreeRTOS
#include <semphr.h>																					//�������� FreeRTOS
#include <MsTimer2.h>																				//���������� �� ������� 2
#include <ArcButton/AntiBounceButtonPortB.h>														//���� �������� ������ �� ����� B
#include <BackgroundADC.h>																			//���������� ����� � ����
#include <EEPROM.h>

//���������� �������
//������ 1
#define BUTTON1_PIN _BV(PB1)																		//���������� ������ �1 �� dPin 9 (PB1)
//������ 2
#define BUTTON2_PIN _BV(PB2)																		//���������� ������ �2 �� dPin 10 (PB2)
//���������
#define PORT_MODE_LED DDRB																			//���������� ��������� �� dPin 13 (PortB5)
#define PORT_STATE_LED PORTB																		//���������� ��������� �� dPin 13 (PortB5)
#define LED_PIN _BV(PB5)																			//���������� ��������� �� dPin 13 (PB5)
//������
#define PORT_MODE_SIREN DDRB																		//���������� ������ �� dPin 12 (PortB4)
#define PORT_STATE_SIREN PORTB																		//���������� ������ �� dPin 12 (PortB4)
#define SIREN_PIN _BV(PB4)																			//���������� ������ �� dPin 12 (PB4)
//������
#define SHLEIF1_PIN A0																				//���������� ����� 1 �� aPin 0
#define SHLEIF2_PIN A1																				//���������� ����� 1 �� aPin 1

//���������
const TickType_t FAST_TIME_FLASH_LED = 100;															//������ ������� ������� ���������� � ��
const TickType_t SLOW_TIME_FLASH_LED = 500;															//������ ������� ������� ���������� � ��
const TickType_t TIME_SET_GUARD	= 20000;															//������ �������� ����� ��� ���������� �� ������ � ��
const TickType_t TIME_ALARM = 60000;																//������ ������ ������� ������� � ��
const TickType_t TIME_BLOCK_ALARM = 30000;															//������ �������� ����� ��� �������� �������� � ��
const TickType_t SET_CODE_TIME = 6000;																//������ ��������� ������ ��� �������� � ����� ����� ���������� ���� � ��
const TickType_t TIME_SET_CODE = 5000;																//������ �������� ������� ������ ��� ����� ���������� ���� � ��
const unsigned int MAX_U = 36864;																	//������� ������ ���������� ������� (3,6�, 3,6*1024/5*50=36864)
const unsigned int MIN_U = 10650;																	//������ ������ ���������� ������� (1,04�, 1,04*1024/5*50=10650)
const byte averageCountNUM = 50;																	//���������� ������� ��� ������� ��� ����������
//������ EEPROM
const byte COD_ADR = 2;																				//����� ���������� ���� � EEPROM
const byte NUMBER_ADR = 4;																			//����� ����� ����� ���������� ���� � EEPROM

//����������
byte mode = 0;																						//����� ������ ��������� �����
byte secretCode = 0;																				//���������� ��� ���������� ����
byte bitNum = 0;																					//����� ������� ������ ��� ����� ���������� ���� (������ ����)
unsigned int sumADC[2];																				//���������� ��� ������������ ����� ���
unsigned int averageADC[2];																			//���������� ���������� ����� ���
boolean flagReadyADC = false;																		//������� ���������� ������ �� ������ ������� ���

TaskHandle_t xTaskScanSensors = NULL;																//���������� ��� ���������� ������ ����� ��������
TaskHandle_t xTaskControlSafety = NULL;																//���������� ��� ���������� ����� ���������� �������
TimerHandle_t xTimerBlynkLed = NULL;																//���������� ��� ���������� ������� ������� ����������
TimerHandle_t xTimerArming = NULL;																	//���������� ��� ���������� ������� ���������� �� ������
SemaphoreHandle_t xSemaphoreScanSensors = NULL;														//���������� ��� ���������� �������� ������� � ������ �������� ������ Button

//TickType_t uiTimerBlynkLedPeriod = FAST_TIME_FLASH_LED/portTICK_PERIOD_MS;							//���������� ��� ������� ������� ������� ���������� � ��������������
boolean flagArmingReady = false;																	//������� ��������� ������� �������� ���������� �� ������

//�������
AntiBounceButtonPortB button1(BUTTON1_PIN, 25);														//������ �1 (�����, ���-�� �������� ������������ �� �������)
AntiBounceButtonPortB button2(BUTTON2_PIN, 25);														//������ �2 (�����, ���-�� �������� ������������ �� �������)

//��������� �������
void vTaskScanSensors(void* pvParameters);															//������� ������ ����� ��������
void vTaskControlSafety(void* pvParameters);														//������� ������ �������� ������
void vTimerBlynkLed(TimerHandle_t xTimer);															//������� ������� ������� �����������
void vTimerArming(TimerHandle_t xTimer);															//������� ������� ���������� �� ������
void timerInterrupt(void);																			//������� ���������� �� ������� 2
boolean secretCodeCheck(void);																		//������� �������� ������������ ���������� ����

void setup()
{
	//���������
	#ifdef DEBUG_ON
		Serial.begin(9600);
	#endif /*	DEBUG_ON	*/
	PORT_MODE_LED |= LED_PIN;																		//��� ���������� � ����� ������ 
	PORT_STATE_LED &= ~LED_PIN;																		//������������� ���� ���������� (���������)
	PORT_MODE_SIREN |= SIREN_PIN;																	//��� ������ � ����� ������
	PORT_STATE_SIREN &= ~SIREN_PIN;																	//������������� ���� ������ (���������)
	MsTimer2::set(2, timerInterrupt);																//������ ��������� ���������� �� ������� 2 ��
	MsTimer2::start();

	//�������� ���������
	//������� ������ �������� ������ Button ********************************************************
	xSemaphoreScanSensors = xSemaphoreCreateBinary();
	if (xSemaphoreScanSensors == NULL)
	{
		//������� �� ������
		#ifdef DEBUG_ON
			Serial.println(F("There was insufficient FreeRTOS heap available for the semaphore to be created successfully."));
		#endif /*	DEBUG_ON	*/
	} 
	else
	{
		//������� ������� ������
	}	//------------------------------------------------------------------------------------------
	
	//�������� ��������
	//������ ������� ����������� *******************************************************************
	xTimerBlynkLed = xTimerCreate("TimerBlynkLed",													//���������� ���
									FAST_TIME_FLASH_LED/portTICK_PERIOD_MS,							//������ (�������� � ������ ��������, ����� ������)
									pdTRUE,															//�������������
									0,																//ID
									vTimerBlynkLed);												//������� �������
	if (xTimerBlynkLed == pdTRUE)
	{
		//������ �� ������
		#ifdef DEBUG_ON
			Serial.println(F("The timer was not created."));
		#endif /*	DEBUG_ON	*/
	}
	else
	{
		//������ ������� ������
	}	//------------------------------------------------------------------------------------------


	//������ ���������� �� ������*******************************************************************
	xTimerArming = xTimerCreate("TimerArming",
								SET_CODE_TIME/portTICK_PERIOD_MS,									//������ (�������� � �������� �������� �������� � ����� ����� ���������� ����, ����� ������)
								pdFALSE,															//������������
								0,
								vTimerArming);
	if (xTimerArming == pdTRUE)
	{
		//������ �� ������
		#ifdef DEBUG_ON
			Serial.println(F("The timer was not created."));
		#endif /*	DEBUG_ON	*/
	}
	else
	{
		//������ ������� ������
	}	//------------------------------------------------------------------------------------------
	
	
	//�������� �����
	//������ ����� ��������*************************************************************************
	if (xTaskCreate(vTaskScanSensors,
					"TaskScanSensors",
					configMINIMAL_STACK_SIZE,
					NULL,
					2,																				//���������
					&xTaskScanSensors) != pdTRUE)
	{
		//������ �� �������
		#ifdef DEBUG_ON
			Serial.println(F("The task could not be created because there was insufficient heap memory remaining."));
		#endif /*	DEBUG_ON	*/
	} 
	else
	{
		//������ �������
	}	//------------------------------------------------------------------------------------------
	
	//������ ����� �������� ������******************************************************************
	if (xTaskCreate(vTaskControlSafety,
					"TaskControlSafety",
					configMINIMAL_STACK_SIZE,
					NULL,
					1,																				//���������
					&xTaskControlSafety) != pdTRUE)
	{
		//������ �� �������
		#ifdef DEBUG_ON
			Serial.println(F("The task could not be created because there was insufficient heap memory remaining."));
		#endif /*	DEBUG_ON	*/
	}
	else
	{
		//������ �������
	}	//------------------------------------------------------------------------------------------

	//����� ���������� ������� �������������� �������� ��������� ������ �������� FreeRTOS �...
	//����� ���������� ������� �������������� ����� ������������ FreeRTOS
	//vTaskStartScheduler();
}

void loop()
{
	/*	�� ����������	*/
}

//����������� �������

/* ������� ������-����������� ���������� ������� 2:
* - ��������� ������ �� ������� � ������� ������� ���������� AntiBounceButtonPortB.h,
* - ��������� ������ �� ������ ��� � ����������� ������ � ������� �������������� ������� � ������� ������� ���������� BackgroundADC.h
* ��������� 2
* ���������� � �������� 2 �� (��� ������������� ������� �������� �� ��������� ������� 2)
* ������������� ������� ��������� ������ flagPress � ������� ������� flagClick �������� ������ Button
* ������������� ������� ���������� ������ ������ ������� ��� flagReadyADC, ����������� ����������� ������ ��������� � ������� ���������� ���������� averageADC[]
*/
void vTaskScanSensors(void* pvParameters)
{
	static byte averageCount = 0;																	//������� ������� ��� ������� ��� ����������
	static byte chanelADC = 0;																		//����� ������ ���	
	
	for (;;)
	{
		xSemaphoreTake(xSemaphoreScanSensors, portMAX_DELAY);										//���� ������� �� ���������� �� ������� 2, ���� ��� �� ��������
		//�������� � ��������
		button1.filterAverage();
		button2.filterAverage();
		
		//�������� �� �������� ���
		sumADC[chanelADC] += BackgroundADC.analogRead();											//����������� ���������� ������� ���
		chanelADC++;																				//��������� ����� ���
		if (chanelADC > 1) chanelADC = 0;															//��������� ����� ������� ���
		BackgroundADC.analogStart(chanelADC);														//������ ��������� ������ ���
		if (chanelADC == 0)
		{
			averageCount++;
			if (averageCount > averageCountNUM)														//��������� ������ ���������� ������� ��� ����������
			{
				averageCount = 0;
				averageADC[0] = sumADC[0];															//���������� ����������� ��������
				averageADC[1] = sumADC[1];
				sumADC[0] = 0;
				sumADC[1] = 0;
				flagReadyADC = true;																//������� ���������� ������ ������� ������� ��� ��� �������������
			}
		}
	}
	vTaskDelete(NULL);																				//��������� �������� ������� ������, ���� ����� ������� �� ����� � ����
}


/*������� ������ ����� �������� ������
* ��������� 1
* ������������ ��������� ������
* ��������� ������ ������� ����������� ����� (0,1 ���) �� 20 ���, ���� ���� ������ 2 ������
* ����� 20 ������ �������� ������ ������� ��� ������� (1 ���) �������
* ���� � ������� 20 ��� ������ ����� ������, �� ������������� ������ �������, ����� ���������
*/
void vTaskControlSafety (void* pvParameters)
{
	#ifdef DEBUG_ON
		vTaskSetApplicationTaskTag(NULL, (void*)2);													//���������� ���
	#endif /*	DEBUG_ON	*/
	
	boolean flagTwoButtons = false;																	//������� ������� ���� ������
	
	for (;;)
	{
		//����������� ������� ���������� �� ������ ��� �������� ������� ���� ������ ������
		//������� flagTwoButtons ��������������� ��� ������� ������ �1 � �2
		
		/*#ifdef DEBUG_ON
			Serial.print(flagTwoButtons);															//���������� �����
			Serial.print("	");
			Serial.print(button1.flagPress);
			Serial.print("	");
			Serial.println(button2.flagPress);
		#endif / *	DEBUG_ON	* /*/
		
		//----------����� 0 = ���������-------------------------------------------------------------
		// - ��������� �� �����
		// - ������ ���������
		// - ���� ������ ��� ������ ������ ������������ ������� � ����� 1 (���������� �� ������)
		// - ���� ������ ��� ������ ������ ������������ �����, ��� 6 ��� ������� � ����� 5 (��������� ���������� ����)
		if (mode == 0)
		{
			#ifdef DEBUG_ON
				Serial.print(F("Mode = "));															//���������� �����
				Serial.println(mode);
				//Serial.print(F("   flagArmingReady = "));
				//Serial.print(flagArmingReady);
				//Serial.print(F("   TimerArmingActive = "));
				//Serial.println(xTimerIsTimerActive(xTimerArming));
			#endif /*	DEBUG_ON	*/
			
			PORT_STATE_LED &= ~LED_PIN;																//��������� �� �����
			PORT_STATE_SIREN &= ~SIREN_PIN;															//������ ���������
			
			if ((button1.flagPress == true) && (button2.flagPress == true))
			{
				flagTwoButtons = true;
				if (!xTimerIsTimerActive(xTimerArming))
				{
					xTimerChangePeriod(xTimerArming, SET_CODE_TIME/portTICK_PERIOD_MS, 0);			//������ ������� ������ � �������� �������� ����� ��������� ����
				}
			}

			while (xTimerIsTimerActive(xTimerArming))												//���� ���� �������� � ����� ����� ���������� ����
			{
				if ((flagTwoButtons == true) && (button1.flagPress == false) && (button2.flagPress == false))	//���� ��� ������ ���� ������, � ����� ������
				{
					flagTwoButtons = false;
					button1.flagClick = false;
					button2.flagClick = false;
					xTimerChangePeriod(xTimerArming, TIME_SET_GUARD/portTICK_PERIOD_MS, 0);			//������ ������� ������ � �������� �������� ���������� �� ������
					xTimerStart(xTimerBlynkLed, 0);													//����� ������� ���������� (�����)
					mode = 1;																		//������� �� ����� 1-���������� �� ������
					break;
				}
			} 
			
			if (flagArmingReady)																	//���� ������ ���������� �� ������ �� �������, �� ��������� ������
			{
				flagTwoButtons = false;
				flagArmingReady = false;
				button1.flagClick = false;
				button2.flagClick = false;
				xTimerStop(xTimerBlynkLed, 0);														//��������� ������� ����������
				xTimerChangePeriod(xTimerArming, TIME_SET_CODE/portTICK_PERIOD_MS, 0);				//������ ������� ������ � �������� �������� ��������� ����� ��� ����� ������ ����
				mode = 5;																			//������� � ����� 5-����_����������_����
			}
		}

		//----------����� 1 = ���������� �� ������----------
		// - ������ ���������
		// - ��������� ������ ����� (5 ��� � ���)
		// - ���� ������ ����� ������ ������, �� ������ -> ������� � ����� 0 (���������)
		// - ���� �� ������ ����� ������ ������ �����, ��� 20 ��� ������� � ����� 2 (������)
		else if (mode == 1)
		{
			#ifdef DEBUG_ON
				Serial.print(F("Mode = "));															//���������� �����
				Serial.println(mode);
			#endif /*	DEBUG_ON	*/
			
			PORT_STATE_SIREN &= ~SIREN_PIN;															//������ ���������
						
			while (xTimerIsTimerActive(xTimerArming))												//���� ������ ���������� �� ������ �������
			{
				if ((button1.flagPress == true) || (button2.flagPress == true))						//���� ������ ����� ������ ���� ������� ������ ���������� �� ������
				{
					button1.flagClick = false;
					button2.flagClick = false;
					xTimerStop(xTimerArming, 0);
					xTimerStop(xTimerBlynkLed, 0);
					mode = 0;																		//������ ���������� �� ������, ������� � ����� 0-���������
				}
			}
			
			if (flagArmingReady)																	//���� ������ ���������� �� ������ �� �������, �� ��������� ������
			{
				flagArmingReady = false;
				xTimerChangePeriod(xTimerBlynkLed, SLOW_TIME_FLASH_LED/portTICK_PERIOD_MS,0);		//������� �� ������ ������� �����������
				mode = 2;																			//������� � ����� 2-������
			}
		}
		
		//----------����� 2 = ������----------
		// - ������ ���������
		// - ��������� ������ ����� (1 ��� � �������)
		// - ����������� ��������� �������
		// - ��� ���������� ������������� ������� ����� ���������� �������� ������� � ����� 3 (��������)
		else if (mode == 2)
		{
			#ifdef DEBUG_ON
				Serial.print(F("Mode = "));															//���������� �����
				Serial.println(mode);
			#endif /*	DEBUG_ON	*/
			
			PORT_STATE_SIREN &= ~SIREN_PIN;															//������ ���������
			
			if (averageADC[0] < MIN_U || averageADC[0] > MAX_U ||									//�������� ��������� �������
				averageADC[1] < MIN_U || averageADC[1] > MAX_U)										//���� ���������� ��������� ������
			{
				xTimerChangePeriod(xTimerBlynkLed, FAST_TIME_FLASH_LED/portTICK_PERIOD_MS,0);		//������� �� ������ ������� �����������
				xTimerChangePeriod(xTimerArming, TIME_BLOCK_ALARM/portTICK_PERIOD_MS, 0);			//������ ������� ������ ��� ������� �������� ��������
				mode = 3;																			//������� � ����� 3-��������
			}
		}
		
		//----------����� 3 = ��������----------
		// - ������ ���������
		// - ��������� ������ ����� (5 ��� � �������)
		// - ��������� ���� ���������� ���� �������� ������ � ������� 30 ���
		// - ��� ����� ���������� ���� ������� � ����� 0 (���������)
		// - ��� ��������� ������� �������� ���������� ���� ������� � ����� 4 (�������)
		else if (mode == 3)
		{
			#ifdef DEBUG_ON
				Serial.print(F("Mode = "));															//���������� �����
				Serial.println(mode);
// 				Serial.print(F("	CheckCode = "));
// 				Serial.println(secretCodeCheck());
			#endif /*	DEBUG_ON	*/
			
			PORT_STATE_SIREN &= ~SIREN_PIN;															//������ ���������
			
			//�������� ����� ���������� ���� ��� ���������� �������
			if (secretCodeCheck())																	//���� ������ ������ ���
			{
				button1.flagClick = false;
				button2.flagClick = false;
				secretCode = 0;
				bitNum = 0;
				xTimerStop(xTimerArming, 0);
				xTimerStop(xTimerBlynkLed, 0);
				mode = 0;																			//������� � ����� 0-���������
			}
			
			if (flagArmingReady)																	//���� ����� ��������� ������� �������� ������� �������
			{
				flagArmingReady = false;
				xTimerChangePeriod(xTimerArming, TIME_ALARM/portTICK_PERIOD_MS, 0);					//������ ������� ������ ��� ������� ������� ������� �������				
				mode = 4;																			//������� � ����� 4-�������
			}
		}
		
		//----------����� 4 = �������----------
		// - ������ �������� �� 1 ������
		// - ��������� ������ ����� (5 ��� � �������)
		// - ��������� ���� ���������� ���� �������� ������
		// - ��� ����� ���������� ���� ������� � ����� 0 (���������)
		// - �� ��������� ������� ������ ������ (1 ������) ������� � ����� 2 (������)
		else if (mode == 4)
		{
			#ifdef DEBUG_ON
				Serial.print(F("Mode = "));															//���������� �����
				Serial.println(mode);
// 				Serial.print(F("	CheckCode = "));
// 				Serial.println(secretCodeCheck());
			#endif /*	DEBUG_ON	*/
			
			PORT_STATE_SIREN |= SIREN_PIN;															//������ ��������
			
			//�������� ����� ���������� ���� ��� ���������� �������
			if (secretCodeCheck())																	//���� ������ ������ ���
			{
				button1.flagClick = false;
				button2.flagClick = false;
				secretCode = 0;
				bitNum = 0;
				xTimerStop(xTimerArming, 0);
				xTimerStop(xTimerBlynkLed, 0);
				mode = 0;																			//������� � ����� 0-���������
			}

			if (flagArmingReady)																	//���� ����� ��������� ������� �������� ������� �������
			{
				flagArmingReady = false;
				xTimerChangePeriod(xTimerBlynkLed, SLOW_TIME_FLASH_LED/portTICK_PERIOD_MS,0);		//������� �� ������ ������� �����������
				mode = 2;																			//������� � ����� 2-������
			}
		}
		
		//----------����� 5 = ��������� ���������� ����----------
		// - ������ ���������
		// - ��������� �������� ���������
		// - ��������� ���� ���������� ���� �������� ������:
		//������ ������� ������ ������� � ��������� ���= ������ �1->0, ������ �2->1
		//����� ������� ������ �� �������������� (������ ���� �� ����� 7, ����� ���������� ���������� ���������� ����)
		// - ��� ���������� ����������� ������� ����� ����� ��������� ������ (5 ���)
		//��������� ��� secretCode � ��� ������ (����� ������� ������ ��� �����) bitNum �������� � EEPROM
		//� ������� � ����� 0 (���������)
		else if (mode == 5)
		{
			#ifdef DEBUG_ON
				Serial.print(F("Mode = "));															//���������� �����
				Serial.println(mode);
// 				Serial.print(F("bitNum = "));
// 				Serial.println(bitNum);
				//Serial.print(F("clickButton1 = "));
				//Serial.println(button1.flagClick);
				//Serial.print(F("	clickButton2 = "));
				//Serial.println(button2.flagClick);
				//Serial.print(F("Mode => 0 "));														//���������� �����
			#endif /*	DEBUG_ON	*/
			
			PORT_STATE_SIREN &= ~SIREN_PIN;															//������ ���������
			PORT_STATE_LED |= LED_PIN;																//��������� �������� ���������
			
			if ((button1.flagClick == true) || (button2.flagClick == true))
			{
				xTimerReset(xTimerArming, 0);														//���������� ������� ������ �� ��� ���� ������ �������� ����� ����� ����
				//Serial.print(xTimerIsTimerActive(xTimerArming));
				secretCode = secretCode << 1;
				if (button1.flagClick == true)														//���� ������ ������ �1
				{
					button1.flagClick = false;
					secretCode &= 0xfe;																//�������� � ������� ��� ���� 0
				}
				if (button2.flagClick == true)														//���� ������ ������ �2
				{
					button2.flagClick = false;
					secretCode |= 1;																//�������� � ������� ��� ���� 1
				}
				bitNum++;																			//������� ���������� ����� (=������� ������)
			}
			
			if (flagArmingReady)																	//���� ����� ��������� ������� ����� ����� ����
			{
				flagArmingReady = false;
				EEPROM.write(COD_ADR, secretCode);													//��������� ����� ��������� ��� � EEPROM
				EEPROM.write(NUMBER_ADR, bitNum);													//��������� ������ ������ ���������� ���� � EEPROM
				button1.flagClick = false;
				button2.flagClick = false;				secretCode = 0;				bitNum = 0;				xTimerStop(xTimerArming, 0);				xTimerStop(xTimerBlynkLed, 0);				mode = 0;																			//������� � ����� 0-���������
			}
		}
		
		else mode = 0;																				//��������� �������������� ������������ � ����� 0 (���������) �� ������ ������������ �������� ���������� mode
	}
	vTaskDelete(NULL);																				//��������� �������� ������� ������, ���� ����� ������� �� ����� � ����
}



/*������� ������� ������� �����������.
* �������-�������. ���������� ������������� �� ��������� ������ ��������.
* �������� - ���������� ������������ ������� ���������� ������������� ��� ������.
* ��������� �������� ������ ���������� � �������� ��������.
*/
void vTimerBlynkLed(TimerHandle_t xTimer)
{
	PORT_STATE_LED ^= LED_PIN;																		//������������� ���������
}



/*������� ������������� ������� ���������� �� ������.
* �������-�������. ���������� ������������� �� ��������� ������ ��������.
* �������� - ���������� ������������ ������� ���������� ������������� ��� ������.
* ��������� �������� ����� ��������� � ����� ������.
*/
void vTimerArming(TimerHandle_t xTimer)
{
	flagArmingReady = true;
}



/*������� ���������� ����������.
* ���������� ����������� �� ������� 2.
* ���� ������� ��� ������������� ������-����������� ����������, ������������ �������.
*/
void timerInterrupt(void)
{
	static BaseType_t xHigherPriorityTaskWoken;
	xSemaphoreGiveFromISR(xSemaphoreScanSensors, &xHigherPriorityTaskWoken);						//���������� ������ ��� ������-����������� ����������
	if (xHigherPriorityTaskWoken == pdTRUE)
	{
		taskYIELD();																				//������������� ����������� �������� ����� ��������� ���������� 
	}
}



/*----------������� �������� ���������� ����-----------
* ��������� ������ �1,2 ������ �� ������� �������
* ���������� ����� ������� � ���������� bitNum, ����������� � EEPROM ��� ��������� ���������� ����
* ���������� ��������� ��������� ��� � ���������� secretCode, ����������� � EEPROM ��� ��������� ���������� ����
* ��� ���������� ���������� � ������������ ����� ��������� ���������� mode=0 (����� ���������).
* ��� ������������ ���������� � ������������ ����� �� ������ ���������� mode (����� �������).
* ��� ������� ����� ������ ������ ���������� ��������� (��������) ���.*/
boolean secretCodeCheck(void)
{
	if ((button1.flagPress == true) && (button2.flagPress == true))									//������ ���� ������� �������� ���� ������ (���� ����)
	{
		button1.flagClick = false;
		button2.flagClick = false;
		secretCode = 0;
		bitNum = 0;
	}
	
	//��������� ������� ������ � ������ ����
	if ((button1.flagClick == true) || (button2.flagClick == true))
	{
		secretCode = secretCode << 1;
		if (button1.flagClick == true)																//���� ������ ������ �1
		{
			button1.flagClick = false;
			secretCode &= 0xfe;																		//�������� � ������� ��� ���� 0
		}
		if (button2.flagClick == true)																//���� ������ ������ �2
		{
			button2.flagClick = false;
			secretCode |= 1;																		//�������� � ������� ��� ���� 1
		}
		bitNum++;																					//������� ���������� ����� (=������� ������)
	}
	
	//�������� ���������� ����
	if (bitNum == EEPROM.read(NUMBER_ADR))															//���� ������� ��� ����� ����
	{
		if (secretCode == EEPROM.read(COD_ADR))														//���� ������ ������ ���
		{
			button1.flagClick = false;
			button2.flagClick = false;			return true;
		} 
		else
		{
			secretCode = 0;
			bitNum = 0;
			return false;
		}
	}
	return false;
}