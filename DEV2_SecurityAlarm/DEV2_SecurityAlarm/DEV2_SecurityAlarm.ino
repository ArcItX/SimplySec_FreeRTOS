/*
 * DEV2_SecurityAlarm.ino
 *
 * Created: 7/3/2018 12:34:26 PM
 * Author: tyutchenko_ai
 * Охранная сигнализация на FreeRTOS
 * Управляется с пульта двумя кнопками
 * Охраняет две зоны
 */ 

#define DEBUG_ON

#include <Arduino_FreeRTOS.h>
#include <timers.h>																					//таймеры FreeRTOS
#include <semphr.h>																					//семафоры FreeRTOS
#include <MsTimer2.h>																				//прерывания от таймера 2
#include <ArcButton/AntiBounceButtonPortB.h>														//скан дребезга кнопок на порту B
#include <BackgroundADC.h>																			//аналоговый замер в фоне
#include <EEPROM.h>

//Назначение выводов
//Кнопка 1
#define BUTTON1_PIN _BV(PB1)																		//подключаем кнопку №1 на dPin 9 (PB1)
//Кнопка 2
#define BUTTON2_PIN _BV(PB2)																		//подключаем кнопку №2 на dPin 10 (PB2)
//Светодиод
#define PORT_MODE_LED DDRB																			//подключаем светодиод на dPin 13 (PortB5)
#define PORT_STATE_LED PORTB																		//подключаем светодиод на dPin 13 (PortB5)
#define LED_PIN _BV(PB5)																			//подключаем светодиод на dPin 13 (PB5)
//Сирена
#define PORT_MODE_SIREN DDRB																		//подключаем сирену на dPin 12 (PortB4)
#define PORT_STATE_SIREN PORTB																		//подключаем сирену на dPin 12 (PortB4)
#define SIREN_PIN _BV(PB4)																			//подключаем сирену на dPin 12 (PB4)
//Шлейфы
#define SHLEIF1_PIN A0																				//подключаем шлейф 1 на aPin 0
#define SHLEIF2_PIN A1																				//подключаем шлейф 1 на aPin 1

//Параметры
const TickType_t FAST_TIME_FLASH_LED = 100;															//период частого мигания светодиода в мс
const TickType_t SLOW_TIME_FLASH_LED = 500;															//период редкого мигания светодиода в мс
const TickType_t TIME_SET_GUARD	= 20000;															//период выдержки паузы при постановке на охрану в мс
const TickType_t TIME_ALARM = 60000;																//период работы сигнала тревоги в мс
const TickType_t TIME_BLOCK_ALARM = 30000;															//период выдержки паузы при сработке датчиков в мс
const TickType_t SET_CODE_TIME = 6000;																//период удержания кнопок для перехода в режим ввода секретного кода в мс
const TickType_t TIME_SET_CODE = 5000;																//период ожидания нажатия кнопок при вводе секретного кода в мс
const unsigned int MAX_U = 36864;																	//верхний предел напряжения шлейфов (3,6В, 3,6*1024/5*50=36864)
const unsigned int MIN_U = 10650;																	//нижний предел напряжения шлейфов (1,04В, 1,04*1024/5*50=10650)
const byte averageCountNUM = 50;																	//количество замеров АЦП шлейфов для усреднения
//Адреса EEPROM
const byte COD_ADR = 2;																				//адрес секретного кода в EEPROM
const byte NUMBER_ADR = 4;																			//адрес числа битов секретного кода в EEPROM

//Переменные
byte mode = 0;																						//режим работы охранного блока
byte secretCode = 0;																				//переменная для секретного кода
byte bitNum = 0;																					//число нажатий кнопок при вводе секретного кода (размер кода)
unsigned int sumADC[2];																				//переменные для суммирования кодов АЦП
unsigned int averageADC[2];																			//результаты накопления кодов АЦП
boolean flagReadyADC = false;																		//признак готовности данных по замеру шлейфов АЦП

TaskHandle_t xTaskScanSensors = NULL;																//переменная под дескриптор задачи скана датчиков
TaskHandle_t xTaskControlSafety = NULL;																//переменная под дескриптор блока управления охраной
TimerHandle_t xTimerBlynkLed = NULL;																//переменная под дескриптор таймера мигания светодиода
TimerHandle_t xTimerArming = NULL;																	//переменная под дескриптор таймера постановки на охрану
SemaphoreHandle_t xSemaphoreScanSensors = NULL;														//переменная под дескриптор семафора доступа к флагам объектов класса Button

//TickType_t uiTimerBlynkLedPeriod = FAST_TIME_FLASH_LED/portTICK_PERIOD_MS;							//переменная для периода таймера мигания светодиода с инициализацией
boolean flagArmingReady = false;																	//признак окончания периода ожидания постановки на охрану

//Объекты
AntiBounceButtonPortB button1(BUTTON1_PIN, 25);														//Кнопка №1 (вывод, кол-во проверок сканирования на дребезг)
AntiBounceButtonPortB button2(BUTTON2_PIN, 25);														//Кнопка №2 (вывод, кол-во проверок сканирования на дребезг)

//Прототипы функций
void vTaskScanSensors(void* pvParameters);															//функция задачи скана датчиков
void vTaskControlSafety(void* pvParameters);														//функция задачи контроля охраны
void vTimerBlynkLed(TimerHandle_t xTimer);															//функция таймера мигания светодиодом
void vTimerArming(TimerHandle_t xTimer);															//функция таймера постановки на охрану
void timerInterrupt(void);																			//функция прерывания от таймера 2
boolean secretCodeCheck(void);																		//функция проверки правильности секретного кода

void setup()
{
	//Настройка
	#ifdef DEBUG_ON
		Serial.begin(9600);
	#endif /*	DEBUG_ON	*/
	PORT_MODE_LED |= LED_PIN;																		//пин светодиода в режим вывода 
	PORT_STATE_LED &= ~LED_PIN;																		//инициализация пина светодиода (выключить)
	PORT_MODE_SIREN |= SIREN_PIN;																	//пин сирены в режим вывода
	PORT_STATE_SIREN &= ~SIREN_PIN;																	//инициализация пина сирены (выключить)
	MsTimer2::set(2, timerInterrupt);																//период генерации прерывания от таймера 2 мс
	MsTimer2::start();

	//Создание семафоров
	//Семафор флагов объектов класса Button ********************************************************
	xSemaphoreScanSensors = xSemaphoreCreateBinary();
	if (xSemaphoreScanSensors == NULL)
	{
		//Семафор не создан
		#ifdef DEBUG_ON
			Serial.println(F("There was insufficient FreeRTOS heap available for the semaphore to be created successfully."));
		#endif /*	DEBUG_ON	*/
	} 
	else
	{
		//Семафор успешно создан
	}	//------------------------------------------------------------------------------------------
	
	//Создание таймеров
	//Таймер мигания светодиодом *******************************************************************
	xTimerBlynkLed = xTimerCreate("TimerBlynkLed",													//Отладочное имя
									FAST_TIME_FLASH_LED/portTICK_PERIOD_MS,							//период (создадим с частым периодом, потом сменим)
									pdTRUE,															//периодический
									0,																//ID
									vTimerBlynkLed);												//функция таймера
	if (xTimerBlynkLed == pdTRUE)
	{
		//Таймер не создан
		#ifdef DEBUG_ON
			Serial.println(F("The timer was not created."));
		#endif /*	DEBUG_ON	*/
	}
	else
	{
		//Таймер успешно создан
	}	//------------------------------------------------------------------------------------------


	//Таймер постановки на охрану*******************************************************************
	xTimerArming = xTimerCreate("TimerArming",
								SET_CODE_TIME/portTICK_PERIOD_MS,									//период (создадим с периодом ожидания перехода в режим ввода секретного кода, потом сменим)
								pdFALSE,															//интервальный
								0,
								vTimerArming);
	if (xTimerArming == pdTRUE)
	{
		//Таймер не создан
		#ifdef DEBUG_ON
			Serial.println(F("The timer was not created."));
		#endif /*	DEBUG_ON	*/
	}
	else
	{
		//Таймер успешно создан
	}	//------------------------------------------------------------------------------------------
	
	
	//Создание задач
	//Задача скана датчиков*************************************************************************
	if (xTaskCreate(vTaskScanSensors,
					"TaskScanSensors",
					configMINIMAL_STACK_SIZE,
					NULL,
					2,																				//приоритет
					&xTaskScanSensors) != pdTRUE)
	{
		//Задача не создана
		#ifdef DEBUG_ON
			Serial.println(F("The task could not be created because there was insufficient heap memory remaining."));
		#endif /*	DEBUG_ON	*/
	} 
	else
	{
		//Задача создана
	}	//------------------------------------------------------------------------------------------
	
	//Задача блока контроля охраны******************************************************************
	if (xTaskCreate(vTaskControlSafety,
					"TaskControlSafety",
					configMINIMAL_STACK_SIZE,
					NULL,
					1,																				//приоритет
					&xTaskControlSafety) != pdTRUE)
	{
		//Задача не создана
		#ifdef DEBUG_ON
			Serial.println(F("The task could not be created because there was insufficient heap memory remaining."));
		#endif /*	DEBUG_ON	*/
	}
	else
	{
		//Задача создана
	}	//------------------------------------------------------------------------------------------

	//далее происходит неявное автоматическое создание служебной задачи таймеров FreeRTOS и...
	//далее происходит неявный автоматический старт планировщика FreeRTOS
	//vTaskStartScheduler();
}

void loop()
{
	/*	Не используем	*/
}

//Определения функций

/* Функция задачи-обработчика прерывания таймера 2:
* - сканирует кнопки на дребезг с помощью вызовов библиотеки AntiBounceButtonPortB.h,
* - сканирует шлейфы на портах АЦП с накоплением данных в течение установленного периода с помощью вызовов библиотеки BackgroundADC.h
* Приоритет 2
* Вызывается с частотой 2 мс (для разблокировки ожидает семафора от прерывния таймера 2)
* Устанавливает признак состояния кнопки flagPress и признак нажатия flagClick объектов класса Button
* Устанавливает признак готовности данных замера шлейфов АЦП flagReadyADC, накопленные усредненные данные сохраняет в массиве глобальных переменных averageADC[]
*/
void vTaskScanSensors(void* pvParameters)
{
	static byte averageCount = 0;																	//счетчик замеров АЦП шлейфов для усреднения
	static byte chanelADC = 0;																		//номер канала АЦП	
	
	for (;;)
	{
		xSemaphoreTake(xSemaphoreScanSensors, portMAX_DELAY);										//ждем семафор от прерывания на таймере 2, пока оно не появится
		//Работаем с кнопками
		button1.filterAverage();
		button2.filterAverage();
		
		//Работаем со шлейфами АЦП
		sumADC[chanelADC] += BackgroundADC.analogRead();											//накапливаем результаты замеров АЦП
		chanelADC++;																				//следующий канал АЦП
		if (chanelADC > 1) chanelADC = 0;															//зациклили смену каналов АЦП
		BackgroundADC.analogStart(chanelADC);														//запуск измерения шлейфа АЦП
		if (chanelADC == 0)
		{
			averageCount++;
			if (averageCount > averageCountNUM)														//проведено нужное количество замеров для усреднения
			{
				averageCount = 0;
				averageADC[0] = sumADC[0];															//перегрузка накопленных значений
				averageADC[1] = sumADC[1];
				sumADC[0] = 0;
				sumADC[1] = 0;
				flagReadyADC = true;																//признак готовности данных замеров шлейфов АЦП для использования
			}
		}
	}
	vTaskDelete(NULL);																				//аварийное удаление текущей задачи, если вдруг вылетим из цикла её тела
}


/*Функция задачи блока контроля охраны
* Приоритет 1
* Контролирует состояние кнопок
* Запускает таймер мигания светодиодом часто (0,1 сек) на 20 сек, если были нажаты 2 кнопки
* После 20 секунд изменяет период таймера для редкого (1 сек) мигания
* Если в течение 20 сек нажата любая кнопка, то останавливает таймер мигания, гасит светодиод
*/
void vTaskControlSafety (void* pvParameters)
{
	#ifdef DEBUG_ON
		vTaskSetApplicationTaskTag(NULL, (void*)2);													//отладочный тэг
	#endif /*	DEBUG_ON	*/
	
	boolean flagTwoButtons = false;																	//признак нажатия двух кнопок
	
	for (;;)
	{
		//Определение команды постановки на охрану при коротком нажатии двух кнопок пульта
		//признак flagTwoButtons устанавливается при нажатии кнопок №1 и №2
		
		/*#ifdef DEBUG_ON
			Serial.print(flagTwoButtons);															//отладочный вывод
			Serial.print("	");
			Serial.print(button1.flagPress);
			Serial.print("	");
			Serial.println(button2.flagPress);
		#endif / *	DEBUG_ON	* /*/
		
		//----------Режим 0 = ОТКЛЮЧЕНО-------------------------------------------------------------
		// - светодиод не горит
		// - сирена отключена
		// - если нажаты две кнопки пульта одновременно перейти в режим 1 (ПОСТАНОВКА НА ОХРАНУ)
		// - если нажаты две кнопки пульта одновременно более, чем 6 сек перейти в режим 5 (УСТАНОВКА СЕКРЕТНОГО КОДА)
		if (mode == 0)
		{
			#ifdef DEBUG_ON
				Serial.print(F("Mode = "));															//отладочный вывод
				Serial.println(mode);
				//Serial.print(F("   flagArmingReady = "));
				//Serial.print(flagArmingReady);
				//Serial.print(F("   TimerArmingActive = "));
				//Serial.println(xTimerIsTimerActive(xTimerArming));
			#endif /*	DEBUG_ON	*/
			
			PORT_STATE_LED &= ~LED_PIN;																//светодиод не горит
			PORT_STATE_SIREN &= ~SIREN_PIN;															//сирена отключена
			
			if ((button1.flagPress == true) && (button2.flagPress == true))
			{
				flagTwoButtons = true;
				if (!xTimerIsTimerActive(xTimerArming))
				{
					xTimerChangePeriod(xTimerArming, SET_CODE_TIME/portTICK_PERIOD_MS, 0);			//запуск таймера охраны с периодом ожидания ввода секретнго кода
				}
			}

			while (xTimerIsTimerActive(xTimerArming))												//пока ждем перехода в режим ввода секретного кода
			{
				if ((flagTwoButtons == true) && (button1.flagPress == false) && (button2.flagPress == false))	//если обе кнопки были нажаты, а затем отжаты
				{
					flagTwoButtons = false;
					button1.flagClick = false;
					button2.flagClick = false;
					xTimerChangePeriod(xTimerArming, TIME_SET_GUARD/portTICK_PERIOD_MS, 0);			//запуск таймера охраны с периодом задержки постановки на охрану
					xTimerStart(xTimerBlynkLed, 0);													//старт мигания светодиода (часто)
					mode = 1;																		//переход на режим 1-ПОСТАНОВКА НА ОХРАНУ
					break;
				}
			} 
			
			if (flagArmingReady)																	//если таймер постановки на охрану не активен, но отработал удачно
			{
				flagTwoButtons = false;
				flagArmingReady = false;
				button1.flagClick = false;
				button2.flagClick = false;
				xTimerStop(xTimerBlynkLed, 0);														//отключить мигание светодиода
				xTimerChangePeriod(xTimerArming, TIME_SET_CODE/portTICK_PERIOD_MS, 0);				//запуск таймера охраны с периодом ожидания окончания паузы при вводе нового кода
				mode = 5;																			//переход в режим 5-ВВОД_СЕКРЕТНОГО_КОДА
			}
		}

		//----------Режим 1 = ПОСТАНОВКА НА ОХРАНУ----------
		// - сирена отключена
		// - светодиод мигает часто (5 раз в сек)
		// - если нажата любая кнопка пульта, то ОТМЕНА -> перейти в режим 0 (ОТКЛЮЧЕНО)
		// - если не нажаты любые кнопки пульта более, чем 20 сек перейти в режим 2 (ОХРАНА)
		else if (mode == 1)
		{
			#ifdef DEBUG_ON
				Serial.print(F("Mode = "));															//отладочный вывод
				Serial.println(mode);
			#endif /*	DEBUG_ON	*/
			
			PORT_STATE_SIREN &= ~SIREN_PIN;															//сирена отключена
						
			while (xTimerIsTimerActive(xTimerArming))												//пока таймер постановки на охрану активен
			{
				if ((button1.flagPress == true) || (button2.flagPress == true))						//если нажата любая кнопка пока активен таймер постановки на охрану
				{
					button1.flagClick = false;
					button2.flagClick = false;
					xTimerStop(xTimerArming, 0);
					xTimerStop(xTimerBlynkLed, 0);
					mode = 0;																		//отмена постановки на охрану, возврат в режим 0-ОТКЛЮЧЕНО
				}
			}
			
			if (flagArmingReady)																	//если таймер постановки на охрану не активен, но отработал удачно
			{
				flagArmingReady = false;
				xTimerChangePeriod(xTimerBlynkLed, SLOW_TIME_FLASH_LED/portTICK_PERIOD_MS,0);		//перейти на редкое мигание светодиодом
				mode = 2;																			//переход в режим 2-ОХРАНА
			}
		}
		
		//----------Режим 2 = ОХРАНА----------
		// - сирена отключена
		// - светодиод мигает редко (1 раз в секунду)
		// - проверяется состояние шлейфов
		// - при отклонении сопротивления шлейфов свыше предельных значений перейти в режим 3 (ЗАДЕРЖКА)
		else if (mode == 2)
		{
			#ifdef DEBUG_ON
				Serial.print(F("Mode = "));															//отладочный вывод
				Serial.println(mode);
			#endif /*	DEBUG_ON	*/
			
			PORT_STATE_SIREN &= ~SIREN_PIN;															//сирена отключена
			
			if (averageADC[0] < MIN_U || averageADC[0] > MAX_U ||									//проверка состояния шлейфов
				averageADC[1] < MIN_U || averageADC[1] > MAX_U)										//если обнаружено нарушение шлейфа
			{
				xTimerChangePeriod(xTimerBlynkLed, FAST_TIME_FLASH_LED/portTICK_PERIOD_MS,0);		//перейти на частое мигание светодиодом
				xTimerChangePeriod(xTimerArming, TIME_BLOCK_ALARM/portTICK_PERIOD_MS, 0);			//запуск таймера охраны для отсчета задержки сработки
				mode = 3;																			//переход в режим 3-ЗАДЕРЖКА
			}
		}
		
		//----------Режим 3 = ЗАДЕРЖКА----------
		// - сирена отключена
		// - светодиод мигает часто (5 раз в секунду)
		// - ожидается ввод секретного кода кнопками пульта в течение 30 сек
		// - при вводе секретного кода перейти в режим 0 (ОТКЛЮЧЕНО)
		// - при истечении времени ожидания секретного кода перейти в режим 4 (ТРЕВОГА)
		else if (mode == 3)
		{
			#ifdef DEBUG_ON
				Serial.print(F("Mode = "));															//отладочный вывод
				Serial.println(mode);
// 				Serial.print(F("	CheckCode = "));
// 				Serial.println(secretCodeCheck());
			#endif /*	DEBUG_ON	*/
			
			PORT_STATE_SIREN &= ~SIREN_PIN;															//сирена отключена
			
			//Проверка ввода секретного кода для отключения тревоги
			if (secretCodeCheck())																	//если введен верный код
			{
				button1.flagClick = false;
				button2.flagClick = false;
				secretCode = 0;
				bitNum = 0;
				xTimerStop(xTimerArming, 0);
				xTimerStop(xTimerBlynkLed, 0);
				mode = 0;																			//переход в режим 0-ОТКЛЮЧЕНО
			}
			
			if (flagArmingReady)																	//ждем флага отработки таймера задержки сигнала тревоги
			{
				flagArmingReady = false;
				xTimerChangePeriod(xTimerArming, TIME_ALARM/portTICK_PERIOD_MS, 0);					//запуск таймера охраны для отсчета времени сигнала тревоги				
				mode = 4;																			//переход в режим 4-ТРЕВОГА
			}
		}
		
		//----------Режим 4 = ТРЕВОГА----------
		// - сирена включена на 1 минуту
		// - светодиод мигает часто (5 раз в секунду)
		// - ожидается ввод секретного кода кнопками пульта
		// - при вводе секретного кода перейти в режим 0 (ОТКЛЮЧЕНО)
		// - по истечении времени работы сирены (1 минута) перейти в режим 2 (ОХРАНА)
		else if (mode == 4)
		{
			#ifdef DEBUG_ON
				Serial.print(F("Mode = "));															//отладочный вывод
				Serial.println(mode);
// 				Serial.print(F("	CheckCode = "));
// 				Serial.println(secretCodeCheck());
			#endif /*	DEBUG_ON	*/
			
			PORT_STATE_SIREN |= SIREN_PIN;															//сирена включена
			
			//Проверка ввода секретного кода для отключения тревоги
			if (secretCodeCheck())																	//если введен верный код
			{
				button1.flagClick = false;
				button2.flagClick = false;
				secretCode = 0;
				bitNum = 0;
				xTimerStop(xTimerArming, 0);
				xTimerStop(xTimerBlynkLed, 0);
				mode = 0;																			//переход в режим 0-ОТКЛЮЧЕНО
			}

			if (flagArmingReady)																	//ждем флага отработки таймера задержки сигнала тревоги
			{
				flagArmingReady = false;
				xTimerChangePeriod(xTimerBlynkLed, SLOW_TIME_FLASH_LED/portTICK_PERIOD_MS,0);		//перейти на редкое мигание светодиодом
				mode = 2;																			//переход в режим 2-ОХРАНА
			}
		}
		
		//----------Режим 5 = УСТАНОВКА СЕКРЕТНОГО КОДА----------
		// - сирена отключена
		// - светодиод светится постоянно
		// - ожидается ввод секретного кода кнопками пульта:
		//каждое нажатие кнопки пишется в отдельный бит= кнопка №1->0, кнопка №2->1
		//число нажатий кнопок не контролируется (должно быть не более 7, иначе произойдет перезапись введенного кода)
		// - при превышении допустимого времени паузы между нажатиями кнопок (5 сек)
		//секретный код secretCode и его размер (число нажатий кнопок при вводе) bitNum записать в EEPROM
		//и перейти в режим 0 (ОТКЛЮЧЕНО)
		else if (mode == 5)
		{
			#ifdef DEBUG_ON
				Serial.print(F("Mode = "));															//отладочный вывод
				Serial.println(mode);
// 				Serial.print(F("bitNum = "));
// 				Serial.println(bitNum);
				//Serial.print(F("clickButton1 = "));
				//Serial.println(button1.flagClick);
				//Serial.print(F("	clickButton2 = "));
				//Serial.println(button2.flagClick);
				//Serial.print(F("Mode => 0 "));														//отладочный вывод
			#endif /*	DEBUG_ON	*/
			
			PORT_STATE_SIREN &= ~SIREN_PIN;															//сирена отключена
			PORT_STATE_LED |= LED_PIN;																//светодиод светится постоянно
			
			if ((button1.flagClick == true) || (button2.flagClick == true))
			{
				xTimerReset(xTimerArming, 0);														//перезапуск таймера охраны на ещё один период ожидания конца ввода кода
				//Serial.print(xTimerIsTimerActive(xTimerArming));
				secretCode = secretCode << 1;
				if (button1.flagClick == true)														//если нажата кнопка №1
				{
					button1.flagClick = false;
					secretCode &= 0xfe;																//записать в младший бит кода 0
				}
				if (button2.flagClick == true)														//если нажата кнопка №2
				{
					button2.flagClick = false;
					secretCode |= 1;																//записать в младший бит кода 1
				}
				bitNum++;																			//счетчик записанных битов (=нажатий кнопок)
			}
			
			if (flagArmingReady)																	//ждем флага отработки таймера конца ввода кода
			{
				flagArmingReady = false;
				EEPROM.write(COD_ADR, secretCode);													//сохраняем новый секретный код в EEPROM
				EEPROM.write(NUMBER_ADR, bitNum);													//сохраняем размер нового секретного кода в EEPROM
				button1.flagClick = false;
				button2.flagClick = false;				secretCode = 0;				bitNum = 0;				xTimerStop(xTimerArming, 0);				xTimerStop(xTimerBlynkLed, 0);				mode = 0;																			//переход в режим 0-ОТКЛЮЧЕНО
			}
		}
		
		else mode = 0;																				//резервное принудительное переключение в режим 0 (ОТКЛЮЧЕНО) на случай некорректных значений переменной mode
	}
	vTaskDelete(NULL);																				//аварийное удаление текущей задачи, если вдруг вылетим из цикла её тела
}



/*Функция таймера мигания светодиодом.
* Функция-коллбэк. Вызывается автоматически из служебной задачи таймеров.
* Параметр - дескриптор сработавшего таймера передается автоматически при вызове.
* Выполняет инверсию вывода светодиода с заданным периодом.
*/
void vTimerBlynkLed(TimerHandle_t xTimer)
{
	PORT_STATE_LED ^= LED_PIN;																		//инвертировать светодиод
}



/*Функция интервального таймера постановки на охрану.
* Функция-коллбэк. Вызывается автоматически из служебной задачи таймеров.
* Параметр - дескриптор сработавшего таймера передается автоматически при вызове.
* Формирует выдержку перед переходом в режим охраны.
*/
void vTimerArming(TimerHandle_t xTimer)
{
	flagArmingReady = true;
}



/*Функция таймерного прерывания.
* Вызывается прерыванием от таймера 2.
* Дает семафор для разблокировки задачи-обработчика прерывания, сканирующего датчики.
*/
void timerInterrupt(void)
{
	static BaseType_t xHigherPriorityTaskWoken;
	xSemaphoreGiveFromISR(xSemaphoreScanSensors, &xHigherPriorityTaskWoken);						//семафорный сигнал для задачи-обработчика прерывания
	if (xHigherPriorityTaskWoken == pdTRUE)
	{
		taskYIELD();																				//принудительно переключить контекст после обработки прерывания 
	}
}



/*----------Функция проверки секретного кода-----------
* Сканирует кнопки №1,2 пульта на предмет нажатия
* Сравнивает число нажатий с переменной bitNum, сохраненной в EEPROM при установке секретного кода
* Сравнивает введенный секретный код с переменной secretCode, сохраненной в EEPROM при установке секретного кода
* При совпадении введенного и сохраненного кодов установит переменную mode=0 (режим ОТКЛЮЧЕНО).
* При несовпадении введенного и сохраненного кодов не меняет переменную mode (режим прежний).
* При нажатии обеих кнопок пульта сбрасывает введенный (неверный) код.*/
boolean secretCodeCheck(void)
{
	if ((button1.flagPress == true) && (button2.flagPress == true))									//начать ввод сначала нажатием двух кнопок (если надо)
	{
		button1.flagClick = false;
		button2.flagClick = false;
		secretCode = 0;
		bitNum = 0;
	}
	
	//Обработка нажатий кнопок и запись кода
	if ((button1.flagClick == true) || (button2.flagClick == true))
	{
		secretCode = secretCode << 1;
		if (button1.flagClick == true)																//если нажата кнопка №1
		{
			button1.flagClick = false;
			secretCode &= 0xfe;																		//записать в младший бит кода 0
		}
		if (button2.flagClick == true)																//если нажата кнопка №2
		{
			button2.flagClick = false;
			secretCode |= 1;																		//записать в младший бит кода 1
		}
		bitNum++;																					//счетчик записанных битов (=нажатий кнопок)
	}
	
	//Проверка введенного кода
	if (bitNum == EEPROM.read(NUMBER_ADR))															//если введены все цифры кода
	{
		if (secretCode == EEPROM.read(COD_ADR))														//если введен верный код
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