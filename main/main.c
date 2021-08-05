#include <stdio.h>
#include "rc522.c"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>
#include "driver/ledc.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"


/** \brief Interupt vector level ref: https://github.com/anoochit/esp32-example/blob/master/2080_GPIO_Interrupt/main/gpio_intr.c. */
#define ESP_INTR_FLAG_LEVEL 0
/** \brief ADC samples to collect for keypad  */
#define MULTISAMPLING		(50)
/** \brief Default key pin from factory. Randomize on every product.  */
#define defaultKEY	1234
/** \brief Length of RFID serial  */
#define serialLen	(5)
/** \brief Keypad IO Pin  */
#define GPIO_KEYPAD	(GPIO_SEL_34)
/** \brief Keypad IO Pin  */
#define GPIO_KEYPAD_IO	34
/** \brief Battery Monitor IO Pin  */
//Might need to move to ADC 2 if confict with Keypad trigger
#define GPIO_BATV	(GPIO_SEL_35)
/** \brief Battery Monitor IO Pin  */
#define GPIO_BATV_IO	35
/** \brief Max pin size is KEYPAD_BUF_SZ - 1 */
#define KEYPAD_BUF_SZ	(17)
/** \brief HW RTC for RGB control.*/
#define LEDC_HS_TIMER          LEDC_TIMER_0
/** \brief RGB PWM timing mode.*/
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
/** \brief Status RGB RED output pin.*/
#define LEDC_HS_RED_GPIO       (18)
/** \brief Status RGB RED LEDC channel.*/
#define LEDC_HS_RED_CHANNEL    LEDC_CHANNEL_0
#define RED  					0
/** \brief Status RGB GREEN output pin.*/
#define LEDC_HS_GREEN_GPIO     (19)
/** \brief Status RGB GREEN LEDC channel.*/
#define LEDC_HS_GREEN_CHANNEL  LEDC_CHANNEL_1
#define GREEN    				1
/** \brief Status RGB BLUE output pin.*/
#define LEDC_HS_BLUE_GPIO      (20)
/** \brief Status RGB BLUE LEDC channel.*/
#define LEDC_HS_BLUE_CHANNEL   LEDC_CHANNEL_2
#define Blue    				2
/** \brief Status RGB channel quantity.*/
#define LEDC_TEST_CH_NUM       3
/** \brief Duty cycle for quick LED flash.*/
#define LEDC_TEST_DUTY         (4000)
/** \brief PWM H-Bridge Input A.*/
#define MTR_PWM0A_OUT 15
/** \brief PWM H-Bridge Input B.*/
#define MTR_PWM0B_OUT 16
/** \brief PWM H-Bridge Sleep Control.*/
#define MTR_Sleep (GPIO_SEL_33)
/** \brief PWM H-Bridge Sleep Control.*/
#define MTR_SleepC (GPIO_NUM_33)
/** \brief PWM H-Bridge PWM Frequency.*/
#define MTR_PWM_FREQ 1000
/** \brief PWM H-Bridge lock cycle time.*/
#define MTR_PERIOD 3000
/** \brief PWM H-Bridge duty cycle.*/
#define MTR_duty 100
	//If falling trigger doesn't work with ADC, attach pin to keypad circuit.
	//#define GPIO_SENSE	(GPIO_SEL_39)
/** \brief QTY of stored NFC keys  */
uint8_t storedKey_QTY = 0;
/** \brief Secret pin, shhhhhhhh  */
uint64_t keypadPin = 0;
/** \brief Logic check to insure password cannot be entered from corrupted memory  */
bool keyPinValid = false;
esp_adc_cal_characteristics_t characteristics;
/** \brief FreeRTOS queue for keypad events. */
static xQueueHandle gpio_evt_queue = NULL;
/** \brief Motor direction control. Default Forward  */
bool mtrForward = true;
/** \brief ADC values for keypad reference. */
int thresholds[12] = {195, 305, 375, 405, 447, 477, 491, 512, 530, 538, 551, 563};
/** \brief Translated keypad values. */
char keypad[12] = {'1', '2', '3', '4', '5', '6', '7', '8', '9', '*', '0', '#'};
/** \brief Buffer storage for keypad input. */
char keypadInputBuffer[KEYPAD_BUF_SZ];
/** \brief Keypad buffer index. */
int keypadInputCnt = 0;
/** \brief Last keypad input tick [ms]. */
unsigned long keypadLastUpdate = 0;
/** \brief Keypad timeout period [ms]. */
const unsigned long keypadInvalidAfter = 5000;
/** \brief NFC pair timeout period [ms]. */
const unsigned long pairInvalidAfter = 5000;
/** \brief NFC enabler. */
bool nfcEnable = true;
/** \brief NFC key list. */
uint8_t *keyList[serialLen];
/** \brief Status RGB LEDC config. */
ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {
	{
		.channel    = LEDC_HS_RED_CHANNEL,
		.duty       = 0,
		.gpio_num   = LEDC_HS_RED_GPIO,
		.speed_mode = LEDC_HS_MODE,
		.hpoint     = 0,
		.timer_sel  = LEDC_HS_TIMER
	},
	{
		.channel    = LEDC_HS_GREEN_CHANNEL,
		.duty       = 0,
		.gpio_num   = LEDC_HS_GREEN_GPIO,
		.speed_mode = LEDC_HS_MODE,
		.hpoint     = 0,
		.timer_sel  = LEDC_HS_TIMER
	},
	{
		.channel    = LEDC_HS_BLUE_CHANNEL,
		.duty       = 0,
		.gpio_num   = LEDC_HS_BLUE_GPIO,
		.speed_mode = LEDC_HS_MODE,
		.hpoint     = 0,
		.timer_sel  = LEDC_HS_TIMER
	}
};

void pairTagHandler(uint8_t* serial_no);
/** \brief rc522 config when pairing NFC tags. */
const rc522_start_args_t pairNFC_args = {
	.miso_io = 25,
	.mosi_io = 23,
	.sck_io = 19,
	.sda_io = 22,
	.callback = &pairTagHandler
};
void tag_handler(uint8_t* serial_no);
/** \brief rc522 config when under normal operation. */
const rc522_start_args_t running_args = {
	.miso_io = 25,
	.mosi_io = 23,
	.sck_io = 19,
	.sda_io = 22,
	.callback = &tag_handler
};
/** \brief H-Bridge Motor config. */
mcpwm_config_t pwm_config = {
	.frequency = MTR_PWM_FREQ,
	.cmpr_a = 0,
	.cmpr_b = 0,
	.counter_mode = MCPWM_UP_COUNTER,
	.duty_mode = MCPWM_DUTY_MODE_0
};
/** \brief Poorly designed timeout bypass for NFC pair. */
bool noPairDect = true;
/** \brief FreeRTOS task handle for changing xqueue on keypad. */
TaskHandle_t xPinHandle;
/** \brief FreeRTOS task handle for normal xqueue on keypad. */
TaskHandle_t xKeypadHandle;


/*! \mainpage ECE 4999 Electromechanical Lock
 * \brief ESP32 based lock that features RFID and keypad to unlock
 * \author Matthew Hait
 * \author Remington Davids
 * \version 1.2
 */


/**
 * @brief Handles motor control and position logic.
 */
void toggle_Lock();
/**
 * @brief Removes NFC keys from heap and NVS.
 */
void unpairNFC();

/**
 * @brief Stores keypad pin into non-voltile storage 
 * 
 * \param secret: Input pin to be stored
*/
void writePinNVS(uint64_t secret);

/** @brief Grabs FreeRTOS runtime to determine timeouts. */
unsigned long runTime() {
	return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

/** @brief Flashes LED if battery is low. */
void batCheck() {
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 5000 / portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount ();
	uint32_t adc1_gpio35 = 0;
	while(true) {
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
		for(int i = 0; i < MULTISAMPLING; i++) {
			adc1_gpio35 = adc1_get_raw(ADC1_CHANNEL_7);
		}
		adc1_gpio35 /= MULTISAMPLING;
		// Read ADC as mV
		uint32_t rightDampVoltage = esp_adc_cal_raw_to_voltage(adc1_gpio35, &characteristics);
		if (rightDampVoltage < 3000) {
			ledc_set_duty(ledc_channel[RED].speed_mode, ledc_channel[RED].channel, LEDC_TEST_DUTY);
			ledc_update_duty(ledc_channel[RED].speed_mode, ledc_channel[RED].channel);
			vTaskDelay(125 / portTICK_PERIOD_MS);
			ledc_set_duty(ledc_channel[RED].speed_mode, ledc_channel[RED].channel, 0);
			ledc_update_duty(ledc_channel[RED].speed_mode, ledc_channel[RED].channel);
		}
	}	
}

/** @brief Keypad GPIO xQueue translate. 
 * \param *arg: Optional switch on GPIO pin.
 */
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

/** @brief Keypad GPIO interupt handler. 
 * 
 * \param *arg: Optional switch on GPIO pin.
 */
static void keypadCallback(void* arg)
{
	uint32_t adc1_gpio34 = 0;
    uint32_t io_num;
	uint64_t bufTranslate;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
			if(keypadLastUpdate != 0 && (keypadLastUpdate + keypadInvalidAfter) > runTime()) {
				//Timeout period reached; reset buffer before input
				memset(keypadInputBuffer, 0, KEYPAD_BUF_SZ);
				keypadInputCnt = 0;
			}
			// Wait 5ms for input to settle
			vTaskDelay(5 / portTICK_PERIOD_MS);
			keypadLastUpdate = runTime();
			adc1_gpio34 = 0;
			for(int i = 0; i < MULTISAMPLING; i++) {
				adc1_gpio34 = adc1_get_raw(ADC1_CHANNEL_6);
			}
			adc1_gpio34 /= MULTISAMPLING;
			for (int i=0; i<12; i++) {
				if(adc1_gpio34 < thresholds[i]) {
					keypadInputBuffer[keypadInputCnt] = keypad[i];
				}
			}
			if(adc1_gpio34 > thresholds[11]) {
				keypadInputBuffer[keypadInputCnt] = keypad[11];
			}
			keypadInputCnt++;
			// EX: "1#1234*" with "1234" being the pin code would disable the NFC.
			if (keypadInputBuffer[keypadInputCnt - 1] == '*' && keypadInputBuffer[1] == '#') {
				//Menu Mode
				//Check Pin
					//Cut out * & keypadInputCnt[0] through '#
					//Double check "-3" and not "-2"
					int numBytes = sizeof(char) * (keypadInputCnt - 3);
					char *pinParse = malloc(numBytes);
					memcpy(pinParse, keypadInputBuffer + 2, numBytes);
					bufTranslate = atoi(pinParse);
					free(pinParse);
				if (bufTranslate == keypadPin && keyPinValid) {
					switch(keypadInputBuffer[0]) {
						case '1':
							//NFC Disable
							nfcEnable = false;
							break;
						case '2':
							//NFC Enable
							nfcEnable = true;
							break;
						case '3':
							//Unpare All NFC
							unpairNFC();
							break;
						case '4':
							//Pair NFC Tag
							rc522_destroy(); 			//Remove running callback
							rc522_start(pairNFC_args);	//Attach pairing callback
							//Setup pair timeout. Reuse keypadLastUpdate because...
							keypadLastUpdate = runTime();
							//Wait until pair or NFC timeout
							while (runTime() < (keypadLastUpdate + keypadInvalidAfter) && noPairDect) {
								vTaskDelay(250 / portTICK_PERIOD_MS);
								//Check every 1/4 sec for result
							}
							if (noPairDect) {
								//No tag was detected
								//Blink Red thrice
								for (int i = 0; i < 6; i++) {
									if ( i % 2) {
										ledc_set_duty(ledc_channel[RED].speed_mode, ledc_channel[RED].channel, 0);
									} else {
										ledc_set_duty(ledc_channel[RED].speed_mode, ledc_channel[RED].channel, LEDC_TEST_DUTY);
									}
									ledc_update_duty(ledc_channel[RED].speed_mode, ledc_channel[RED].channel);
									vTaskDelay(250 / portTICK_PERIOD_MS);
								}
							}
							noPairDect = true;			//Reset for next time.
							rc522_destroy(); 			//Remove pairing callback
							rc522_start(running_args);	//Attach running callback
							break;
						case '5':
							//Change Pin

							//Clear buffer for new process
							memset(keypadInputBuffer, 0, KEYPAD_BUF_SZ);
							keypadInputCnt = 0;
							//Resume change pin task and suspend this task
							vTaskResume( xPinHandle );
 							vTaskSuspend( NULL );
							break;
						default:
							//A proper menu code was not selected.
							//Blink Red thrice
							for (int i = 0; i < 6; i++) {
								if ( i % 2) {
									ledc_set_duty(ledc_channel[RED].speed_mode, ledc_channel[RED].channel, 0);
								} else {
									ledc_set_duty(ledc_channel[RED].speed_mode, ledc_channel[RED].channel, LEDC_TEST_DUTY);
								}
								ledc_update_duty(ledc_channel[RED].speed_mode, ledc_channel[RED].channel);
								vTaskDelay(250 / portTICK_PERIOD_MS);
							}
							break;
					}
					//Flash Green
					ledc_set_duty(ledc_channel[GREEN].speed_mode, ledc_channel[GREEN].channel, LEDC_TEST_DUTY);
					ledc_update_duty(ledc_channel[GREEN].speed_mode, ledc_channel[GREEN].channel);
					vTaskDelay(250 / portTICK_PERIOD_MS);
					ledc_set_duty(ledc_channel[GREEN].speed_mode, ledc_channel[GREEN].channel, 0);
					ledc_update_duty(ledc_channel[GREEN].speed_mode, ledc_channel[GREEN].channel);
				} else {
					//Blink Red twice
					for (int i = 0; i < 4; i++) {
						if ( i % 2) {
							ledc_set_duty(ledc_channel[RED].speed_mode, ledc_channel[RED].channel, 0);
						} else {
							ledc_set_duty(ledc_channel[RED].speed_mode, ledc_channel[RED].channel, LEDC_TEST_DUTY);
						}
						ledc_update_duty(ledc_channel[RED].speed_mode, ledc_channel[RED].channel);
						vTaskDelay(125 / portTICK_PERIOD_MS);
					}
				}
				memset(keypadInputBuffer, 0, KEYPAD_BUF_SZ);
				keypadInputCnt = 0;
			}
			if (keypadInputBuffer[keypadInputCnt - 1] == '*') {
				//Check Pin
				keypadInputBuffer[keypadInputCnt - 1] = 0;
				bufTranslate = atoi(keypadInputBuffer);
				if (bufTranslate == keypadPin && keyPinValid) {
					ledc_set_duty(ledc_channel[GREEN].speed_mode, ledc_channel[GREEN].channel, LEDC_TEST_DUTY);
					ledc_update_duty(ledc_channel[GREEN].speed_mode, ledc_channel[GREEN].channel);
					toggle_Lock();
					ledc_set_duty(ledc_channel[GREEN].speed_mode, ledc_channel[GREEN].channel, 0);
					ledc_update_duty(ledc_channel[GREEN].speed_mode, ledc_channel[GREEN].channel);
				} else {
					//Blink Red twice
					for (int i = 0; i < 4; i++) {
						if ( i % 2) {
							ledc_set_duty(ledc_channel[RED].speed_mode, ledc_channel[RED].channel, 0);
						} else {
							ledc_set_duty(ledc_channel[RED].speed_mode, ledc_channel[RED].channel, LEDC_TEST_DUTY);
						}
						ledc_update_duty(ledc_channel[RED].speed_mode, ledc_channel[RED].channel);
						vTaskDelay(125 / portTICK_PERIOD_MS);
					}
				}
				memset(keypadInputBuffer, 0, KEYPAD_BUF_SZ);
				keypadInputCnt = 0;
			}
			if (keypadInputCnt == KEYPAD_BUF_SZ) {
				//buffer limit reached; wrong code or menu option entered.
				memset(keypadInputBuffer, 0, KEYPAD_BUF_SZ);
				keypadInputCnt = 0;
				//Do error message or something
			}
        }
    }
}

/** @brief Keypad GPIO interupt handler for setting new pin. 
 * 
 * \param *arg: Optional switch on GPIO pin.
 */
static void keypadPINCallback(void* arg)
{
	uint32_t adc1_gpio34 = 0;
    uint32_t io_num;
	uint64_t bufTranslate;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
			if(keypadLastUpdate != 0 && (keypadLastUpdate + keypadInvalidAfter) > runTime()) {
				//Timeout period reached; reset buffer before input
				memset(keypadInputBuffer, 0, KEYPAD_BUF_SZ);
				keypadInputCnt = 0;
			}
			// Wait 5ms for input to settle
			vTaskDelay(5 / portTICK_PERIOD_MS);
			keypadLastUpdate = runTime();
			adc1_gpio34 = 0;
			for(int i = 0; i < MULTISAMPLING; i++) {
				adc1_gpio34 = adc1_get_raw(ADC1_CHANNEL_6);
			}
			adc1_gpio34 /= MULTISAMPLING;
			for (int i=0; i<12; i++) {
				if(adc1_gpio34 < thresholds[i]) {
					keypadInputBuffer[keypadInputCnt] = keypad[i];
				}
			}
			if(adc1_gpio34 > thresholds[11]) {
				keypadInputBuffer[keypadInputCnt] = keypad[11];
			}
			keypadInputCnt++;
			if (keypadInputBuffer[keypadInputCnt - 1] == '*') {
				//Check Pin
				keypadInputBuffer[keypadInputCnt - 1] = 0;
				bufTranslate = atoi(keypadInputBuffer);
				//Update pin in RAM
				keypadPin = bufTranslate;
				//Update pin in NVS
				writePinNVS(keypadPin);
				//Flash Blue
				ledc_set_duty(ledc_channel[Blue].speed_mode, ledc_channel[Blue].channel, LEDC_TEST_DUTY);
				ledc_update_duty(ledc_channel[Blue].speed_mode, ledc_channel[Blue].channel);
				toggle_Lock();
				ledc_set_duty(ledc_channel[Blue].speed_mode, ledc_channel[Blue].channel, 0);
				ledc_update_duty(ledc_channel[Blue].speed_mode, ledc_channel[Blue].channel);
				//Clear buffer for other process
				memset(keypadInputBuffer, 0, KEYPAD_BUF_SZ);
				keypadInputCnt = 0;
				//Resume other task and suspend self
				vTaskResume( xKeypadHandle );
				vTaskSuspend( NULL );
			}
			if (keypadInputCnt == KEYPAD_BUF_SZ) {
				//buffer limit reached;
				memset(keypadInputBuffer, 0, KEYPAD_BUF_SZ);
				keypadInputCnt = 0;
				//Resume other task and suspend self
				vTaskResume( xKeypadHandle );
				vTaskSuspend( NULL );
			}
        }
    }
}



/**
 * @brief Initialize GPIO pins. 
 * Ref: https://github.com/espressif/esp-idf/blob/master/examples/peripherals/gpio/generic_gpio/main/gpio_example_main.c
*/
void initGPIO() {
	//Start ADC setup
		gpio_config_t io_conf;

		//Keypad Input
		io_conf.mode = GPIO_MODE_INPUT;
		io_conf.intr_type = GPIO_INTR_NEGEDGE;
		io_conf.pin_bit_mask = GPIO_KEYPAD;
		io_conf.pull_down_en = 0;
		io_conf.pull_up_en = 0;
		if(gpio_config(&io_conf) != ESP_OK)
			printf("Error setting up GPIO 34 | Keypad Input");

		//Start Battery check ADC
		io_conf.mode = GPIO_MODE_INPUT;
		io_conf.intr_type = GPIO_INTR_NEGEDGE;
		io_conf.pin_bit_mask = GPIO_BATV;
		io_conf.pull_down_en = 0;
		io_conf.pull_up_en = 0;
		if(gpio_config(&io_conf) != ESP_OK)
			printf("Error setting up GPIO 35 | BatV Input");

		adc_power_on();
		adc1_config_width(ADC_WIDTH_BIT_12);
		adc1_config_channel_atten(ADC1_CHANNEL_6,ADC_ATTEN_DB_11);	//and attetntuation | Channel 6 = gpio34 | 11 db = 0 to 3.9v attentuation
		adc1_config_channel_atten(ADC1_CHANNEL_7,ADC_ATTEN_DB_11);	//and attetntuation | Channel 7 = gpio35 | 11 db = 0 to 3.9v attentuation
		esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1101, &characteristics);
		gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
		//Create FreeRTOS task here for keypad events. Maybe move to MAIN later.
		xTaskCreate(keypadCallback, "keypadCallback_task", 2048, NULL, 10, &xKeypadHandle);
		//Create FreeRTOS task for changing pin. Immediately suspend.
		xTaskCreate(keypadPINCallback, "keypadPINCallback_task", 2048, NULL, 10, &xPinHandle);
		vTaskSuspend( xPinHandle );
		gpio_install_isr_service(ESP_INTR_FLAG_LEVEL);
		gpio_isr_handler_add(GPIO_KEYPAD_IO, gpio_isr_handler, (void*) GPIO_KEYPAD_IO);
	//End Keypad Setup
	//Start Status RGB Setup
		ledc_timer_config_t ledc_timer = {
			.duty_resolution = LEDC_TIMER_13_BIT,
			.freq_hz = 5000,
			.speed_mode = LEDC_HS_MODE,
			.timer_num = LEDC_HS_TIMER,
			.clk_cfg = LEDC_AUTO_CLK,
		};
    	ledc_timer_config(&ledc_timer);
		for (int ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
			ledc_channel_config(&ledc_channel[ch]);
		}
		//Dance and say hello.
		for (int i = 0; i < 6; i++) {
			if ( i % 2) {
				ledc_set_duty(ledc_channel[GREEN].speed_mode, ledc_channel[GREEN].channel, 0);
			} else {
				ledc_set_duty(ledc_channel[GREEN].speed_mode, ledc_channel[GREEN].channel, LEDC_TEST_DUTY);
			}
			ledc_update_duty(ledc_channel[GREEN].speed_mode, ledc_channel[GREEN].channel);
			vTaskDelay(167 / portTICK_PERIOD_MS);
		}
	//End Status RGB Setup
	//Start Motor Control Setup
		mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MTR_PWM0A_OUT);
		mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MTR_PWM0B_OUT);
		io_conf.mode = GPIO_MODE_OUTPUT;
		io_conf.pin_bit_mask = MTR_Sleep;
		io_conf.pull_down_en = true;
		io_conf.pull_up_en = false;
		if(gpio_config(&io_conf) != ESP_OK)
			printf("Error setting up GPIO 33 | Motor Sleep");
		gpio_set_level(MTR_SleepC, true);
		mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
	//End Motor Control Setup
}

/**
 * @brief Initialize NVS; pulls key qty and pin from storage into memory. 
 * Ref: https://github.com/espressif/esp-idf/tree/1d7068e/examples/storage/nvs_rw_value
*/
void init_NVS() {
    esp_err_t err = nvs_flash_init();
	//Error handling as recommended by example. May be able to remove later.
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
    nvs_handle_t my_handle;
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
		//Error condition if NVS cannot be opened. Turn into LED status.
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
		//NVS handle open, withdraw data.
        uint8_t tmpStoredKey_QTY = 0;
		err = nvs_get_u8(my_handle, "storedKey_QTY", &tmpStoredKey_QTY);
        switch (err) {
            case ESP_OK:
				storedKey_QTY = tmpStoredKey_QTY;
                printf("Done\n");
                break;
            case ESP_ERR_NVS_NOT_FOUND:
				//Must be first time device is being powered up. YAY!!!
                printf("The value is not initialized yet!\n");
                break;
            default :
				printf("Error (%s) reading!\n", esp_err_to_name(err));
        }
        uint64_t tmpkeypadPin = 0;
		err = nvs_get_u64(my_handle, "keypadPin", &tmpkeypadPin);
        switch (err) {
            case ESP_OK:
				keypadPin = tmpkeypadPin;
				keyPinValid = true;
                printf("Done\n");
                break;
            case ESP_ERR_NVS_NOT_FOUND:
				//Must be first time device is being powered up. YAY!!!
                printf("The value is not initialized yet!\n");
				//Default Key Pin
				keypadPin = defaultKEY;
				keyPinValid = true;
                break;
            default :
				printf("Error (%s) reading!\n", esp_err_to_name(err));
        }
        nvs_close(my_handle);
	}
}

/**
 * @brief Stores keypad pin into non-voltile storage 
 * 
 * \param secret: Input pin to be stored
*/
void writePinNVS(uint64_t secret) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
    nvs_handle_t place_handle;
    err = nvs_open("storage", NVS_READWRITE, &place_handle);
    if (err != ESP_OK) {
		//Error condition if NVS cannot be opened. Turn into LED status.
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        err = nvs_set_u64(place_handle, "keypadPin", secret);
        err = nvs_commit(place_handle);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
        nvs_close(place_handle);
	}
}

/**
 * @brief Grabs NFC keylist from NVS.
 * 
 * \param *buf: Key Output
 * \param count: Key Index
 */
void grabKeyList(uint8_t *buf, int count) {
    esp_err_t err = nvs_flash_init();
	//Error handling as recommended by example. May be able to remove later.
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
    nvs_handle_t my_handle;
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
		//Error condition if NVS cannot be opened. Turn into LED status.
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
		uint8_t tmp_key[serialLen];
		size_t size = sizeof(tmp_key);
		char* tmpNvsKey;
		tmpNvsKey = "NFCKEY";
		strcat(tmpNvsKey, (char*)count);
		err = nvs_get_blob(my_handle, tmpNvsKey, tmp_key, &size);
		switch (err) {
			case ESP_OK:
				*buf = *tmp_key; //double check this line, thx
				printf("Grabbed Key: %u\n",count);
				break;
			case ESP_ERR_NVS_NOT_FOUND:
				//Stored keys do not equal qty reported. Bad.
				*buf = 0;
				printf("The key %u is not initialized yet!\n",count);
				break;
			default :
				*buf = 0;
				printf("Error (%s) reading!\n", esp_err_to_name(err));
		}
		tmpNvsKey[0] = '\0';
		if (err != ESP_OK) {
			printf("Error (%s) grabbing key!\n", esp_err_to_name(err));
		}
        nvs_close(my_handle);
	}
}

/**
 * @brief Writes NFC key to NVS.
 *  Only writes one key, not all NFC keys at once. (For use in pairing new NFC Tag)
 * 
 * \param *buf: Key Input
 * \param count: Key Index
 */
void writeNFCKey(uint8_t *buf, int count) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
    nvs_handle_t blobHandle;
    err = nvs_open("storage", NVS_READWRITE, &blobHandle);
    if (err != ESP_OK) {
		//Error condition if NVS cannot be opened. Turn into LED status.
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
		uint8_t tmp_key[serialLen];
		//Replace line below with known size
		size_t size = sizeof(tmp_key);
		char* tmpNvsKey;
		tmpNvsKey = "NFCKEY";
		strcat(tmpNvsKey, (char*)count);
		err = nvs_set_blob(blobHandle, tmpNvsKey, buf, size);
        err = nvs_commit(blobHandle);
		switch (err) {
			case ESP_OK:
				//Successful Write
				printf("Wrote Key: %u\n",count);
				break;
			default :
				printf("Error (%s) Writing!\n", esp_err_to_name(err));
		}
        nvs_close(blobHandle);
	}
}

/**
 * @brief Removes NFC keys from heap and NVS.
 */
void unpairNFC() {
	//First, free from heap
	for (int i=0; i<storedKey_QTY-1; i++) {
		free(keyList[i]);
	}
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
    nvs_handle_t unpairHandle;
    err = nvs_open("storage", NVS_READWRITE, &unpairHandle);
    if (err != ESP_OK) {
		//Error condition if NVS cannot be opened. Turn into LED status.
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
		//Replace lines below with known size
		char* tmpNvsKey;
		for (int i=0; i<storedKey_QTY-1; i++) {
			tmpNvsKey = "NFCKEY";
			strcat(tmpNvsKey, (char*)i);
			nvs_erase_key(unpairHandle, tmpNvsKey);
		}
        err = nvs_commit(unpairHandle);
		switch (err) {
			case ESP_OK:
				//Successful Write
				printf("Successful Clear: \n");
				break;
			default :
				printf("Error (%s) Writing!\n", esp_err_to_name(err));
		}
        nvs_close(unpairHandle);
	}
	storedKey_QTY = 0;
}

/**
 * @brief Stores NFC key qty into non-voltile storage 
 * 
 * \param qty: Value to be stored
*/
void writeKeyQtyNVS(uint8_t qty) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
    nvs_handle_t placeqtyHand;
    err = nvs_open("storage", NVS_READWRITE, &placeqtyHand);
    if (err != ESP_OK) {
		//Error condition if NVS cannot be opened. Turn into LED status.
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        err = nvs_set_u8(placeqtyHand, "storedKey_QTY", qty);
        err = nvs_commit(placeqtyHand);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
        nvs_close(placeqtyHand);
	}
}

/**
 * @brief Callback function for rc522 library under normal operation.
 */
void tag_handler(uint8_t* serial_no) {
	bool thisIsNotTheKeyYouSeek = true;
	//Check every stored NFC key
	for(int i = 0; i < storedKey_QTY-1; i++) {
		if (serial_no == keyList[i]) {
			ledc_set_duty(ledc_channel[GREEN].speed_mode, ledc_channel[GREEN].channel, LEDC_TEST_DUTY);
			ledc_update_duty(ledc_channel[GREEN].speed_mode, ledc_channel[GREEN].channel);
			toggle_Lock();
			ledc_set_duty(ledc_channel[GREEN].speed_mode, ledc_channel[GREEN].channel, 0);
			ledc_update_duty(ledc_channel[GREEN].speed_mode, ledc_channel[GREEN].channel);
			thisIsNotTheKeyYouSeek = false;
		}
	}
	if (thisIsNotTheKeyYouSeek) {
		//Blink Red thrice
		for (int i = 0; i < 6; i++) {
			if ( i % 2) {
				ledc_set_duty(ledc_channel[RED].speed_mode, ledc_channel[RED].channel, 0);
			} else {
				ledc_set_duty(ledc_channel[RED].speed_mode, ledc_channel[RED].channel, LEDC_TEST_DUTY);
			}
			ledc_update_duty(ledc_channel[RED].speed_mode, ledc_channel[RED].channel);
			vTaskDelay(250 / portTICK_PERIOD_MS);
		}
	}
}

/**
 * @brief Callback function for rc522 when pairing new NFC tags.
 */
void pairTagHandler(uint8_t* serial_no) {
	//Allocate heap and update
	keyList[storedKey_QTY] = (uint8_t *)malloc(serialLen * sizeof(uint8_t));
	keyList[storedKey_QTY] = serial_no;
	//Update NVS
	writeNFCKey(keyList[storedKey_QTY], storedKey_QTY);
	storedKey_QTY++;
	writeKeyQtyNVS(storedKey_QTY);
	noPairDect = false;
}

/**
 * @brief Handles motor control and position logic.
 */
void toggle_Lock() {
	gpio_set_level(MTR_SleepC, false);
	vTaskDelay(5 / portTICK_PERIOD_MS);
	if (mtrForward) {
		mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
		mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MTR_duty);
		mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
		mtrForward = false;
	} else {
		mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    	mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MTR_duty);
    	mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
		mtrForward = true;
	}
	vTaskDelay(MTR_PERIOD / portTICK_PERIOD_MS);
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
	vTaskDelay(5 / portTICK_PERIOD_MS);
	gpio_set_level(MTR_SleepC, true);
}

void app_main(void) {
	// Runs rc522_init or handles esp32 error.
	// rc522_init runs startup, tests, and starts "rc522_task";
	// Maybe adjust "rc522_task" to run on FreeRTOS HW interupt and not timer.
	init_NVS();
	if(storedKey_QTY != 0) {
		for (int i=0; i<storedKey_QTY-1; i++) {
			keyList[i] = (uint8_t *)malloc(serialLen * sizeof(uint8_t));
			grabKeyList(keyList[i],i);
		}
	}
	initGPIO();
	rc522_start(running_args);
	xTaskCreate(batCheck, "Battery_Check_Task", 1024, NULL, 3, NULL);
}