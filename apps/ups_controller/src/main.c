/*
 * Copyright (c) 2023 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c/stm32.h>
#include <zephyr/drivers/smbus.h>
#include <zephyr/shell/shell.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/smf.h>
#include <zephyr/devicetree.h>

#include "batt.h"

LOG_MODULE_REGISTER(ups_controller, LOG_LEVEL_DBG);

#define ZEPHYR_USR_NODE DT_PATH(zephyr_user)

#define DRV_COMPAT "st,stm32-timers"
#define DAC_NODE_1 DT_PHANDLE(ZEPHYR_USR_NODE, dac1)
#define DAC_NODE_2 DT_PHANDLE(ZEPHYR_USR_NODE, dac2)
#define DAC_CHANNEL_ID DT_PROP(ZEPHYR_USR_NODE, dac_channel_id)
#define DAC_RESOLUTION DT_PROP(ZEPHYR_USR_NODE, dac_resolution)
#define I2C_DEV_NODE DT_NODELABEL(i2c3)

/* both DACs use channel 1 and 12 bit resolution */

static const struct device *const dac_dev[] = {

	DEVICE_DT_GET(DAC_NODE_1),
	DEVICE_DT_GET(DAC_NODE_2),
};

/* NOTE: same configuration used by all DAC devices */
static const struct dac_channel_cfg dac_ch_cfg = {
	.channel_id = DAC_CHANNEL_ID,
	.resolution = DAC_RESOLUTION,
#if defined(CONFIG_DAC_BUFFER_NOT_SUPPORT)
	.buffered = false,
#else
	.buffered = true,
#endif
};
/* remember, we use the same config and setup for both DAC's */

#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
	!DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
			     DT_SPEC_AND_COMMA)
};
// something jacked up with DT_PROP
// #define ADC_SPAN_CORR (3000/DT_PROP(DT_NODELABEL(adc1), vref-mv))
#define ADC_SPAN_CORR 1.0  //controller has external precision 3.0V reference
// So ADC_SPAN_CORR should be 3/<actual value of vref>
//fixed by hardware design:
//NOTE: LSB values are normalized to Amps and Volts.  Smart battery parameters
//      return values in mA and mV (integers, no floating point format).
//      Be sure to check your math in the measurement routines.
#define BATT_CHG_CUR_LSB 0.0096265  //39.43A full scale, ADC measurement
#define BATT_DISCHG_CUR_LSB 0.0070521 // 28.885A full scale. Hey, I didn't design this crazy supply... ADC measurement
#define VC_SUPPLY_CTRL_V 0.0009801 // D/A output to drive VC, DAC output
#define VC_SUPPLY_CTRL_V_OFFSET 10.483 //Control range is limited by stopping resistors to 10.483 to 14.501V
#define VC_SUPPLY_VOLT 0.003662 // These two incorporate a divide by 5 in external HW in the supply, ADC measurement
#define VC_INVERTER_BOOST_CURRENT_MEAS 0.0061035  // adc 4 LSB, full scale = 25A ADC measurement
#define VC_SUPPLY_VOLTAGE_MEAS 0.0036621 // adc 3 lsb, full scale = 15V
#define BATT_FLOAT_VOLTAGE 13.8
#define BATT_MAX_CHG_VOLTAGE 14.4
#define BATT_MAX_CHG_CURRENT 15  //in mA note. for now, depends on how much oomph VC can give us plus inverter
#define BATT_MIN_CHG_CURRENT 5  //purely arbitrary
#define VC_SUPPLY_MIN_VOLTAGE 12.5  //lower than this the Dc Dc boost for inverter will start to fall over at full load
#define BATT_MAX_SOC 99  //as in 99% which is when we switch to float charging.
#define MAX_TOTAL_VC_CURRENT 20000 //mA  Safe value for now.
#define INVERTER_START_WAIT_TIME 500 // milliseconds
#define MAX_BATT_DISCH_CURRENT 20000 //maximum continuous current.
#define BATT_EMERGENCY_SHUTDOWN_CURRENT 27000 //If it hits this even for an instant it's nitey nite
#define BATT_SOC_CRITICAL 2 // as in 2%- have to leave a bit of something to keep system sane
#define VC_SUPPLY_CURR_LIMIT 0.0061035 // 25A full scale, THIS IS ONLY A GUESS FOR NOW

#define BATT_CHECK_MS 5000 /* smart batt spec says check every 10 seconds but that's too slow */
// Some numbers:
// Valid voltage range for VC is ~10.5V to ~14.5V
// Float voltage for LiPoFe batt pack is 13.8V
// Maximum charge voltage is 14.4V
// Maximum battery charge current is 18A
// Maximum inverter current consumption is about 18A I think (hasn't been established).
// Maximum continuous battery discharge current is 20A, 28A for less than 10secs (sound warning), over 28A shut it down.

// index values for analog stuff:
#define DAC_CHG_CURRENT 1
#define DAC_VC_VOLT 0
#define ADC_BATT_CHG_CUR 0
#define ADC_BATT_DISCHG_CUR 1
#define ADC_VC_VOLTAGE 2
#define ADC_VC_CURRENT 3
#define ADC_SUPPLY_TEMP 4  //Crude thermistor analog voltage from the supply, no resistor spec as of 12/8/25
#define SYS_TEMP_HIGH 4095 //arbitrary numbers until we figure out what the thermistor is doing
#define SYS_TEMP_LOW 4000 //need a little hysteresis in here

// NOTE, lsb values were derived mathematically from sumulations
// and are referenced to 3.00V a/d span.

double raw_to_real(double lsb_value, int16_t raw_param) {
	return ((lsb_value * raw_param) * ADC_SPAN_CORR);  //ADC_SPAN_CORR being slightly less or greater than 1
}

int real_to_raw(double lsb_value, double real_param) {
	double test;

	test = ((real_param/lsb_value) * ADC_SPAN_CORR);
	if (test > 4095) {
		return 4095;
	}
	else {
		return (int)test;
	}
}

static const struct smf_state ups_states[];

#define GPIOA_DEV DEVICE_DT_GET(DT_NODELABEL(gpioa))
#define GPIOB_DEV DEVICE_DT_GET(DT_NODELABEL(gpiob))
#define GPIOC_DEV DEVICE_DT_GET(DT_NODELABEL(gpioc))

struct gpio {
        char *desc;
        const struct device *port;
        gpio_pin_t gpio;
        gpio_flags_t flags;
};

enum {
	GPIO_VC_TO_VINA_EN,
	GPIO_BATT_DISCHG_PROT_OVERRIDE,
	GPIO_AC_OUT_DETECT,
	GPIO_BUZZER,
	GPIO_BATT_DISCONNECT,
	GPIO_AC_INPUT_DETECT,
	GPIO_ENABLE_PB,
	GPIO_VB_AND_VD_ASSERTED,
	GPIO_FAN_ENABLE,
	GPIO_MUTE_PB,
	GPIO_ON_OFF_PB,
	GPIO_AC_OUT_LED,
	GPIO_LED_1_RED,
	GPIO_LED_1_GREEN,
	GPIO_LED_2_RED,
	GPIO_LED_2_GREEN,
	GPIO_LED_3,
	GPIO_LED_4,
	GPIO_INVERTER_ENABLE,
	GPIO_USB_VBUS_DETECT,
	GPIO_FAN_TACH,
	GPIO_USER_LED,
};

// the order of the gpios in the enum must match the order in which they are defined below:

const struct gpio gpios_upsctl[] = {
	{"VC_TO_VINA_EN", GPIOA_DEV, 7, GPIO_OUTPUT},
	{"BATT_DISCHG_PROT_OVERRIDE", GPIOA_DEV, 8, GPIO_OUTPUT},
	{"AC_OUT_DETECT", GPIOA_DEV, 9, GPIO_INPUT | GPIO_ACTIVE_LOW},
	{"BUZZER", GPIOB_DEV, 0, GPIO_OUTPUT},
	{"BATT_DISCONNECT", GPIOB_DEV, 6, GPIO_OUTPUT},
	{"AC_INPUT_DETECT", GPIOB_DEV, 7, GPIO_INPUT | GPIO_ACTIVE_LOW},
	{"ENABLE_PB", GPIOB_DEV, 9, GPIO_INPUT | GPIO_ACTIVE_LOW},
	{"VB_AND_VD_ASSERTED", GPIOB_DEV, 10, GPIO_INPUT | GPIO_ACTIVE_LOW},
	{"FAN_ENABLE", GPIOB_DEV, 11, GPIO_OUTPUT},
	{"MUTE_PB", GPIOB_DEV, 14, GPIO_INPUT | GPIO_ACTIVE_LOW},
	{"ON_OFF_PB", GPIOB_DEV, 15, GPIO_INPUT | GPIO_ACTIVE_LOW},
	{"AC_OUT_LED", GPIOC_DEV, 0, GPIO_OUTPUT},
	{"LED_1_RED", GPIOC_DEV, 2, GPIO_OUTPUT | GPIO_ACTIVE_LOW},
	{"LED_1_GREEN", GPIOC_DEV, 1, GPIO_OUTPUT | GPIO_ACTIVE_LOW},
	{"LED_2_RED", GPIOC_DEV, 4, GPIO_OUTPUT | GPIO_ACTIVE_LOW},
	{"LED_2_GREEN", GPIOC_DEV, 3, GPIO_OUTPUT | GPIO_ACTIVE_LOW},
	{"LED_3", GPIOC_DEV, 5, GPIO_OUTPUT},
	{"LED_4", GPIOC_DEV, 6, GPIO_OUTPUT},
	{"INVERTER_ENABLE", GPIOC_DEV, 13, GPIO_OUTPUT},
	{"USB_VBUS_DETECT", GPIOC_DEV, 14, GPIO_INPUT | GPIO_ACTIVE_LOW},
	{"FAN_TACH", GPIOC_DEV, 15, GPIO_INPUT},
	{"USER_LED",GPIOA_DEV, 5, GPIO_OUTPUT},
};

const struct gpio *gpios;
size_t gpios_len;

// tie everything together
int gpio_init()
{
	gpios = gpios_upsctl;
	gpios_len = ARRAY_SIZE(gpios_upsctl);
	
	for (int i = 0; i < gpios_len; i++ ) {
		int ret = gpio_pin_configure(gpios[i].port, gpios[i].gpio, gpios[i].flags);
		if (ret != 0) {
			LOG_ERR("Error configuring GPIO for %s\n", gpios[i].desc);
		}
	}
	return 0;
}

// some simplification for accessing gpio ports:
int gpio_get(int id)
{
        if (id < 0 || id >= gpios_len) {
//                LOG_ERR("GPIO ID out of bounds: %i", id);
		LOG_ERR("GPIO ID out of bounds: %i\n", id);
                return 0;
        }

        return gpio_pin_get(gpios[id].port, gpios[id].gpio);
}

int gpio_set(int id, int state)
{
        if (id < 0 || id >= gpios_len) {
//                LOG_ERR("GPIO ID out of bounds: %i", id);
		LOG_ERR("GPIO ID out of bounds: %i\n", id);
                return 0;
        }

        return gpio_pin_set(gpios[id].port, gpios[id].gpio, state);
}


// Additional shell commands for the analog stuff:

static int cmd_get_adc(const struct shell *shell, size_t argc, char **argv) {
	uint16_t buf;
	int32_t err;
	struct adc_sequence sequence = {
                .buffer = &buf,
                /* buffer size in bytes, not number of samples */
                .buffer_size = sizeof(buf),
        };
 /* Configure channels individually prior to sampling. */
	shell_print(shell, "Found %d a/d channels", ARRAY_SIZE(adc_channels));
        for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
                if (!adc_is_ready_dt(&adc_channels[i])) {
                        shell_error(shell, "ADC controller device %s not ready\n", adc_channels[i].dev->name);
                        return -EINVAL;
                }

                err = adc_channel_setup_dt(&adc_channels[i]);
                if (err < 0) {
                        shell_error(shell, "Could not setup channel #%d (%d)\n", i, err);
                        return -EINVAL;
                }
        }

	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
		int32_t val_mv;

		shell_print(shell, "- %s, channel %d: ",
		       adc_channels[i].dev->name,
		       adc_channels[i].channel_id);

		(void)adc_sequence_init_dt(&adc_channels[i], &sequence);
		err = adc_read_dt(&adc_channels[i], &sequence);
		if (err < 0) {
			shell_error(shell, "Could not read (%d)\n", err);
			continue;
		}

		/*
		 * If using differential mode, the 16 bit value
		 * in the ADC sample buffer should be a signed 2's
		 * complement value.
		 */
		if (adc_channels[i].channel_cfg.differential) {
			val_mv = (int32_t)((int16_t)buf);
		} else {
			val_mv = (int32_t)buf;
		}
		shell_print(shell, "%"PRId32, val_mv);
	}
	return 0;
}

SHELL_CMD_REGISTER(get_adc, NULL, "Read ADC's", cmd_get_adc);

static int cmd_write_dac(const struct shell *shell, size_t argc, char **argv) {
	if (argc < 3) {
		shell_error(shell, "Please provide <dac id> <dac_value>.");
		return -EINVAL;
	}
	int i = atoi(argv[1]) -1; // we are expecting "1" or "2"
	int val_mv = atoi(argv[2]);
	if (val_mv > 4095 || val_mv < 0) {
		shell_error(shell, "Invalid value for DAC passed in write_dac: %d\n", val_mv);
		return -EINVAL;
	}
	
	if (i == 0 || i == 1) { //just fail silently if buggered input
		int ret = dac_write_value(dac_dev[i], DAC_CHANNEL_ID, val_mv);
		if (ret != 0) {
			shell_error(shell, "dac_write_value() for DAC%d failed with code %d\n", i+1, ret);
			return -EINVAL;
		}
	}
	return 0;
}

SHELL_CMD_REGISTER(write_dac, NULL, "Write DAC's", cmd_write_dac);

const struct device *i2c_dev = DEVICE_DT_GET(I2C_DEV_NODE);

bool ac_mode = false; // false when no AC line present
 
typedef struct {
	bool state;
	bool flashing;
	int led_gpio;
} led_mode;

led_mode leds_array[6] = {
	{false, false, GPIO_LED_1_GREEN},
	{false, false, GPIO_LED_1_RED},
	{false, false, GPIO_LED_2_GREEN},
	{false, false, GPIO_LED_2_RED},
	{false, false, GPIO_LED_3},
	{false, false, GPIO_LED_4},
};

void batt_svc_handler(struct k_work *work)
{
	uint16_t buf;
	struct adc_sequence sequence = {
                .buffer = &buf,
                /* buffer size in bytes, not number of samples */
                .buffer_size = sizeof(buf),
        };
	uint16_t sys_temp;

	(void)adc_sequence_init_dt(&adc_channels[ADC_SUPPLY_TEMP],&sequence);
	int err = adc_read_dt(&adc_channels[ADC_SUPPLY_TEMP],&sequence);
	if (err < 0) {
		LOG_ERR("Could not read ADC3 in work queue.");
	}
	sys_temp = (uint16_t)buf; 
	if (sys_temp > SYS_TEMP_HIGH) {
		gpio_set(GPIO_FAN_ENABLE, 1);
	}
	else {
		if (sys_temp < SYS_TEMP_LOW) {
			gpio_set(GPIO_FAN_ENABLE, 0);
		}
	}

	load_batt_info(&batt_info, i2c_dev);
	LOG_INF("System temp: %d", sys_temp);
	LOG_INF("Batt temp: %d", batt_info.temperature);
	LOG_INF("Batt voltage: %d", batt_info.voltage);
	LOG_INF("Batt current: %d", batt_info.current);
	LOG_INF("Batt avg current: %d", batt_info.avg_current);
	LOG_INF("Batt SOC: %d", batt_info.state_of_chg);
	LOG_INF("Remaining capacity: %d", batt_info.remaining_capacity);
	LOG_INF("Average time to empty: %d", batt_info.avg_time_to_empty);
	LOG_INF("Batt status: %d", batt_info.batt_stats);
// Update LED array:
	if (ac_mode) { //when charging
		if (batt_info.state_of_chg <= 30) {
			leds_array[0].state = true;
			leds_array[0].flashing = false;
			leds_array[1].state = false;
			leds_array[2].state = false;
			leds_array[3].state = false;
			leds_array[4].state = false;
			leds_array[5].state = true;
			leds_array[5].flashing = true;
		}
		else if (batt_info.state_of_chg > 30 && batt_info.state_of_chg < 60) {
			leds_array[0].state = true;
			leds_array[0].flashing = false;
			leds_array[1].state = false;
			leds_array[2].state = true;
			leds_array[2].flashing = false;
			leds_array[3].state = false;
			leds_array[4].state = false;
			leds_array[5].state = true;
			leds_array[5].flashing = true;
		}
		else if (batt_info.state_of_chg > 59 && batt_info.state_of_chg < 90) {
			leds_array[0].state = true;
			leds_array[0].flashing = false;
			leds_array[1].state = false;
			leds_array[2].state = true;
			leds_array[2].flashing = false;
			leds_array[3].state = false;
			leds_array[4].state = true;
			leds_array[4].flashing = false;
			leds_array[5].state = true;
			leds_array[5].flashing = true;
		}
		else {
			leds_array[0].state = true;
			leds_array[0].flashing = false;
			leds_array[1].state = false;
			leds_array[2].state = true;
			leds_array[2].flashing = false;
			leds_array[3].state = false;
			leds_array[4].state = true;
			leds_array[4].flashing = false;
			leds_array[5].state = true;
		}
	}
	else {
		if (batt_info.state_of_chg > 89 ) {
			leds_array[0].state = true;
			leds_array[0].flashing = false;
			leds_array[1].state = false;
			leds_array[2].state = true;
			leds_array[2].flashing = false;
			leds_array[3].state = false;
			leds_array[4].state = true;
			leds_array[4].flashing = false;
			leds_array[5].state = true;
			leds_array[5].flashing = false;
		}
		else if (batt_info.state_of_chg > 59 && batt_info.state_of_chg < 90) {
			leds_array[0].state = true;
			leds_array[0].flashing = false;
			leds_array[1].state = false;
			leds_array[2].state = true;
			leds_array[2].flashing = false;
			leds_array[3].state = false;
			leds_array[4].state = true;
			leds_array[4].flashing = false;
			leds_array[5].state = false;
		}
		else if (batt_info.state_of_chg > 10 && batt_info.state_of_chg < 60) {
			leds_array[0].state = true;
			leds_array[0].flashing = false;
			leds_array[1].state = false;
			leds_array[2].state = true;
			leds_array[2].flashing = false;
			leds_array[3].state = false;
			leds_array[4].state = false;
			leds_array[5].state = false;
		}
		else if (batt_info.state_of_chg > 5 && batt_info.state_of_chg < 11) {
			leds_array[0].state = true;
			leds_array[0].flashing = false;
			leds_array[1].state = true;
			leds_array[1].flashing = false;
			leds_array[2].state = true;
			leds_array[3].state = true;
			leds_array[4].state = false;
			leds_array[5].state = false;
		}
		else { //we are about to die...
			leds_array[0].state = false;
			leds_array[1].state = true;
			leds_array[1].flashing = true;
			leds_array[2].state = false;
			leds_array[3].state = false;
			leds_array[4].state = false;
			leds_array[5].state = false;
		}
	}
}

K_WORK_DEFINE(batt_svc, batt_svc_handler);

void batt_timer_handler(struct k_timer *dummy)
{
	k_work_submit(&batt_svc);
}

K_TIMER_DEFINE(my_timer, batt_timer_handler, NULL);

// and lastly, front panel keys:
bool on_off_key_pressed = false;
bool on_off_key_released = false;  //useful for long press detections if we ever need that
bool mute_key_pressed = false;
bool mute_key_relased = false;

// Fun stuff now, state machine definitions:

enum ups_state {
	OFF,
	ON_CHG,
	ON_IDLE,
	ON_FULL_CHG,
	ON_FULL_FLOAT,
	ON_BATTERY,
};

struct s_object {
	struct smf_ctx ctx;
	int64_t state_entry_time_ms;
	int64_t beep_time_start;
	int64_t state_delay_time_ms;
	uint16_t batt_state_of_chg;
	uint16_t dac_vc_volt;
	uint16_t dac_batt_current;
	uint16_t desired_chg_current; //in mA, read from the battery
	bool debug;
	bool next; // I may need to add one or 2 more here...
} s_obj;


//Cliff's helper macros:

#define STATE_INIT(o)                                                                              \
	LOG_DBG("");                                                                               \
	struct s_object *s = (struct s_object *)(o);                                               \
	s->state_entry_time_ms = k_uptime_get();                                                   \
	if (s->debug) {                                                                            \
		s->next = false;                                                                   \
	}

#define STATE_OBJ(o) struct s_object *s = (struct s_object *)(o);

#define STATE_TIME_MS() (k_uptime_get() - s->state_entry_time_ms)

#define SET_STATE(state)                                                                           \
	do {                                                                                       \
		if (s->next) {                                                                     \
			smf_set_state(SMF_CTX(&s_obj), &ups_states[state]);                        \
		}                                                                                  \
	} while (0)

static void s_off_entry(void *o)
/**
 * Lots of things to do in this block.  We have come here as a result of one
 * of several possibilities:
 * 1. Initial power up, either new battery connect or battery wake up
 * 2. AC input suddenly appearing with battery initially switched off
 * 3. Conclusion of a normal shutdown request from the front panel
 * 4. Emergency shutdown as battery nears full discharge.
 * Ideally we don't wait until the battery switches off because that kills
 * everything.  Leave a little capacity left so at least we can warn the user
 * through the leds and beeper that "Dude, your batt is flat, plug it in."
 * So first time through completely initialize the system to the OFF state.
 */
{

	STATE_INIT(o); 
	uint8_t rx_buf[2];
	uint8_t tx_buf[2];

	k_timer_stop(&my_timer);	//disable LEDs service and battery data loading
	ac_mode = false;
	gpio_set(GPIO_INVERTER_ENABLE, 0); 
	gpio_set(GPIO_VC_TO_VINA_EN, 0);
	gpio_set(GPIO_BATT_DISCONNECT, 0);
// those were the critical ones, now the rest, we don't like floaters:
	gpio_set(GPIO_BATT_DISCHG_PROT_OVERRIDE, 0);
	gpio_set(GPIO_BUZZER, 0);
	gpio_set(GPIO_FAN_ENABLE, 0);
	gpio_set(GPIO_AC_OUT_LED, 0);
	gpio_set(GPIO_LED_1_RED, 0);
	gpio_set(GPIO_LED_1_GREEN, 0);
	gpio_set(GPIO_LED_2_RED, 0);
	gpio_set(GPIO_LED_2_GREEN, 0);
	gpio_set(GPIO_LED_3, 0);
	gpio_set(GPIO_LED_4, 0);

	s->beep_time_start = -1;         //Give users a moment to get off the button.

	tx_buf[0] = BATT_SOC;
	i2c_write_read(i2c_dev, BATT_SMBUS_ADDR, tx_buf, 1, rx_buf, 2);
	s->batt_state_of_chg = (uint16_t)(rx_buf[0] + 256*rx_buf[1]);  // if batt is offline returns 0, so result is the same as low SOC
	LOG_INF("Entering the OFF state...");
}

/**
 * GPIO_ENABLE_PB is actually a system interlock.  If the front panel is
 * disconnected we can't do anything so just stay here and wait for someone
 * to plug it in.  Note we also should check to see if we have a valid
 * battery attached to the system.  Run without battery?  Dunno, have to
 * discuss this with the project people.
 */

enum smf_state_result s_off_run(void *o)
{
	STATE_OBJ(o);  //must always do this first


	if (gpio_get(GPIO_ENABLE_PB)) { 	//if interlock open just spin here forever 
	
		if (on_off_key_pressed) {  	//on/off button
			if (s->batt_state_of_chg > 5) {
				LOG_DBG("off_run: on/off keypress detected.");
				on_off_key_pressed = false;
				SET_STATE(ON_BATTERY);
				return SMF_EVENT_HANDLED;
			}
			else {
				gpio_set(GPIO_LED_1_RED, 1);
				gpio_set(GPIO_LED_2_RED, 1);
//				gpio_set(GPIO_BUZZER, 1);  //annoy the user
				s->beep_time_start = 100;  //show for 1 second
			}
		}

		if (s->beep_time_start > 0) {   	// in this state we do not have 5V to beep the beeper so all
			s->beep_time_start--;		// we can do is flash the LEDs.
		}
		else if (s->beep_time_start == 0) {
				gpio_set(GPIO_LED_1_RED, 0);
				gpio_set(GPIO_LED_2_RED, 0);
				gpio_set(GPIO_BUZZER, 0);  //peace again
				s->beep_time_start--;  //underflow it to gate off and reset everything.
		}
		if (gpio_get(GPIO_AC_INPUT_DETECT)) {
			SET_STATE(ON_CHG);
		}
	}
	return 0;
}

/**
 *  AC input but inverter off:
 * OK we've got a lot to do here whilst waiting around for user input.  First,
 * there's the battery that needs charging so we'll spend a lot of time here
 * when the user has us turned off but plugged in.  Bring the battery up to 
 * full charge then switch to float mode.
 * This state is entered in one of two ways:
 * 1. from the power off state and someone plugged us in;
 * 2. from the ON_FULL state in response to user input.
 */

static void s_on_chg_entry(void *o)
{
	uint8_t rx_buf[2];
	uint8_t tx_buf[2];
	int ret;
	STATE_INIT(o);

	if (!gpio_get(GPIO_ENABLE_PB)) {
		SET_STATE(OFF);
		return;
	}

	if (!gpio_get(GPIO_AC_INPUT_DETECT)) {
		SET_STATE(OFF);
		return;
	}

	gpio_set(GPIO_LED_1_RED, 0);
	gpio_set(GPIO_LED_2_RED, 0);
	gpio_set(GPIO_BUZZER, 0);
	gpio_set(GPIO_AC_OUT_LED, 0);
	ac_mode = true; // Used by the LEDs rourine in the background
//	We may have come here from a state where the batt svc timer was already running
	if (k_timer_status_get(&my_timer) == 0 && k_timer_remaining_get(&my_timer) == 0) {
		k_timer_start(&my_timer, K_SECONDS(10), K_SECONDS(10));
	}
	gpio_set(GPIO_VC_TO_VINA_EN, 1); //connect battery to analog circuitry

	tx_buf[0] = BATT_SOC;
	i2c_write_read(i2c_dev, BATT_SMBUS_ADDR, tx_buf, 1, rx_buf, 2);
	s->batt_state_of_chg = (uint16_t)(rx_buf[0] + 256*rx_buf[1]);  // if batt is offline returns 0, so result is the same as low SOC

	tx_buf[0] = DESIRED_CHG_CURRENT;
	i2c_write_read(i2c_dev, BATT_SMBUS_ADDR, tx_buf, 1, rx_buf, 2);
	s->desired_chg_current = (uint16_t)(rx_buf[0] + 256*rx_buf[1])/1000;  // if batt is offline returns 0, disabling charging


	s->dac_batt_current = real_to_raw(VC_SUPPLY_CURR_LIMIT,s->desired_chg_current);
	LOG_DBG("VC current limit value: %d", s->dac_batt_current);
	ret = dac_write_value(dac_dev[DAC_VC_VOLT], DAC_CHANNEL_ID, s->dac_batt_current);
	if (ret != 0) {
		LOG_ERR("DAC write failed in state on_chg_run.  Can't set current limit.");
	}
	if (s->batt_state_of_chg < 99) {
		s->dac_vc_volt = real_to_raw(VC_SUPPLY_CTRL_V, (BATT_FLOAT_VOLTAGE - VC_SUPPLY_CTRL_V_OFFSET));
	}
	else {
		s->dac_vc_volt = real_to_raw(VC_SUPPLY_CTRL_V, (BATT_MAX_CHG_VOLTAGE - VC_SUPPLY_CTRL_V_OFFSET));
	}
	LOG_DBG("VC voltage control raw value: %d", s->dac_vc_volt);
	ret = dac_write_value(dac_dev[DAC_VC_VOLT], DAC_CHANNEL_ID, s->dac_vc_volt);
	if (ret != 0) {
		LOG_ERR("DAC write failed in state on_chg_run.  Can't set VC voltage.");
	}
	LOG_INF("Entering the standby ON CHARGE state...");
}

/**
 * OK, so what happens next is we run the current up to whatever the battery wants. While its max charge current spec is 20A, we are much better
 * off running the charger current at the value the battery requests because that will ensure highest accuracy of the state of charge measurements
 * plus maximize the battery life.
 * 
 * As the battery approaches the CV phase the voltage regulator side of VC control will start to take over control of VC from the CC control loop
 * that our DAC feeds here in this routine.  At this point VC will hover around maximum charge voltage until we hit 99% SOC then drops back to
 * float charging. So we really have nothing to do here except keep an eye on the battery and wait for button presses or AC to go away.
 */

enum smf_state_result s_on_chg_run(void *o)
{
	STATE_OBJ(o);

// Safety first:
	if (!gpio_get(GPIO_AC_INPUT_DETECT) || !gpio_get(GPIO_ENABLE_PB)) {
		SET_STATE(OFF);
		return SMF_EVENT_HANDLED;
	}

	if (on_off_key_pressed) {  	//
		on_off_key_pressed = false;
		SET_STATE(ON_FULL_CHG);
		return SMF_EVENT_HANDLED;
	}

//safe to set float voltage first
// so once every batt update we check the state of charge.  If it's charged, then just set the float
// voltage.  Current loop won't be doing anything now so just leave it be.  Remember, inverter boost supply
// current is measured in a separate loop so battery current limit has no effect on the inverter.

	if (k_timer_status_get(&my_timer) > 0 ) {	//only run this loop after batt stats are updated
		s->batt_state_of_chg = batt_info.state_of_chg;
		if (s->batt_state_of_chg >= BATT_MAX_SOC) {
			s->dac_vc_volt = real_to_raw(VC_SUPPLY_CTRL_V, (BATT_FLOAT_VOLTAGE - VC_SUPPLY_CTRL_V_OFFSET));
			LOG_DBG("VC voltage control raw value: %d", s->dac_vc_volt);
			int ret = dac_write_value(dac_dev[DAC_VC_VOLT], DAC_CHANNEL_ID, s->dac_vc_volt);
			if (ret != 0) {
				LOG_ERR("DAC write failed in state on_chg_run.  Can't set VC voltage.");
			}
			SET_STATE(ON_IDLE);
		}
	}
	return SMF_EVENT_HANDLED;
}

static void s_idle_entry(void *o)
{
	STATE_INIT(o);
	// not really much to do here...
	LOG_DBG("Entering IDLE state...");
}

enum smf_state_result s_idle_run(void *o)
{
	STATE_OBJ(o);

	if (!gpio_get(GPIO_AC_INPUT_DETECT) || !gpio_get(GPIO_ENABLE_PB)) {
		SET_STATE(OFF);
		return SMF_EVENT_HANDLED;
	}

	if (on_off_key_pressed) {  	//
		on_off_key_pressed = false;
		SET_STATE(ON_FULL_CHG);
	}
	return SMF_EVENT_HANDLED;
// Monitor things here, What?  temp? Probably batt SOC in case it runs down a bit.
}

/** 
 * On state entry, set battery current to minimum and voltage to max charge value.  That helps bootstrap
 * the VC to 179VDC forward boost converter for the inverter circuit.  Not sure how long it takes the
 * inverter circuit to ramp up.  May have to blank the GPIO_AC_OUT_DETECT before using it.  There should
 * be enough current capacity in VC to run both the inverter at full power and battery at minimum charge
 * level.
 */

static void s_on_full_entry(void *o)
{
	STATE_INIT(o);
	int ret;
	uint8_t tx_buf[2];
	uint8_t rx_buf[2];

	tx_buf[0] = BATT_SOC;
	i2c_write_read(i2c_dev, BATT_SMBUS_ADDR, tx_buf, 1, rx_buf, 2);
	s->batt_state_of_chg = (uint16_t)(rx_buf[0] + 256*rx_buf[1]);  // if batt is offline returns 0, so result is the same as low SOC

	tx_buf[0] = DESIRED_CHG_CURRENT;
	i2c_write_read(i2c_dev, BATT_SMBUS_ADDR, tx_buf, 1, rx_buf, 2);
	s->desired_chg_current = (uint16_t)(rx_buf[0] + 256*rx_buf[1])/1000;  // if batt is offline returns 0, disabling charging

	s->dac_vc_volt = real_to_raw(VC_SUPPLY_CTRL_V, (BATT_FLOAT_VOLTAGE - VC_SUPPLY_CTRL_V_OFFSET));
	ret = dac_write_value(dac_dev[DAC_VC_VOLT], DAC_CHANNEL_ID, s->dac_vc_volt);
	if (ret != 0) {
		LOG_ERR("dac write failed in state on_full_entry.  Can't set current limit");
	}
// It's safe to set the float charge voltage without messing with the current limiting loop at this point, we just need to get the
// inverter up and running then we'll worry about how much power we have to charge the battery in the run function.

	s->state_entry_time_ms = k_uptime_get();  //seed inverter startup timeout
	gpio_set(GPIO_INVERTER_ENABLE, 1); // Inverter on
	gpio_set(GPIO_AC_OUT_LED, 1);
	LOG_INF("Entering the ON state and Charging...");
}

/**
 * So now we are set to run full tilt.  On entry we dropped VC voltage to the float charge level.
 * The battery, if the SOC is _really_ low, will begin sucking a little charging current but there's
 * no need to worry about that until we figure out what the load on the inverter is. Setting VC
 * to the float value should allow the inverter to start OK
 * even with the maximum overload attached.  The battery will help the VC supply in any case to deal
 * with a momentary overload condition.  After we start we monitor the inverter current needed, and if
 * we are running within normal limits we can increase the battery charging current limit.  Now we have
 * to be careful here, since diverting more VC current to the battery can affect VC voltage regulation, 
 * and decreasing VC voltage will result in an INCREASE of inverter input current.
 * 1. If batt SOC > 98% on entry then just leave things as they are, VC set to float and batt charge
 *    current at minimum.  When batt charging current in float mode is less than the current limit set
 *    then the constant current control loop is effectively out of the circuit and the constant voltage
 *    loop for VC takes over.
 * 2. If batt SOC < 98% we still have some charging work to do.  Starting with float voltage and minimum
 *    current should be safe, first then set VC voltage to max charge voltage.  Voltage increase will stop
 *    when constant current loop kicks in.  Inverter input current should drop slightly on voltage increase
 *    as well.  But we need to be sensitive to the needs of the inverter and let its need for current
 *    override battery maximum charging current.
 */

enum smf_state_result s_on_full_run(void *o)
{
	STATE_OBJ(o);
	uint16_t raw_inv_current;
	double real_inv_current;
	double reserve_current;
	uint16_t buf;
	struct adc_sequence sequence = {
                .buffer = &buf,
                /* buffer size in bytes, not number of samples */
                .buffer_size = sizeof(buf),
        };
	int32_t err;

//NOTE: Because the A/D spans are different for _every_ analog measurement thanks to noddy analog design,
//      we have to translate everything in order to stay sane.

// again, safety first:
	if (!gpio_get(GPIO_ENABLE_PB)) {
		SET_STATE(OFF);
		return SMF_EVENT_HANDLED;
	}

	if (!gpio_get(GPIO_AC_INPUT_DETECT)) {
		SET_STATE(ON_BATTERY);
		return SMF_EVENT_HANDLED;
	}

	if (on_off_key_pressed) {  	
		on_off_key_pressed = false;
		SET_STATE(OFF);
		return SMF_EVENT_HANDLED;
	}

	if (k_uptime_get() - s->state_entry_time_ms < INVERTER_START_WAIT_TIME) {
		return SMF_EVENT_HANDLED;
	} // just skip everything this time until the inverter gets its act together
/**
 * OK, this isn't exactly what we call a PID loop, the rate of change is only 10A per second,
 * will that be fast enough?  Easy enough to change the step sizes.  Hopefully when the system
 * reaches steady state all we'll do is simply monitor VC current.  Once the battery reaches
 * full charge we switch to float mode anyway, after that there's really nothing else to do here
 * except monitor VC current for overloads.
 */

	if (k_timer_status_get(&my_timer) > 0 ) {	//only run this loop after batt stats are updated
		s->batt_state_of_chg = batt_info.state_of_chg; // keep updated each time through even though it only changes every 10 secs
		if (s->batt_state_of_chg >= BATT_MAX_SOC) {
			SET_STATE(ON_FULL_FLOAT); // Charging not needed.
			return SMF_EVENT_HANDLED;
		}
	}		

	if (s->batt_state_of_chg < BATT_MAX_SOC) {  // then we have to run the charging loop
		// voltage to charge level, we came in with it set to float
		s->dac_vc_volt = real_to_raw(VC_SUPPLY_CTRL_V, (BATT_MAX_CHG_VOLTAGE - VC_SUPPLY_CTRL_V_OFFSET));
		LOG_DBG("VC voltage control raw value: %d", s->dac_vc_volt);
		int ret = dac_write_value(dac_dev[DAC_VC_VOLT], DAC_CHANNEL_ID, s->dac_vc_volt);
		if (ret != 0) {
			LOG_ERR("DAC write failed in state on_full_run.  Can't set VC voltage.");
		}

		(void)adc_sequence_init_dt(&adc_channels[ADC_VC_CURRENT],&sequence);
		err = adc_read_dt(&adc_channels[ADC_VC_CURRENT],&sequence);
		if (err < 0) {
			LOG_ERR("Could not read ADC in on_full_run state.");
		}
		raw_inv_current = (uint16_t)buf;  //raw value from A/D for VC, must translate it
		LOG_DBG("raw inv cur: %d", raw_inv_current);
		real_inv_current = raw_to_real(VC_INVERTER_BOOST_CURRENT_MEAS, raw_inv_current); // A/D returns fractions of Amps 
		LOG_DBG("real inv cur: %f", real_inv_current);

		reserve_current = MAX_TOTAL_VC_CURRENT - real_inv_current;

		if (reserve_current > s->desired_chg_current) { //plenty of oomph to charge the batt
			s->dac_batt_current = real_to_raw(VC_SUPPLY_CURR_LIMIT, s->desired_chg_current);
			LOG_DBG("Batt chg current raw val: %d", s->dac_batt_current);
			int ret = dac_write_value(dac_dev[DAC_CHG_CURRENT], DAC_CHANNEL_ID, s->dac_batt_current);
			if (ret != 0) {
				LOG_ERR("dac write failed in state on_full_run.  Can't set current limit");		
			}
			gpio_set(GPIO_BUZZER, 0);  //Clear it		
		}
		else {
			if (reserve_current < s->desired_chg_current) {
				s->dac_batt_current = real_to_raw(VC_SUPPLY_CURR_LIMIT, (MAX_TOTAL_VC_CURRENT - real_inv_current));
				int ret = dac_write_value(dac_dev[DAC_CHG_CURRENT], DAC_CHANNEL_ID, s->dac_batt_current);
				if (ret != 0) {
					LOG_ERR("dac write failed in state on_full_run.  Can't set current limit");		
				}		
//				gpio_set(GPIO_BUZZER, 1);  //Annoy users for overloading the supply
			}
		}
	}
	else {
		s->dac_vc_volt = real_to_raw(VC_SUPPLY_CTRL_V, (BATT_FLOAT_VOLTAGE - VC_SUPPLY_CTRL_V_OFFSET));
		LOG_DBG("VC voltage control raw value: %d", s->dac_vc_volt);
		int ret = dac_write_value(dac_dev[DAC_VC_VOLT], DAC_CHANNEL_ID, s->dac_vc_volt);
		if (ret != 0) {
			LOG_ERR("DAC write failed in state on_full_run.  Can't set VC voltage.");
		}

		s->dac_batt_current = real_to_raw(VC_SUPPLY_CURR_LIMIT, s->desired_chg_current);
		LOG_DBG("VC voltage control raw value: %d", s->dac_batt_current);
		ret = dac_write_value(dac_dev[DAC_CHG_CURRENT], DAC_CHANNEL_ID, s->dac_batt_current);
		if (ret != 0) {
			LOG_ERR("dac write failed in state on_full_run.  Can't set current limit");		
		}
		SET_STATE(ON_FULL_FLOAT);
	}

	return SMF_EVENT_HANDLED;
}

static void s_on_float_entry(void *o)
{
	STATE_INIT(o);
	int ret;

	ret = dac_write_value(dac_dev[DAC_CHG_CURRENT], DAC_CHANNEL_ID, real_to_raw(VC_SUPPLY_CURR_LIMIT, BATT_MIN_CHG_CURRENT));
	if (ret != 0) {
		LOG_ERR("dac write failed in state on_float_entry.  Can't set current limit");
	}
	int vc_setpoint = real_to_raw(VC_SUPPLY_CTRL_V, (BATT_FLOAT_VOLTAGE - VC_SUPPLY_CTRL_V_OFFSET));
	ret = dac_write_value(dac_dev[DAC_VC_VOLT], DAC_CHANNEL_ID, vc_setpoint);
	if (ret != 0) {
		LOG_ERR("DAC write failed in state on_float_entry.  Can't set VC voltage.");
	}
	gpio_set(GPIO_AC_OUT_LED, 1);
	LOG_INF("Entering RUN state but float charging...");		
}

/**
 * There's not much to do in the float state except monitor things and watch for overloads on the VC supply
 * feeding the inverter.  The current is actually for the forward boost converter that generates the 179VDC 
 * for the inverter section.
 */

enum smf_state_result s_on_float_run(void *o)
{
	STATE_OBJ(o);
	uint16_t vc_current;
	float vc_result;
	int32_t err;
	uint16_t buf;
	struct adc_sequence sequence = {
                .buffer = &buf,
                /* buffer size in bytes, not number of samples */
                .buffer_size = sizeof(buf),
        };

	if (!gpio_get(GPIO_ENABLE_PB)) {
		SET_STATE(OFF);
		return SMF_EVENT_HANDLED;
	}

	if (!gpio_get(GPIO_AC_INPUT_DETECT)) {
		SET_STATE(ON_BATTERY);
		return SMF_EVENT_HANDLED;
	}

	if (on_off_key_pressed) {  	
		on_off_key_pressed = false;
		SET_STATE(OFF);
		return SMF_EVENT_HANDLED;
	}

	if (k_timer_status_get(&my_timer) > 0 ) {	//only run this loop after batt stats are updated
		s->batt_state_of_chg = batt_info.state_of_chg;
		if (s->batt_state_of_chg < BATT_MAX_SOC) { //in theory this code should never execute, but one never knows...
			SET_STATE(ON_FULL_CHG);
			return SMF_EVENT_HANDLED;
		}
	}
	
	(void)adc_sequence_init_dt(&adc_channels[ADC_VC_CURRENT],&sequence);
	err = adc_read_dt(&adc_channels[ADC_VC_CURRENT],&sequence);
	if (err < 0) {
		LOG_ERR("Could not read ADC in on_float state.");
	}
	vc_current = (uint16_t)buf;  //raw value from A/D for VC, must translate it
	vc_result = raw_to_real(VC_INVERTER_BOOST_CURRENT_MEAS, vc_current);
	if (vc_result > MAX_TOTAL_VC_CURRENT) {
		gpio_set(GPIO_BUZZER, 1);  //annoy the user
	}
	else {
		gpio_set(GPIO_BUZZER, 0);  // basically just scream until the overload disappears
	}
	return SMF_EVENT_HANDLED;
}

static void s_on_battery_entry(void *o)
{
	STATE_INIT(o);
	gpio_set(GPIO_VC_TO_VINA_EN, 1); // Connect batt pwr to VINA bus to power analog control and PMIC devices for DC-DC boost ckt
	gpio_set(GPIO_INVERTER_ENABLE, 1); // Inverter on
	ac_mode = false;
// If we came here from the OFF state timer will need to be started for LED service
	if (k_timer_status_get(&my_timer) == 0 && k_timer_remaining_get(&my_timer) == 0) {
		k_timer_start(&my_timer, K_SECONDS(10), K_SECONDS(10));
	}
	gpio_set(GPIO_AC_OUT_LED, 1);
	LOG_INF("Entering the ON BATTERY state...");

// inverter on, running on battery
}

/**
 * During the on_battery state there's not much to do except monitor the front panel switch,
 * AC input's possible return, and the battery discharge current and state of charge.
 */

enum smf_state_result s_on_battery_run(void *o)
{
	STATE_OBJ(o);
	uint8_t rx_buf[2];
	uint8_t tx_buf[2];
	uint16_t real_batt_current;

// lots to do here when on batt
	if (!gpio_get(GPIO_ENABLE_PB)) {
		SET_STATE(OFF);
		return SMF_EVENT_HANDLED;
	}

	if (on_off_key_pressed) {  	
		on_off_key_pressed = false;
		SET_STATE(OFF);
		return SMF_EVENT_HANDLED;
	}
	
	if (gpio_get(GPIO_AC_INPUT_DETECT)) {
		SET_STATE(ON_FULL_CHG);
		return SMF_EVENT_HANDLED;
	}
	
	if (batt_info.state_of_chg <= BATT_SOC_CRITICAL) {
		SET_STATE(OFF);  //sorry, that's all folks
		return SMF_EVENT_HANDLED;
	}
// LEDs should have been flashing red long before this time
// And rather than trust the noddy analog design of the supply, let's rely on the
// battery itself to tell us its actual current drain.

	tx_buf[0] = BATT_CURRENT;
	i2c_write_read(i2c_dev, BATT_SMBUS_ADDR, tx_buf, 1, rx_buf, 2);
	real_batt_current = abs(((int16_t)(rx_buf[0] + 256*rx_buf[1])))/1000; //remember, batt current is signed 16bit int.
	if (real_batt_current >= BATT_EMERGENCY_SHUTDOWN_CURRENT) { //BAD!!!
		SET_STATE(OFF); // Immediate shutdown, sorry
		return SMF_EVENT_HANDLED;
	}
	
	if (real_batt_current >= MAX_BATT_DISCH_CURRENT) {
		gpio_set(GPIO_BUZZER, 1); // Annoy the user
	}
	else {
		gpio_set(GPIO_BUZZER, 0);
	}
	return SMF_EVENT_HANDLED;
}

static const struct smf_state ups_states[] = {
	[OFF] = SMF_CREATE_STATE(s_off_entry, s_off_run, NULL, NULL, NULL),
	[ON_CHG] = SMF_CREATE_STATE(s_on_chg_entry, s_on_chg_run, NULL, NULL, NULL),
	[ON_IDLE] = SMF_CREATE_STATE(s_idle_entry, s_idle_run, NULL, NULL, NULL),
	[ON_FULL_CHG] = SMF_CREATE_STATE(s_on_full_entry, s_on_full_run, NULL, NULL, NULL),
	[ON_FULL_FLOAT] = SMF_CREATE_STATE(s_on_float_entry, s_on_float_run, NULL, NULL, NULL),
	[ON_BATTERY] = SMF_CREATE_STATE(s_on_battery_entry, s_on_battery_run, NULL, NULL, NULL),
};

/*-------------------------------------------------------------------------------------------- */

int main(void)
{
	int ret;
	int msecs_count;
	int on_off_pressed_start = 0;
	bool on_off_key_down = false;
	bool mute_key_down = false;
	int mute_pressed_start = 0;
	int32_t err;

	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("ERROR: failed to get handle to I2C3 device.\n");
		return -ENODEV;
	};

	if (!device_is_ready(dac_dev[0]) || !device_is_ready(dac_dev[1])) {
		LOG_ERR("One or both DAC devices is not ready.\n");
		return 0;
	}
	
	for (size_t j = 0U; j < ARRAY_SIZE(dac_dev); j++) {
		ret = dac_channel_setup(dac_dev[j], &dac_ch_cfg);
		if (ret != 0) {
			LOG_ERR("setting up of DAC channel %d failed with code %d\n", j+1, ret);
			return 0;
		}
	}
 /* Configure channels individually prior to sampling. */
        for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
                if (!adc_is_ready_dt(&adc_channels[i])) {
                        LOG_ERR("ADC controller device %s not ready\n", adc_channels[i].dev->name);
                        return -EINVAL;
                }

                err = adc_channel_setup_dt(&adc_channels[i]);
                if (err < 0) {
                        LOG_ERR("Could not setup channel #%d (%d)\n", i, err);
                        return -EINVAL;
                }
        }
	
	gpio_init();
//NOTE: when we are in standby state, powered off, with no AC input, we don't start the battery service,
//      and we shut it down when entering low power standby (on/off OFF, no AC input)

//	Do this inside one of the states instead
//	printk("Starting batt svc timer service\n");
//	k_timer_start(&my_timer, K_SECONDS(10), K_SECONDS(10));

	msecs_count = 0;
	s_obj.debug = false;
	s_obj.next = true;
	smf_set_initial(SMF_CTX(&s_obj), &ups_states[OFF]);

// In main we run the state machine state monitor every 10mSec.  We also update the front panel
// LEDs once every half second in order to implement flashing.  LEDs themselves update only once every
// BATT_CHECK_MS miliseconds as part of the battery monitor work.  Two input push buttons are handled by
// the main loop outside of the state machines.

	while (true) {
//		LOG_DBG("About to run state: %s", smf_get_current_executing_state(SMF_CTX(&s_obj)));
		ret = smf_run_state(SMF_CTX(&s_obj));
		if (ret) {
			LOG_ERR("State machine exited -- system DIED!");
			// what do we do now???
		}

		k_sleep(K_MSEC(10));
// Don't need to service LEDs if the current state is OFF
		if (smf_get_current_executing_state(SMF_CTX(&s_obj)) != &ups_states[OFF]) {
			msecs_count++;
			if (msecs_count == 50) {
				for (size_t j = 0U; j < ARRAY_SIZE(leds_array); j++) {
					if (leds_array[j].state) {
						gpio_set(leds_array[j].led_gpio,1);
					}
					else {
						 gpio_set(leds_array[j].led_gpio,0);
					}
				}	
			}	

			if (msecs_count >=99) {
				for (size_t j = 0U; j < ARRAY_SIZE(leds_array); j++) {
					if (leds_array[j].flashing) {
						gpio_set(leds_array[j].led_gpio,0);
					}
				}
				
				msecs_count = 0;
			}
		}	
//But we do have to service the front panel buttons even when off:
		if (!on_off_key_pressed && !on_off_key_down) {
			if (gpio_get(GPIO_ON_OFF_PB)) {		//the classic Telxon key debounce routine
			 	on_off_pressed_start++;
			} 
			else {
				if (on_off_pressed_start > 0) {
					on_off_pressed_start--;
				}
			}
		}		

		if (on_off_pressed_start > 5 && !on_off_key_down) {  	//10mS state tick timing
			on_off_key_pressed = true;
			on_off_key_down = true;
			LOG_DBG("on/off key pressed");
		}
		
		if (on_off_key_down) {
			if (!gpio_get(GPIO_ON_OFF_PB)) {
				on_off_pressed_start--;
				if (on_off_pressed_start <= 0) {
					on_off_key_down = false;
					LOG_DBG("on/off key released");
				}
			}
			else {
				on_off_pressed_start++;
				if (on_off_pressed_start > 5) {
					on_off_pressed_start = 5; //otherwise it will wind up if someone holds the button
				}
			}
		}

		if (!mute_key_pressed && !mute_key_down) {
			if (gpio_get(GPIO_MUTE_PB)) {		//the classic Telxon key debounce routine
				mute_pressed_start++;
			} 
			else {
				if (mute_pressed_start > 0) {
					mute_pressed_start--;
				}
			}
		}		

		if (mute_pressed_start > 5 && !mute_key_down) {  	//10mS state tick timing
			mute_key_pressed = true;
			mute_key_down = true;
			LOG_DBG("mute button pressed");
		}

		if (mute_key_down) {
			if (!gpio_get(GPIO_MUTE_PB)) {
				mute_pressed_start--;
				if (mute_pressed_start <= 0) {
					mute_key_down = false;
					LOG_DBG("mute button released");
				}
			}
			else {
				mute_pressed_start++;
				if (mute_pressed_start > 5) {
					mute_pressed_start = 5; //otherwise it will wind up if someone holds the button
				}
			}
		}
	}
	// the shell is currently doing all the work...
	return 0;
}
