/* Battery communications support */

// Following commands are all read only, and return two bytes.
// Batt supposedly has a write command to put the battery in "ship" mode
// but we need to figure out what that is from the battery manufacturer.
// See the smart batt spec for a complete list of commands, these are only
// a few.  May need to implement others depending on customer requirements.

#define BATT_SMBUS_ADDR 0x0b
// A word to the wise: Inventus batt needs 100KHz I2C speed to work.
// Faster than that the bugger won't respond.

#define BATT_VOLTAGE 0x9 // mV
#define BATT_CURRENT 0x0a // mA
#define BATT_AVG_CURRENT 0x0b // rolling 1 minute average
#define BATT_SOC_MAX_ERROR 0x0c // percent
#define BATT_SOC  0x0d  // state of charge, percent
#define BATT_ABS_SOC 0x0e // based on design capacity, percent
#define BATT_CAP_REMAIN 0x0f  // maAr or 10mWh
#define BATT_AVG_TO_EMPTY 0x12 // in minutes
#define BATT_TEMP 0x08  // 0.1 Kelvin. Divide by 10 then subtract 273.15 to get C.  F is uglier...
#define BATT_STAT 0x16  //bit mapped
#define DESIRED_CHG_CURRENT 0x14 // battery's desired charging current
#define DESIRED_CHG_VOLTAGE 0x15 // battery's desired charging voltage.

//Status bits:
#define ALM_OVER_CHGED 0x8000
#define ALM_TERM_CHARGE 0x4000
#define ALM_OVERTEMP 0x1000
#define ALM_TERM_DISCHARGE 0x0800
#define ALM_REMAIN_CAPACITY 0x0200
#define ALM_REMAIN_TIME 0x0100

#define STAT_INITIALIZED 0x0080
#define STAT_DISCHARGING 0x0040
#define STAT_FULL_CHARGED 0x0020
#define STAT_FULL_DISCHARGED 0x0010

// the 4 lsb's of the status register define comms error codes:

#define FUNC_OK 0x0000
#define FUNC_BUSY 0x0001
#define FUNC_UNDEFINED 0x0002
#define FUNC_UNSUPPORTED 0x0003
#define FUNC_ACCESS_DENIED 0x0004
#define FUNC_OFLOW_UFLOW 0x0005
#define FUNC_BAD_DATA_SIZE 0x0006
#define FUNC_UNK_ERROR 0x0007
//NOTE: error conditions are indicated by the battery's refusal to acknowledge a read or write command.

static struct batt_health {
	uint16_t temperature; //degrees kelvin
	uint16_t voltage;
	int16_t current; //positive is charging, negative is discharging, mA
	int16_t avg_current; // 1 minute rolling average, charging or discharging
	uint16_t state_of_chg; // predicted remaining capacity in percent of "full charge capacity"
	uint16_t abs_state_of_chg; // remaining capacity as percentage of design capacity.  Could be >100%
	uint16_t remaining_capacity; // mAhr
	uint16_t avg_time_to_empty; // minutes
	uint16_t batt_stats; // flag register
} batt_info;

/**
 * This might seem a bit daft since the battery is using SMBus verses an I2C interface,
 * but since there are no other I2C devices in the system we can get away with this.
 * Note that SMBus interface is limited to 100KHz maximum speed, and that pullup
 * resistors must tie to 3.3V ONLY.  Since the battery has no 'alert' signal, for all
 * intents and purposes it's just an I2C device with restrictions.
 */

void load_batt_info( struct batt_health *stats, const struct device *i2c_dev) {
	uint8_t rx_buf[2];
	uint8_t tx_buf[2];

	// These all return raw values, user context determines converstion needed
	tx_buf[0] = BATT_TEMP;
	i2c_write_read(i2c_dev, BATT_SMBUS_ADDR, tx_buf, 1, rx_buf, 2);
	stats->temperature = (uint16_t)(rx_buf[0] + rx_buf[1] * 256);

	tx_buf[0] = BATT_VOLTAGE;
	i2c_write_read(i2c_dev, BATT_SMBUS_ADDR, tx_buf, 1, rx_buf, 2);
	stats->voltage = (int32_t)(uint16_t)(rx_buf[0] + (rx_buf[1] * 256));
	
	tx_buf[0] = BATT_CURRENT;  //signed:w

	i2c_write_read(i2c_dev, BATT_SMBUS_ADDR, tx_buf, 1, rx_buf, 2);
	stats->current =(int16_t)(rx_buf[0] + 256*rx_buf[1]);

	tx_buf[0] = BATT_AVG_CURRENT; //signed
	i2c_write_read(i2c_dev, BATT_SMBUS_ADDR, tx_buf, 1, rx_buf, 2);
	stats->avg_current = (int16_t)(rx_buf[0] + 256*rx_buf[1]);
	
	tx_buf[0] = BATT_SOC;
	i2c_write_read(i2c_dev, BATT_SMBUS_ADDR, tx_buf, 1, rx_buf, 2);
	stats->state_of_chg = (uint16_t)(rx_buf[0] + 256*rx_buf[1]);
	
	tx_buf[0] = BATT_ABS_SOC;
	i2c_write_read(i2c_dev, BATT_SMBUS_ADDR, tx_buf, 1, rx_buf, 2);
	stats->abs_state_of_chg = (uint16_t)(rx_buf[0] + 256*rx_buf[1]);
	
	tx_buf[0] = BATT_CAP_REMAIN;
	i2c_write_read(i2c_dev, BATT_SMBUS_ADDR, tx_buf, 1, rx_buf, 2);
	stats->remaining_capacity = (uint16_t)(rx_buf[0] + 256*rx_buf[1]);

	tx_buf[0] = BATT_AVG_TO_EMPTY;
	i2c_write_read(i2c_dev, BATT_SMBUS_ADDR, tx_buf, 1, rx_buf, 2);
	stats->avg_time_to_empty = (uint16_t)(rx_buf[0] + 256*rx_buf[1]);

	tx_buf[0] = BATT_STAT;
	i2c_write_read(i2c_dev, BATT_SMBUS_ADDR, tx_buf, 1, rx_buf, 2);
	stats->batt_stats = (uint16_t)(rx_buf[0] + 256*rx_buf[1]);
};
