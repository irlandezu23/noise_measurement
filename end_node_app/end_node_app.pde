#include <cayenne_lpp.h>
#include <WaspSensorCities_PRO.h>
#include <WaspLoRaWAN.h>        

#define DEVICE_EUI                          "BE7A00000000303F"
#define DEVICE_ADDR                         "31EC34E0"
#define APP_EUI                             "BE7A000000002547"
#define NWK_SESSION_KEY                     "0A36E5ED7319D09BB6D47A1A078C10EE"
#define APP_SESSION_KEY                     "7152F3C5123B1EF85F86117C5082C523"

#define DO_NOTHING                          ((uint8_t)0)
#define GET_NOISE                           ((uint8_t)1)
#define SEND_DATA                           ((uint8_t)2)

#define PING_CMD                            ((uint8_t)0)
#define GET_NOISE_COUNTER_CMD               ((uint8_t)1)
#define SEND_DATA_COUNTER_CMD               ((uint8_t)2)
#define SET_TIME_CMD                        ((uint8_t)3)
#define REBOOT_CMD                          ((uint8_t)-1)

#define PING_CMD_LENGTH                     ((uint8_t)2)
#define GET_NOISE_COUNTER_CMD_LENGTH        ((uint8_t)4)
#define SEND_DATA_COUNTER_CMD_LENGTH        ((uint8_t)4)
#define SET_TIME_CMD_LENGTH                 ((uint8_t)42)
#define REBOOT_CMD_LENGTH                   ((uint8_t)2)

#define STATUS_OK                           ((uint8_t)0)
#define STATUS_ERROR                        ((uint8_t)-1)

#define UPDATE_OK                           "0"
#define UPDATE_BAD_CMD                      "1"
#define UPDATE_BAD_PARAMETER                "2"

#define MIN_MEASUREMENTS_NUMBER             ((uint8_t)5)
#define MAX_BUFFER_LENGTH                   ((uint16_t)256)
#define DEFAULT_SEND_DATA_MAX_COUNTER       ((uint16_t)120)
#define DEFAULT_GET_NOISE_MAX_COUNTER       ((uint16_t)4)
#define MAX_SEND_RETRIES                    ((uint8_t)3)
#define MAX_SENSOR_ERROR                    ((uint8_t)5)

#define LORA_SOCKET                         SOCKET0
#define LORA_DATA_RATE                      ((uint8_t)5)
#define LORA_PORT                           ((uint8_t)3)
#define LORA_RX1_DELAY                      ((uint16_t)1000)
#define LORA_RX2_FREQUENCY                  ((uint32_t)869525000)
#define LORA_RX2_DATA_RATE                  ((uint8_t)0)

#define LORA_RETRY_DELAY                    ((uint32_t)1000)
#define LORA_UPDATE_DELAY                   ((uint32_t)3000)

cayenne_lpp_t lpp = { 0 };

volatile uint8_t app_state;
volatile uint8_t noise_buffer_length;
volatile float noise_buffer[MAX_BUFFER_LENGTH];
volatile uint16_t send_data_max_counter;
volatile uint16_t get_noise_max_counter;
volatile uint16_t send_data_counter;
volatile uint16_t get_noise_counter;
volatile uint8_t sensor_error_counter;

char* app_time = "21:02:17:04:14:00:00";
char* app_alarm_config = "15:12:00:00";

static float peak_noise;
static float average_noise;

static void check_state(void);
static uint8_t compute_noise_stats(void);
static void lora_setup(void);
static inline void sleep_mode_on(void);
static inline void send_data(void);
static inline void build_frame(void);
static inline void get_update(void);
static inline uint8_t check_measurements_number(uint16_t get_counter, uint16_t send_counter);
static inline void reset_noise_buffer(void);


void setup()
{
    USB.ON();
    USB.println(F("Application started..."));

    RTC.ON();
    RTC.setTime(app_time);
    RTC.setAlarm1(app_alarm_config, RTC_ABSOLUTE, RTC_ALM1_MODE6);
        
    noise.configure();
    
    send_data_max_counter = DEFAULT_SEND_DATA_MAX_COUNTER;
    get_noise_max_counter = DEFAULT_GET_NOISE_MAX_COUNTER;
    lora_setup();
}


void loop()
{
    check_state();
    switch(app_state)
    {
        case GET_NOISE:
        {
            uint8_t status = noise.getSPLA(FAST_MODE);
            if(status == STATUS_OK)
            {
                noise_buffer[noise_buffer_length++] = noise.SPLA;
                sensor_error_counter = 0;
                USB.print(F("Noise read ")); USB.print(int(noise_buffer_length));
                USB.print(F(" with dBA value: ")); USB.println(noise_buffer[noise_buffer_length-1]);
            }
            else
            {
                USB.println(F("Error: No response from the audio sensor!"));
                if(MAX_SENSOR_ERROR <= sensor_error_counter++)
                {
                    USB.println(F("Noise sensor disconnected ..."));
                    USB.println(F("Rebooting the system..."));
                    PWR.reboot();
                }
            }
            break;
        }
        case SEND_DATA:
        {
            if(STATUS_OK == compute_noise_stats())
            {
                send_data();
            }
            reset_noise_buffer();
            break;
        }
        case DO_NOTHING:
        {
            break;
        }
        default:
        {
            break;
        }
    }
    sleep_mode_on();
}

static void check_state(void)
{
    if( intFlag & RTC_INT )
    {   
        if(send_data_max_counter <= ++send_data_counter)
        {
            app_state = SEND_DATA;
            send_data_counter = 0;
        }
        else if(get_noise_max_counter <= ++get_noise_counter)
        {
            app_state = GET_NOISE;
            get_noise_counter = 0;
        }
        else
        {
            app_state = DO_NOTHING;
        }
        RTC.clearAlarmFlag();
    }
    else
    {
        app_state = DO_NOTHING;
    }
    intFlag &= ~(RTC_INT);
}

static uint8_t compute_noise_stats(void)
{    
    peak_noise = 0;
    average_noise = 0;

    if(MIN_MEASUREMENTS_NUMBER > noise_buffer_length)
    {
        return STATUS_ERROR;
    }
    
    for(uint8_t i = 0; i < noise_buffer_length; i++)
    {
        average_noise += noise_buffer[i];
        if(noise_buffer[i] > peak_noise)
        {
            peak_noise = noise_buffer[i];        
        }
    }
    average_noise = average_noise/noise_buffer_length;
    return STATUS_OK;    
}

static void lora_setup(void)
{
    uint8_t status = STATUS_OK;
    uint8_t send_retries = 0;
    do{
        status |= LoRaWAN.ON(LORA_SOCKET);
        status |= LoRaWAN.setDataRate(LORA_DATA_RATE);
        status |= LoRaWAN.setDeviceEUI(DEVICE_EUI);
        status |= LoRaWAN.setDeviceAddr(DEVICE_ADDR);
        status |= LoRaWAN.setAppEUI(APP_EUI);
        status |= LoRaWAN.setNwkSessionKey(NWK_SESSION_KEY);
        status |= LoRaWAN.setAppSessionKey(APP_SESSION_KEY);
        status |= LoRaWAN.setDownCounter(0);
        status |= LoRaWAN.setRX2Parameters(LORA_RX2_DATA_RATE, LORA_RX2_FREQUENCY);
        status |= LoRaWAN.saveConfig();
        status |= LoRaWAN.OFF(LORA_SOCKET);
        if(STATUS_OK == status)
        {
            USB.println(F("LoRa Module succesfully configured..."));
            break;
        }
        if(MAX_SEND_RETRIES <= send_retries++)
        {
            USB.println(F("LoRa Module not configured..."));
            USB.println(F("Rebooting the system..."));
            PWR.reboot();
        }
        delay(LORA_RETRY_DELAY);
    }while(1);
}

static inline void sleep_mode_on(void)
{
    PWR.sleep(ALL_OFF);
    USB.ON();
    noise.configure();
}

static inline void send_data(void)
{
    build_frame();    
    uint8_t send_retries = 0;
    uint8_t status = STATUS_OK;

    USB.println(F("Sending data to the LoRa server..."));
    
    do{
        status |= LoRaWAN.ON(LORA_SOCKET);
        status |= LoRaWAN.setRX1Delay(LORA_RX1_DELAY);
        status |= LoRaWAN.joinABP();
        status |= LoRaWAN.sendUnconfirmed( LORA_PORT, lpp.buffer, lpp.cursor);
        if(STATUS_OK == status)
        {
            get_update();
            status |= LoRaWAN.OFF(LORA_SOCKET);
            break;
        }        
        if(MAX_SEND_RETRIES <= send_retries++)
        {
            USB.println(F("Unable to send the LoRa frame..."));
            USB.println(F("Rebooting the system..."));
            PWR.reboot();
        }
        delay(LORA_RETRY_DELAY);    
    }while(1);
}

static inline void build_frame(void)
{
    cayenne_lpp_reset(&lpp);
    cayenne_lpp_add_analog_input(&lpp, 1, average_noise);
    cayenne_lpp_add_analog_input(&lpp, 2, peak_noise);  
}

static inline void get_update(void)
{
    if(LoRaWAN._dataReceived == true)
    {
        char* received_data = LoRaWAN._data;
        uint8_t cmd = Utils.str2hex(received_data+0);
        switch(cmd)
        {
            case PING_CMD:
            {
                if(strlen(received_data) == PING_CMD_LENGTH)
                {
                    USB.println(F("Ping received from the LoRa server..."));
                    LoRaWAN.sendUnconfirmed(LORA_PORT, UPDATE_OK);
                    break;
                }
                USB.println(F("Bad parameter received for ping..."));
                LoRaWAN.sendUnconfirmed(LORA_PORT, UPDATE_BAD_PARAMETER);
                break;
            }
            case SEND_DATA_COUNTER_CMD:
            {
                if(strlen(received_data) == SEND_DATA_COUNTER_CMD_LENGTH)
                {
                    uint16_t aux = (Utils.str2hex(received_data+2))*60;
                    if(STATUS_OK == check_measurements_number(get_noise_max_counter, aux))
                    {
                        send_data_max_counter = aux;
                        USB.print(F("Sending data period (in seconds) was updated to: "));
                        USB.println(send_data_max_counter);
                        LoRaWAN.sendUnconfirmed(LORA_PORT, UPDATE_OK);
                        break;
                    }
                }
                USB.println(F("Bad parameter received for sending data period update..."));
                LoRaWAN.sendUnconfirmed(LORA_PORT, UPDATE_BAD_PARAMETER);
                break;
            }
            case GET_NOISE_COUNTER_CMD:
            {
                if(strlen(received_data) == GET_NOISE_COUNTER_CMD_LENGTH)
                {
                    uint16_t aux = Utils.str2hex(received_data+2);
                    if(STATUS_OK == check_measurements_number(aux, send_data_max_counter))
                    {
                        get_noise_max_counter = aux;
                        USB.print(F("Getting noise period (in seconds) was updated to: "));
                        USB.println(get_noise_max_counter);
                        LoRaWAN.sendUnconfirmed(LORA_PORT, UPDATE_OK);
                        break;                        
                    }
                }
                USB.println(F("Bad parameter received for getting noise period update..."));
                LoRaWAN.sendUnconfirmed(LORA_PORT, UPDATE_BAD_PARAMETER);
                break;
            }
            case SET_TIME_CMD:
            {
                if(strlen(received_data) == SET_TIME_CMD_LENGTH)
                {
                    char aux[25];
                    uint8_t i;
                    for(i = 2; i < SET_TIME_CMD_LENGTH; i = i + 2)
                    {
                        aux[(i-2)>>1] = Utils.str2hex(received_data+i);
                        /* every two received bytes correspond to a character;
                           received parameter starts from index 2(cmd has index 0)
                           but aux starts from 0*/
                    }
                    aux[(i-2)>>1] = '\0';
                    if(STATUS_OK == RTC.setTime(aux))
                    {
                        app_time = aux;
                        USB.print(F("System time updated to: "));
                        USB.println(app_time);
                        LoRaWAN.sendUnconfirmed(LORA_PORT, UPDATE_OK);
                        break;
                    }
                }
                USB.println(F("Bad parameter received for system time update..."));
                LoRaWAN.sendUnconfirmed(LORA_PORT, UPDATE_BAD_PARAMETER);
                break;
            }
            case REBOOT_CMD:
            {
                if(strlen(received_data) == REBOOT_CMD_LENGTH)
                {
                    USB.println(F("System reboot command received..."));
                    USB.println(F("Rebooting the system..."));
                    LoRaWAN.sendUnconfirmed(LORA_PORT, UPDATE_OK);
                    PWR.reboot();
                    break;
                }
                USB.println(F("Bad parameter received for system reboot..."));
                LoRaWAN.sendUnconfirmed(LORA_PORT, UPDATE_BAD_PARAMETER);
                break;
            }
            default:
            {
                USB.println(F("Unknown command received..."));
                LoRaWAN.sendUnconfirmed(LORA_PORT, UPDATE_BAD_CMD);
                break;
            }
        }
    }
}

static inline uint8_t check_measurements_number(uint16_t get_counter, uint16_t send_counter)
{
    if(0 != get_counter)
    {
        uint16_t measurements_number = send_counter/get_counter;
        if(MIN_MEASUREMENTS_NUMBER <= measurements_number && MAX_BUFFER_LENGTH > measurements_number)
        {
            return STATUS_OK;
        }
    }
    return STATUS_ERROR;
}

static inline void reset_noise_buffer(void)
{
    noise_buffer_length = 0;
}








