#include <cayenne_lpp.h>
#include <WaspSensorCities_PRO.h>
#include <WaspLoRaWAN.h>        

#define DEVICE_EUI                          "009BEC9EF331B290"
#define DEVICE_ADDR                         "260138B8"
#define APP_EUI                             "70B3D57ED003EC45"
#define NWK_SESSION_KEY                     "0858F977EA4136BB366051F5351A616D"
#define APP_SESSION_KEY                     "54F787A07E844D89E6B17289D4CD5804"

#define DO_NOTHING                          ((uint8_t)0)
#define GET_NOISE                           ((uint8_t)1)
#define SEND_DATA                           ((uint8_t)2)

#define STATUS_OK                           ((uint8_t)0)
#define STATUS_ERROR                        ((uint8_t)-1)

#define MIN_MEAS_NO                         ((uint8_t)5)
#define MAX_BUFFER_LENGTH                   ((uint8_t)200)
#define DEFAULT_SEND_DATA_MAX_COUNTER       ((uint16_t)120)
#define DEFAULT_GET_NOISE_MAX_COUNTER       ((uint16_t)4)
#define MAX_SEND_RETRIES                    ((uint8_t)3)   

#define LORA_SOCKET                         SOCKET0
#define LORA_DATA_RATE                      ((uint8_t)5)
#define LORA_PORT                           ((uint8_t)3)
#define LORA_RX1_DELAY                      ((uint16_t)1000)
#define LORA_RX2_FREQUENCY                  ((uint32_t)869525000)
#define LORA_RX2_DATA_RATE                  ((uint8_t)3)

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

char* app_time = "21:2:17:4:14:00:00";
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
                USB.print(F("Noise read ")); USB.print(int(noise_buffer_length));
                USB.print(F(" with dBA value: ")); USB.println(noise_buffer[noise_buffer_length-1]);
            }
            else
            {
                USB.println(F("Error: No response from the audio sensor!"));
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
    intFlag &= ~(RTC_INT | WTD_INT);
}

static uint8_t compute_noise_stats(void)
{    
    peak_noise = 0;
    average_noise = 0;

    if(MIN_MEAS_NO > noise_buffer_length)
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
            break;
        }
        if(MAX_SEND_RETRIES <= send_retries++)
        {
            USB.println(F("REBOOT!!!"));
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

    USB.println(F("Sending data to TTN..."));
    
    do{
        status |= LoRaWAN.ON(LORA_SOCKET);
        status |= LoRaWAN.setRX1Delay(LORA_RX1_DELAY);
        //status |= LoRaWAN.setAR("on");
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
            USB.println(F("REBOOT!!!"));
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

static inline void get_update(void) //TODO - This is not functional yet
{
    if(LoRaWAN._dataReceived == true)
    {
        USB.print(F("   There's data on port number "));
        USB.print(LoRaWAN._port,DEC);
        USB.print(F(".\r\n   Data: "));
        USB.println(LoRaWAN._data);

        uint8_t local_app_max_counter = 200;
        char* local_app_time = "19:2:17:4:14:00:00";
        char* received_data = LoRaWAN._data;
        uint8_t minutes_flag = Utils.str2hex(received_data+0);
        uint8_t time_flag = Utils.str2hex(received_data+1);
        if(minutes_flag)
        {
            local_app_max_counter = (Utils.str2hex(received_data+2))*10 + Utils.str2hex(received_data+3);
            if(time_flag)
            {
                for(int i = 0; i < strlen(local_app_time);i++)
                {
                    *(local_app_time+i) = *(received_data+4+i);
                }
            }
        }
        else
        {
            if(time_flag)
            {
                for(int i = 0; i < strlen(local_app_time);i++)
                {
                    *(local_app_time+i) = *(received_data+2+i);
                }
            }
        }
        USB.println(local_app_max_counter);
        for(int i = 0; i < strlen(local_app_time) ; i++)
        {
            USB.print(*(local_app_time+i));
        }
        USB.println(F("---"));
    }    
}

static inline void reset_noise_buffer(void)
{
    noise_buffer_length = 0;
}








