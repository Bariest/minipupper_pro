#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "lwip/ip4_addr.h"
#include "driver/gpio.h"
#include "driver_board.h"   // SPI controller-driver-board backend (replaces SCServo)
#include "stanford_gait.h"  // Stanford Pupper trot gait port (forward walk)
#include "mqtt_client.h"

#define TAG "PUPPER"
#define PI 3.14159265358979f

/* ---- legacy "speed" -> current(mA) limit mapping (new driver board) ----
 * The driver board does POSITION control with a current/torque CAP; it has no
 * speed field. Feetech convention used by the gait code: goal_speed 0 == "max
 * speed". We translate every legacy speed value into a current limit:
 *   speed 0      -> CUR_MAX_MA   (full torque / snap)
 *   speed 1..ref -> CUR_MIN_MA .. CUR_MAX_MA  (gentler/slower = lower cap)
 * NOTE: this is a CAP, not a forced draw. A lightly loaded leg only sources the
 * current it needs to hold its position; the cap just limits peak/stall current.
 * Tune these three constants for your SCS0009 servos. */
#define CUR_MAX_MA   1200

static inline uint16_t speed_to_current_mA(uint16_t spd){ (void)spd; return CUR_MAX_MA; }

// ---- WiFi (station) + MQTT configuration ----
// Fill these in for your network / broker before flashing.
#define WIFI_SSID        "Mangdang"
#define WIFI_PASS        "mangdang"
#define MQTT_BROKER_URI  "mqtt://192.168.1.117:1883"
#define MQTT_CMD_TOPIC   "minipupper/cmd"
#define MQTT_STATE_TOPIC "minipupper/state"

static nvs_handle_t nvs;

static float offset[13] = {0};
static float L1 = 50, L2 = 56;

static int Ini=0, Step=0, Roll=0, Pitch=0, Stretch=0;
static int Advance=0, Back=0, Left=0, Right=0, TurnL=0, TurnR=0;
static int Twerk=0, Jump=0, JumpFwd=0, TestSpeed=0, Mate=0, Stanford=0;
static int sg_started=0;   // Stanford gait state initialised for this activation

static int period=80, height=70, upHeight=10, stride=10, tilt=10;

static uint16_t goal[13];
static uint16_t goal_speed[13];   // legacy "speed" units; converted to mA on flush

// Manual override for servo 8 (Rear Right shoulder). When manual8 is set the
// gait task holds a neutral stand but drives servo 8 to manual8_pos.
static int manual8 = 0;
static uint16_t manual8_pos = 511;   // SCS position 0..1023 (511 = centre)

static inline void servo_speed(int ch, uint16_t spd){
    goal_speed[ch] = spd;
}
static inline void servo_speed_all(uint16_t spd){
    for(int i=1;i<=12;i++) goal_speed[i] = spd;
}

static void servo_flush(void){
    static int64_t last_us = 0;
    while(esp_timer_get_time() - last_us < 5000){
        vTaskDelay(1);
    }
    last_us = esp_timer_get_time();

    uint16_t pos[12], cur[12];
    for(int i=0; i<12; i++){
        pos[i] = goal[i+1];
        cur[i] = speed_to_current_mA(goal_speed[i+1]);  // speed -> current limit (mA)
    }
    driver_board_sync_write(pos, cur);
}

static inline uint32_t millis(void){ return (uint32_t)(esp_timer_get_time()/1000ULL); }

static void reset_all_modes(void){
    Ini=Step=Roll=Pitch=Stretch=0;
    Advance=Back=Left=Right=TurnL=TurnR=Twerk=Jump=JumpFwd=TestSpeed=Mate=Stanford=0; // <-- add TestSpeed here
    manual8=0;
    sg_started=0;
}

// Toggle a motion flag the same way the web buttons do: pressing the
// active motion's button turns it off, pressing any other turns that
// one on (and everything else off).
static void toggle_motion(int *flag){
    if(*flag){ *flag=0; reset_all_modes(); }
    else     { reset_all_modes(); *flag=1; }
}

// Command name -> flag table, shared between the web UI and MQTT.
typedef struct { const char *name; int *flag; } motion_cmd_t;
static const motion_cmd_t motion_cmds[] = {
    {"ini",       &Ini},      {"step",      &Step},     {"roll",   &Roll},
    {"pitch",     &Pitch},    {"stretch",   &Stretch},  {"advance",&Advance},
    {"back",      &Back},     {"left",      &Left},     {"right",  &Right},
    {"turnl",     &TurnL},    {"turnr",     &TurnR},    {"twerk",  &Twerk},
    {"jump",      &Jump},     {"jumpfwd",   &JumpFwd},  {"testspeed",&TestSpeed},
    {"mate",      &Mate},     {"stanford",  &Stanford},
};
#define MOTION_CMD_COUNT (sizeof(motion_cmds)/sizeof(motion_cmds[0]))

static void nvs_put_float(const char*k, float v){
    nvs_set_blob(nvs, k, &v, sizeof(float)); nvs_commit(nvs);
}
static float nvs_get_float(const char*k, float def){
    float v=def; size_t sz=sizeof(float);
    if(nvs_get_blob(nvs,k,&v,&sz)!=ESP_OK) v=def;
    return v;
}
static void nvs_put_int(const char*k, int v){
    nvs_set_i32(nvs,k,v); nvs_commit(nvs);
}

static void servo_write(int ch, float ang){
    int sig = 511 + (int)(ang / 0.263f);
    if(sig<0) sig=0;
    if(sig>1023) sig=1023;
    goal[ch] = (uint16_t)sig;
}
static void fRIK(float x,float th0,float z){
    float zd=z/cosf(th0/180.0f*PI);
    float ld=sqrtf(x*x+zd*zd);
    float phi=atan2f(x,zd);
    float th1=phi-acosf((L1*L1+ld*ld-L2*L2)/(2*L1*ld));
    float th2=asinf((ld*ld-L1*L1-L2*L2)/(2*L1*L2))-th1;
    servo_write(1,  th0            + offset[1]);
    servo_write(2, -(th1*180.0f/PI)+ offset[2]);
    servo_write(3,  th2*180.0f/PI  + offset[3]);
}
static void rRIK(float x,float th0,float z){
    float zd=z/cosf(th0/180.0f*PI);
    float ld=sqrtf(x*x+zd*zd);
    float phi=atan2f(x,zd);
    float th1=phi-acosf((L1*L1+ld*ld-L2*L2)/(2*L1*ld));
    float th2=asinf((ld*ld-L1*L1-L2*L2)/(2*L1*L2))-th1;
    servo_write(7,  th0            + offset[7]);
    servo_write(8, -(th1*180.0f/PI)+ offset[8]);
    servo_write(9,  th2*180.0f/PI  + offset[9]);
}
static void fLIK(float x,float th0,float z){
    float zd=z/cosf(th0/180.0f*PI);
    float ld=sqrtf(x*x+zd*zd);
    float phi=atan2f(x,zd);
    float th1=phi-acosf((L1*L1+ld*ld-L2*L2)/(2*L1*ld));
    float th2=asinf((ld*ld-L1*L1-L2*L2)/(2*L1*L2))-th1;
    servo_write(4,  th0            + offset[4]);
    servo_write(5,  th1*180.0f/PI  + offset[5]);
    servo_write(6, -(th2*180.0f/PI)+ offset[6]);
}
static void rLIK(float x,float th0,float z){
    float zd=z/cosf(th0/180.0f*PI);
    float ld=sqrtf(x*x+zd*zd);
    float phi=atan2f(x,zd);
    float th1=phi-acosf((L1*L1+ld*ld-L2*L2)/(2*L1*ld));
    float th2=asinf((ld*ld-L1*L1-L2*L2)/(2*L1*L2))-th1;
    servo_write(10, th0            + offset[10]);
    servo_write(11, th1*180.0f/PI  + offset[11]);
    servo_write(12,-(th2*180.0f/PI)+ offset[12]);
}

static esp_err_t send_root(httpd_req_t *req){
    char *b = malloc(12000);
    if(!b) return ESP_ERR_NO_MEM;
    int n=0;
    #define A(...) n += snprintf(b+n, 12000-n, __VA_ARGS__)
    #define ON(x) ((x)?"on":"off")

    A("<!DOCTYPE html><html lang=\"ja\"><head><meta charset=\"utf-8\">"
      "<title>Mini Pupper 2</title>"
      "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"><style>"
      ".container{margin:auto;text-align:center;font-size:1.2rem;}"
      "span,.pm{display:inline-block;border:1px solid #ccc;width:50px;height:30px;"
      "vertical-align:middle;margin-bottom:8px;}span{width:120px;}"
      "button{width:100px;height:40px;font-weight:bold;margin-bottom:8px;}"
      "button.on{background:lime;color:white;}"
      ".column-3{max-width:330px;margin:auto;text-align:center;display:flex;"
      "justify-content:space-between;flex-wrap:wrap;}"
      "button.twerk-btn{width:200px;background:#9b59b6;color:white;}"
      "button.twerk-btn.on{background:lime;color:white;}</style></head><body>"
      "<div class=\"container\"><h3>Mini Pupper 2</h3><div class=\"column-3\">");

    A("<button class=\"%s\" type=\"button\"><a href=\"/roll\">Roll</a></button><br>", ON(Roll));
    A("<button class=\"%s\" type=\"button\"><a href=\"/ad\">Advance</a></button><br>", ON(Advance));
    A("<button class=\"%s\" type=\"button\"><a href=\"/pitch\">Pitch</a></button><br>", ON(Pitch));
    A("<button class=\"%s\" type=\"button\"><a href=\"/left\">Left</a></button><br>", ON(Left));
    A("<button class=\"%s\" type=\"button\"><a href=\"/right\">Right</a></button><br>", ON(Right));
    A("<button class=\"%s\" type=\"button\"><a href=\"/turnL\">TurnL</a></button><br>", ON(TurnL));
    A("<button class=\"%s\" type=\"button\"><a href=\"/back\">Back</a></button><br>", ON(Back));
    A("<button class=\"%s\" type=\"button\"><a href=\"/turnR\">TurnR</a></button><br>", ON(TurnR));
    A("<button class=\"%s\" type=\"button\"><a href=\"/ini\">Ini</a></button><br>", ON(Ini));
    A("<button class=\"%s\" type=\"button\"><a href=\"/step\">Step</a></button><br>", ON(Step));
    A("<button class=\"%s\" type=\"button\"><a href=\"/stretch\">Stretch</a></button><br>", ON(Stretch));
    A("</div>");

    A("<div style=\"margin:8px auto;\"><button class=\"twerk-btn %s\" type=\"button\">"
      "<a href=\"/twerk\" style=\"color:white;\">&#127926; Twerk</a></button></div>", ON(Twerk));
    A("<div style=\"margin:8px auto;\"><button class=\"twerk-btn %s\" type=\"button\" "
      "style=\"background:#e67e22;\"><a href=\"/jump\" style=\"color:white;\">&#11014; Jump</a>"
      "</button></div>", ON(Jump));
    A("<div style=\"margin:8px auto;\"><button class=\"twerk-btn %s\" type=\"button\" "
      "style=\"background:#27ae60;\"><a href=\"/jumpfwd\" style=\"color:white;\">&#8599; Jump Fwd</a>"
      "</button></div>", ON(JumpFwd));
    A("<div style=\"margin:8px auto;\"><button class=\"twerk-btn %s\" type=\"button\" "
      "style=\"background:#2980b9;\"><a href=\"/testspeed\" style=\"color:white;\">&#9881; Test Speed</a>"
      "</button></div>", ON(TestSpeed));
    A("<div style=\"margin:8px auto;\"><button class=\"twerk-btn %s\" type=\"button\" "
      "style=\"background:#c0392b;\"><a href=\"/mate\" style=\"color:white;\">&#10084; Mate</a>"
      "</button></div>", ON(Mate));
    A("<div style=\"margin:8px auto;\"><button class=\"twerk-btn %s\" type=\"button\" "
      "style=\"background:#16a085;\"><a href=\"/stanford\" style=\"color:white;\">&#128021; Stanford Walk</a>"
      "</button></div>", ON(Stanford));
    A("<div style=\"margin:8px auto;\"><form action=\"/leg8\" method=\"get\" "
      "style=\"display:inline;\">Leg 8 pos (0-1023): "
      "<input type=\"number\" name=\"v\" value=\"%d\" min=\"0\" max=\"1023\" "
      "style=\"width:80px;height:34px;\"><button type=\"submit\" "
      "style=\"width:110px;background:%s;color:white;\">Set Leg 8</button>"
      "</form></div>", manual8_pos, manual8?"lime":"#555");
    A("period (msec)<br><a class=\"pm\" href=\"/periodM\">-</a><span>%d</span>"
      "<a class=\"pm\" href=\"/periodP\">+</a><br>", period);
    A("height (mm)<br><a class=\"pm\" href=\"/heightM\">-</a><span>%d</span>"
      "<a class=\"pm\" href=\"/heightP\">+</a><br>", height);
    A("upHeight (mm)<br><a class=\"pm\" href=\"/upHeightM\">-</a><span>%d</span>"
      "<a class=\"pm\" href=\"/upHeightP\">+</a><br>", upHeight);
    A("stride (mm)<br><a class=\"pm\" href=\"/strideM\">-</a><span>%d</span>"
      "<a class=\"pm\" href=\"/strideP\">+</a><br>", stride);
    A("tilt (deg)<br><a class=\"pm\" href=\"/tiltM\">-</a><span>%d</span>"
      "<a class=\"pm\" href=\"/tiltP\">+</a><br>", tilt);

    A("<hr><h4>Servo Calibration (Offsets in degrees)</h4>"
      "<p style=\"font-size:0.9rem;color:#666;\">Press 'Ini' first, then adjust offsets</p>");

    const char* legN[4]={"Front Right Leg","Front Left Leg","Rear Right Leg","Rear Left Leg"};
    const char* legC[4]={"#ffe4e1","#e1f5ff","#fff4e1","#e8ffe1"};
    const char* jN[3]={"Hip","Shoulder","Knee"};
    for(int leg=0; leg<4; leg++){
        A("<div style=\"background:%s;padding:10px;margin:10px 0;border-radius:5px;\">"
          "<strong>%s</strong><br>", legC[leg], legN[leg]);
        for(int j=0;j<3;j++){
            int id = leg*3 + j + 1;
            A("S%d (%s): <a class=\"pm\" href=\"/cal%dM\">-</a><span>%.1f</span>"
              "<a class=\"pm\" href=\"/cal%dP\">+</a><br>", id, jN[j], id, offset[id], id);
        }
        A("</div>");
    }
    A("<button type=\"button\" style=\"background:#ff6b6b;color:white;width:200px;\">"
      "<a href=\"/calReset\" style=\"color:white;\">Reset All Offsets to 0</a></button><br>"
      "</div></body></html>");

    #undef A
    #undef ON
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, b, n);
    free(b);
    return ESP_OK;
}

#define MOTION(name, var) \
static esp_err_t name(httpd_req_t*r){ \
    uint32_t t0 = millis(); \
    toggle_motion(&var); \
    esp_err_t ret = send_root(r); \
    ESP_LOGI(TAG, "HTTP %s handled in %lu ms", r->uri, (unsigned long)(millis()-t0)); \
    return ret; }
MOTION(h_ini,Ini)   MOTION(h_step,Step)   MOTION(h_roll,Roll)
MOTION(h_pitch,Pitch) MOTION(h_stretch,Stretch) MOTION(h_ad,Advance)
MOTION(h_back,Back) MOTION(h_left,Left)   MOTION(h_right,Right)
MOTION(h_turnL,TurnL) MOTION(h_turnR,TurnR) MOTION(h_twerk,Twerk)
MOTION(h_jump,Jump)
MOTION(h_jumpfwd,JumpFwd)
MOTION(h_testspeed,TestSpeed)
MOTION(h_mate,Mate)
MOTION(h_stanford,Stanford)

static esp_err_t h_root(httpd_req_t*r){ return send_root(r); }

static esp_err_t h_periodM(httpd_req_t*r){ if(period>30){period-=5;nvs_put_int("period",period);} return send_root(r);}
static esp_err_t h_periodP(httpd_req_t*r){ if(period<=1000){period+=5;nvs_put_int("period",period);} return send_root(r);}
static esp_err_t h_heightM(httpd_req_t*r){ if(height>30){height-=5;nvs_put_int("height",height);} return send_root(r);}
static esp_err_t h_heightP(httpd_req_t*r){ if(height<=90){height+=5;nvs_put_int("height",height);} return send_root(r);}
static esp_err_t h_upM(httpd_req_t*r){ if(upHeight>0){upHeight-=2;nvs_put_int("upHeight",upHeight);} return send_root(r);}
static esp_err_t h_upP(httpd_req_t*r){ if(upHeight<=40){upHeight+=2;nvs_put_int("upHeight",upHeight);} return send_root(r);}
static esp_err_t h_strM(httpd_req_t*r){ if(stride>0){stride-=2;nvs_put_int("stride",stride);} return send_root(r);}
static esp_err_t h_strP(httpd_req_t*r){ if(stride<=40){stride+=2;nvs_put_int("stride",stride);} return send_root(r);}
static esp_err_t h_tiltM(httpd_req_t*r){ if(tilt>0){tilt-=2;nvs_put_int("tilt",tilt);} return send_root(r);}
static esp_err_t h_tiltP(httpd_req_t*r){ if(tilt<=40){tilt+=2;nvs_put_int("tilt",tilt);} return send_root(r);}

static esp_err_t h_cal(httpd_req_t*r){
    int id=0; sscanf(r->uri, "/cal%d", &id);
    char sign = r->uri[strlen(r->uri)-1];
    if(id>=1 && id<=12){
        offset[id] += (sign=='P') ? 1.0f : -1.0f;
        char k[12]; snprintf(k,sizeof k,"offset%d",id);
        nvs_put_float(k, offset[id]);
    }
    return send_root(r);
}
static esp_err_t h_calReset(httpd_req_t*r){
    for(int i=1;i<=12;i++){ offset[i]=0; char k[12];
        snprintf(k,sizeof k,"offset%d",i); nvs_put_float(k,0); }
    return send_root(r);
}

// /leg8?v=NNN  -> hold servo 8 at raw SCS position NNN (0..1023, 511=centre)
static esp_err_t h_leg8(httpd_req_t*r){
    char q[32], val[8];
    if(httpd_req_get_url_query_str(r,q,sizeof q)==ESP_OK &&
       httpd_query_key_value(q,"v",val,sizeof val)==ESP_OK){
        int p=atoi(val);
        if(p<0) p=0;
        if(p>1023) p=1023;
        reset_all_modes();
        manual8_pos=(uint16_t)p;
        manual8=1;
    }
    return send_root(r);
}

static void reg(httpd_handle_t s,const char*uri,esp_err_t(*h)(httpd_req_t*)){
    httpd_uri_t u={.uri=uri,.method=HTTP_GET,.handler=h};
    httpd_register_uri_handler(s,&u);
}

static void start_webserver(void){
    httpd_handle_t s=NULL;
    httpd_config_t cfg=HTTPD_DEFAULT_CONFIG();
    cfg.max_uri_handlers=60;
    cfg.stack_size=8192;
    cfg.core_id = 0;
    cfg.lru_purge_enable=true;
    ESP_ERROR_CHECK(httpd_start(&s,&cfg));

    reg(s,"/",h_root);
    reg(s,"/ini",h_ini);     reg(s,"/step",h_step);   reg(s,"/roll",h_roll);
    reg(s,"/pitch",h_pitch); reg(s,"/stretch",h_stretch);
    reg(s,"/ad",h_ad);       reg(s,"/back",h_back);   reg(s,"/left",h_left);
    reg(s,"/right",h_right); reg(s,"/turnL",h_turnL); reg(s,"/turnR",h_turnR);
    reg(s,"/twerk",h_twerk); reg(s,"/jump",h_jump); reg(s,"/jumpfwd",h_jumpfwd); reg(s,"/testspeed",h_testspeed);
    reg(s,"/mate",h_mate);
    reg(s,"/stanford",h_stanford);
    reg(s,"/periodM",h_periodM); reg(s,"/periodP",h_periodP);
    reg(s,"/heightM",h_heightM); reg(s,"/heightP",h_heightP);
    reg(s,"/upHeightM",h_upM);   reg(s,"/upHeightP",h_upP);
    reg(s,"/strideM",h_strM);    reg(s,"/strideP",h_strP);
    reg(s,"/tiltM",h_tiltM);     reg(s,"/tiltP",h_tiltP);
    reg(s,"/calReset",h_calReset);
    reg(s,"/leg8",h_leg8);
    char uri[12];
    for(int i=1;i<=12;i++){
        snprintf(uri,sizeof uri,"/cal%dM",i); reg(s,strdup(uri),h_cal);
        snprintf(uri,sizeof uri,"/cal%dP",i); reg(s,strdup(uri),h_cal);
    }
}

static esp_mqtt_client_handle_t mqtt_client = NULL;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data){
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    switch(event_id){
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected, subscribing to %s", MQTT_CMD_TOPIC);
            esp_mqtt_client_subscribe(mqtt_client, MQTT_CMD_TOPIC, 0);
            break;
        case MQTT_EVENT_DATA: {
            char cmd[32] = {0};
            int len = event->data_len < (int)sizeof(cmd)-1 ? event->data_len : (int)sizeof(cmd)-1;
            memcpy(cmd, event->data, len);
            for(int i=0;i<len;i++) cmd[i] = (char)tolower((unsigned char)cmd[i]);

            for(size_t i=0;i<MOTION_CMD_COUNT;i++){
                if(strcmp(cmd, motion_cmds[i].name)==0){
                    toggle_motion(motion_cmds[i].flag);
                    esp_mqtt_client_publish(mqtt_client, MQTT_STATE_TOPIC, cmd, 0, 0, 0);
                    break;
                }
            }
            break;
        }
        default: break;
    }
}

static void mqtt_app_start(void){
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data){
    if(event_base==WIFI_EVENT && event_id==WIFI_EVENT_STA_START){
        esp_wifi_connect();
    }else if(event_base==WIFI_EVENT && event_id==WIFI_EVENT_STA_DISCONNECTED){
        ESP_LOGW(TAG, "WiFi disconnected, reconnecting...");
        esp_wifi_connect();
    }else if(event_base==IP_EVENT && event_id==IP_EVENT_STA_GOT_IP){
        ip_event_got_ip_t *event = (ip_event_got_ip_t*)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        mqtt_app_start();
    }
}

static void wifi_init_sta(void){
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wcfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t sc = {0};
    strncpy((char*)sc.sta.ssid, WIFI_SSID, sizeof(sc.sta.ssid)-1);
    strncpy((char*)sc.sta.password, WIFI_PASS, sizeof(sc.sta.password)-1);
    sc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sc));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_wifi_set_ps(WIFI_PS_NONE);

    ESP_LOGI(TAG, "Connecting to WiFi SSID: %s", WIFI_SSID);
}

static void gait_task(void *arg){
    float tim, tt;
    uint32_t time_mSt;

    for(int i=1;i<=12;i++) goal[i] = 511;
    servo_speed_all(0);

    for(;;){
        if(Ini){
            servo_speed_all(0);
            for(int i=1; i<=12; i++) servo_write(i, offset[i]);
            servo_flush();
            vTaskDelay(1);

        }else if(Step){
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK(0,0,height-upHeight*sinf(tt)); rLIK(0,0,height-upHeight*sinf(tt)); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK(0,0,height-upHeight*cosf(tt)); rLIK(0,0,height-upHeight*cosf(tt)); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                rRIK(0,0,height-upHeight*sinf(tt)); fLIK(0,0,height-upHeight*sinf(tt)); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                rRIK(0,0,height-upHeight*cosf(tt)); fLIK(0,0,height-upHeight*cosf(tt)); servo_flush(); }

        }else if(Roll){
            time_mSt=millis(); tim=0;
            while(tim<period*8){ tim=millis()-time_mSt; tt=(float)(tim*2*PI/(period*8));
                fRIK(0,-tilt*sinf(tt),height); rLIK(0,tilt*sinf(tt),height);
                rRIK(0,tilt*sinf(tt),height);  fLIK(0,-tilt*sinf(tt),height); servo_flush(); }

        }else if(Pitch){
            time_mSt=millis(); tim=0;
            while(tim<period*8){ tim=millis()-time_mSt; tt=(float)(tim*2*PI/(period*8));
                fRIK(0,0,height-upHeight*sinf(tt)); rLIK(0,0,height+upHeight*sinf(tt));
                rRIK(0,0,height+upHeight*sinf(tt)); fLIK(0,0,height-upHeight*sinf(tt)); servo_flush(); }

        }else if(Stretch){
            time_mSt=millis(); tim=0;
            while(tim<period*8){ tim=millis()-time_mSt; tt=(float)(tim*2*PI/(period*8));
                fRIK(0,0,height+upHeight*sinf(tt)); rLIK(0,0,height+upHeight*sinf(tt));
                rRIK(0,0,height+upHeight*sinf(tt)); fLIK(0,0,height+upHeight*sinf(tt)); servo_flush(); }

        }else if(Advance){
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK(-stride*cosf(tt),0,height-upHeight*sinf(tt)); rLIK(-stride*cosf(tt),0,height-upHeight*sinf(tt));
                rRIK( stride*cosf(tt),0,height);                   fLIK( stride*cosf(tt),0,height); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK( stride*sinf(tt),0,height-upHeight*cosf(tt)); rLIK( stride*sinf(tt),0,height-upHeight*cosf(tt));
                rRIK(-stride*sinf(tt),0,height);                   fLIK(-stride*sinf(tt),0,height); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK( stride*cosf(tt),0,height);                   rLIK( stride*cosf(tt),0,height);
                rRIK(-stride*cosf(tt),0,height-upHeight*sinf(tt)); fLIK(-stride*cosf(tt),0,height-upHeight*sinf(tt)); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK(-stride*sinf(tt),0,height);                   rLIK(-stride*sinf(tt),0,height);
                rRIK( stride*sinf(tt),0,height-upHeight*cosf(tt)); fLIK( stride*sinf(tt),0,height-upHeight*cosf(tt)); servo_flush(); }
            ESP_LOGI(TAG, "cur(mA): 1=%d 2=%d 3=%d 4=%d 5=%d 6=%d 7=%d 8=%d 9=%d 10=%d 11=%d 12=%d",
                driver_board_present_current(1),  driver_board_present_current(2),
                driver_board_present_current(3),  driver_board_present_current(4),
                driver_board_present_current(5),  driver_board_present_current(6),
                driver_board_present_current(7),  driver_board_present_current(8),
                driver_board_present_current(9),  driver_board_present_current(10),
                driver_board_present_current(11), driver_board_present_current(12));

        }else if(Back){
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK( stride*cosf(tt),0,height-upHeight*sinf(tt)); rLIK( stride*cosf(tt)+15,0,height-upHeight*sinf(tt));
                rRIK(-stride*cosf(tt)+15,0,height);                fLIK(-stride*cosf(tt),0,height); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK(-stride*sinf(tt),0,height-upHeight*cosf(tt)); rLIK(-stride*sinf(tt)+15,0,height-upHeight*cosf(tt));
                rRIK( stride*sinf(tt)+15,0,height);                fLIK( stride*sinf(tt),0,height); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK(-stride*cosf(tt),0,height);                   rLIK(-stride*cosf(tt)+15,0,height);
                rRIK( stride*cosf(tt)+15,0,height-upHeight*sinf(tt)); fLIK( stride*cosf(tt),0,height-upHeight*sinf(tt)); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK( stride*sinf(tt),0,height);                   rLIK( stride*sinf(tt)+15,0,height);
                rRIK(-stride*sinf(tt)+15,0,height-upHeight*cosf(tt)); fLIK(-stride*sinf(tt),0,height-upHeight*cosf(tt)); servo_flush(); }

        }else if(Left){
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK(0, tilt-2*tilt*sinf(tt),height-upHeight*sinf(tt)); rLIK(0,-tilt+2*tilt*sinf(tt),height-upHeight*sinf(tt));
                rRIK(0, tilt*cosf(tt),height-upHeight*cosf(tt));        fLIK(0,-tilt*cosf(tt),height-upHeight*cosf(tt)); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                rRIK(0,-tilt*sinf(tt),height); fLIK(0,tilt*sinf(tt),height); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK(0,-tilt*cosf(tt),height-upHeight*cosf(tt)); rLIK(0,tilt*cosf(tt),height-upHeight*cosf(tt));
                rRIK(0,-tilt+2*tilt*sinf(tt),height-upHeight*sinf(tt)); fLIK(0,tilt-2*tilt*sinf(tt),height-upHeight*sinf(tt)); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK(0,tilt*sinf(tt),height); rLIK(0,-tilt*sinf(tt),height); servo_flush(); }

        }else if(Right){
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK(0,-tilt+2*tilt*sinf(tt),height-upHeight*sinf(tt)); rLIK(0,tilt-2*tilt*sinf(tt),height-upHeight*sinf(tt));
                rRIK(0,-tilt*cosf(tt),height-upHeight*cosf(tt));        fLIK(0,tilt*cosf(tt),height-upHeight*cosf(tt)); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                rRIK(0,tilt*sinf(tt),height); fLIK(0,-tilt*sinf(tt),height); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK(0,tilt*cosf(tt),height-upHeight*cosf(tt)); rLIK(0,-tilt*cosf(tt),height-upHeight*cosf(tt));
                rRIK(0,tilt-2*tilt*sinf(tt),height-upHeight*sinf(tt)); fLIK(0,-tilt+2*tilt*sinf(tt),height-upHeight*sinf(tt)); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK(0,-tilt*sinf(tt),height); rLIK(0,tilt*sinf(tt),height); servo_flush(); }

        }else if(TurnL){
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK(0, tilt-2*tilt*sinf(tt),height-upHeight*sinf(tt)); rLIK(0, tilt-2*tilt*sinf(tt),height-upHeight*sinf(tt));
                rRIK(0,-tilt*cosf(tt),height-upHeight*cosf(tt));        fLIK(0,-tilt*cosf(tt),height-upHeight*cosf(tt)); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                rRIK(0,tilt*sinf(tt),height); fLIK(0,tilt*sinf(tt),height); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK(0,-tilt*cosf(tt),height-upHeight*cosf(tt));         rLIK(0,-tilt*cosf(tt),height-upHeight*cosf(tt));
                rRIK(0,tilt-2*tilt*sinf(tt),height-upHeight*sinf(tt));   fLIK(0,tilt-2*tilt*sinf(tt),height-upHeight*sinf(tt)); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK(0,tilt*sinf(tt),height); rLIK(0,tilt*sinf(tt),height); servo_flush(); }

        }else if(TurnR){
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK(0,-tilt+2*tilt*sinf(tt),height-upHeight*sinf(tt)); rLIK(0,-tilt+2*tilt*sinf(tt),height-upHeight*sinf(tt));
                rRIK(0, tilt*cosf(tt),height-upHeight*cosf(tt));        fLIK(0, tilt*cosf(tt),height-upHeight*cosf(tt)); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                rRIK(0,-tilt*sinf(tt),height); fLIK(0,-tilt*sinf(tt),height); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK(0,tilt*cosf(tt),height-upHeight*cosf(tt));          rLIK(0,tilt*cosf(tt),height-upHeight*cosf(tt));
                rRIK(0,-tilt+2*tilt*sinf(tt),height-upHeight*sinf(tt));  fLIK(0,-tilt+2*tilt*sinf(tt),height-upHeight*sinf(tt)); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK(0,-tilt*sinf(tt),height); rLIK(0,-tilt*sinf(tt),height); servo_flush(); }

        }else if(Twerk){
            // Fast up/down vibration: one quick full cycle per pass, which
            // the outer loop repeats continuously while Twerk is held.
            // Front and rear bounce in opposite phase for a bigger shake.
            float twerkAmp  = upHeight * 1.0f;
            float frontAmp  = upHeight * 0.6f;  // front legs bounce less than rear
            float zLo = 15.0f, zHi = 100.0f; // keep within safe leg reach

            servo_speed_all(300); // slower servo travel for a gentler shake
            time_mSt=millis(); tim=0;
            while(tim<period*5){ tim=millis()-time_mSt; tt=(float)(tim*2.0*PI/(period*5));
                float zf = fmaxf(zLo, fminf(zHi, height - frontAmp*sinf(tt)));
                float zr = fmaxf(zLo, fminf(zHi, height + twerkAmp*sinf(tt)));
                fRIK(0,0,zf); fLIK(0,0,zf);
                rRIK(0,0,zr); rLIK(0,0,zr); servo_flush(); }

        }else if(Mate){
            // Front legs stand tall and stay still; rear end thrusts up/down.
            float frontZ   = 90.0f;          // raised front stance, held fixed
            float rearMidZ = 50.0f;          // rear sits lower -> mounting posture
            float rearAmp  = upHeight * 1.5f;
            float zLo = 15.0f, zHi = 100.0f; // keep within safe leg reach

            servo_speed_all(400);
            fRIK(0,0,frontZ); fLIK(0,0,frontZ);
            servo_flush();

            time_mSt=millis(); tim=0;
            while(tim<period*3){ tim=millis()-time_mSt; tt=(float)(tim*2.0*PI/(period*3));
                float zr = fmaxf(zLo, fminf(zHi, rearMidZ + rearAmp*sinf(tt)));
                fRIK(0,0,frontZ); fLIK(0,0,frontZ);
                rRIK(0,0,zr); rLIK(0,0,zr); servo_flush(); }

        }else if(Jump){
            float crouchZ = 40;
            float pushZ   = 105;
            float tuckZ   = 45;

            time_mSt=millis(); tim=0;
            while(tim<period*2){ tim=millis()-time_mSt;
                tt = (float)(tim * PI / 2.0 / (period*2));
                float z = height - (height - crouchZ) * sinf(tt);
                fRIK(0,0,z); fLIK(0,0,z); rRIK(0,0,z); rLIK(0,0,z); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<20){ tim=millis()-time_mSt;
                fRIK(0,0,crouchZ); fLIK(0,0,crouchZ); rRIK(0,0,crouchZ); rLIK(0,0,crouchZ); servo_flush(); }

            servo_speed_all(0);
            fRIK(0,0,pushZ); fLIK(0,0,pushZ); rRIK(0,0,pushZ); rLIK(0,0,pushZ);
            servo_flush();
            servo_flush();
            int airMs = (int)(160.0f + (70.0f - crouchZ) * 1.0f);
            vTaskDelay(pdMS_TO_TICKS(airMs));

            time_mSt=millis(); tim=0;
            int tuckMs = 50;
            while(tim<tuckMs){ tim=millis()-time_mSt;
                float frac = sinf((float)tim * PI / 2.0f / (float)tuckMs);
                float z = pushZ - (pushZ - tuckZ) * frac;
                fRIK(0,0,z); fLIK(0,0,z); rRIK(0,0,z); rLIK(0,0,z); servo_flush(); }

            time_mSt=millis(); tim=0;
            while(tim<period*3){ tim=millis()-time_mSt;
                tt = (float)(tim * PI / 2.0 / (period*3));
                float z = tuckZ + (height - tuckZ) * sinf(tt);
                fRIK(0,0,z); fLIK(0,0,z); rRIK(0,0,z); rLIK(0,0,z); servo_flush(); }

            Jump = 0;

        }else if(JumpFwd){
            float crouchZrear  = 60;
            float crouchZfront = 40;
            float pushZ        = 105;
            float tuckZ        = 45;

            // ----------------------------------------------------------------
            // Phase 1: CONTROLLED CROUCH
            //   Medium speed (70) so it looks like the dog is deliberately
            //   loading up energy rather than just falling down fast.
            // ----------------------------------------------------------------
            // Phase 1: CONTROLLED CROUCH
            //   Medium speed (100) so it looks like the dog is deliberately
            //   loading up energy rather than just falling down fast.
            // ----------------------------------------------------------------
            servo_speed_all(100);   // NOW this actually controls how slow it lowers
            fRIK(0,0,crouchZfront); fLIK(0,0,crouchZfront);
            rRIK(0,0,crouchZrear);  rLIK(0,0,crouchZrear);
            servo_flush();
            vTaskDelay(pdMS_TO_TICKS(900));  // give it time to travel slowly
            // Brief settle so every servo actually reaches its crouch pose
            time_mSt=millis(); tim=0;
            while(tim<25){ tim=millis()-time_mSt;
                fRIK(0,0,crouchZfront); fLIK(0,0,crouchZfront);
                rRIK(0,0,crouchZrear);  rLIK(0,0,crouchZrear);
                servo_flush(); }

            // ----------------------------------------------------------------
            // Phase 2: EXPLOSIVE EXTENSION — max speed (0)
            //   The speed contrast with the slow crouch is what makes this
            //   feel snappy. Two packets for bus reliability.
            // ----------------------------------------------------------------
            servo_speed_all(0);
            fRIK(0,0,pushZ); fLIK(0,0,pushZ);
            rRIK(0,0,pushZ); rLIK(0,0,pushZ);
            servo_flush();
            servo_flush();

            // Read torque (load %) and current (mA) on the rear knee servos
            // right at the moment of the push, to see how hard they're working.
            {
                // feedback now comes back on each SPI transaction (position + current)
                ESP_LOGI(TAG, "JumpFwd push RR knee(9):  pos=%u cur=%dmA",
                         driver_board_present_position(9),  driver_board_present_current(9));
                ESP_LOGI(TAG, "JumpFwd push RL knee(12): pos=%u cur=%dmA",
                         driver_board_present_position(12), driver_board_present_current(12));
            }

            // Airborne window
            int airMs = (int)(160.0f + (70.0f - crouchZrear) * 1.0f);
            vTaskDelay(pdMS_TO_TICKS(airMs));

            // ----------------------------------------------------------------
            // Phase 3: POUNCE TUCK — still max speed (0)
            //   Must stay fast: we're airborne and need legs repositioned
            //   before the dog hits the ground.
            //   Rear tucks first while front stays reaching → pounce look.
            // ----------------------------------------------------------------
            // servo_speed already 0 from Phase 2, no need to set again
            time_mSt=millis(); tim=0;
            int rearTuckMs = 45;
            while(tim<rearTuckMs){ tim=millis()-time_mSt;
                float frac = sinf((float)tim * PI / 2.0f / (float)rearTuckMs);
                float zr = pushZ - (pushZ - tuckZ) * frac;
                rRIK(0,0,zr); rLIK(0,0,zr);
                fRIK(0,0,pushZ); fLIK(0,0,pushZ);
                servo_flush(); }

            time_mSt=millis(); tim=0;
            int frontTuckMs = 45;
            while(tim<frontTuckMs){ tim=millis()-time_mSt;
                float frac = sinf((float)tim * PI / 2.0f / (float)frontTuckMs);
                float zf = pushZ - (pushZ - tuckZ) * frac;
                fRIK(0,0,zf); fLIK(0,0,zf);
                rRIK(0,0,tuckZ); rLIK(0,0,tuckZ);
                servo_flush(); }

            // ----------------------------------------------------------------
            // Phase 4: SOFT LANDING RECOVERY — slower speed (250)
            //   Legs extend gently to absorb the impact instead of snapping
            //   down hard. Looks springy, like the dog sticks the landing.
            // ----------------------------------------------------------------
            // Phase 4: SOFT LANDING — one command, servo speed controls how fast it arrives
            servo_speed_all(0);    // NOW this actually does something
            fRIK(0,0,height); fLIK(0,0,height);
            rRIK(0,0,height); rLIK(0,0,height);
            servo_flush();
            vTaskDelay(pdMS_TO_TICKS(period * 4));  // wait for it to finish travelling
            // Always reset to max speed so other motions are unaffected
            servo_speed_all(0);
            JumpFwd = 0;
        }else if(TestSpeed){
            ESP_LOGI(TAG, "--- Speed Test START ---");

            // Step 1: go to a neutral mid position at max speed
            servo_speed_all(2047);
            fRIK(0,0,70); fLIK(0,0,70); rRIK(0,0,70); rLIK(0,0,70);
            servo_flush();
            vTaskDelay(pdMS_TO_TICKS(1500));

            // Step 2: move to a lower position SLOWLY — you should see it creep down
            ESP_LOGI(TAG, "Moving SLOW (speed=30)");
            servo_speed_all(30);
            fRIK(0,0,100); fLIK(0,0,100); rRIK(0,0,100); rLIK(0,0,100);
            servo_flush();
            vTaskDelay(pdMS_TO_TICKS(3000));   // watch it move slowly

            // Step 3: snap back FAST — you should see it jump back instantly
            ESP_LOGI(TAG, "Moving FAST (speed=2047)");
            servo_speed_all(2047);
            fRIK(0,0,70); fLIK(0,0,70); rRIK(0,0,70); rLIK(0,0,70);
            servo_flush();
            vTaskDelay(pdMS_TO_TICKS(2000));   // watch it snap back

            ESP_LOGI(TAG, "--- Speed Test DONE ---");
            servo_speed_all(0);
            TestSpeed = 0;   // auto-clears after one run

        }else if(Stanford){
            // Stanford Pupper trot gait (forward only), ported from
            // mangdangroboticsclub/StanfordQuadruped. Uses the NATIVE Mini
            // Pupper parameters from the BSP Config.py (height 80 mm,
            // clearance 30 mm, 15 ms tick) — independent of the web
            // sliders, so height/period/stride/upHeight are untouched.
            static int64_t sg_next_us = 0;

            if(!sg_started){
                stanford_gait_reset(SG_NATIVE_HEIGHT_MM);
                sg_next_us = esp_timer_get_time();
                servo_speed_all(0);
                sg_started = 1;
            }

            sg_foot_t feet[4];
            stanford_gait_step(SG_NATIVE_VX_MM_S, SG_NATIVE_HEIGHT_MM,
                               SG_NATIVE_CLEARANCE_MM, feet);
            fRIK(feet[0].x, 0, feet[0].z);   // Front Right
            fLIK(feet[1].x, 0, feet[1].z);   // Front Left
            rRIK(feet[2].x, 0, feet[2].z);   // Rear Right
            rLIK(feet[3].x, 0, feet[3].z);   // Rear Left
            servo_flush();

            // Pace to the next 10 ms tick; resync if we fell far behind.
            sg_next_us += (int64_t)(SG_DT * 1e6f);
            int64_t now = esp_timer_get_time();
            if(now > sg_next_us + 100000) sg_next_us = now;
            while(esp_timer_get_time() < sg_next_us && Stanford) vTaskDelay(1);

        }else if(manual8){
            // Hold a neutral stand, but drive servo 8 to the manually entered
            // position instead of the value the IK just computed.
            servo_speed_all(0);
            fRIK(0,0,height); rRIK(0,0,height); fLIK(0,0,height); rLIK(0,0,height);
            goal[8] = manual8_pos;   // override just servo 8
            servo_flush();
            vTaskDelay(1);

        }else{
            fRIK(0,0,height); rRIK(0,0,height); fLIK(0,0,height); rLIK(0,0,height);
            servo_flush();
            vTaskDelay(1);
        }
    }
}

void app_main(void){
    esp_err_t r = nvs_flash_init();
    if(r==ESP_ERR_NVS_NO_FREE_PAGES || r==ESP_ERR_NVS_NEW_VERSION_FOUND){
        nvs_flash_erase(); nvs_flash_init();
    }
    nvs_open("parameter", NVS_READWRITE, &nvs);

    driver_board_init();                 // SPI bus + 4 AT32 driver boards + servo power ON
    vTaskDelay(pdMS_TO_TICKS(1000));     // let servo power rails settle

    int32_t v;
    if(nvs_get_i32(nvs,"period",&v)==ESP_OK) period=v;
    if(nvs_get_i32(nvs,"height",&v)==ESP_OK) height=v;
    for(int i=1;i<=12;i++){ char k[12]; snprintf(k,sizeof k,"offset%d",i);
        offset[i]=nvs_get_float(k, offset[i]); }

    wifi_init_sta();
    start_webserver();

    xTaskCreatePinnedToCore(gait_task, "gait", 8192, NULL, 22, NULL, 1);
}