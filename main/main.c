#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
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
#include "SCServo.h"

#define TAG "PUPPER"
#define PI 3.14159265358979f

#define SERVO_TX_PIN    4
#define SERVO_RX_PIN    5
#define SERVO_BAUD_RATE 1000000

static nvs_handle_t nvs;

static float offset[13] = {0};
static float L1 = 50, L2 = 56;

static int Ini=0, Step=0, Roll=0, Pitch=0, Stretch=0;
static int Advance=0, Back=0, Left=0, Right=0, TurnL=0, TurnR=0;
static int Twerk=0, Jump=0;

static int period=80, height=70, upHeight=10, stride=10, tilt=10;

/* ---------- sync write buffer ---------- */
/* goal[1..12]       = target signal (0..1023) for each servo
   goal_speed[1..12] = per-servo speed (0 = full/max speed)            */
static uint16_t goal[13];
static uint16_t goal_speed[13];                 // all start at 0 (full speed)
static const uint8_t sync_ids[12] = {1,2,3,4,5,6,7,8,9,10,11,12};

/* set the speed for ONE servo (call before servo_flush) */
static inline void servo_speed(int ch, uint16_t spd){
    goal_speed[ch] = spd;
}
/* set the same speed for ALL servos */
static inline void servo_speed_all(uint16_t spd){
    for(int i=1;i<=12;i++) goal_speed[i] = spd;
}

/* push positions + per-servo speeds to all 12 servos in ONE packet.
 *
 * IMPORTANT FIX:
 *  - This loop used to be hammered with no yield, which flooded the
 *    half-duplex servo bus and starved the idle task -> servos die while
 *    the ESP keeps running. We now pace it to ~5 ms/frame and yield, which
 *    mimics the natural pacing of the per-servo WritePos() version.
 *  - speed is sent as 0 (= max speed), exactly like WritePos(ch,sig,0,0).
 */
static void servo_flush(void){
    static int64_t last_us = 0;

    /* pace to ~5 ms per frame: never floods the bus, and vTaskDelay lets the
       idle task run (feeds the watchdog) and lets the UART TX drain */
    while(esp_timer_get_time() - last_us < 5000){
        vTaskDelay(1);
    }
    last_us = esp_timer_get_time();

    uint16_t pos[12], spd[12], tim[12];
    for(int i=0; i<12; i++){
        pos[i] = goal[i+1];
        tim[i] = 0;                 /* time=0: no timed control */
        spd[i] = goal_speed[i+1];   /* 0 == max speed, same as WritePos(ch,sig,0,0) */
    }
    SyncWritePos((uint8_t*)sync_ids, 12, pos, tim, spd);
}

/* ---------- helpers ---------- */
static inline uint32_t millis(void){ return (uint32_t)(esp_timer_get_time()/1000ULL); }

static void reset_all_modes(void){
    Ini=Step=Roll=Pitch=Stretch=0;
    Advance=Back=Left=Right=TurnL=TurnR=Twerk=Jump=0;
}

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

/* ---------- servo + IK ---------- */
/* NOTE: only stores into goal[]; transmission happens in servo_flush() */
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

/* ---------- HTML page ---------- */
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

/* ---------- handlers ---------- */
#define MOTION(name, var) \
static esp_err_t name(httpd_req_t*r){ \
    if(var){var=0;reset_all_modes();} else {reset_all_modes();var=1;} \
    return send_root(r); }
MOTION(h_ini,Ini)   MOTION(h_step,Step)   MOTION(h_roll,Roll)
MOTION(h_pitch,Pitch) MOTION(h_stretch,Stretch) MOTION(h_ad,Advance)
MOTION(h_back,Back) MOTION(h_left,Left)   MOTION(h_right,Right)
MOTION(h_turnL,TurnL) MOTION(h_turnR,TurnR) MOTION(h_twerk,Twerk)
MOTION(h_jump,Jump)

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

static void reg(httpd_handle_t s,const char*uri,esp_err_t(*h)(httpd_req_t*)){
    httpd_uri_t u={.uri=uri,.method=HTTP_GET,.handler=h};
    httpd_register_uri_handler(s,&u);
}

static void start_webserver(void){
    httpd_handle_t s=NULL;
    httpd_config_t cfg=HTTPD_DEFAULT_CONFIG();
    cfg.max_uri_handlers=60;
    cfg.stack_size=8192;
    cfg.core_id = 0;  // Pin HTTP server to core 0
    cfg.lru_purge_enable=true;
    ESP_ERROR_CHECK(httpd_start(&s,&cfg));

    reg(s,"/",h_root);
    reg(s,"/ini",h_ini);     reg(s,"/step",h_step);   reg(s,"/roll",h_roll);
    reg(s,"/pitch",h_pitch); reg(s,"/stretch",h_stretch);
    reg(s,"/ad",h_ad);       reg(s,"/back",h_back);   reg(s,"/left",h_left);
    reg(s,"/right",h_right); reg(s,"/turnL",h_turnL); reg(s,"/turnR",h_turnR);
    reg(s,"/twerk",h_twerk); reg(s,"/jump",h_jump);
    reg(s,"/periodM",h_periodM); reg(s,"/periodP",h_periodP);
    reg(s,"/heightM",h_heightM); reg(s,"/heightP",h_heightP);
    reg(s,"/upHeightM",h_upM);   reg(s,"/upHeightP",h_upP);
    reg(s,"/strideM",h_strM);    reg(s,"/strideP",h_strP);
    reg(s,"/tiltM",h_tiltM);     reg(s,"/tiltP",h_tiltP);
    reg(s,"/calReset",h_calReset);
    char uri[12];
    for(int i=1;i<=12;i++){
        snprintf(uri,sizeof uri,"/cal%dM",i); reg(s,strdup(uri),h_cal);
        snprintf(uri,sizeof uri,"/cal%dP",i); reg(s,strdup(uri),h_cal);
    }
}

/* ---------- WiFi softAP with fixed IP ---------- */
static void wifi_init_softap(void){
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t* ap = esp_netif_create_default_wifi_ap();

    esp_netif_ip_info_t ipinfo;
    IP4_ADDR(&ipinfo.ip,      192,168,55,22);
    IP4_ADDR(&ipinfo.gw,      192,168,55,22);
    IP4_ADDR(&ipinfo.netmask, 255,255,255,0);
    esp_netif_dhcps_stop(ap);
    esp_netif_set_ip_info(ap, &ipinfo);
    esp_netif_dhcps_start(ap);

    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wcfg));

    wifi_config_t ac = {0};
    strcpy((char*)ac.ap.ssid, "MiniPupper2");
    ac.ap.ssid_len = strlen("MiniPupper2");
    strcpy((char*)ac.ap.password, "password");
    ac.ap.max_connection = 4;
    ac.ap.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ac));
    ESP_ERROR_CHECK(esp_wifi_start());

    // *** DISABLE WIFI POWER SAVING TO REDUCE LAG ***
    esp_wifi_set_ps(WIFI_PS_NONE);

    ESP_LOGI(TAG, "AP started: 192.168.55.22");
}

/* ---------- gait task (matches Arduino loop exactly) ---------- */
static void gait_task(void *arg){
    float tim, tt;
    uint32_t time_mSt;

    /* start with all servos centered + ALL SPEEDS = 0 (full speed) */
    for(int i=1;i<=12;i++) goal[i] = 511;
    servo_speed_all(0);          // <<< everything defaults to 0 here

    for(;;){
        if(Ini){
            servo_speed_all(0);  // keep full speed (change here if you want slow homing)
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
            // Phase 1: Slow descent
            time_mSt=millis(); tim=0;
            while(tim<period*4){ tim=millis()-time_mSt; tt=(float)(tim*PI/2.0/(period*4));
                fRIK(0,0,height-upHeight*sinf(tt)); fLIK(0,0,height-upHeight*sinf(tt));
                rRIK(0,0,height+upHeight*sinf(tt)); rLIK(0,0,height+upHeight*sinf(tt)); servo_flush(); }

            // Phase 2: The shake
            time_mSt=millis(); tim=0;
            while(tim<period*6){ tim=millis()-time_mSt; tt=(float)(tim*2.0*PI/period);
                fRIK(0,0,height-upHeight); fLIK(0,0,height-upHeight);
                rRIK(0,0,height+upHeight*(1.0f+0.5f*sinf(tt))); rLIK(0,0,height+upHeight*(1.0f+0.5f*sinf(tt))); servo_flush(); }

            // Phase 3: Slow return
            time_mSt=millis(); tim=0;
            while(tim<period*4){ tim=millis()-time_mSt; tt=(float)(tim*PI/2.0/(period*4));
                fRIK(0,0,height-upHeight*cosf(tt)); fLIK(0,0,height-upHeight*cosf(tt));
                rRIK(0,0,height+upHeight*cosf(tt)); rLIK(0,0,height+upHeight*cosf(tt)); servo_flush(); }

        }else if(Jump){
            float crouchZ = 40;
            float pushZ   = 100;
            float tuckZ   = 45;

            if(crouchZ < 25) crouchZ = 25;
            if(pushZ > 105)  pushZ = 105;

            // Phase 1: Deep crouch
            time_mSt=millis(); tim=0;
            while(tim<period*3){ tim=millis()-time_mSt; tt=(float)(tim*PI/2.0/(period*3));
                float z = height - (height - crouchZ) * sinf(tt);
                fRIK(0,0,z); fLIK(0,0,z); rRIK(0,0,z); rLIK(0,0,z); servo_flush(); }

            // Hold crouch
            time_mSt=millis(); tim=0;
            while(tim<period*2){ tim=millis()-time_mSt;
                fRIK(0,0,crouchZ); fLIK(0,0,crouchZ); rRIK(0,0,crouchZ); rLIK(0,0,crouchZ); servo_flush(); }

            // Phase 2: Explosive extension — force max speed on all legs
            servo_speed_all(0);   // 0 = full speed = maximum pop
            fRIK(0,0,pushZ); fLIK(0,0,pushZ); rRIK(0,0,pushZ); rLIK(0,0,pushZ);
            servo_flush();
            vTaskDelay(pdMS_TO_TICKS(210));

            // Phase 3: Quick tuck
            time_mSt=millis(); tim=0;
            int tuckMs = 60;
            while(tim<tuckMs){ tim=millis()-time_mSt;
                float frac = (float)tim / (float)tuckMs;
                float z = pushZ - (pushZ - tuckZ) * frac;
                fRIK(0,0,z); fLIK(0,0,z); rRIK(0,0,z); rLIK(0,0,z); servo_flush(); }

            // Phase 4: Land / recover
            time_mSt=millis(); tim=0;
            while(tim<period*3){ tim=millis()-time_mSt; tt=(float)(tim*PI/2.0/(period*3));
                float z = tuckZ + (height - tuckZ) * sinf(tt);
                fRIK(0,0,z); fLIK(0,0,z); rRIK(0,0,z); rLIK(0,0,z); servo_flush(); }

            Jump = 0;

        }else{
            fRIK(0,0,height); rRIK(0,0,height); fLIK(0,0,height); rLIK(0,0,height);
            servo_flush();
            vTaskDelay(1);  // Small yield to prevent watchdog
        }
    }
}

/* ---------- entry ---------- */
void app_main(void){
    esp_err_t r = nvs_flash_init();
    if(r==ESP_ERR_NVS_NO_FREE_PAGES || r==ESP_ERR_NVS_NEW_VERSION_FOUND){
        nvs_flash_erase(); nvs_flash_init();
    }
    nvs_open("parameter", NVS_READWRITE, &nvs);

    gpio_set_direction(8, GPIO_MODE_OUTPUT);
    gpio_set_level(8, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));

    ftServo_InitWithType(SERVO_SCSCL, SERVO_TX_PIN, SERVO_RX_PIN, -1, SERVO_BAUD_RATE);

    int32_t v;
    if(nvs_get_i32(nvs,"period",&v)==ESP_OK) period=v;
    if(nvs_get_i32(nvs,"height",&v)==ESP_OK) height=v;
    for(int i=1;i<=12;i++){ char k[12]; snprintf(k,sizeof k,"offset%d",i);
        offset[i]=nvs_get_float(k, offset[i]); }

    wifi_init_softap();
    start_webserver();

    // *** HIGH PRIORITY GAIT TASK ON CORE 1 ***
    xTaskCreatePinnedToCore(gait_task, "gait", 8192, NULL, 22, NULL, 1);
}