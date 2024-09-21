/*
programa: Servidor OBD II para el diagnostico de un vehículo mediante CAN
Desarrollado por: Juan Diego Restrepo

Descripción: El servidor se encarga de recibir comandos del estandar OBD II, procesarlos y enviar una respuesta ante una solicitud

 */

#include "driver/gpio.h"
#include "driver/twai.h"
#include <map>
#include <vector>

const int POT1 = 34; //pin 34 VELOCIMETRO
const int POT2 = 35; //pin 35 TACOMETRO 
const int MIL = 26; //luz MIL
const int DTC = 27; // indicador de DTC


volatile bool ledState = false;
volatile bool flagDtc = false;

//variable para almacenar la lectura en digital del adc
int vel,rev,revoluciones;
float x;
uint8_t velocimetro;
uint8_t A_rev, B_rev;

std::vector<uint8_t*> frame_errors_all;
int index_frame_error = 0;

unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 100; // 100ms para antirrebote

void setup(){
  Serial.begin(115200);
     pinMode(POT1, INPUT);
     pinMode(POT2, INPUT);
     pinMode(MIL, OUTPUT);
     pinMode(DTC,INPUT_PULLUP);

     attachInterrupt(digitalPinToInterrupt(DTC), interrup_DTC, FALLING);

    //Initialize configuration structures using macro initializers
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_5, GPIO_NUM_4, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    //Install TWAI driver
    bool driver_installed=false;
    bool attemps=0;
    while(!driver_installed && attemps < 3){
      if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
          driver_installed=true;
          printf("Driver installed\n");
      }
      attemps++;
    }
    if(attemps==3){
        printf("Failed to install driver\n");
        return;
    }

    //Start TWAI driver
    bool driver_started=false;
    bool attemps_start=0;
    while(!driver_started && attemps_start < 3){
      if (twai_start() == ESP_OK) {
          driver_started=true;
          printf("Driver started\n");
      } 
      attemps_start++;
    }
     if(attemps_start==3){
         printf("Failed to start driver\n");
          return;
    }

    

}

void loop() {

  //Wait for message to be received
twai_message_t message;
if (twai_receive(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    printf("Message received\n");
} else {
    printf("Failed to receive message\n");
    return;
}

// llamar funcion de dtc cuando se presione el boton
if(flagDtc == true){
uint8_t* frame_error = generate_error();
if(frame_error!=nullptr){
  frame_errors_all.push_back(frame_error);
  printf("frame_error size");
}
}

//Process received message
if (message.extd) {
    printf("Message is in Extended Format\n");
} else {
   printf("Message is in Standard Format\n");
}
printf("ID is %d\n", message.identifier);
if (!(message.rtr)) {
    for (int i = 0; i < message.data_length_code; i++) {
        printf("Data byte %d = %d\n", i, message.data[i]);
    }
    process_message(message);
}
}


void mode_01(twai_message_t message_rec,twai_message_t message_pro){
  uint8_t pid = message_rec.data[2];

  if(pid==0){
    uint8_t frame_response[8]={6,65,0,152,26,0,18,0};
    transmit_messsage(message_pro, frame_response);
  }else if(pid==1){
    uint8_t frame_response[8]={6,65,1,0,4,0,0,0};
    transmit_messsage(message_pro, frame_response);

  }else if(pid==28){
    uint8_t frame_response[8]={3,65,28,9,0,0,0,0};
    transmit_messsage(message_pro, frame_response);
  }
  else if(pid==12){
       respond_revolutions(message_pro);
  }
  else if(pid==13){
      respond_speed(message_pro);
  }
}

void mode_03(twai_message_t message_rec,twai_message_t msg){

  printf("responde dtc");
  if(frame_errors_all.empty()){
    uint8_t frame_response[8]={2,67,0,0,0,0,0,0};
    transmit_messsage(msg, frame_response);
  }else{
    for (uint8_t* frame_error : frame_errors_all) {
      transmit_messsage(msg, frame_error);
    }
  }

}
void process_message(twai_message_t message_rec){
  twai_message_t message_pro;
  message_pro.identifier = 0x7E9;
  message_pro.extd = 0;
  message_pro.data_length_code = 8;

//tramas de solicitud de conexión
  uint8_t frame_3[8] ={2,6,0,0,0,0,0,0};
  uint8_t frame_4[8] ={2,8,0,0,0,0,0,0};
  uint8_t frame_5[8] ={2,9,0,0,0,0,0,0};
  uint8_t frame_8[8] ={1,7,0,0,0,0,0,0};
  uint8_t frame_9[8] ={3,2,2,0,0,0,0,0};
  
  if(message_rec.data[1] == 1){
    mode_01(message_rec,message_pro);
  }else if(message_rec.data[1] == 3){
    mode_03(message_rec,message_pro);
  }
  else if(compare_msg(frame_3, message_rec.data)==true){
    message_pro.identifier = 0x7E8;
    uint8_t frame_response[8]={6,70,0,192,0,0,1,0};
    transmit_messsage(message_pro, frame_response);

  }else if(compare_msg(frame_4, message_rec.data)==true){
    message_pro.identifier = 0x7E8;
    uint8_t frame_response[8]={6,72,0,0,0,0,0,0};
    transmit_messsage(message_pro, frame_response);

  }else if(compare_msg(frame_5, message_rec.data)==true){
    uint8_t frame_response[8]={6,73,0,20,64,0,0,0};
    transmit_messsage(message_pro, frame_response);
  }
  else if(compare_msg(frame_8, message_rec.data)==true){
    uint8_t frame_response[8]={2,71,0,0,0,0,0,0};
    transmit_messsage(message_pro, frame_response);
  }
  else if(compare_msg(frame_9, message_rec.data)==true){
    uint8_t frame_response[8]={5,66,2,0,0,0,0,0};
    transmit_messsage(message_pro, frame_response);
  }
}
void respond_revolutions(twai_message_t msg){
    x=0;
    A_rev=0;
    B_rev=0;
    rev = analogRead (POT2);
    revoluciones = map(rev,0,4095,0,10000);
    while( x<= revoluciones){
      A_rev ++;
      x=x+64; 
    }
    A_rev--;
    B_rev=(4*revoluciones)-(256*A_rev);
    delay(100);
    uint8_t frame_response_rpm[8]={4,65,12,A_rev,B_rev,0,0,0};      // revoluciones del motor
    transmit_messsage(msg, frame_response_rpm);
}
void respond_speed(twai_message_t msg){
    vel = analogRead (POT1);
    velocimetro = map(vel,0,4095,0,255);
    delay(100);
    uint8_t frame_response[8]={4,65,13,velocimetro,0,0,0,0};  // velocidad del vehiculo
    transmit_messsage(msg, frame_response);
}

uint8_t* generate_error(){
  frame_errors_all.clear();
  static uint8_t frame_error_1[8] = {4, 67, 1, 7, 4, 0, 0, 0}; 
  static uint8_t frame_error_2[8] = {6, 67, 2, 7, 4, 7, 5, 0};
 
  static uint8_t* frame_errors[] = {frame_error_1, frame_error_2};
  uint8_t* selected_frame_error = nullptr;
  
  if(index_frame_error<2){
    ledState = true;
    digitalWrite(MIL, ledState);
    selected_frame_error = frame_errors[index_frame_error];
    index_frame_error++;
  }else{
    ledState = false;
    digitalWrite(MIL, ledState);
    index_frame_error=0;
  }
  flagDtc=false;
  return selected_frame_error;
  
}

void transmit_messsage(twai_message_t msg, uint8_t frame[]){
    for (int i = 0; i < msg.data_length_code; i++) {
      msg.data[i] = frame[i];  
    }

    if (twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK) {
      printf("Message queued for transmission\n");
    } else {
        printf("Failed to queue message for transmission\n");
    }
    
}

bool compare_msg(uint8_t msg_rec[], uint8_t msg[]){
  bool eq=true;
  for (int i = 0; i < sizeof(msg_rec); i++) {
    if(msg_rec[i]!=msg[i]){
      eq=false;
    }
  }
  return eq;
}

void interrup_DTC(){
    if ((millis() - lastDebounceTime) > debounceDelay) {
      lastDebounceTime = millis();
      flagDtc = true;
  }
 
}