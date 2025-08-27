#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <pthread.h>
#include <time.h>

#define BUF_SIZE 100
#define SPI_CH 0
#define SPI_SPEED 500000

// ADC 관련 정의
#define ADC_CS 8
#define ADC_MISO 13
#define ADC_MOSI 12
#define ADC_SCLK 14
#define GAS_CH 6

// PIR 센서 정의
#define PIR_D 2

// 초음파 센서 정의
#define TRIG 28
#define ECHO 29


// DHT11 센서 정의
#define DHT11PIN 25
#define MAX_TIME 100
int dht11_val[5]={0,0,0,0,0};
int dht11_temp[5]={0,0,0,0,0};
float farenheit_temp;

// 부저 정의 
#define BUZZER 15


// 전역 변수
int sock;
char pir_flag = 0;
volatile int is_pir_active = 0;  // PIR 센서 활성화 상태
volatile int is_ultrasonic_active = 0;  // 초음파 센서 활성화 상태
int is_temp_active = 0; //온도센서 활성화 상태

void error_handling(char* message);
void getCurrentTime(char* timeStr);
int mcp3208(int channel);

// PIR 인터럽트 핸들러
void PIR_interrupt() {
    pir_flag = 1;
}


// 가스 센서 스레드 함수
void* gas_sensor_thread(void* arg) {
    char message[BUF_SIZE];
    char timeStr[20];

    while (1) {
        if(is_ultrasonic_active){
        }
        else{
        int gas_value = mcp3208(GAS_CH);
        getCurrentTime(timeStr);
        sprintf(message, "GAS/%d/%s", gas_value, timeStr);
        write(sock, message, strlen(message));
        printf("Send to server: %s\n", message);  // 전송 확인용 출력
        delay(1000);  // 1초 간격으로 전송
    }
    }
    return NULL;
}

// PIR 센서 스레드 함수
void* pir_sensor_thread(void* arg) {
    char message[BUF_SIZE];
    char timeStr[20];

    while (1) {
        if (is_pir_active) {  // PIR 센서가 활성화된 경우에만 동작
            if (pir_flag == 1) {
                getCurrentTime(timeStr);
                sprintf(message, "PIR/Detected/%s", timeStr);
                write(sock, message, strlen(message));
                printf("Send to server: %s\n", message);
                printf("PIR Detected !!\n");

                // 부저 동작
                printf("BUZZER ON !!!\n");
                digitalWrite(BUZZER, HIGH);
                sleep(3);  // 3초 동안 부저 ON
                digitalWrite(BUZZER, LOW);
                printf("BUZZER OFF !!!\n");

                pir_flag = 0;
            }
            else {
                printf("PIR Not detect !!\n");
            }
        }
        else {
             // PIR 센서가 비활성화 상태임을 알림
        }
        usleep(1000000);
    }
    return NULL;
}
// 초음파 센서 스레드 함수
void* ultrasonic_sensor_thread(void* arg) {
    char message[BUF_SIZE];
    char timeStr[20];
    long startTime, travelTime;
    int distance;

    while (1) {
        if (is_ultrasonic_active) {  // 초음파 센서가 활성화된 경우에만 동작
            digitalWrite(TRIG, LOW);
            usleep(2);
            digitalWrite(TRIG, HIGH);
            usleep(20);
            digitalWrite(TRIG, LOW);

            while (digitalRead(ECHO) == LOW);
            startTime = micros();
            while (digitalRead(ECHO) == HIGH);
            travelTime = micros() - startTime;

            distance = travelTime * 17 / 1000;

            getCurrentTime(timeStr);
            sprintf(message, "ULTSONIC/%d/%s", distance, timeStr);
            int str_len = write(sock, message, strlen(message));
            if (str_len > 0) {
                printf("Send to server: %s\n", message);
            }
           // write(sock, message, strlen(message));
            //printf("Send to server: %s\n", message);
        }
        else {
              // 초음파 센서가 비활성화 상태임을 알림
        }
        sleep(1);
    }
    return NULL;
}

//온도센서
void dht11_read_val(float *celsius)  
{  
    unsigned char lststate=HIGH;  
    unsigned char counter=0;  
    unsigned char j=0,i;  
    float farenheit;  
    
    for(i=0;i<5;i++)  
        dht11_val[i]=0;  
    
    pinMode(DHT11PIN,OUTPUT);  
    digitalWrite(DHT11PIN,0);  
    delay(18);  
    digitalWrite(DHT11PIN,1);  
    delayMicroseconds(40);  
    pinMode(DHT11PIN,INPUT);
    
    for(i=0;i<MAX_TIME;i++)  
    {  
        counter=0;  
        while(digitalRead(DHT11PIN)==lststate){  
            counter++;  
            delayMicroseconds(1);  
            if(counter==255)  
                break;  
        }  
        lststate=digitalRead(DHT11PIN);  
        if(counter==255)  
            break;  
        if((i>=4)&&(i%2==0)){  
            dht11_val[j/8]<<=1;  
            if(counter>16)  
                dht11_val[j/8]|=1;  
            j++;  
        }  
    }  
  
    if((j>=40)&&(dht11_val[4]==((dht11_val[0]+dht11_val[1]+dht11_val[2]+dht11_val[3])& 0xFF))){  
        *celsius = dht11_val[2] + dht11_val[3] * 0.1;
        for(i=0; i<5; i++)
            dht11_temp[i] = dht11_val[i];
    }else {
        *celsius = dht11_temp[2] + dht11_temp[3] * 0.1;
    }
}

void* temperature_sensor_thread(void* arg) {
    char message[BUF_SIZE];
    char timeStr[20];
    float temperature;

    while(1) {
        if(is_ultrasonic_active)
        {}
        else{
        dht11_read_val(&temperature);
        getCurrentTime(timeStr);
        sprintf(message, "TEMP/%.1f/%s", temperature, timeStr);
        write(sock, message, strlen(message));
        printf("Send to server: %s\n", message);
        sleep(1);
    }
    }
    return NULL;
}
int main(int argc, char* argv[]) {
    struct sockaddr_in serv_addr;
    pthread_t gas_thread, pir_thread, ultrasonic_thread, temp_thread;

    char message[BUF_SIZE];
    
    if(argc != 3) {
        printf("Usage : %s <IP> <port>\n", argv[0]);
        exit(1);
    }
    
    // WiringPi 초기화
    if(wiringPiSetup() == -1)
        return 1;
        
    // 핀 모드 설정
    pinMode(ADC_CS, OUTPUT);
    pinMode(PIR_D, INPUT);
    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);
    pinMode(BUZZER, OUTPUT);  
    
    // PIR 인터럽트 설정
    wiringPiISR(PIR_D, INT_EDGE_RISING, &PIR_interrupt);
    
    // SPI 설정
    if(wiringPiSPISetup(SPI_CH, SPI_SPEED) == -1) {
        printf("wiringPi SPI Setup failed!\n");
        exit(0);
    }
    
    // 소켓 생성 및 연결
    sock = socket(PF_INET, SOCK_STREAM, 0);
    if(sock == -1)
        error_handling("socket() error");
    
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = inet_addr(argv[1]);
    serv_addr.sin_port = htons(atoi(argv[2]));
    
    if(connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) == -1)
        error_handling("connect() error");
        
    // ROLE 전송
    write(sock, "ROLE:SENSOR", strlen("ROLE:SENSOR"));
    
    // 센서 스레드 생성
    pthread_create(&gas_thread, NULL, gas_sensor_thread, NULL);
    pthread_create(&pir_thread, NULL, pir_sensor_thread, NULL);
    pthread_create(&ultrasonic_thread, NULL, ultrasonic_sensor_thread, NULL);
    pthread_create(&temp_thread, NULL, temperature_sensor_thread, NULL);
    
    // 서버로부터 메시지 수신
    while(1) {
        memset(message, 0, BUF_SIZE);
        int str_len = read(sock, message, BUF_SIZE - 1);
        if(str_len <= 0)
            break;
            
        message[str_len] = '\0';
        printf("Received command: %s\n", message);
        // 서버로부터 받은 명령 처리
        if (strncmp(message, "PIR SENSOR ACTIVATION", strlen("PIR SENSOR ACTIVATION")) == 0) {
            is_pir_active = 1;
            printf("PIR sensor activated\n");
        }
        else if (strncmp(message, "PIR SENSOR DEACTIVATION", strlen("PIR SENSOR DEACTIVATION")) == 0) {
            is_pir_active = 0;
            printf("PIR sensor deactivated\n");
        }
        else if (strncmp(message, "ULTRASONIC SENSOR ACTIVATION", strlen("ULTRASONIC SENSOR ACTIVATION")) == 0) {
            is_ultrasonic_active = 1;
            printf("Ultrasonic sensor activated\n");
        }
        else if (strncmp(message, "ULTRASONIC SENSOR DEACTIVATION", strlen("ULTRASONIC SENSOR DEACTIVATION")) == 0) {
            is_ultrasonic_active = 0;
            printf("Ultrasonic sensor deactivated\n");
        }
    }
    
    close(sock);
    return 0;
}

// ADC 제어 함수
int mcp3208(int channel) {
    int adc_value = 0;
    unsigned char buf[3];
    
    buf[0] = 0x06 | ((channel & 0x07)>>2);
    buf[1] = ((channel & 0x07)<<6);
    buf[2] = 0x00;
    
    digitalWrite(ADC_CS, LOW);
    wiringPiSPIDataRW(SPI_CH, buf, 3);
    buf[1] = 0x0F & buf[1];
    adc_value = (buf[1] << 8) | buf[2];
    digitalWrite(ADC_CS, HIGH);
    
    return adc_value;
}

// 현재 시간 문자열 반환 함수
void getCurrentTime(char* timeStr) {
    time_t now = time(NULL);
    struct tm* t = localtime(&now);
    strftime(timeStr, 20, "%Y-%m-%d %H:%M:%S", t);
}

void error_handling(char* message) {
    fputs(message, stderr);
    fputc('\n', stderr);
    exit(1);
}
