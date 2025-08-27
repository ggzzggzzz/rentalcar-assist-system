#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <time.h>
#include <pthread.h>

#define BUF_SIZE 100
#define KEY_UP 3
#define KEY_DOWN 4
#define KEY_LEFT 5
#define KEY_RIGHT 6
#define SEL 16
#define ADC_CS 8
#define ADC_CH4 4
#define ADC_CH5 5
#define SPI_CH 0
#define SPI_SPEED 500000

#define NEUTRAL_V 2024
#define NEUTRAL_H 2024
#define DEADZONE 1000

#define PIR_D 2
#define BUZZER 15

extern int sock;

char pir_flag = 0;
pthread_t pir_thread;
pthread_t server_thread;
pthread_t ultrasonic_thread;
volatile int is_pir_active = 0;  // PIR 센서 활성화 상태
volatile int is_ultrasonic_active = 0;  // 초음파 센서 활성화 상태

void PIR_interrupt() {
    pir_flag = 1;
}

void getCurrentTime(char* timeStr) {
    time_t now = time(NULL);
    struct tm* t = localtime(&now);
    strftime(timeStr, 20, "%Y-%m-%d %H:%M:%S", t);
}

typedef enum {
    GEAR_P = 0,
    GEAR_R,
    GEAR_N,
    GEAR_D,
    GEAR_MAX
} GearState;

char adChannel;
const char* gearNames[] = { "P", "R", "N", "D" };
GearState currentGear = GEAR_P;
int sock;
char message[BUF_SIZE];

void error_handling(char* message) {
    fputs(message, stderr);
    fputc('\n', stderr);
    exit(1);
}

int Joy_V() {
    int adcValue = 0;
    unsigned char buf[3];
    adChannel = ADC_CH4;
    buf[0] = 0x06 | ((adChannel & 0x07) >> 2);
    buf[1] = ((adChannel & 0x07) << 6);
    buf[2] = 0x00;
    digitalWrite(ADC_CS, 0);
    wiringPiSPIDataRW(SPI_CH, buf, 3);
    buf[1] = 0x0F & buf[1];
    adcValue = (buf[1] << 8) | buf[2];
    digitalWrite(ADC_CS, 1);
    return adcValue;
}

int Joy_H() {
    int adcValue = 0;
    unsigned char buf[3];
    adChannel = ADC_CH5;
    buf[0] = 0x06 | ((adChannel & 0x07) >> 2);
    buf[1] = ((adChannel & 0x07) << 6);
    buf[2] = 0x00;
    digitalWrite(ADC_CS, 0);
    wiringPiSPIDataRW(SPI_CH, buf, 3);
    buf[1] = 0x0F & buf[1];
    adcValue = (buf[1] << 8) | buf[2];
    digitalWrite(ADC_CS, 1);
    return adcValue;
}

void* pir_sensor_thread(void* arg) {
    char message[BUF_SIZE];
    char timeStr[20];

    while (1) {
        if (is_pir_active && pir_flag == 1) {  // 활성화 상태일 때만 동작
            getCurrentTime(timeStr);
            sprintf(message, "PIR/Detected/%s", timeStr);
            write(sock, message, strlen(message));
            printf("Send to server: %s\n", message);

            // 부저 동작
            printf("BUZZER ON !!!\n");
            digitalWrite(BUZZER, HIGH);
            sleep(3);
            digitalWrite(BUZZER, LOW);
            printf("BUZZER OFF !!!\n");

            pir_flag = 0;
        }
        usleep(100000);
    }
    return NULL;
}

void* server_response_thread(void* arg) {
    char message[BUF_SIZE];
    int str_len;

    while (1) {
        str_len = read(sock, message, BUF_SIZE - 1);
        if (str_len > 0) {
            message[str_len] = '\0';
            printf("Received from server: %s\n", message);
            
            // PIR 센서 제어 메시지 처리
            if (strncmp(message, "PIR SENSOR ACTIVATION", strlen("PIR SENSOR ACTIVATION")) == 0) {
                is_pir_active = 1;
                printf("PIR sensor activated\n");
            }
            else if (strncmp(message, "PIR SENSOR DEACTIVATION", strlen("PIR SENSOR DEACTIVATION")) == 0) {
                is_pir_active = 0;
                printf("PIR sensor deactivated\n");
            }
            // 초음파 센서 제어 메시지 처리
            else if (strncmp(message, "ULTRASONIC SENSOR ACTIVATION", strlen("ULTRASONIC SENSOR ACTIVATION")) == 0) {
                is_ultrasonic_active = 1;
                printf("Ultrasonic sensor activated\n");
            }
            else if (strncmp(message, "ULTRASONIC SENSOR DEACTIVATION", strlen("ULTRASONIC SENSOR DEACTIVATION")) == 0) {
                is_ultrasonic_active = 0;
                printf("Ultrasonic sensor deactivated\n");
                
                
            }
        }
        usleep(100000);
    }
    return NULL;
}

void* ultrasonic_sensor_thread(void* arg) {
    char message[BUF_SIZE];
    char timeStr[20];
    long startTime, travelTime;
    int distance;

    while (1) {
        if (is_ultrasonic_active) {  // 초음파 센서가 활성화된 경우에만 동작
            

            getCurrentTime(timeStr);
            sprintf(message, "ULTSONIC/170/%s", timeStr);
            int str_len = write(sock, message, strlen(message));
            if (str_len > 0) {
                printf("Send to server: %s\n", message);
            }
        }
        else {
             // 초음파 센서가 비활성화 상태임을 알림
        }
        sleep(1);
    }
    return NULL;
}


int main(int argc, char* argv[]) {
    struct sockaddr_in serv_addr;
    int str_len;
    int ret = 0; 
    int v_value, h_value;
    int prev_v_state = 0;
    int prev_h_state = 0;
    int prev_sel_state = 1;

    if (argc != 3) {
        printf("Usage : %s <IP> <port>\n", argv[0]);
        exit(1);
    }

    // GPIO 초기화
    if (wiringPiSetup() == -1)
        return 1;

    pinMode(KEY_UP, INPUT);
    pinMode(KEY_DOWN, INPUT);
    pinMode(KEY_LEFT, INPUT);
    pinMode(KEY_RIGHT, INPUT);
    pinMode(SEL, INPUT);
    pinMode(ADC_CS, OUTPUT);
    pinMode(PIR_D, INPUT);
    pinMode(BUZZER, OUTPUT);

    wiringPiISR(PIR_D, INT_EDGE_RISING, &PIR_interrupt);

    if (wiringPiSPISetup(SPI_CH, SPI_SPEED) == -1) {
        printf("wiringPi SPI Setup failed!\n");
        exit(0);
    }

    // 소켓 생성 및 서버 연결
    sock = socket(PF_INET, SOCK_STREAM, 0);
    if (sock == -1)
        error_handling("socket() error");

    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = inet_addr(argv[1]);
    serv_addr.sin_port = htons(atoi(argv[2]));

    if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) == -1)
        error_handling("connect() error");

    // 컨트롤러 역할 전송
    strcpy(message, "ROLE:CONTROLLER");
    write(sock, message, strlen(message));

    printf("Connected to server. Controller ready.\n");
    printf("Current Gear: %s\n", gearNames[currentGear]);

    pthread_create(&server_thread, NULL, server_response_thread, NULL);
    pthread_create(&pir_thread, NULL, pir_sensor_thread, NULL);
    pthread_create(&ultrasonic_thread, NULL, ultrasonic_sensor_thread, NULL);

    while (1) {
        char timeStr[20];
        ret = digitalRead(SEL);
        if (ret == 0 && prev_sel_state == 1) {
            getCurrentTime(timeStr);
            sprintf(message, "JOYSTICK/SEL/%s", timeStr);
            write(sock, message, strlen(message));
            printf("Send to server: %s\n", message);
        }
        prev_sel_state = ret;

        v_value = Joy_V();
        h_value = Joy_H();

        int current_v_state = 0;
        int current_h_state = 0;

        if (v_value > NEUTRAL_V + DEADZONE)
            current_v_state = 1;
        else if (v_value < NEUTRAL_V - DEADZONE)
            current_v_state = -1;

        if (h_value > NEUTRAL_H + DEADZONE)
            current_h_state = 1;
        else if (h_value < NEUTRAL_H - DEADZONE)
            current_h_state = -1;

        // 기어 변경 처리
        if (current_v_state != prev_v_state) {
            if (current_v_state == 1) {
                if (currentGear > GEAR_P) {
                    currentGear--;
                    getCurrentTime(timeStr);
                    sprintf(message, "JOYSTICK/%s/%s", gearNames[currentGear], timeStr);
                    write(sock, message, strlen(message));
                    printf("Send to server: %s\n", message);
                }
            }
            else if (current_v_state == -1) {
                if (currentGear < GEAR_D) {
                    currentGear++;
                    getCurrentTime(timeStr);
                    sprintf(message, "JOYSTICK/%s/%s", gearNames[currentGear], timeStr);
                    write(sock, message, strlen(message));
                    printf("Send to server: %s\n", message);
                }
            }
        }

        // 잠금/해제 처리
        if (current_h_state != prev_h_state) {
            if (current_h_state == 1) {
                getCurrentTime(timeStr);
                sprintf(message, "JOYSTICK/LOCK/%s", timeStr);
                write(sock, message, strlen(message));
                printf("Send to server: %s\n", message);
            }
            else if (current_h_state == -1) {
                getCurrentTime(timeStr);
                sprintf(message, "JOYSTICK/UNLOCK/%s", timeStr);
                write(sock, message, strlen(message));
                printf("Send to server: %s\n", message);
            }
        }

        prev_v_state = current_v_state;
        prev_h_state = current_h_state;

        usleep(500000);
    }

    close(sock);
    return 0;
}
