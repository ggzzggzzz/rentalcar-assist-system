#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <wiringPi.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <time.h>
#include <pthread.h>

#define BUF_SIZE 100
#define BUZZER 15
#define DC_INA_PIN 26
#define DC_INB_PIN 23
#define PIR_D 2

#define TRIG 28
#define ECHO 29
extern int sock;
volatile int is_pir_active = 0;  
volatile int is_ultrasonic_active = 0;

void getCurrentTime(char* timeStr) {
    time_t now = time(NULL);
    struct tm* t = localtime(&now);
    strftime(timeStr, 20, "%Y-%m-%d %H:%M:%S", t);
}

pthread_t ultrasonic_thread;

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
        }
        else {
             // 초음파 센서가 비활성화 상태임을 알림
        }
        sleep(1);
    }
    return NULL;
}



// PIR 센서 관련 전역 변수
volatile char pir_flag = 0;
pthread_t pir_thread;
int sock;

// PIR 인터럽트 핸들러
void PIR_interrupt() {
    pir_flag = 1;
}

// PIR 센서 스레드 함수
void* pir_sensor_thread(void* arg) {
    char msg[BUF_SIZE];
    char timeStr[20];

    while (1) {
        if (is_pir_active) {  // PIR 센서가 활성화된 경우에만 동작
            if (pir_flag == 1) {
                getCurrentTime(timeStr);
                sprintf(msg, "PIR/Detected/%s", timeStr);
                write(sock, msg, strlen(msg));
                printf("Send to server: %s\n", msg);

                // 부저 동작
                printf("BUZZER ON !!!\n");
                digitalWrite(BUZZER, HIGH);
                sleep(3);
                digitalWrite(BUZZER, LOW);
                printf("BUZZER OFF !!!\n");

                pir_flag = 0;
            }
        }
        else {
             // PIR 센서가 비활성화 상태임을 알림
        }
        usleep(100000);
    }
    return NULL;
}
void error_handling(char* message) {
    fputs(message, stderr);
    fputc('\n', stderr);
    exit(1);
}

// DC 모터 제어 함수들
void dc_forward(int state) {
    digitalWrite(DC_INB_PIN, LOW);
    digitalWrite(DC_INA_PIN, state ? HIGH : LOW);
}

void dc_backward(int state) {
    digitalWrite(DC_INA_PIN, LOW);
    digitalWrite(DC_INB_PIN, state ? HIGH : LOW);
}

void dc_stop() {
    digitalWrite(DC_INA_PIN, LOW);
    digitalWrite(DC_INB_PIN, LOW);
}

int main(int argc, char* argv[]) {
    struct sockaddr_in serv_addr;
    char message[BUF_SIZE];
    int str_len;

    if (argc != 3) {
        printf("Usage : %s <IP> <port>\n", argv[0]);
        exit(1);
    }

    // WiringPi 초기화
    if (wiringPiSetup() == -1)
        return 1;

    // GPIO 핀 설정
    pinMode(BUZZER, OUTPUT);
    pinMode(DC_INA_PIN, OUTPUT);
    pinMode(DC_INB_PIN, OUTPUT);
    pinMode(PIR_D, INPUT);  
    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);

    // PIR 인터럽트 설정
    wiringPiISR(PIR_D, INT_EDGE_RISING, &PIR_interrupt);

    // 초기 상태 설정
    digitalWrite(BUZZER, LOW);
    dc_stop();

    // 소켓 생성 및 연결
    sock = socket(PF_INET, SOCK_STREAM, 0);
    if (sock == -1)
        error_handling("socket() error");

    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = inet_addr(argv[1]);
    serv_addr.sin_port = htons(atoi(argv[2]));

    if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) == -1)
        error_handling("connect() error");

    // ROLE 전송
    write(sock, "ROLE:ACTUATOR", strlen("ROLE:ACTUATOR"));
    printf("Connected to server as ACTUATOR\n");

    // PIR 센서 스레드 생성
    pthread_create(&pir_thread, NULL, pir_sensor_thread, NULL);
    pthread_create(&ultrasonic_thread, NULL, ultrasonic_sensor_thread, NULL);

    while (1) {
        str_len = read(sock, message, BUF_SIZE - 1);
        if (str_len <= 0)
            break;

        message[str_len] = '\0';
        printf("Received command: %s\n", message);

        if (strncmp(message, "BUZZER: LEVEL 2", strlen("BUZZER: LEVEL 2")) == 0) {
            digitalWrite(BUZZER, HIGH);
        }
        else if (strncmp(message, "BUZZER: LEVEL 1", strlen("BUZZER: LEVEL 1")) == 0) {
            digitalWrite(BUZZER, HIGH);
            usleep(500000);
            digitalWrite(BUZZER, LOW);
            usleep(500000);
        }
        else if (strncmp(message, "BUZZER: OFF", strlen("BUZZER: OFF")) == 0) {
            digitalWrite(BUZZER, LOW);
        }
        else if (strncmp(message, "Temperature action: HEATING", strlen("Temperature action: HEATING")) == 0) {
            printf("Heating mode - Moving forward\n");
            dc_forward(1);
            usleep(500000);  // 0.5초 대기
            dc_stop();
        }
        else if (strncmp(message, "Temperature action: COOLING", strlen("Temperature action: COOLING")) == 0) {
            printf("Cooling mode - Moving backward\n");
            dc_backward(1);
            usleep(500000);  // 0.5초 대기
            dc_stop();
        }
        else if (strncmp(message, "NO ACTION", strlen("NO ACTION")) == 0) {
            printf("Normal temperature - Stopping motor\n");
            dc_stop();
        }
        else if (strncmp(message, "PIR SENSOR ACTIVATION",strlen("PIR SENSOR ACTIVATION")) == 0) {
            is_pir_active = 1;
            printf("PIR sensor activated\n");
        }
        else if (strncmp(message, "PIR SENSOR DEACTIVATION",strlen("PIR SENSOR DEACTIVATION")) == 0) {
            is_pir_active = 0;
            printf("PIR sensor deactivated\n");
        }
        else if (strncmp(message, "ULTRASONIC SENSOR ACTIVATION",strlen("ULTRASONIC SENSOR ACTIVATION")) == 0) {
            is_ultrasonic_active = 1;
            printf("Ultrasonic sensor activated\n");
        }
        else if (strncmp(message, "ULTRASONIC SENSOR DEACTIVATION",strlen("ULTRASONIC SENSOR DEACTIVATION")) == 0) {
            is_ultrasonic_active = 0;
            printf("Ultrasonic sensor deactivated\n");
        }
    }

    // 연결 종료 시 정리
    digitalWrite(BUZZER, LOW);
    dc_stop();
    close(sock);
    return 0;
}
