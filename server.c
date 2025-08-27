#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <time.h>
#include <errno.h>
#include <sys/file.h> // flock 사용
#include <sys/select.h> // select() 함수와 관련된 헤더
#include <fcntl.h>      // 파일 열기 함수
#include <stdbool.h>

#define BUF_SIZE 100
#define MAX_CLIENTS 3
#define PIR_LOG_FILE "pir_sensor_log.txt"  // PIR 센서 로그 파일 이름
#define GAS_LOG_FILE "gas_sensor_log.txt"  // 가스 센서 로그 파일 이름
#define JOYSTICK_LOG_FILE "joystick_log.txt" // 조이스틱 로그 파일 이름
#define TEMP_LOG_FILE "temperature_log.txt" // 온도 로그 파일 이름
#define ULTRA_LOG_FILE "ultrasonic_log.txt" // 초음파 로그 파일 이름
#define HEATING "HEATING"
#define COOLING "COOLING"
#define NO_ACTION "NO ACTION"
#define ULTRASONIC_CMD_ACTIVATE "ULTRASONIC SENSOR: ACTIVATE"
#define ULTRASONIC_CMD_DEACTIVATE "ULTRASONIC SENSOR: DEACTIVATE"
#define BUZZER_CMD_LEVEL_2 "BUZZER: LEVEL 2"
#define BUZZER_CMD_LEVEL_1 "BUZZER: LEVEL 1"
#define BUZZER_CMD_OFF "BUZZER: OFF"

// 에러 메시지 출력 후 종료
void error_handling(char *message);

// 자식 프로세스 종료 처리
void read_childproc(int sig);

// 로그 파일에 로그 메세지 저장
void log_message(const char *log_file, const char *message);

// 파이프 생성 및 초기화 
void create_pipes(); 

// select() 초기화
void initialize_resources();

// 모든 클라이언트 연결 확인  
int check_all_clients_connected();


// 전역 변수
int clnt_sock[MAX_CLIENTS] = {-1, -1, -1};
const char *clnt_name[MAX_CLIENTS] = {"SENSOR", "CONTROLLER", "ACTURATOR"};
char last_action[20] = NO_ACTION;       // 마지막 냉난방 상태
int buzzer_msg_send = 0;
int distances[MAX_CLIENTS] = {999, 999, 999}; // 거리 데이터 초기화
int min_distance = 999; // 최소 거리 저장
int closest_client_idx = -1; // 가장 가까운 클라이언트 인덱스 저장
int pipes[MAX_CLIENTS][2]; // Pipe용 파일 디스크립터 (0: 읽기, 1: 쓰기)
int stdin_fd;  // stdin 파일 디스크립터
int dummy_fd;  // 더미 파일 디스크립터
int first = 0;
int first2 = 0;



int main(int argc, char *argv[]) {
    int serv_sock; // 서버소켓 선언
    struct sockaddr_in serv_adr, clnt_adr;                // 서버, 클라이언트 주소 구조체
    pid_t pids[MAX_CLIENTS];                              // 자식 프로세스 ID 저장배열 
    struct sigaction act;                                 // 시그널 액션 구조체
    socklen_t adr_sz;                                     // 클라이언트 주소 크기
    int str_len;                                          // 수신된 메시지 길이
    char buf[BUF_SIZE];                                   // 메시지 버퍼
    char ipc_buf[BUF_SIZE];                      // ipc 메시지 버퍼
    char lock_state[10] = "UNLOCK";                      // 현재 잠금 상태
    char prev_lock_state[10] = "UNLOCK";        // 이전 잠금 상태 
    char gear_state[10] = "P";                      // 현재 기어 상태
    char prev_gear_state[10] = "P";        // 이전 기어 상태 
    int clnt_idx = -1;               //클라이언트 및 자식프로세스 식별 번호 
    
    // 명령행 인자 확인
    if (argc != 2) {
        printf("Usage : %s <port>\n", argv[0]);
        exit(1);
    }

    // SIGCHLD 시그널 처리 설정 (자식 프로세스 종료 시 처리)
    act.sa_handler = read_childproc;
    sigemptyset(&act.sa_mask);
    act.sa_flags = 0;
    sigaction(SIGCHLD, &act, 0);

    // 서버 소켓 생성
    serv_sock = socket(PF_INET, SOCK_STREAM, 0);
    memset(&serv_adr, 0, sizeof(serv_adr));
    serv_adr.sin_family = AF_INET;
    serv_adr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_adr.sin_port = htons(atoi(argv[1]));

    // 소켓 바인딩
    if (bind(serv_sock, (struct sockaddr *)&serv_adr, sizeof(serv_adr)) == -1)
        error_handling("bind() error");

    // 소켓 리스닝
    if (listen(serv_sock, 5) == -1)
        error_handling("listen() error");
        
        create_pipes(); //파이프 생성  
        
   initialize_resources(); //select 초기화 함수 

    printf("Server is running...\n");

    while (1) {
    
    int is_full = check_all_clients_connected(); // 모든 소켓이 꽉 찼는지 확인    
    
    if (!is_full) {
       // 새로운 클라이언트 연결 대기
    adr_sz = sizeof(clnt_adr);
        int new_sock = accept(serv_sock, (struct sockaddr *)&clnt_adr, &adr_sz);

        if (new_sock == -1) {
                perror("accept() error"); // 오류의 경우만 출력
                close(new_sock);
                continue;
        }

    printf("New client connected.\n");
    
    // 클라이언트 역할 식별
    memset(buf, 0, BUF_SIZE);
    int str_len = read(new_sock, buf, BUF_SIZE - 1);
    if (str_len <= 0) {
        if (str_len == 0) {
            printf("Client disconnected before sending role.\n");
        } else {
            perror("Read error during role identification");
        }
        close(new_sock);
        continue;
    }
    buf[str_len] = '\0';
    
    

    // 역할 식별 및 소켓 매핑
    if (strstr(buf, "ROLE:SENSOR") != NULL) {
        if (clnt_sock[0] == -1) {
            clnt_sock[0] = new_sock;
            printf("Client identified as SENSOR.\n");
            clnt_idx = 0;
        } else {
            printf("SENSOR role already assigned. Closing connection.\n");
            close(new_sock);
            continue;
        }
    } else if (strstr(buf, "ROLE:CONTROLLER") != NULL) {
        if (clnt_sock[1] == -1) {
            clnt_sock[1] = new_sock;
            printf("Client identified as CONTROLLER.\n");
            clnt_idx = 1;
        } else {
            printf("CONTROLLER role already assigned. Closing connection.\n");
            close(new_sock);
            continue;
        }
    } else if (strstr(buf, "ROLE:ACTUATOR") != NULL) {
        if (clnt_sock[2] == -1) {
            clnt_sock[2] = new_sock;
            printf("Client identified as ACTUATOR.\n");
            clnt_idx = 2;
        } else {
            printf("ACTUATOR role already assigned. Closing connection.\n");
            close(new_sock);
            continue;
        }
    } else {
        printf("Invalid role message: %s\n", buf);
        close(new_sock);
        continue;
    }
    
    
    pids[clnt_idx] = fork();
    if (pids[clnt_idx] == -1) { //fork 오류시 연결 끊기 
        perror("fork() error");
        close(clnt_sock[clnt_idx]);
        clnt_sock[clnt_idx] = -1;
        continue;
    } 
    
    if (pids[clnt_idx] == 0) { // 자식 프로세스
    close(serv_sock); // 자식 프로세스에서 서버 소켓 닫기
    
   if (clnt_idx == 2) { //2번 클라이언트일 경우 쓰기 끝을 닫는다 
    close(pipes[2][1]); // 파이프의 쓰기 끝 닫기
}

     str_len = read(pipes[clnt_idx][0], ipc_buf, BUF_SIZE - 1); //모든 자식프로세스 동시시작을 위해 부모로부터 
                             //시작 메세지 받을때까지 블로킹 모드로 대기

if (str_len > 0) {
    // 자식프로세스 시작을 알림  
    printf("Child process idx: %d - Starting now.\n", clnt_idx);

    // 클라이언트에게 연결 성공 메시지 전송
    char success_msg[] = "Connection established successfully.\n";
    if (write(clnt_sock[clnt_idx], success_msg, strlen(success_msg)) == -1) {
        perror("Failed to send message to client");
    } else {
        printf("Successfully sent success message to client %d\n", clnt_idx);
    }
} else {
    // IPC 메시지를 받지 못했을 경우 에러 처리
    if (str_len == 0) {
        printf("IPC connection closed by parent for client %d\n", clnt_idx);
    } else {
        perror("Failed to read IPC message");
    }
}

            // 데이터 처리 루프
while ((str_len = read(clnt_sock[clnt_idx], buf, BUF_SIZE - 1)) > 0) {
    buf[str_len] = '\0'; // 수신된 메시지 문자열 종료 처리

                // 가스 센서 메시지 처리
                if (strstr(buf, "GAS") != NULL && clnt_idx==0) {
                 
                    int gas_value = 0;
                    if (sscanf(buf, "GAS/%d", &gas_value) == 1 && gas_value >= 3000) { // 가스 값이 3000 이상인 경우 처리
                    printf("Received from %s client: %s\n", clnt_name[clnt_idx], buf); 
                    
                    log_message(GAS_LOG_FILE, buf); //받은 메세지 로그 저장
                    }
                }

                // 조이스틱 메시지 처리 (잠금 상태 및 기어 상태 업데이트)
if (strstr(buf, "JOYSTICK") != NULL && clnt_idx == 1) {
    // 클라이언트로부터 받은 메시지 출력
    printf("Received message from CONTROLLER client: %s\n", buf);

    // 메시지 파싱 (구분자 '/' 기준)
    char field1[50], field2[50], field3[50];
    int parsed = sscanf(buf, "%49[^/]/%49[^/]/%49s", field1, field2, field3);

    if (parsed < 2) {
        printf("Invalid JOYSTICK message format: %s\n", buf);
        continue; // 잘못된 형식으로 메시지 처리 중단
    }

    // 잠금 상태 처리
    if (strcmp(field2, "LOCK") == 0 || strcmp(field2, "UNLOCK") == 0) {
    if(strcmp(gear_state, "P") == 0){ //기어가 P인 경우에만 처리 
        if (strcmp(lock_state, field2) != 0) { // 상태가 변경된 경우에만 처리
            strcpy(lock_state, field2); //현재 잠금상태 업데이트 

            const char *message = (strcmp(lock_state, "LOCK") == 0) ? 
                                  "PIR SENSOR ACTIVATION\n" : "PIR SENSOR DEACTIVATION\n";

            if (write(pipes[clnt_idx][1], message, strlen(message)) == -1) { //파이프를 통해 부모프로세스에게 메세지 전달 
                perror("Failed to write IPC message");
            } else {
                printf("child process%d message sent: %s\n", clnt_idx, message);
            }

            printf("Lock state updated: %s\n", lock_state);
            strcpy(prev_lock_state, lock_state); //이전 잠금상태 업데이트
        }
    }
    }

    // 기어 상태 처리
    else if (strcmp(field2, "P") == 0 || strcmp(field2, "R") == 0 ||
             strcmp(field2, "N") == 0 || strcmp(field2, "D") == 0) {
             if(strcmp(lock_state, "UNLOCK") == 0){ // 잠금상태가 unlock인 경우에만 처리 
        if (strcmp(gear_state, field2) != 0) { // 상태가 변경된 경우에만 처리
            strcpy(gear_state, field2); //현재 기어상태 업데이트

            if (strcmp(prev_gear_state, "R") != 0 && strcmp(gear_state, "R") == 0) {
                // 기어가 R로 변경된 경우 부모프로세스에게 초음파 센서 활성화 메시지 전송
                char message[] = "ULTRASONIC SENSOR ACTIVATION\n";
                if (write(pipes[clnt_idx][1], message, strlen(message)) == -1) {
                    perror("Failed to write IPC message");
                } else {
                printf("child process%d message sent: %s\n", clnt_idx, message);
            }
                
            } else if (strcmp(prev_gear_state, "R") == 0 && strcmp(gear_state, "R") != 0) {
                // 기어가 R에서 다른 상태로 변경된 경우 초음파 센서 비활성화 메시지 전송
                char message[] = "ULTRASONIC SENSOR DEACTIVATION\n";
                if (write(pipes[clnt_idx][1], message, strlen(message)) == -1) {
                    perror("Failed to write IPC message");
                } else {
               printf("child process%d message sent: %s\n", clnt_idx, message);
            }
                
            }
            printf("Gear state updated: %s\n", gear_state);
            // 이전 기어상태 업데이트
    strcpy(prev_gear_state, gear_state);
        }
        }
    } else {
        printf("Unknown JOYSTICK command: %s\n", field2);
    }

    // 로그 기록
    log_message(JOYSTICK_LOG_FILE, buf);
}


                // PIR 메시지 처리
if (strstr(buf, "PIR") != NULL) {
    
    // 클라이언트로부터 받은 메시지 출력
    printf("Received message from %s client: %s\n", clnt_name[clnt_idx], buf);

    // 로그 파일 열기 및 저장
    log_message(PIR_LOG_FILE, buf);
}

// 온도 메시지 처리
if (strstr(buf, "TEMP") != NULL && clnt_idx == 0) {
    double temperature = 0.0;

    if (sscanf(buf, "TEMP/%lf", &temperature) == 1) { //온도값 파싱 
        // 클라이언트로부터 받은 메시지 출력
        printf("Received message from Sensor client: %s\n", buf);

        // 로그 기록 코드 추가
        log_message(TEMP_LOG_FILE, buf);

        char current_action[20]; // 냉난방 판단용 임시 변수

        if (temperature <= 28.1) { // 온도가 28.1도 이하면 난방 
            strncpy(current_action, HEATING, sizeof(current_action) - 1);
        } else if (temperature >= 28.9) { // 온도가 28.9도 이상이면 냉방 
            strncpy(current_action, COOLING, sizeof(current_action) - 1);
        } else {
            strncpy(current_action, NO_ACTION, sizeof(current_action) - 1);
        }
        current_action[sizeof(current_action) - 1] = '\0'; // 문자열 종료 보장

        // 상태 변경 시 부모 프로세스에 메시지 전송
        if (strcmp(current_action, last_action) != 0) {
            // 마지막 상태 업데이트
            strncpy(last_action, current_action, sizeof(last_action) - 1);
            last_action[sizeof(last_action) - 1] = '\0'; // 문자열 종료 보장

            // Pipe를 통해 부모 프로세스에 냉난방 제어 메시지 전송
            char Temp_message[BUF_SIZE];
            snprintf(Temp_message, sizeof(Temp_message), "Temperature action: %s\n", current_action);

            if (write(pipes[clnt_idx][1], Temp_message, strlen(Temp_message)) == -1) {
                perror("Failed to write temperature action message");
            } else {
                printf("child process%d message sent: %s\n", clnt_idx, Temp_message);
            }
        }
    }
}


memset(buf, 0, BUF_SIZE); //버퍼 초기화

}           
            close(pipes[clnt_idx][0]); //자식프로세스 종료 전 읽기 파이프 닫기
            if(clnt_idx != 2){
            close(pipes[clnt_idx][1]); //자식프로세스 종료 전 쓰기 파이프 닫기
            }
            
            // 자식 프로세스에서 종료시 clnt_idx를 반환
            exit(clnt_idx); // 부모가 처리하도록 종료 코드로 전달
    } else { //부모프로세스  
    
    if(clnt_idx == 2 && pipes[2][0] != -1){ // 2번 자식프로세스의 읽기 끝 닫기
    
    close(pipes[2][0]); //읽기 파이프 닫기 
    pipes[2][0] = -1; // 닫힌 상태를 표시
    
    }
    
    is_full = check_all_clients_connected(); // 모든 소켓이 꽉 찼는지 확인    
    
    if (!is_full) {
    // while문 상단으로 돌아가서 다른 클라이언트 연결 받기
    continue;
    }
    } 
    } // 서버가 새로운 클라이언트 받는 if문 마지막        
    
    // 부모 프로세스
    
    if(!first){ //처음 한번만 실행하도록 설계 
    first++;
    // 자식에게 이제 시작해도 좋다는 메세지를 보내는 코드         
    const char *start_msg = "START"; // 자식 프로세스에게 보낼 메시지

for (int i = 0; i < MAX_CLIENTS; i++) {
    if (write(pipes[i][1], start_msg, strlen(start_msg)) == -1) { // 파이프로 모든 자식에게 메시지 전송
        perror("Failed to send start message to child");
    } else {
        printf("Sent start message to child %d\n", i);
    }
     usleep(10000); // 10ms 지연 추가
}
        }
    
    //select 사용
    fd_set read_fds;
    int max_fd = 0;
     
// 파일 디스크립터 집합 초기화
FD_ZERO(&read_fds);

// 0번과 1번 자식의 파이프 읽기 끝을 감시 대상으로 추가
for (int i = 0; i < 2; i++) {
    FD_SET(pipes[i][0], &read_fds); // 파이프의 읽기 끝 추가
    if (pipes[i][0] > max_fd) {
        max_fd = pipes[i][0]; // max_fd 갱신
    }
}

// select 호출
int activity = select(max_fd + 1, &read_fds, NULL, NULL, NULL);
if (activity < 0) {
    perror("select() error");
    return 0;
}

// 도착한 데이터 처리
for (int i = 0; i < 2; i++) {
    if (FD_ISSET(pipes[i][0], &read_fds)) { // 해당 파이프에 데이터 도착 확인
        int bytes_read = read(pipes[i][0], ipc_buf, BUF_SIZE - 1);
        if (bytes_read > 0) {
            ipc_buf[bytes_read] = '\0'; // 문자열 종료
            printf("Received message via IPC from child %d: %s\n", i, ipc_buf);

            // 메시지에 따라 처리
            if (strstr(ipc_buf, "PIR") != NULL) {
                // 1번 자식에게 받은 PIR센서 활성/비활성여부를 모든 클라이언트에 메시지 전송
                for (int j = 0; j < MAX_CLIENTS; j++) {
                    if (clnt_sock[j] != -1) {
                        int result = write(clnt_sock[j], ipc_buf, bytes_read);
                        if (result == -1) {
                            perror("write() error");
                            close(clnt_sock[j]);
                            clnt_sock[j] = -1;
                        }
                    }
                }
            } else if (strstr(ipc_buf, "Temp") != NULL) { //0번 자식에게 받은 냉난방 제어 메세지를 2번 클라이언트에게 전송
                if (clnt_sock[2] != -1) {
                    int result = write(clnt_sock[2], ipc_buf, bytes_read);
                    if (result == -1) {
                        perror("write() error");
                        close(clnt_sock[2]);
                        clnt_sock[2] = -1;
                    }
                }
            } else if (strstr(ipc_buf, "ULTRASONIC SENSOR ACTIVATION") != NULL) { //1번 자식에게 받은 초음파센서 활성화 메세지를 
                buzzer_msg_send = 1;                         // 모든 클라이언트에게 전송
                for (int j = 0; j < MAX_CLIENTS; j++) {
                    if (clnt_sock[j] != -1) {
                        int result = write(clnt_sock[j], ipc_buf, bytes_read);
                        if (result == -1) {
                            perror("write() error");
                            close(clnt_sock[j]);
                            clnt_sock[j] = -1;
                        }
                    }
                }
            } else if (strstr(ipc_buf, "ULTRASONIC SENSOR DEACTIVATION") != NULL) { //1번 자식에게 받은 초음파센서 비활성화 메세지를
                buzzer_msg_send = 0;                      // 모든 클라이언트에게 전송 
                for (int j = 0; j < MAX_CLIENTS; j++) {
                    if (clnt_sock[j] != -1) {
                        int result = write(clnt_sock[j], ipc_buf, bytes_read);
                        if (result == -1) {
                            perror("write() error");
                            close(clnt_sock[j]);
                            clnt_sock[j] = -1;
                        }
                    }
                }
            }   
        }   
    }
}

// 거리 데이터 및 초음파 활성화 루프
while (buzzer_msg_send) {
    fd_set read_fds;
    FD_ZERO(&read_fds);

    // pipes[1][0] 감시 추가
    FD_SET(pipes[1][0], &read_fds);
    int max_fd = pipes[1][0];

    // 클라이언트 소켓도 감시 대상에 추가
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (clnt_sock[i] != -1) {
            FD_SET(clnt_sock[i], &read_fds);
            if (clnt_sock[i] > max_fd) {
                max_fd = clnt_sock[i]; // max_fd 갱신
            }
        }
    }

    // select 호출
    int activity = select(max_fd + 1, &read_fds, NULL, NULL, NULL);
    if (activity < 0) {
        perror("select() error");
        break;
    }

    // pipes[1][0]에 데이터가 도착한 경우
    if (FD_ISSET(pipes[1][0], &read_fds)) {
        int bytes_read = read(pipes[1][0], ipc_buf, BUF_SIZE - 1);
        if (bytes_read > 0) {
            ipc_buf[bytes_read] = '\0'; // 문자열 종료
            printf("Received IPC message: %s\n", ipc_buf);

            if (strstr(ipc_buf, "ULTRASONIC SENSOR DEACTIVATION") != NULL) { // 초음파센서 비활성화 메세지일 경우
                buzzer_msg_send = 0;
                printf("Received deactivation message. Exiting loop.\n");
                for (int j = 0; j < MAX_CLIENTS; j++) { //모든 클라이언트에게 해당 메세지 전달
                    if (clnt_sock[j] != -1) {
                        int result = write(clnt_sock[j], ipc_buf, bytes_read);
                        if (result == -1) {
                            perror("write() error");
                            close(clnt_sock[j]);
                            clnt_sock[j] = -1;
                        }
                    }
                }
                break; //while문 탈출
            }
        }
    }

    // 클라이언트 소켓으로부터 거리 데이터 읽기
    min_distance = 999; // 최소 거리 초기화
    closest_client_idx = -1; // 인덱스 초기화
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (clnt_sock[i] != -1 && FD_ISSET(clnt_sock[i], &read_fds)) {
            int clnt_str_len = read(clnt_sock[i], buf, BUF_SIZE - 1);
            if (clnt_str_len > 0) {
                buf[clnt_str_len] = '\0'; // 문자열 종료

                int distance;
                if (sscanf(buf, "ULTSONIC/%d", &distance) == 1) { //거리데이터 파싱
                    if (distance < min_distance) {
                        min_distance = distance;
                        closest_client_idx = i;
                    }
                }
            }
        }
    }

    // 부저 메시지 결정
    const char *buzzer_command = BUZZER_CMD_OFF;
    if (min_distance <= 30) {
        buzzer_command = BUZZER_CMD_LEVEL_2;
    } else if (min_distance <= 60) {
        buzzer_command = BUZZER_CMD_LEVEL_1;
    }

    // 부저 메시지를 클라이언트2에게 전송
    if (clnt_sock[2] != -1) {
    usleep(500000);
        int result = write(clnt_sock[2], buzzer_command, strlen(buzzer_command));
        if (result == -1) {
            perror("write() error");
            close(clnt_sock[2]);
            clnt_sock[2] = -1; // 소켓 초기화
        } else {
            printf("Sent buzzer command to ACTURATOR client: %s\n", buzzer_command);
        }
    }
}
     
}
close(serv_sock); // 서버 소켓 닫기
// 서버 종료 시 파이프 닫기
for (int i = 0; i < MAX_CLIENTS; i++) {
    // 모든 자식 프로세스의 쓰기 끝 닫기
    close(pipes[i][1]); 

    if(i != 2){
        close(pipes[i][0]);
    }
}

    return 0;
}

void read_childproc(int sig) {
    pid_t pid;
    int status;

    // 종료된 자식 프로세스 처리
    while ((pid = waitpid(-1, &status, WNOHANG)) > 0) {
        int clnt_idx = WEXITSTATUS(status); // 자식이 반환한 클라이언트 인덱스

        if (clnt_idx >= 0 && clnt_idx < MAX_CLIENTS) {
            printf("Cleaning up %s client socket\n", clnt_name[clnt_idx]);
            close(clnt_sock[clnt_idx]);    // 부모에서 소켓 닫기
            clnt_sock[clnt_idx] = -1;     // 배열 초기화
        } else {
    // 비정상 종료 처리
    printf("Warning: Child process exited with invalid clnt_idx: %d\n", clnt_idx);
}
    }
}


// 에러 처리 함수
void error_handling(char *message) {
    fputs(message, stderr);
    fputc('\n', stderr);
    exit(1);
}

// 로그 저장 함수
void log_message(const char *log_file, const char *message) {
    FILE *file = fopen(log_file, "a");
    if (!file) {
        perror("Failed to open log file");
        return;
    }

    // 파일에 쓰기 잠금 설정
    int fd = fileno(file);
    if (flock(fd, LOCK_EX) == -1) {
        perror("Failed to lock file");
        fclose(file);
        return;
    }

    // 로그 메시지 기록
    fprintf(file, "%s\n", message);

    // 잠금 해제
    flock(fd, LOCK_UN);

    fclose(file);
}

//파이프 생성 및 초기화 
void create_pipes() {
    // 파이프 초기화
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (pipe(pipes[i]) == -1) {
            perror("Pipe creation failed");
            exit(EXIT_FAILURE);
        }
    }
}

//select 초기화
void initialize_resources() {
    // 표준 입력 파일 디스크립터
    stdin_fd = fileno(stdin);

    // 더미 파일 디스크립터 (예: /dev/null)
    dummy_fd = open("/dev/null", O_RDONLY);
    if (dummy_fd == -1) {
        perror("open() error");
        exit(EXIT_FAILURE);
    }
      printf("Resources initialized successfully.\n");
    
    }
    
//모든 클라이언트와의 연결을 체크    
int check_all_clients_connected() {
    // 모든 소켓이 꽉 찼는지 확인
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (clnt_sock[i] == -1) { // 비어 있는 슬롯이 있으면
            return 0; // 아직 모든 클라이언트가 연결되지 않음
        }
    }
    return 1; // 모든 클라이언트가 연결됨
}

    