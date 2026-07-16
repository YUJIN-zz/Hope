#include <driver/i2s.h>
#include <Arduino.h>
// ========== 하드웨어 핀 설정 ==========
// 그룹 1 (앞방향: 마이크 1, 2) - I2S 포트 0
#define I2S0_SCK_PIN  15
#define I2S0_WS_PIN   16
#define I2S0_SD_PIN   17

// 그룹 2 (뒷방향: 마이크 3, 4) - I2S 포트 1
#define I2S1_SCK_PIN  4
#define I2S1_WS_PIN   5
#define I2S1_SD_PIN   6

// ========== I2S 통신 설정 ==========
#define SAMPLE_RATE   16000 // 음성 및 방향 감지(DoA)에 적합한 주파수
#define DMA_BUF_COUNT 8
#define DMA_BUF_LEN   256

void setup_i2s() {
  // I2S 공통 통신 규격 설정
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, // INMP441은 32비트 프레임 내에 24비트 데이터 전송
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, // 좌/우 채널 동시 수신 필수
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = DMA_BUF_COUNT,
    .dma_buf_len = DMA_BUF_LEN,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  // 1. I2S 포트 0번, 1번 드라이버 설치
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_driver_install(I2S_NUM_1, &i2s_config, 0, NULL);

  // 2. 그룹 1 핀 맵핑 (I2S_NUM_0)
  i2s_pin_config_t pin_config_0 = {
    .bck_io_num = I2S0_SCK_PIN,
    .ws_io_num = I2S0_WS_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S0_SD_PIN
  };
  i2s_set_pin(I2S_NUM_0, &pin_config_0);

  // 3. 그룹 2 핀 맵핑 (I2S_NUM_1)
  i2s_pin_config_t pin_config_1 = {
    .bck_io_num = I2S1_SCK_PIN,
    .ws_io_num = I2S1_WS_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S1_SD_PIN
  };
  i2s_set_pin(I2S_NUM_1, &pin_config_1);
}

void setup() {
  Serial.begin(115200);
  Serial.println("4채널 INMP441 마이크 초기화 시작...");

  setup_i2s();

  Serial.println("I2S 포트 0, 1 초기화 완료! 데이터 수신 대기 중...");
}

void loop() {
  // DMA 버퍼 길이만큼 데이터를 담을 배열 (L, R 데이터를 모두 담아야 하므로 크기는 * 2)
  int32_t i2s0_read_buff[DMA_BUF_LEN * 2];
  int32_t i2s1_read_buff[DMA_BUF_LEN * 2];

  size_t bytes_read_0 = 0;
  size_t bytes_read_1 = 0;

  // 두 포트에서 동시에 데이터 읽기
  i2s_read(I2S_NUM_0, &i2s0_read_buff, sizeof(i2s0_read_buff), &bytes_read_0, portMAX_DELAY);
  i2s_read(I2S_NUM_1, &i2s1_read_buff, sizeof(i2s1_read_buff), &bytes_read_1, portMAX_DELAY);

  // 32비트(4바이트) * 2채널(좌,우) = 8바이트당 1샘플 세트
  int samples_read = bytes_read_0 / 8;

  // 배열에 담긴 데이터 파싱
  for (int i = 0; i < samples_read; i++) {
    /*
       INMP441은 24비트 데이터를 32비트 프레임에 담아 보냅니다.
       하위 8비트는 노이즈(쓰레기값)이므로 오른쪽으로 8칸 시프트(>> 8)하여 제거해야 정확한 파형이 나옵니다.
       배열의 짝수 인덱스는 Left 채널, 홀수 인덱스는 Right 채널 데이터입니다.
    */
    int32_t mic1_left  = i2s0_read_buff[i * 2] >> 8;     // 그룹 1 - 마이크 1 (L/R -> GND)
    int32_t mic2_right = i2s0_read_buff[i * 2 + 1] >> 8; // 그룹 1 - 마이크 2 (L/R -> 3.3V)

    int32_t mic3_left  = i2s1_read_buff[i * 2] >> 8;     // 그룹 2 - 마이크 3 (L/R -> GND)
    int32_t mic4_right = i2s1_read_buff[i * 2 + 1] >> 8; // 그룹 2 - 마이크 4 (L/R -> 3.3V)

    // 아두이노 IDE 시리얼 플로터용 출력 포맷 (콤마로 구분)
    // 주의: 출력 속도가 너무 빠르면 멈출 수 있으므로, 테스트 시에는 샘플을 건너뛰며 출력하는 것이 좋습니다.
    if (i % 8 == 0) { // 8개 샘플 중 1개만 시리얼 플로터로 출력 (과부하 방지)
      Serial.printf("%d,%d,%d,%d\n", mic1_left, mic2_right, mic3_left, mic4_right);
    }
  }
}
