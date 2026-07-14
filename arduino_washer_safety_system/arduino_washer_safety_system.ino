/*
 * 세탁기 안전 제어 시스템 (Arduino Uno)
 *
 * 실제 세탁기의 안전 로직을 아두이노로 재현한 프로젝트.
 * 대기 -> 급수 -> 세탁 -> 탈수 -> 완료 순서로 진행되며,
 * 동작 중 문 열림이나 탈수 중 진동을 감지하면 자동으로 정지한다.
 *
 * 주요 동작
 *  - 문이 열려 있으면 시작 자체가 안 됨 (도어 인터록)
 *  - 동작 중 문이 2초 이상 열리면 일시정지 + 경보
 *  - 탈수 중 진동 감지 시 일시정지, 재개하면 탈수를 처음부터 다시
 *  - 급수가 제한시간 안에 안 끝나면 알람 (밸브/센서 이상 가정)
 *  - 대기 상태에서 버튼2로 절전 진입/해제 (토글)
 *
 * 절전모드 관련 참고
 *  처음엔 AVR Power-down(sleep_mode)으로 구현했는데, 그 방식은
 *  외부 인터럽트가 있는 핀(D2, D3)으로만 깨울 수 있어서
 *  버튼(A1, 아날로그 핀)으로는 깨울 수가 없었다. 그래서
 *  완전한 MCU 절전 대신, 릴레이/LED를 끄고 LCD 표시를 꺼둔 채
 *  버튼 폴링만 유지하는 소프트웨어 방식으로 바꿨다.
 *  MCU 자체 소비전력은 그대로지만 주변 부품 전력은 줄어든다.
 *  (LCD 백라이트는 결선상 상시 전원이라 소프트웨어로는 못 끈다)
 *
 * 핀맵
 *  D2      리드스위치 DO (도어 열림/닫힘 감지)
 *  D3      기울기센서 (INPUT_PULLUP, 반대쪽 핀은 GND)
 *  D4~D9   1602 LCD (RS, E, D4~D7)
 *  D10     수동부저
 *  D11     릴레이 (급수밸브 역할)
 *  D12     빨강 LED (문 닫힘 표시)
 *  D13     초록 LED (문 열림 표시)
 *  A0      물높이센서
 *  A1      버튼 3개 (저항 래더로 1핀에 통합)
 *  A2~A5   ULN2003 IN1~IN4 (28BYJ-48 스텝모터)
 */
#include <LiquidCrystal.h>
#include <Stepper.h>
#include <EEPROM.h>
//---------------- 핀 ----------------
#define PIN_DOOR      2
#define PIN_TILT      3
#define PIN_BUZZER    10
#define PIN_RELAY     11
#define PIN_LED_LOCK  12
#define PIN_LED_OPEN  13
#define PIN_WATER     A0
#define PIN_BTN       A1
LiquidCrystal lcd(4, 5, 6, 7, 8, 9);
//Stepper 라이브러리는 (IN1, IN3, IN2, IN4) 순서로 넣어야 정방향 회전한다
//실제 배선: A2=IN1, A3=IN2, A4=IN3, A5=IN4
Stepper stepper(2048, A2, A4, A3, A5);
//---------------- 상태 머신 ----------------
enum State {
  IDLE, FILLING, WASHING, SPINNING, DONE, ALARM, PAUSED, SLEEPING
};
State currentState = IDLE;
//---------------- 시간 설정 ----------------
const unsigned long LCD_INTERVAL  = 300;
const unsigned long FILL_DURATION = 20000; //손으로 물 붓는 걸 감안해 넉넉히
const unsigned long WASH_DURATION = 10000;
const unsigned long SPIN_DURATION = 8000;
//---------------- 물높이 (실측 기반) ----------------
//전도성 접점식이라 물이 조금만 닿아도 값이 400대로 튀는 비선형 특성이 있어서
//단순 비례식 대신 실측 지점을 구간별 직선 보간해서 %를 계산한다.
//실측값: 건조 0 / 소량 410 / 중간 520 / 목표 수위(종이컵 절반) 660
const int waterAdcPoints[]     = {0, 410, 520, 660};
const int waterPercentPoints[] = {0, 33, 67, 100};
const int WATER_POINTS_COUNT   = 4;
const int WATER_FULL_THRESHOLD = 660;
//---------------- 버튼 (아날로그 래더, 실측 기반) ----------------
//버튼 3개를 저항 분압으로 A1 한 핀에서 읽는다.
//실측 ADC: 버튼1(시작)=1019, 버튼2(절전)=509, 버튼3(일시정지/재개)=338
//풀다운 10k를 빼먹었을 때는 핀이 떠서 아무것도 안 눌러도
//200~300대 잡음이 계속 찍혔다. 풀다운 필수.
const int BTN1_ADC = 1019;
const int BTN2_ADC = 509;
const int BTN3_ADC = 338;
const int BTN_TOLERANCE = 40;
unsigned long lastBtnTime = 0;
const unsigned long BTN_DEBOUNCE = 250;
//리드스위치: 자석 붙음(문 닫힘) = LOW (실측 확인)
#define DOOR_CLOSED_LEVEL LOW
//문 열림 판정 디바운스.
//리드스위치가 예민해서 자석이 살짝만 흔들려도 순간적으로 열림이 잡히길래,
//2초 이상 계속 열려 있어야 진짜 열림으로 인정하도록 했다.
unsigned long doorOpenSince = 0;
bool doorWasClosed = true;
const unsigned long DOOR_OPEN_CONFIRM_TIME = 2000;
//일시정지 관련. 어느 상태에서 얼마나 진행하다 멈췄는지 기억해뒀다가 이어서 재개
State pausedFromState;
unsigned long pausedElapsed = 0;
bool pausedByDoor = false;
bool pausedByImbalance = false;
//급수 100% 도달 후 화면을 잠깐 유지했다가 다음 단계로 넘어가기 위한 플래그
//(바로 넘기면 100% 표시가 보일 틈이 없다)
bool fillCompleteShown = false;
unsigned long fillCompleteTime = 0;
const unsigned long FILL_COMPLETE_HOLD = 2000;
//문 열린 채로 시작 버튼을 눌렀을 때 안내 문구 표시용
bool showCloseDoorMsg = false;
unsigned long closeDoorMsgTime = 0;
const unsigned long CLOSE_DOOR_MSG_DURATION = 2000;
unsigned long lastLcdUpdate = 0;
unsigned long stateTimer = 0;
#define EEPROM_ADDR_STATE 0
//---------------- 부저 멜로디 (수동부저, tone 사용) ----------------
//멜로디 데이터를 구조체로 묶어 함수 인자로 넘기는 방식을 먼저 시도했는데,
// 아두이노 IDE가 함수 프로토타입을 구조체 정의보다 앞에 자동 생성하는 바람에
//컴파일이 깨졌다. 그래서 배열 + 번호(0=시작, 1=경보, 2=완료)로 우회했다.
const int startNotes[] = {880, 1175}; //시작음: 짧게 상승
const unsigned int startDur[] = {100, 150};
const int startLen = 2;
const int alarmNotes[] = {1800, 0, 1800, 0}; //경보음: 높은음 반복 (0은 무음 구간)
const unsigned int alarmDur[] = {120, 80, 120, 400};
const int alarmLen = 4;
const int completeNotes[] = {523, 659, 784, 1047}; //완료음: 도-미-솔-도
const unsigned int completeDur[] = {150, 150, 150, 300};
const int completeLen = 4;
int activeMelody = -1; //-1=없음, 0=시작, 1=경보, 2=완료
int melodyIndex = 0;
unsigned long melodyNoteStart = 0;
bool melodyPlaying = false;
void playNoteAt(int melodyType, int idx) {
  int freq = 0;
  if (melodyType == 0) freq = startNotes[idx];
  else if (melodyType == 1) freq = alarmNotes[idx];
  else if (melodyType == 2) freq = completeNotes[idx];
  if (freq > 0) tone(PIN_BUZZER, freq);
  else noTone(PIN_BUZZER);
}
unsigned int getDurationAt(int melodyType, int idx) {
  if (melodyType == 0) return startDur[idx];
  if (melodyType == 1) return alarmDur[idx];
  if (melodyType == 2) return completeDur[idx];
  return 100;
}
int getMelodyLen(int melodyType) {
  if (melodyType == 0) return startLen;
  if (melodyType == 1) return alarmLen;
  if (melodyType == 2) return completeLen;
  return 0;
}
void startMelodyPlay(int melodyType) {
  activeMelody = melodyType;
  melodyIndex = 0;
  melodyPlaying = true;
  melodyNoteStart = millis();
  playNoteAt(activeMelody, 0);
}
void stopMelodyPlay() {
  melodyPlaying = false;
  activeMelody = -1;
  noTone(PIN_BUZZER);
}
//매 루프 호출. tone()은 백그라운드로 소리를 내므로
//음 전환 타이밍만 millis로 관리하면 루프를 막지 않는다
void updateMelody() {
  if (!melodyPlaying || activeMelody == -1) return;
  unsigned int dur = getDurationAt(activeMelody, melodyIndex);
  if (millis() - melodyNoteStart >= dur) {
    melodyIndex++;
    int len = getMelodyLen(activeMelody);
    if (melodyIndex >= len) {
      if (activeMelody == 1) {
        melodyIndex = 0; //경보음만 무한 반복
      } else {
        noTone(PIN_BUZZER);
        melodyPlaying = false;
        return;
      }
    }
    melodyNoteStart = millis();
    playNoteAt(activeMelody, melodyIndex);
  }
}
//---------------- 도어 / LED ----------------
bool isDoorClosed() {
  return (digitalRead(PIN_DOOR) == DOOR_CLOSED_LEVEL);
}
void setDoorLED(bool locked) {
  digitalWrite(PIN_LED_LOCK, locked ? HIGH : LOW);
  digitalWrite(PIN_LED_OPEN, locked ? LOW : HIGH);
}
//---------------- 상태 관리 ----------------
bool isRunningState(State s) {
  return (s == FILLING || s == WASHING || s == SPINNING);
}
void saveState(State s) { EEPROM.update(EEPROM_ADDR_STATE, (byte)s); }
void changeState(State s) {
  currentState = s;
  stateTimer = millis();
  saveState(s);
  Serial.print("State -> ");
  Serial.println(s);
}
//일시정지 진입. 원인 구분:
//0 = 사용자가 버튼으로 멈춤 (경보 없음)
//1 = 동작 중 문 열림 (경보)
//2 = 탈수 중 진동 감지 (경보)
void enterPaused(int reason) {
  pausedFromState = currentState;
  pausedElapsed = millis() - stateTimer;
  pausedByDoor = (reason == 1);
  pausedByImbalance = (reason == 2);
  digitalWrite(PIN_RELAY, LOW);
  currentState = PAUSED;
  saveState(PAUSED);
  Serial.println(reason == 1 ? "PAUSED (door opened)" : reason == 2 ? "PAUSED (imbalance)" : "PAUSED (button)");
  if (reason == 1 || reason == 2) startMelodyPlay(1);
}
//절전 진입: 대기 상태에서만 가능. 릴레이/LED를 끄고 LCD 표시를 끈다.
//버튼은 계속 폴링하니 하드웨어 인터럽트 없이도 즉시 복귀 가능
void enterSleepMode() {
  stopMelodyPlay();
  digitalWrite(PIN_RELAY, LOW);
  digitalWrite(PIN_LED_LOCK, LOW);
  digitalWrite(PIN_LED_OPEN, LOW);
  lcd.noDisplay(); //백라이트는 결선상 못 끄고, 화면 표시만 끈다
  currentState = SLEEPING;
  Serial.println("Entering sleep mode");
}
//절전 해제: 버튼2를 다시 누르면 대기 상태로 복귀
void exitSleepMode() {
  lcd.display();
  currentState = IDLE;
  setDoorLED(isDoorClosed());
  Serial.println("Exiting sleep mode");
}
void setup() {
  Serial.begin(9600);
  pinMode(PIN_DOOR, INPUT); //3핀 모듈이라 자체 풀업 있음
  pinMode(PIN_TILT, INPUT_PULLUP); //2핀 단품이라 내부 풀업 사용
  pinMode(PIN_RELAY, OUTPUT);
  pinMode(PIN_LED_LOCK, OUTPUT);
  pinMode(PIN_LED_OPEN, OUTPUT);
  digitalWrite(PIN_RELAY, LOW);
  lcd.begin(16, 2);
  stepper.setSpeed(10);
  //마지막 상태를 EEPROM에서 복원한다.
  //단, 동작 중(급수/세탁/탈수)이거나 완료/알람/일시정지/절전으로
  //저장돼 있었다면 전부 대기로 되돌린다. 정전 후 아무도 없는 상태에서
  //밸브나 모터가 멋대로 재가동되는 걸 막기 위한 처리다.
  //(처음엔 저장된 상태를 그대로 복원했다가, DONE으로 저장된 채 껐다 켜니
  //계속 완료 화면에 갇히는 문제를 겪고 이렇게 바꿨다)
byte saved = EEPROM.read(EEPROM_ADDR_STATE);
if (saved <= SLEEPING && (isRunningState((State)saved) || saved == DONE || saved == ALARM || saved == PAUSED || saved == SLEEPING)) {
  currentState = IDLE;
} else if (saved <= SLEEPING) {
  currentState = (State)saved;
} else {
  currentState = IDLE; //에러 방지 (EEPROM 초기값 255 등)
}
  setDoorLED(isDoorClosed());
  lcd.clear();
  lcd.print("Washer Ready");
  Serial.println("=== System Boot ===");
}
//버튼 읽기. 실측 ADC값과 비교해서 어느 버튼인지 판별
int readButton() {
  int adc = analogRead(PIN_BTN);
  if (millis() - lastBtnTime < BTN_DEBOUNCE) return 0;
  int btn = 0;
  if (abs(adc - BTN1_ADC) < BTN_TOLERANCE) btn = 1;
  else if (abs(adc - BTN2_ADC) < BTN_TOLERANCE) btn = 2;
  else if (abs(adc - BTN3_ADC) < BTN_TOLERANCE) btn = 3;
  if (btn != 0) lastBtnTime = millis();
  return btn;
}
//기울기센서: 평상시 LOW, 흔들리면 HIGH (실측 확인)
bool checkImbalance() {
  return (digitalRead(PIN_TILT) == HIGH);
}
//ADC -> % 변환. 실측 4점 사이를 구간별로 직선 보간
int waterAdcToPercent(int adc) {
  if (adc <= waterAdcPoints[0]) return waterPercentPoints[0];
  if (adc >= waterAdcPoints[WATER_POINTS_COUNT - 1]) return waterPercentPoints[WATER_POINTS_COUNT - 1];
  for (int i = 0; i < WATER_POINTS_COUNT - 1; i++) {
    if (adc >= waterAdcPoints[i] && adc <= waterAdcPoints[i + 1]) {
      long adcRange = waterAdcPoints[i + 1] - waterAdcPoints[i];
      long pctRange = waterPercentPoints[i + 1] - waterPercentPoints[i];
      long offset = adc - waterAdcPoints[i];
      return waterPercentPoints[i] + (offset * pctRange) / adcRange;
    }
  }
  return 0;
}
void updateLcd() {
  if (currentState == SLEEPING) return; // 절전 중엔 화면 자체를 안 씀
  if (millis() - lastLcdUpdate < LCD_INTERVAL) return;
  lastLcdUpdate = millis();
  lcd.clear();
  lcd.setCursor(0, 0);
  switch (currentState) {
    case IDLE: lcd.print("1.Standby"); break;
    case FILLING: lcd.print("2.Filling..."); break;
    case WASHING: lcd.print("3.Washing..."); break;
    case SPINNING: lcd.print("4.Spinning..."); break;
    case DONE: lcd.print("5.Complete!"); break;
    case ALARM: lcd.print("! ALARM !"); break;
    case PAUSED:
      if (pausedByDoor) lcd.print("!PAUSED(Door)!");
      else if (pausedByImbalance) lcd.print("!PAUSED(Shake)!");
      else lcd.print("-- PAUSED --");
      break;
    default: break;
  }
  lcd.setCursor(0, 1);
  if (currentState == IDLE) {
    if (showCloseDoorMsg && millis() - closeDoorMsgTime < CLOSE_DOOR_MSG_DURATION) {
      lcd.print("Close the door");
    } else {
      showCloseDoorMsg = false;
      lcd.print("Btn2:Sleep");
    }
  } else if (currentState == PAUSED) {
    //문이 열려 있으면 재개 불가 상태라는 걸 안내
    lcd.print(isDoorClosed() ? "Btn3:Resume" : "Close the door");
  } else if (currentState == FILLING) {
    //5회 평균으로 표시 잔떨림 완화
    int avgWater = 0;
    for (int i = 0; i < 5; i++) avgWater += analogRead(PIN_WATER);
    avgWater /= 5;
    lcd.print("Water:");
    lcd.print(waterAdcToPercent(avgWater));
    lcd.print("%");
  } else if (currentState == WASHING || currentState == SPINNING) {
    //남은 시간(초) 표시. 일시정지 후 재개해도 이어서 계산된다
    unsigned long duration = (currentState == WASHING) ? WASH_DURATION : SPIN_DURATION;
    unsigned long elapsed = millis() - stateTimer;
    long remaining = (long)(duration - elapsed) / 1000;
    if (remaining < 0) remaining = 0;
    lcd.print("Time left:");
    lcd.print(remaining);
    lcd.print("s");
  }
}
void loop() {
  bool doorClosedNow = isDoorClosed();
  //도어 감시는 동작 중(급수/세탁/탈수)에만 한다.
  //대기 상태에서 문을 여닫는 건 정상 사용이라 감시 대상이 아니다.
  if (isRunningState(currentState)) {
    if (!doorClosedNow) {
      if (doorWasClosed) {
        doorOpenSince = millis(); //열리기 시작한 시점 기록
        doorWasClosed = false;
      }
      //2초 이상 계속 열려 있어야 확정(짧은 접촉은 무시)
      if (millis() - doorOpenSince > DOOR_OPEN_CONFIRM_TIME) {
        Serial.println("!! DOOR OPENED (confirmed) - PAUSED !!");
        enterPaused(1);
      }
    } else {
      doorWasClosed = true; //중간에 닫히면 타이머 리셋
    }
  } else {
    doorWasClosed = true;
  }
  int btn = readButton();
  switch (currentState) {
    case IDLE:
      digitalWrite(PIN_RELAY, LOW);
      setDoorLED(doorClosedNow); //LED는 오직 실제 문 상태만 반영
      if (btn == 2) {
        enterSleepMode();
        break;
      }
      if (btn == 1) {
        if (doorClosedNow) {
          changeState(FILLING);
          startMelodyPlay(0);
        } else {
          //문 열린 채 시작 시도 -> 안내만 띄우고 대기 유지 (도어 인터록)
          showCloseDoorMsg = true;
          closeDoorMsgTime = millis();
          Serial.println("Cannot start: door open");
        }
      }
      break;
    //절전 상태: 버튼2를 다시 누르면 즉시 복귀. 그 외엔 아무것도 안 함
    case SLEEPING:
      if (btn == 2) {
        exitSleepMode();
      }
      break;
    case FILLING: {
      //급수 완료 전까지만 릴레이 ON 유지.
      //조건 없이 매 루프 HIGH를 쓰면 완료 판정에서 꺼도
      //다음 루프에서 다시 켜져 버려서 이렇게 처리했다.
      if (!fillCompleteShown) {
        digitalWrite(PIN_RELAY, HIGH);
      }
      int rawWater = analogRead(PIN_WATER);
      if (rawWater >= WATER_FULL_THRESHOLD) {
        if (!fillCompleteShown) {
          fillCompleteShown = true;
          fillCompleteTime = millis();
          digitalWrite(PIN_RELAY, LOW); //밸브는 즉시 잠금
        }
        //100% 화면을 잠깐 보여준 뒤 다음 단계로
        if (millis() - fillCompleteTime > FILL_COMPLETE_HOLD) {
          fillCompleteShown = false;
          changeState(WASHING);
        }
      } else if (millis() - stateTimer > FILL_DURATION) {
        //제한시간 내에 목표 수위 도달 실패 = 밸브/센서 이상으로 간주
        Serial.println("FAULT: Fill timeout");
        digitalWrite(PIN_RELAY, LOW);
        changeState(ALARM);
        startMelodyPlay(1);
      }
      if (btn == 3) enterPaused(0);
      break;
    }
    case WASHING:
      if (millis() - stateTimer > WASH_DURATION) {
        changeState(SPINNING);
      }
      if (btn == 3) enterPaused(0);
      break;
    case SPINNING:
      //step()이 실제로 도는 동안은 블로킹이라 엄밀히는 완전한 논블로킹이 아니다.
      //한 번에 50스텝씩만 돌려서 루프 지연을 체감 안 되는 수준으로 유지.
      stepper.step(50);
      if (checkImbalance()) {
        Serial.println("Imbalance detected -> PAUSED");
        enterPaused(2);
        break; //같은 루프에서 아래 완료 판정이 PAUSED를 덮어쓰지 않도록
      }
      if (millis() - stateTimer > SPIN_DURATION) {
        changeState(DONE);
        startMelodyPlay(2);
        break;
      }
      if (btn == 3) enterPaused(0);
      break;
    case PAUSED:
      digitalWrite(PIN_RELAY, LOW);
      setDoorLED(doorClosedNow);
      //재개 조건: 문이 닫힌 상태에서 버튼3.
      //진동으로 멈춘 경우엔 탈수를 처음부터 다시 돌린다
      //(실제 세탁기도 세탁물 재배치 후 탈수를 재시도하는 방식)
      if (btn == 3 && doorClosedNow) {
        stopMelodyPlay();
        currentState = pausedFromState;
        if (pausedByImbalance) {
          stateTimer = millis(); //탈수 처음부터
        } else {
          stateTimer = millis() - pausedElapsed; //멈춘 지점부터 이어서
        }
        saveState(currentState);
        Serial.print("Resumed -> ");
        Serial.println(currentState);
      }
      break;
    case DONE:
      setDoorLED(false); //완료 후 도어 잠금 해제 표시
      if (btn == 1 || btn == 3) {
        stopMelodyPlay();
        changeState(IDLE);
      }
      break;
    case ALARM:
      digitalWrite(PIN_RELAY, LOW);
      if (btn == 1 || btn == 3) {
        stopMelodyPlay();
        changeState(IDLE);
      }
      break;
  }
  updateMelody();
  updateLcd();
}
 