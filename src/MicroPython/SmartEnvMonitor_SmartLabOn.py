# ******************************************************************************************
# FileName     : SmartEnvMonitor_SmartLabOn.py
# Description  : 환경 모니터링 시스템 - 온습도, 미세먼지, 산소, 이산화탄소, 소음 센서
# Description  : 지능형 과학실 ON 연동
# Author       : 박은정
# Created Date : 25.07.30 : 박은정
# Modified     : 
# ******************************************************************************************

import time
import math
import ujson
from machine import ADC, Pin, UART, Timer
from ETboard.lib.pin_define import *                     # ETboard 핀 관련 모듈


#===========================================================================================
# IoT 프로그램 사용하기                       
#===========================================================================================
from ET_IoT_App import ET_IoT_App, setup, loop
app = ET_IoT_App()


#===========================================================================================
# 미세먼지 센서 사용하기
#===========================================================================================
from ET_PPD42NS import ET_PPD42NS                        # 미세먼지 센서
dust_sensor = ET_PPD42NS(A3)                             # A3 핀에 센서 연결


#===========================================================================================
# 산소 센서 사용하기
#===========================================================================================
o2_sensor = ADC(Pin(A4))                                 # A4 핀에 센서 연결
o2_sensor.atten(ADC.ATTN_11DB)                           # 입력 전압 범위 확장 (3.3V까지)
o2_sensor.width(ADC.WIDTH_12BIT)                         # 12비트 ADC 해상도 설정 (0~4095)

VREF = 3.3                                               # 기준 전압
ADC_RESOLUTION = 4095                                    # 12비트 ADC 최대값


#===========================================================================================
# 사운드 센서 사용하기
#===========================================================================================
sound_sensor = ADC(Pin(A5))                              # A5번 핀에 센서 연결
sound_sensor.atten(ADC.ATTN_11DB)                        # 입력 전압 범위 확장 (3.3V까지)
sound_sensor.width(ADC.WIDTH_12BIT)                      # 12비트 ADC 해상도 설정 (0~4095)


#===========================================================================================
# DHT 온도 센서 사용하기
#===========================================================================================
from ET_DHTSensor import ET_DHTSensor                    # 온습도 센서
dht_sensor = ET_DHTSensor(D2)                            # D2 핀에 연결


#===========================================================================================
# 이산화탄소 센서 사용하기
#===========================================================================================
co2_uart = UART(1, baudrate=9600, tx=Pin(D9), rx=Pin(D8))# D8, D9 핀에 센서 연결
VREF = 3.3                                               # 기준 전압
ADC_RESOLUTION = 4095                                    # 12비트 ADC 최대값


#===========================================================================================
# OLED 표시 장치 사용하기
#===========================================================================================
from ETboard.lib.OLED_U8G2 import *
oled = oled_u8g2()


#===========================================================================================
# 전역 변수
#===========================================================================================
o2 = 0
co2 = 0

# 사운드 센서 관련 변수
SAMPLE_BUFFER_SIZE = 1024
SAMPLING_FREQ = 8000
sample_buffer = []

# 캘리브레이션 변수
dc_baseline = 2048
calibration_samples = 1000

# 측정 결과 변수
current_rms = 0
dB = 0

# 히스토리 및 상수
HISTORY_SIZE = 128
rms_history = [0] * HISTORY_SIZE
ADC_TO_VOLTAGE = 3.3 / 4095
RMS_DB_DATA = [
    (1.7, 39.9), (2.1, 46.3), (3.5, 66.8), (7.4, 88.2),
    (13.9, 96.5), (20.4, 98.4), (29.3, 103.6), (31.8, 103.9)
]

sound_timer = None


#===========================================================================================
def et_setup():                                          # 응용 프로그램 설정하기
#===========================================================================================
    dust_sensor.begin()

    # 사운드 센서 초기화
    print("RMS 측정 시스템 초기화...")
    calibrate_dc_offset()
    sound_timer = Timer(0)
    sound_timer.init(freq=SAMPLING_FREQ, mode=Timer.PERIODIC, callback=sample_sound_sensor)

    recv_message()                                       # 메시지 수신


#===========================================================================================
def et_loop():                                           # 응용 프로그램 루프
#===========================================================================================
    do_sensing_proces()                                  # 센싱 처리


#===========================================================================================
def do_sensing_proces():                                 # 센싱 처리
#===========================================================================================
    dust_sensor.update()
    dht_sensor.update()
    o2_sensor_update()
    co2_sensor_update()
    sound_sensor_update()


#===========================================================================================
def o2_sensor_update():                                  # 산소 센서 값 업데이트
#===========================================================================================
    global o2

    raw = o2_sensor.read()
    voltage = raw / ADC_RESOLUTION * VREF
    o2 = voltage * 0.21 / 2.0 * 100

    time.sleep(1)


#===========================================================================================
def co2_sensor_update():                                 # 이산화탄소 센서 값 업데이트
#===========================================================================================\
    global co2
    command = bytes([0xFE, 0x44, 0x00, 0x08, 0x02, 0x9F, 0x25])

    # 명령 전송 재시도
    for _ in range(5):
        co2_uart.write(command)
        time.sleep_ms(50)
        if co2_uart.any(): break
    else:
        co2 = -55
        return

    # 응답 대기
    timeout_counter = 0
    while co2_uart.any() < 7:
        time.sleep_ms(50)
        timeout_counter += 1
        if timeout_counter > 10:
            while co2_uart.any(): co2_uart.read(1)
            co2 = -55
            return

    # 데이터 처리
    response = co2_uart.read(7)
    if len(response) == 7:
        co2 = response[3] * 256 + response[4]
    else:
        co2 = -55
    
    
#===========================================================================================
def calibrate_dc_offset():                               # DC 바이어스 제거용 캘리브레이션
#===========================================================================================
    global dc_baseline

    print("DC 오프셋 캘리브레이션 중...")

    calibration_buffer = []
    for i in range(calibration_samples):
        calibration_buffer.append(sound_sensor.read())
        time.sleep_ms(1)

    dc_baseline = sum(calibration_buffer) / len(calibration_buffer)

    print(f"DC 기준값: {dc_baseline:.2f}")


#===========================================================================================
def calculate_rms(buffer):                               # RMS 계산
#===========================================================================================
    if not buffer or len(buffer) < 2: return 0, 0

    dc_component = sum(buffer) / len(buffer)
    rms_raw = math.sqrt(sum((sample - dc_component) ** 2 for sample in buffer) / len(buffer))

    return rms_raw, rms_to_db(rms_raw)


#===========================================================================================
def rms_to_db(rms):                                      # dB 변환
#===========================================================================================
    if rms <= RMS_DB_DATA[0][0]: return RMS_DB_DATA[0][1]

    if rms >= RMS_DB_DATA[-1][0]: return RMS_DB_DATA[-1][1]

    for i in range(len(RMS_DB_DATA) - 1):
        x1, y1 = RMS_DB_DATA[i]
        x2, y2 = RMS_DB_DATA[i + 1]
        if x1 <= rms <= x2:
            return y1 + (rms - x1) * (y2 - y1) / (x2 - x1)

    return 0


#===========================================================================================
def sample_sound_sensor(timer):                          # 샘플 수집
#===========================================================================================
    global sample_buffer

    if len(sample_buffer) < SAMPLE_BUFFER_SIZE:
        sample_buffer.append(sound_sensor.read())


#===========================================================================================
def sound_sensor_update():                               # RMS 계산
#===========================================================================================
    global sample_buffer, current_rms, dB

    if len(sample_buffer) >= SAMPLE_BUFFER_SIZE:
        data = sample_buffer[:]
        sample_buffer.clear()
        current_rms, dB = calculate_rms(data)


#===========================================================================================
def do_automatic_process():                              # 자동화 처리
#===========================================================================================
    pass


#===========================================================================================
def et_short_periodic_process():                         # 사용자 주기적 처리 (예 : 1초마다)
#===========================================================================================
    display_information()


#===========================================================================================
def display_information():                               # oled 표시
#===========================================================================================
    global o2, co2, dB

    string_temp = "{:.1f}".format(dht_sensor.get_average_temperature())
    string_humi = "{:.1f}".format(dht_sensor.get_average_humidity())
    string_dust = "{:.6f}".format(dust_sensor.get_ugm3());
    string_o2 = "{:.2f}".format(o2)
    string_co2 = "{:d}".format(co2)
    string_db = "{:.1f}".format(dB)

    oled.clear()
    oled.setLine(2, 'temp: ' + string_temp + 'C')               # OLED 모듈 2번 줄에 저장
    oled.setLine(3, 'humi: ' + string_humi + '%')               # OLED 모듈 3번 줄에 저장
    oled.setLine(4, 'dust: ' + string_dust + 'ppm')             # OLED 모듈 4번 줄에 저장
    oled.setLine(5, 'O2: ' + string_o2 + '%')                   # OLED 모듈 5번 줄에 저장
    oled.setLine(6, 'CO2: ' + string_co2 + 'ppm')               # OLED 모듈 6번 줄에 저장
    oled.setLine(7, 'dB: ' + string_db + 'dB')                  # OLED 모듈 7번 줄에 저장

    oled.display()                                              # 저장된 내용을 oled에 보여줌


#===========================================================================================
def et_long_periodic_process():                          # 사용자 주기적 처리 (예 : 5초마다)
#===========================================================================================
    send_message()                                       # 메시지 송신


#===========================================================================================
def send_message():                                      # 메시지 송신
#===========================================================================================
    global o2, co2, dB

    app.add_sensor_data("temp", dht_sensor.get_average_temperature())
    app.add_sensor_data("humi", dht_sensor.get_average_humidity())
    app.add_sensor_data("dust", dust_sensor.get_ugm3())
    app.add_sensor_data("o2", o2)
    app.add_sensor_data("co2", co2)
    app.add_sensor_data("dB", dB)
    app.send_sensor_data()


#===========================================================================================
def recv_message():                                      # 메시지 수신
#===========================================================================================
    # "get_sensor_type" 메시지를 받으면 send_sensor_type() 실행
    app.setup_recv_message('get_sensor_type', handle_get_sensor_type_request)


#===========================================================================================
def handle_get_sensor_type_request(topic, msg):          # 센서 타입 송신 처리
#===========================================================================================
    send_sensor_type()


#===========================================================================================
def json_to_unicode_escaped(data):                       # 직렬화, 이스케이프
#===========================================================================================
    # JSON 직렬화
    json_string = ujson.dumps(data)

    # JSON 문자열에서 비-ASCII 문자를 Unicode 이스케이프 형식으로 변환
    return ''.join(f'\\u{ord(c):04x}' if ord(c) > 127 else c for c in json_string)


#===========================================================================================
def send_sensor_type():                                  # 센서 타입 전송
#===========================================================================================
    sensors = [
        {"sensorId": "temp", "sensorType": "temp", "sensorNicNm": "온도", "channelCode": "01", "collectUnit": "°C"},
        {"sensorId": "humi", "sensorType": "humi", "sensorNicNm": "습도", "channelCode": "01", "collectUnit": "%"},
        {"sensorId": "o2", "sensorType": "O2", "sensorNicNm": "산소", "channelCode": "01", "collectUnit": "%"},
        {"sensorId": "co2", "sensorType": "CO2", "sensorNicNm": "이산화탄소", "channelCode": "01", "collectUnit": "ppm"},
        {"sensorId": "dust", "sensorType": "dust", "sensorNicNm": "미세먼지", "channelCode": "01", "collectUnit": "ppm"},
        {"sensorId": "db", "sensorType": "dB", "sensorNicNm": "소음", "channelCode": "01", "collectUnit": "dB"}
    ]
    
    for sensor in sensors:
        payload = ujson.dumps(sensor).encode('unicode_escape').decode('ascii')
        app.send_data("sensor_types", sensor["sensorId"], payload)


#===========================================================================================
# 시작 지점                     
#===========================================================================================
if __name__ == "__main__":
    setup(app, et_setup)    
    while True:
        loop(app, et_loop, et_short_periodic_process, et_long_periodic_process)


#===========================================================================================
#                                                    
# (주)한국공학기술연구원 http://et.ketri.re.kr       
#
#===========================================================================================
