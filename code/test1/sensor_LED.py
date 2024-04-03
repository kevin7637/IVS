import RPi.GPIO as GPIO
import time
import neopixel

# 초음파 센서 핀 설정
Tr = 11
Ec = 8

# 라인 트래킹 센서 핀 설정
line_pin_right = 19
line_pin_middle = 16
line_pin_left = 20

# RGB LED 핀 설정
left_R = 22
left_G = 23
left_B = 24
right_R = 10
right_G = 9
right_B = 25

# WS2812 LED 설정
LED_COUNT = 8
LED_PIN = 18
LED_BRIGHTNESS = 50
LED_ORDER = neopixel.GRB

on = GPIO.LOW
off = GPIO.HIGH

def setup():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Tr, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(Ec, GPIO.IN)
    GPIO.setup(line_pin_right, GPIO.IN)
    GPIO.setup(line_pin_middle, GPIO.IN)
    GPIO.setup(line_pin_left, GPIO.IN)
    GPIO.setup(left_R, GPIO.OUT)
    GPIO.setup(left_G, GPIO.OUT)
    GPIO.setup(left_B, GPIO.OUT)
    GPIO.setup(right_R, GPIO.OUT)
    GPIO.setup(right_G, GPIO.OUT)
    GPIO.setup(right_B, GPIO.OUT)
    both_off()

def checkdist():
    # 초음파 센서로 거리 측정
    for i in range(5):
        GPIO.output(Tr, GPIO.LOW)
        time.sleep(0.000002)
        GPIO.output(Tr, GPIO.HIGH)
        time.sleep(0.000015)
        GPIO.output(Tr, GPIO.LOW)
        
        while not GPIO.input(Ec):
            pass
        t1 = time.time()
        
        while GPIO.input(Ec):
            pass
        t2 = time.time()
        
        dist = (t2 - t1) * 340 / 2
        
        if dist > 9 and i < 4:
            continue
        else:
            return dist

def line_tracking():
    # 라인 트래킹 센서 측정
    status_right = GPIO.input(line_pin_right)
    status_middle = GPIO.input(line_pin_middle)
    status_left = GPIO.input(line_pin_left)
    return status_right, status_middle, status_left

def both_on():
    GPIO.output(left_R, on)
    GPIO.output(left_G, on)
    GPIO.output(left_B, on)
    GPIO.output(right_R, on)
    GPIO.output(right_G, on)
    GPIO.output(right_B, on)

def both_off():
    GPIO.output(left_R, off)
    GPIO.output(left_G, off)
    GPIO.output(left_B, off)
    GPIO.output(right_R, off)
    GPIO.output(right_G, off)
    GPIO.output(right_B, off)

def side_on(side_X):
    GPIO.output(side_X, on)

def side_off(side_X):
    GPIO.output(side_X, off)

def red():
    side_on(right_R)
    side_on(left_R)

def green():
    side_on(right_G)
    side_on(left_G)

def blue():
    side_on(right_B)
    side_on(left_B)

def ws2812_control(color):
    pixels = neopixel.NeoPixel(LED_PIN, LED_COUNT, brightness=LED_BRIGHTNESS / 255, auto_write=False, pixel_order=LED_ORDER)
    for i in range(LED_COUNT):
        pixels[i] = color
    pixels.show()

if __name__ == '__main__':
    setup()
    
    try:
        while True:
            distance = checkdist() * 100
            print("Distance: %.2f cm" % distance)
            
            if distance < 20:  # 일정 거리 내에 장애물이 있을 경우
                red()  # RGB LED 빨간색 점등
                ws2812_control((255, 0, 0))  # WS2812 LED 빨간색 점등
            else:
                both_off()  # RGB LED 끄기
                ws2812_control((0, 0, 0))  # WS2812 LED 끄기
            
            status_right, status_middle, status_left = line_tracking()
            print('LF3: %d LF2: %d LF1: %d' % (status_right, status_middle, status_left))
            
            if status_left == 0:  # 왼쪽 라인 감지
                side_on(left_R)  # 왼쪽 온보드 LED 빨간색 점등
            else:
                side_off(left_R)  # 왼쪽 온보드 LED 끄기
            
            if status_middle == 0:  # 가운데 라인 감지
                side_on(left_G)  # 왼쪽 온보드 LED 초록색 점등
            else:
                side_off(left_G)  # 왼쪽 온보드 LED 끄기
            
            if status_right == 0:  # 오른쪽 라인 감지
                side_on(right_R)  # 오른쪽 온보드 LED 빨간색 점등
            else:
                side_off(right_R)  # 오른쪽 온보드 LED 끄기
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        GPIO.cleanup()