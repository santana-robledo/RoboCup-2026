import pygame
import serial
import time

SERIAL_PORT = "COM5"
BAUD = 115200

AXIS_LX = 0
AXIS_LY = 1
AXIS_RT = 5   # Puede variar según tu control

BTN_LB = 4
BTN_RB = 5
BTN_A  = 0

DEADZONE = 0.15

def to_pwm(v):
    if abs(v) < DEADZONE:
        return 0
    return int(v * 255)

def send_line(ser, text):
    print("PY ->", text.strip())
    ser.write((text + "\n").encode("ascii"))

def main():

    pygame.init()
    pygame.joystick.init()

    js = pygame.joystick.Joystick(0)
    js.init()

    ser = serial.Serial(SERIAL_PORT, BAUD)
    time.sleep(2)

    try:
        while True:

            pygame.event.pump()

            x = js.get_axis(AXIS_LX)
            y = -js.get_axis(AXIS_LY)

            send_line(ser, f"X,{to_pwm(x)}")
            send_line(ser, f"Y,{to_pwm(y)}")

            lb = js.get_button(BTN_LB)
            rb = js.get_button(BTN_RB)
            a  = js.get_button(BTN_A)

            rt_value = js.get_axis(AXIS_RT)
            rt = 1 if rt_value > 0.5 else 0

            send_line(ser, f"L,{lb}")
            send_line(ser, f"R,{rb}")
            send_line(ser, f"T,{rt}")
            send_line(ser, f"A,{a}")

            time.sleep(0.05)

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        pygame.quit()

if __name__ == "__main__":
    main()
