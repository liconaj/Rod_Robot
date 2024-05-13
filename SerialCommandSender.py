import serial
import time

EOC = "!EOC"

arduino = serial.Serial(port="COM3", baudrate=250000, timeout=.1)

print("\nRodRobot Serial Communication\n")

def send_command(value):
    arduino.write(bytes(value, "utf-8"))

try:
    while True:
        cmd = input("G-CODE> ").upper()
        if ";EXIT" in cmd:
            break
        elif not cmd or cmd.startswith(";"):
            continue

        cmd = cmd.split(";")[0]
        send_command(cmd.strip()+"\r\n")
        value = ""
        while not EOC in value:
            value = arduino.readline().decode()
            if not EOC in value :
                print(value, end="")
        print()
except KeyboardInterrupt:
    print("\n\nPROGRAM ENDED BY USER\n")