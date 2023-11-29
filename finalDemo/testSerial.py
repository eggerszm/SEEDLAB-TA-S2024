import serial

def main():
    ard = serial.Serial("/dev/ttyACM0", baudrate=9600, timeout=0.5)
    print(ard.name)

    ard.reset_output_buffer()

    while True:
        uin = input(">")
        for b in [ord(x) for x in uin]:
            ard.write(b)
        while ard.in_waiting > 0:
            print(ard.read())

if __name__ == "__main__":
    main()
