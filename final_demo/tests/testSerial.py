import serial

def main():
    ard = serial.Serial("/dev/ttyACM0")
    print(ard.name)
    print(ard.read())
    ard.write(b"test")

if __name__ == "__main__":
    main()
