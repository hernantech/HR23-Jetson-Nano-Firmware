import serial

Ser=serial.Serial(port='/dev/ttyTHS2',baudrate='115200',timeout=0.1)
Ser1=serial.Serial(port='/dev/ttyTHS1',baudrate='115200',timeout=0.1)
while True:
    print("uart2" , Ser.readline())
    print("uart1",Ser1.readline())
