import serial
import time

# Otwórz port szeregowy
ser = serial.Serial('COM4', 115200, timeout=0)
time.sleep(2)  # odczekaj aż urządzenie się zresetuje (np. Marlin/GRBL)
print("serial conneted")
def send_gcode(cmd):
    ser.write((cmd + "\n").encode())
    
    ser.flush() # <--- czeka aż write fizycznie się zakończy

    print("Wysłałem:", cmd)

    time.sleep(0.4)
    # Odczyt odpowiedzi (opcjonalnie)
    #response = ser.readline().decode().strip()
    #print("Odpowiedź:", response)

# Przykładowe komendy G-code:
#6400 - pełen obrót
# F60000 S400000 nie przekraczać 

#dwa pełne obroty o 45 stopni
send_gcode("G1 X800 F10000")
send_gcode("G1 X1600")
send_gcode("G1 X2400")
send_gcode("G1 X3200")
send_gcode("G1 X4000")
send_gcode("G1 X4800")
send_gcode("G1 X5600")
send_gcode("G1 X6400")
send_gcode("G1 X7200")
send_gcode("G1 X8000  F10000")
send_gcode("G1 X8800  F10000")
send_gcode("G1 X9600  F10000")
send_gcode("G1 X10400  F10000")
send_gcode("G1 X11200  F10000")
send_gcode("G1 X12000  F10000")
send_gcode("G1 X12800  F10000")
#powrót do pozycji początkowej
send_gcode("G1 X0  F30000")
#10 obrotów
send_gcode("G1 X64000 F50000")
send_gcode("G1 X0 F50000")

ser.close()