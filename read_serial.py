import serial, time
try:
    ser = serial.Serial('COM3', 115200, timeout=1)
    
    end_time = time.time() + 15.0
    while time.time() < end_time:
        line = ser.readline()
        if line:
            result = line.decode('utf-8', 'ignore').strip()
            if result:
                print(result)
    ser.close()
except Exception as e:
    print("Error:", e)
