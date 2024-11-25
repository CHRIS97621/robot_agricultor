import time
import serial
import re

# Variables para almacenar datos GPS
utctime = ''
lat = ''
ulat = ''
lon = ''
ulon = ''
numSv = ''
msl = ''
cogt = ''
cogm = ''
sog = ''
kph = ''
gps_t = 0

# Configuración del puerto serial
try:
    ser = serial.Serial("/dev/ttyUSB0", 9600, timeout=1)
    if ser.isOpen():
        print("GPS Serial Opened! Baudrate=9600")
    else:
        print("GPS Serial Open Failed!")
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit(1)

def GPS_read():
    global utctime, lat, ulat, lon, ulon, numSv, msl, cogt, cogm, sog, kph, gps_t
    try:
        if ser.in_waiting:
            line = ser.readline().decode('ascii', errors='replace').strip()
            if line.startswith('$GNGGA'):
                GGA = line.split(',')
                if len(GGA) < 15:
                    print("GPS no found")
                    gps_t = 0
                    return 0
                else:
                    utctime = GGA[1]
                    lat = GGA[2][0:2] + '°' + GGA[2][2:] + "'" 
                    ulat = GGA[3]
                    lon = GGA[4][0:3] + '°' + GGA[4][3:] + "'"
                    ulon = GGA[5]
                    numSv = GGA[7]
                    msl = GGA[9] + " m"
                    gps_t = 1
                    return 1
            elif line.startswith('$GNVTG') and gps_t == 1:
                VTG = line.split(',')
                cogt = VTG[1] + 'T'
                cogm = VTG[3] + 'M' if VTG[3] else '0.00M'
                sog = VTG[5] + 'Kn'
                kph = VTG[7] + 'Km/h'
    except Exception as e:
        print(f"Error reading GPS data: {e}")
        return 0

try:
    while True:
        if GPS_read():
            print("*********************")
            print(f'UTC Time: {utctime}')
            print(f'Latitude: {lat} {ulat}')
            print(f'Longitude: {lon} {ulon}')
            print(f'Number of satellites: {numSv}')
            print(f'Altitude: {msl}')
            print(f'True north heading: {cogt}')
            print(f'Magnetic north heading: {cogm}')
            print(f'Ground speed: {sog}')
            print(f'Ground speed: {kph}')
            print("*********************")
        time.sleep(1)

except KeyboardInterrupt:
    ser.close()
    print("GPS serial Close!")
