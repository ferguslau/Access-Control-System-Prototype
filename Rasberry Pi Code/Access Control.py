import requests
import serial

ser=serial.Serial('/dev/ttyUSB1',9600)
chatId="-658029637"
botToken="5276803832:AAFt1lMnEeg5gicneyAs8AoHJbHuu0OTW5M"

def senddata():
    while True:
        msgText = ser.readline().decode()
        r = requests.get("https://api.telegram.org/bot"+botToken+"/sendMessage?chat_id="+chatId+"&text="+msgText)

try:
    senddata();
except:
    pass