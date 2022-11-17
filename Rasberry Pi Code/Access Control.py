import requests
import serial

ser=serial.Serial('/dev/ttyUSB1',9600)
chatId="-000000000" #personal charId hidden for privacy reason
botToken="9999999999:AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA" #personal chatId is hidden for privacy reason

def senddata():
    while True:
        msgText = ser.readline().decode()
        r = requests.get("https://api.telegram.org/bot"+botToken+"/sendMessage?chat_id="+chatId+"&text="+msgText)

try:
    senddata();
except:
    pass
