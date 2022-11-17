# Access-Control-System-Prototype

Project Description: 

This project is to build an embedded system prototype for Smart Access Control System in low cost which aims to control and 
monitor access to a sensitive site. An effective Access Control System can mitigate the risk of information 
and sensitive sites being accessed without the appropriate authorization, unlawfully and risk of a data breach. 
This project will improve the access control system by sending a notification not only to monitor in security room or booth, 
but also to Telegram App so that everyone in the security team can receive alert everywhere. Even if the guard go to patrol, 
they will still receive the alert as well when there are abnormal situations.

After incomer input the 4-digit password code by pressing push buttons, a notification showing ‘Access Granted/Access Denied’ 
will be shown in OLED display and sent to the monitor and conversation in the Telegram App through Raspberry PI. 
The notification in the App can be saved as history. 

If the password is correct, the door will open by driving Stepper Motor and the RGB will turn to Green Colour. 
Otherwise, the door will not open, the RGB Led will turn to Red Colour, as well as the system will trigger a multi-tone auditory signal 
exciting a buzzer by applying the PWM signals at different frequencies. After 5 seconds, incomer can enter password again.

Addition Function: A temperature sensor is added to detect fire constantly. If the temperature detected is higher than a certain degree,
an alert will be shown to OLED display and the app.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------


![WhatsApp Image 2022-11-16 at 10 29 20 PM](https://user-images.githubusercontent.com/118412269/202348065-70528f2a-9010-4974-94e4-114fcdcc5397.jpeg)


![Telegram app](https://user-images.githubusercontent.com/118412269/202347252-b5aa6995-577e-4730-bc9d-2a610292cff5.jpeg)
