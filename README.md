# SolarControl
Arduino Mega2560 Pro, MAX31865 - PT1000, 4 Relay, MQTT, Temperaturdifferenz-Steuerung, Pool-Modus, Pegelsonde

Dieses Projekt dient zur Steuerung meiner Warmwasser-Solaranlage.
Das System ist so ausgelegt, dass die Temperaturdifferenz-Steuerung autonom funktioniert.
Die Solarpumpe wird bei einer Temeraturdifferenz von 8° Celsius eingeschaltet und bei 2° Celsius wieder ausgeschaltet.

Entworfen und Gebaut habe ich das System, weil ich meine Solaranlage im Sommer zur Beheizung meines Pools nutze.
Aus diesem Grund ist ein 3-Wege-Ventil eingebunden mit dem ein Heizkreis mit Wärmetauscher meinen Pool speist.

# Automatik
Die Steuerung hat einen Automatik und zwei manuelle Modi.
-> Auto:  Grundsätzlich läuft die Anlage im Pufferspeicher-Modus. Wenn die Poolsteuerung über MQTT mitteilt, dass die Poolsteuerung im Automatikmodus läuft und die Solaranforderung mit dem Topic Solar_Mode "Pool" übermittelt, dann schaltet der Heizmodus auf Pool um. Sprich die Referenztemperatur wird vom Pool verwendet.

Mit den manuellen Modi kann zwischen Pool und Pufferspeicher umgeschaltet werden.
Umgeschaltet wird mit dem Taster am Gerät oder über MQTT.

# Anschlüße:

• 4 Relays:
  - Solar-Umwälzpumpe
  - 3-Wege-Ventil (Umschaltung Heizkreis Pool und Pufferspeicher)
  - Zirkulationspumpe für Warmwasser Heizkreis -> Steuerung über MQTT (FHEM)
  - Beleuchtung

• Bewegungsmelder zur Steuerung des Raumlichts und des Displays

• Pegelsonde zur Messung eines Heizöltanks oder einer Zisterne

• Status Abfrage des 3-Wege-Ventils

• 3x Temperatursensoren für zusätzliche Messwerte (DS18B20 One-Wire)

• Taster zur Modusauswahl am Gerät selbst

![SolarControl_01](https://user-images.githubusercontent.com/99981722/155893874-d54c76ad-b0a4-45d2-a5e0-2a3c6586c876.jpg)
![SolarControl_02](https://user-images.githubusercontent.com/99981722/155893879-5aab6682-acb7-49e3-96d5-f574d1bd9db6.jpg)
![SolarControl_03](https://user-images.githubusercontent.com/99981722/155893880-f963ca49-6563-42df-99df-23db3f62d530.jpg)

# Platine:
![PCB-Board-3D](https://user-images.githubusercontent.com/99981722/155893882-25ff9f20-6d0f-46f7-8c29-2073359e50b0.jpg)
![PCB-Board](https://user-images.githubusercontent.com/99981722/155893887-806404dc-4511-4801-9ebc-17b270b96d50.jpg)
