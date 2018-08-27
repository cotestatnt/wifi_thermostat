# wifi_thermostat

Questo sketch è poco più di un semplice termostato. 
La funzione richiesta è quella di attivare un'elettrovalvola di scambio quando la temperatura T2 è maggiore di T1 più un'eventuale isteresi ed è maggiore anche di un setpoint impostabile a piacere.

Le due temperature però sono da rilevare in due locali diversi e difficilmente collegabili tra loro.
E' stato quindi deciso di usare una coppia di Wemos D1 Mini (ESP8266) e due sonde di temperatura Dallas DS18B20.

Il Wemos principale è configurato come access point e comunica con il secondo solo con una semplice connessione WebSocket.
Gestisce inoltre un display oled ed un rotary encoder per consentire di variare agevolmente setpoint, soglia di isteresi e tipo di funzionamento delle due temperature.
Per avere una maggiore flessibilità per eventuali applicazioni future, è possibile impostare la temperatura T1 come "locale" oppure "remota".
Nel primo caso la temperatura T1 è quella misurata dalla sonda collegata al Wemos direttamente mentre nel secondo caso il valore di temperatura T1 viene inviato da remoto e quindi corrisponde a quello misurato dal secondo Wemos.

Al secono Wemos è collegata la sola sonda di temperatura DS18B20.
