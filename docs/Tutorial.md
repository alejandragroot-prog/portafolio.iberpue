#**PRÁCTICAS PRIMER SEMESTRE**

##Microcontroladores 

##SP32

<!-- Control de tamaño usando HTML (cuando se requiera) -->
<img src="../recursos/imgs/esp32.png"  alt="Diagrama del sistema" width="420">

  + El ESP32 es un microcontrolador, al igual que el Arduino, pero tiene conectividad Bluetooth y WiFi ya integrada en la placa. Esto facilita mucho los proyectos de IoT, ya que intercambiarán información constantemente con la red.


??? info "Antes de iniciar"
    Toda la programación se realiza con el programa arduino, asi mismo asegurate de tener descargada la biblioteca ESP 32 by Arduino

    
 	###Práctica 1
  
+	**Materiales:** FFM Fused Filament Fabrication
+	ESP 32
+	1 resistencia de 1K
+	Jumpers
+	Led
+	Botón

 	#Parte 1: 
 	<!-- Control de tamaño usando HTML (cuando se requiera) -->
<img src="../recursos/imgs/prac1prt1.jpeg" alt="Diagrama del sistema" width="420">

``` codigo
  const int led = 33;
  const int led = 25;
  void setup(){
    serial.begin(115200); //INICIO DE LA COMUNICACION
  pinMode(led, OUTPUT);
  pinMode(btn, INPUT);
  }
  void loop(){
    int estado= digitalRead(btn);
    if (estado==1){
      digitalWrite(led,1);
    }
  else{
    digitalWrite(led,0)
  }
} 
```

#Parte 2: Conectar ESP32 por Bluetooth
 	<!-- Control de tamaño usando HTML (cuando se requiera) -->
<img src="../recursos/imgs/bluetoothvideo.mp4" alt="Diagrama del sistema" width="420">

``` codigo
  const int led = 33;
  const int led = 25;
  void setup(){
    serial.begin(115200); //INICIO DE LA COMUNICACION
    serialBT.begin ("Arduinito") // Nombre del dispositivo Bluetooth
  }
  void loop(){
    if (SerialBT.available()){
          String mensaje= SerialBT.readString();
          Serial.printlin("Recibido: "+mensaje);
      if (mensaje==1){
      digitalWrite(led,1);
    }
    else{
    digitalWrite(led,0)
     }
  }
  delay (1000);
}
```

###Práctica 1
  
+	**Materiales:** FFM Fused Filament Fabrication
+	ESP 32
+	1 Micro servomotor 9g
+	Jumpers
+	Motor
+	Puente H

<!-- Control de tamaño usando HTML (cuando se requiera) -->
<img src="../recursos/imgs/materiales2.jpeg" alt="Diagrama del sistema" width="420">

##Puente H

<!-- Control de tamaño usando HTML (cuando se requiera) -->
<img src="../recursos/imgs/puenteH.png" alt="Diagrama del sistema" width="420">
  
+	El puente H es un circuito electrónico que se utiliza para controlar el movimiento de motores, en particular motores de corriente continua (DC). Su nombre se debe a la forma típica del circuito, que se asemeja a la letra «H». El principal objetivo del puente H es permitir que un motor gire en ambas direcciones: hacia adelante y hacia atrás. Para lograr esto, se utilizan conmutadores o interruptores, que pueden ser mecánicos (como relés) o electrónicos (como transistores).


 	#Parte 1: 
 	<!-- Control de tamaño usando HTML (cuando se requiera) -->
<img src="../recursos/imgs/pract2pt1.jpeg" alt="Diagrama del sistema" width="420">

``` codigo
  #define in1 2
  #define in2 15
  void setup(){
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
  }
  void loop(){
   digitalWrite (in1,1);
   digitalWrite (in2,0);
   delay(4000);
   digitalWrite (in1,0);
   digitalWrite (in2,0);
   delay(2000);
   digitalWrite (in1,0);
   digitalWrite (in2,1);
   delay(4000);
} 
```

#Parte 2: Conectar ESP32 por Bluetooth
 	<!-- Control de tamaño usando HTML (cuando se requiera) -->
<img src="../recursos/imgs/bluetoothvideo.mp4" alt="Diagrama del sistema" width="420">

``` codigo
  const int led = 33;
  const int led = 25;
  void setup(){
    serial.begin(115200); //INICIO DE LA COMUNICACION
    serialBT.begin ("Arduinito") // Nombre del dispositivo Bluetooth
  }
  void loop(){
    if (SerialBT.available()){
          String mensaje= SerialBT.readString();
          Serial.printlin("Recibido: "+mensaje);
      if (mensaje==1){
      digitalWrite(led,1);
    }
    else{
    digitalWrite(led,0)
     }
  }
  delay (1000);
}
```

??? info "Antes de iniciar"
    Instala Ultimaker Cura


