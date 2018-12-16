/*
  Incluyendo las librerías necesarias
*/
#include <ESP8266WiFi.h>
#include "ArduinoJson.h"
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <time.h>
#define BotToken "488545891:AAGZ2kbRj4BOQ-I6m89h9xvUskLQb-eKln4"
#define bit0 14
#define bit1 12

/*
  Declaración de variables e instancias de clases
*/
//Configuración de la red WIFI
char nombreDeRed[] = "Familia EB";
char claveDeRed[] = "conectese123";

WiFiClientSecure cliente;
UniversalTelegramBot bot(BotToken, cliente);

int tiempoEntreMensajes = 10;
long tiempoUltimoMensaje;
int banderaAlimento = 0;
int banderaAgua = 0;
String chat_id;
String fechaAlimento;
String horaAlimento;
String fechaAgua;
String horaAgua;

void manejadorDeMensajes(int numeroDeMensajes) {
  for (int i = 0; i < numeroDeMensajes; i++) {

    //se obtiene el id del chat con el bot
    String id = String(bot.messages[i].chat_id);
    chat_id = id;
    //Se obtiene el texto del mensaje
    String comando = bot.messages[i].text;


    //Acciones a tomar en cada caso de comando enviado por el usuario
    if ( comando == "/start" || comando == "/start@AlonRobIncBot") {
        String mensajeDeBienvenida = "Bienvenido a Happpy Pets, su solución en mascotas\n";
        mensajeDeBienvenida += "Utilice el comando /opciones y le diremos las acciones disponibles.\n\n";
        bot.sendMessage(chat_id, mensajeDeBienvenida);
    }

    if (comando == "/opciones" || comando == "/opciones@AlonRobIncBot") {
        String opcionesComandos = "[[\"/DispensarAgua\", \"/DispensarAlimento\"],[\"/UltimaDispensacion\"]]";
        bot.sendMessageWithReplyKeyboard(chat_id, "Seleccione una de las siguientes opciones:", "", opcionesComandos, true);
    }

    if (comando == "/DispensarAgua") {
        banderaAgua = 1;
        time_t now = time(nullptr);
        struct tm* date = localtime(&now);
        fechaAgua = String(date->tm_mday) + "/" + String(date->tm_mon + 1) + "/" + String(date-> tm_year + 1900);
        horaAgua = String(date->tm_hour) + ":" + String(date->tm_min) + ":" + String(date->tm_sec);
        digitalWrite(bit0, HIGH);
        bot.sendMessage(chat_id, "En este momento estamos dispensando una porción de agua a su mascota");
    }

    if (comando == "/DispensarAlimento") {
        banderaAlimento = 1;
        time_t now = time(nullptr);
        struct tm* date = localtime(&now);
        fechaAlimento = String(date->tm_mday) + "/" + String(date->tm_mon + 1) + "/" + String(date-> tm_year + 1900);
        horaAlimento = String(date->tm_hour) + ":" + String(date->tm_min) + ":" + String(date->tm_sec);
        digitalWrite(bit1, LOW);
        bot.sendMessage(chat_id, "En este momento estamos dispensando una porción de alimento a su mascota");
    }

    if (comando == "/UltimaDispensacion") {
        String mensaje = "";
        if(banderaAlimento == 1 && banderaAgua == 1){
            mensaje += "Se dispensó alimento por última vez el " + fechaAlimento + " a las " + horaAlimento;
            mensaje += "\n\n Se dispensó agua por última vez el " + fechaAgua + " a las " + horaAgua;
        }
        else if(banderaAgua == 1 && banderaAlimento == 0){
            mensaje += "\n\n Se dispensó agua por última vez el " + fechaAgua + " a las " + horaAgua;
        }
        else if (banderaAlimento == 1 && banderaAgua == 0) {
            mensaje += "Se dispensó alimento por última vez el " + fechaAlimento + " a las " + horaAlimento;
        }
        else{
            mensaje = "Aún no se han dispensado alimento ni agua";
        }

        bot.sendMessage(chat_id, mensaje);
    }
  }

}

void setup() {

    //Cerrando conexiones anteriores
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    //Iniciando nueva conexion
    WiFi.begin(nombreDeRed, claveDeRed);

    //Pines utilizados
    pinMode(bit0, OUTPUT);//equivalente al D5 en el NodeMCU
    pinMode(bit1, OUTPUT);//equivalente al D6 en el NodeMCU

    //configurando opciones de tiempo real
    int zonaHoraria = -6*3600;
    int dst = 0;
    configTime(zonaHoraria, dst, "pool.ntp.org", "time.nist.gov");
    while(!time(nullptr)){
        delay(1000);
    }
}

void loop() {

    if (millis() > (tiempoUltimoMensaje + tiempoEntreMensajes))  {
        int numeroDeMensajes = bot.getUpdates(bot.last_message_received + 1);
        while (numeroDeMensajes) {
            manejadorDeMensajes(numeroDeMensajes);
            numeroDeMensajes = bot.getUpdates(bot.last_message_received + 1);
        }

        tiempoUltimoMensaje = millis();
    }
}
