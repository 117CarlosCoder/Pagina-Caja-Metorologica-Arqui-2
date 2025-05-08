#include <SoftwareSerial.h>
#include <avr/pgmspace.h>
#include <string.h>
// Usar DHTStable en lugar de DHT estándar
#include <DHTStable.h>
// NUEVO: Incluir librería para PMS5003
#include <PMS.h> // *** Asegúrate de instalar esta librería ***

// --- Configuración WiFi ---
const char WIFI_SSID[] PROGMEM = "HUAWEI Y9 Prime 2019"; // Reemplaza con tu SSID
const char WIFI_PASS[] PROGMEM = "rudyreyes";    // Reemplaza con tu contraseña

#define WIFI_RX_PIN 2 // Pin RX de Arduino, conectado a TX del ESP8266
#define WIFI_TX_PIN 3 // Pin TX de Arduino, conectado a RX del ESP8266
SoftwareSerial wifiSerial(WIFI_RX_PIN, WIFI_TX_PIN);

// --- Configuración Ubidots ---
const char UBIDOTS_TOKEN[] PROGMEM = "BBUS-1AgE3imgxz5DDJIRwt8HKBw3W7e2nE"; // Tu Token de Ubidots
const char DEVICE_LABEL[] PROGMEM = "arduino-caja-meteorologica"; // Nombre del dispositivo en Ubidots
const char UBIDOTS_SERVER[] PROGMEM = "industrial.api.ubidots.com";

// --- DHT11 con librería DHTStable ---
#define DHTPIN 4 // Pin donde está conectado el sensor DHT11/DHT22
DHTStable dht;

// --- MQ Sensors ---
#define MQ7_PIN A0  // Pin analógico para MQ-7 (CO)
#define MQ135_PIN A1 // Pin analógico para MQ-135 (Calidad Aire: NH3, NOx, CO2 est.)
#define MQ131_PIN A2 // Pin analógico para MQ-131 (Ozono O3 est.)

// --- NUEVO: Configuración PMS5003 ---
#define PMS_RX_PIN 5 // Pin RX de Arduino (Conectar a TX del PMS5003 cuando lo instales)
#define PMS_TX_PIN 6 // Pin TX de Arduino (Conectar a RX del PMS5003 cuando lo instales)
SoftwareSerial pmsSerial(PMS_RX_PIN, PMS_TX_PIN); // Nueva instancia de SoftwareSerial para PMS
PMS pms(pmsSerial); // Objeto PMS usando la instancia serial creada
PMS::DATA pmsData; // Estructura para guardar los datos leídos del PMS (si se leen)

// --- Intervalo de envío y chequeo ---
unsigned long previousSensorReadMillis = 0;
unsigned long previousWifiCheckMillis = 0;
unsigned long previousDhtReadMillis = 0;
const long sensorInterval = 60000;  // Leer sensores y enviar cada 60 segundos
const long wifiCheckInterval = 15000; // Chequear conexión WiFi cada 15 segundos
const long dhtReadInterval = 3000;    // Intentar leer DHT cada 3 segundos

// --- Buffers globales ---
char cmdBuffer[120];       // Buffer para comandos AT
char payloadBuffer[220];   // Buffer para JSON (AUMENTADO para datos PMS)
char requestBuffer[480];   // Buffer para petición HTTP (AUMENTADO para datos PMS)
char responseBuffer[128];  // Buffer para respuestas AT

// --- Estado del Programa ---
bool isWifiConnected = false;
bool sensorsInitialized = false; // Aunque ya no se usa mucho, mantenido por si acaso

// --- Variables para almacenar lecturas de sensores ---
float temperatura = 0.0; // Usar 0.0 o NAN si se prefiere indicar ausencia de lectura
float humedad = 0.0;
bool dhtReadSuccess = false;
int adc_mq7 = 0;
int adc_mq135 = 0;
int adc_mq131 = 0;

// --- Variables para cálculos del MQ135 ---
float ppm_nh3 = 0.0;
float ppm_nox = 0.0;
float ppm_co2 = 0.0; // Inicializar a 0, se sumará la base en el cálculo

// --- NUEVO: Variables para almacenar lecturas del PMS5003 ---
// Inicializadas a 0, se actualizarán si el sensor responde
int pm1_0_env = 0; // PM1.0 Ambiental (ug/m3)
int pm2_5_env = 0; // PM2.5 Ambiental (ug/m3)
int pm10_0_env = 0;// PM10  Ambiental (ug/m3)
bool pmsReadSuccess = false; // Para saber si la última lectura PMS fue exitosa

// --- Prototipos de Funciones ---
bool attemptAutoConnect();
bool checkWifiConnection();
bool sendCommand(const char* cmd, unsigned long timeout, const char* expectedResponse, bool printOutput = true);
// MODIFICADO: Prototipo para incluir datos PMS en el envío
void sendDataToUbidots(float temp, float hum, float co, float o3, int mq135_adc, float nh3, float nox, float co2, int pm1, int pm25, int pm10);
bool readDhtWithRetry();
void calculateMQ135Gases(int adc_value);
// NUEVO: Prototipo para leer PMS5003
bool readPmsSensor();
void getStringFromProgmem(const char* progmemString, char* ramBuffer); // Añadido prototipo

// ==========================================================
//                     SETUP
// ==========================================================
void setup() {
  // Iniciar comunicación serial con el PC (Monitor Serie)
  Serial.begin(9600);
  while (!Serial); // Esperar a que el puerto serie se conecte (necesario para Leonardo/Micro, no daña a otros)

  // Iniciar comunicación serial con el módulo WiFi ESP8266
  wifiSerial.begin(9600);

  // NUEVO: Iniciar comunicación serial con el PMS5003 (aunque no esté conectado)
  pmsSerial.begin(9600); // PMS5003 usa 9600 baudios por defecto

  Serial.println(F("\n-----------------------------------------"));
  Serial.println(F("ARDUINO ESTACIÓN METEO (v1.9 - PMS Fixes)")); // Versión actualizada
  Serial.println(F("-----------------------------------------"));
  Serial.println(F("Comunicación a 9600 baud. Monitor Serie en 9600."));

  // Inicializar sensores (lógica simple ahora)
  Serial.println(F("Iniciando configuración de sensores..."));
  // No se necesita dht.begin() con DHTStable
  // NUEVO: Intentar "despertar" el PMS (no causa error si no está)
  // pms.wakeUp(); // Opcional: Podría enviarse comando wakeUp
  // Serial.println(F("Comando wakeUp enviado a PMS5003 (no requiere respuesta)."));
  Serial.println(F("Sensores listos para lectura."));

  // Intentar conectar al WiFi
  Serial.println(F("Intentando conexión WiFi automática..."));
  isWifiConnected = attemptAutoConnect();

  if (isWifiConnected) {
    Serial.println(F(">>> Conexión WiFi automática EXITOSA!"));
  } else {
    Serial.println(F(">>> ADVERTENCIA: Conexión WiFi automática FALLÓ."));
    Serial.println(F(">>> El sistema intentará leer sensores pero no enviará datos hasta conectar."));
    Serial.println(F(">>> Verificar SSID/PASS, señal WiFi y conexiones del ESP8266."));
  }
  Serial.println(F("-----------------------------------------"));
  Serial.println(F("--- Entrando al loop principal ---"));

  // Realizar una lectura inicial de sensores para tener valores de partida
  Serial.println(F("Realizando lectura inicial de sensores..."));
  dhtReadSuccess = readDhtWithRetry();
  if (dhtReadSuccess) {
    Serial.print(F("  DHT OK: T=")); Serial.print(temperatura, 1);
    Serial.print(F("C, H=")); Serial.print(humedad, 1); Serial.println(F("%"));
  } else {
    Serial.println(F("  Lectura inicial DHT falló."));
  }

  // NUEVO: Lectura inicial PMS (fallará si no está conectado)
  pmsReadSuccess = readPmsSensor();
  if (pmsReadSuccess) {
      Serial.print(F("  PMS OK: PM2.5=")); Serial.println(pm2_5_env);
  } else {
      Serial.println(F("  Lectura inicial PMS falló (esperado si no está conectado)."));
  }
  Serial.println(F("-----------------------------------------"));

  // Establecer tiempo inicial para el primer ciclo de lectura/envío
  // Hacerlo al final asegura que el primer ciclo ocurra 'sensorInterval' ms después de setup()
  previousSensorReadMillis = millis() - sensorInterval + 5000; // Ejecutar primer ciclo pronto (5s tras setup)
  previousWifiCheckMillis = millis();
  previousDhtReadMillis = millis();
}

// ==========================================================
//                      LOOP
// ==========================================================
void loop() {
  unsigned long currentMillis = millis();

  // --- Passthrough Serial <-> WiFi Serial (para comandos AT manuales si es necesario) ---
  if (wifiSerial.available()) {
    Serial.write(wifiSerial.read());
  }
  if (Serial.available()) {
    // Lee comando del usuario y lo envía al ESP8266
    char userInputBuffer[128];
    byte bytesRead = Serial.readBytesUntil('\n', userInputBuffer, sizeof(userInputBuffer)-1);
    if (bytesRead > 0) {
      userInputBuffer[bytesRead] = '\0'; // Null-terminate
      // Remover posible '\r' al final
      if (bytesRead > 0 && userInputBuffer[bytesRead-1] == '\r') {
        userInputBuffer[bytesRead-1] = '\0';
      }
      // Enviar comando al ESP8266 si no está vacío
      if (strlen(userInputBuffer) > 0) {
         wifiSerial.println(userInputBuffer); // println añade \r\n
      }
    }
  }

  // --- Lectura del DHT (más frecuente que el envío) ---
  if (currentMillis - previousDhtReadMillis >= dhtReadInterval) {
    previousDhtReadMillis = currentMillis;
    dhtReadSuccess = readDhtWithRetry(); // Intenta leer y actualiza 'temperatura', 'humedad'
    // Log de lectura DHT solo ocasionalmente para no saturar
    if (dhtReadSuccess && (currentMillis / 10000) % 3 == 0) { // Cada 30 segundos aprox
        //Serial.print(F("[DHT Status] T:")); Serial.print(temperatura, 1); // Descomentar si se necesita log frecuente
        //Serial.print(F("C H:")); Serial.print(humedad, 1); Serial.println(F("%"));
    } else if (!dhtReadSuccess && (currentMillis / 10000) % 3 == 0) {
        // Serial.println(F("[DHT Status] Failed read attempt")); // Log opcional de fallo
    }
  }

  // --- Chequeo de Conexión WiFi (periódico) ---
  if (currentMillis - previousWifiCheckMillis >= wifiCheckInterval) {
    previousWifiCheckMillis = currentMillis;
    bool previousConnectionState = isWifiConnected;
    isWifiConnected = checkWifiConnection();
    if (isWifiConnected && !previousConnectionState) {
        Serial.println(F(">>> Conexión WiFi RESTABLECIDA."));
    } else if (!isWifiConnected && previousConnectionState) {
        Serial.println(F(">>> Conexión WiFi PERDIDA."));
    } else if (!isWifiConnected) {
        // Serial.println(F("[WiFi Status] Still disconnected.")); // Log opcional
        // Podría intentar reconectar aquí si se desea:
        // attemptAutoConnect();
    }
  }

  // --- Ciclo Principal: Leer TODOS los sensores y Enviar a Ubidots ---
  if (currentMillis - previousSensorReadMillis >= sensorInterval) {
    previousSensorReadMillis = currentMillis;

    Serial.println(F("\n--- INICIO CICLO LECTURA/ENVÍO ---"));

    // 1. Leer sensores Analógicos MQ
    adc_mq7 = analogRead(MQ7_PIN);
    adc_mq135 = analogRead(MQ135_PIN);
    adc_mq131 = analogRead(MQ131_PIN);

    // 2. Calcular gases a partir de lectura MQ135
    calculateMQ135Gases(adc_mq135); // Actualiza ppm_nh3, ppm_nox, ppm_co2

    // 3. (Re)Intentar lectura DHT por si falló la última vez
    if (!dhtReadSuccess) {
      dhtReadSuccess = readDhtWithRetry();
      if(dhtReadSuccess) Serial.println(F("  INFO: Lectura DHT recuperada antes de envío."));
    }

    // 4. NUEVO: Intentar leer sensor PMS5003
    pmsReadSuccess = readPmsSensor(); // Actualiza pmX_env y devuelve éxito/fallo

    // 5. Mostrar lecturas en Monitor Serie
    Serial.print(F("  MQ7(ADC):")); Serial.print(adc_mq7);
    Serial.print(F(" | MQ135(ADC):")); Serial.print(adc_mq135);
    Serial.print(F(" | MQ131(ADC):")); Serial.println(adc_mq131);
    Serial.print(F("  Gases (est): NH3=")); Serial.print(ppm_nh3, 1);
    Serial.print(F(" NOx=")); Serial.print(ppm_nox, 1);
    Serial.print(F(" CO2=")); Serial.print(ppm_co2, 1); Serial.println(F(" ppm"));
    if(dhtReadSuccess){
      Serial.print(F("  DHT: T=")); Serial.print(temperatura, 1);
      Serial.print(F("C H=")); Serial.print(humedad, 1); Serial.println(F("%"));
    } else {
      Serial.println(F("  DHT: Lectura fallida (usando valor anterior o 0)."));
    }
    if(pmsReadSuccess){
      Serial.print(F("  PMS: PM1.0=")); Serial.print(pm1_0_env);
      Serial.print(F(" PM2.5=")); Serial.print(pm2_5_env);
      Serial.print(F(" PM10=")); Serial.print(pm10_0_env); Serial.println(F(" ug/m3"));
    } else {
      Serial.println(F("  PMS: Lectura fallida (sensor no conectado o error). Enviando ceros."));
    }

    // 6. Enviar datos a Ubidots (SOLO si hay conexión WiFi)
    if (isWifiConnected) {
      Serial.println(F("  WiFi OK. Preparando envío a Ubidots..."));
      // Calcular valores estimados para CO y O3 (AJUSTAR FACTORES SEGÚN CALIBRACIÓN)
      float ppm_co_estimado = map(adc_mq7, 0, 1023, 0, 100); // Mapeo MUY simple, ajustar!
      float ppm_o3_estimado = map(adc_mq131, 0, 1023, 0, 50); // Mapeo MUY simple, ajustar!

      // Llamar a la función de envío con todos los datos
      sendDataToUbidots(temperatura, humedad, ppm_co_estimado, ppm_o3_estimado, adc_mq135,
                        ppm_nh3, ppm_nox, ppm_co2,
                        pm1_0_env, pm2_5_env, pm10_0_env); // Se envían los valores actuales (0 si falló PMS)
    } else {
      Serial.println(F("  WiFi desconectado. Envío a Ubidots OMITIDO."));
    }
    Serial.println(F("--- FIN CICLO LECTURA/ENVÍO ---"));
  }
} // Fin loop()

// ==========================================================
//         FUNCIONES AUXILIARES Y DE SENSORES
// ==========================================================

// --- Función para copiar string desde PROGMEM a RAM ---
void getStringFromProgmem(const char* progmemString, char* ramBuffer) {
  strcpy_P(ramBuffer, progmemString);
}

// --- Función para calcular gases del MQ135 (estimación) ---
void calculateMQ135Gases(int adc_value) {
  // Convertir ADC a voltaje (asumiendo Vref = 5V, ADC 10 bits)
  float voltaje = adc_value * (5.0 / 1023.0);

  // Factores de conversión MUY APROXIMADOS - REQUIEREN CALIBRACIÓN REAL
  // Estos son solo ejemplos y pueden no ser precisos para tu sensor específico.
  ppm_nh3 = voltaje * 10.0; // Factor ejemplo
  ppm_nox = voltaje * 15.0; // Factor ejemplo
  // Para CO2, a menudo se usa una relación logarítmica o exponencial,
  // pero una aproximación lineal simple sobre el nivel base puede ser un inicio.
  // Asumimos ~400ppm como base atmosférica. El sensor mide el *aumento* sobre esto.
  // Esta fórmula es MUY básica y probablemente incorrecta:
  ppm_co2 = 400.0 + (voltaje * 1000.0); // Base 400ppm + factor muy especulativo

  // Aplicar límites razonables para evitar valores absurdos
  ppm_nh3 = constrain(ppm_nh3, 0, 200);     // Rango típico NH3
  ppm_nox = constrain(ppm_nox, 0, 300);     // Rango típico NOx
  ppm_co2 = constrain(ppm_co2, 400, 10000); // Rango CO2 (ambiente a muy contaminado)
}

// --- Función de lectura DHT con reintentos ---
bool readDhtWithRetry() {
  for (int i = 0; i < 3; i++) {
    int chk = dht.read11(DHTPIN); // Usar read22 si tienes un DHT22
    if (chk == DHTLIB_OK) {
      float h = dht.getHumidity();
      float t = dht.getTemperature();
      // Validar rangos razonables (evita lecturas erróneas como -999)
      if (t > -40 && t < 85 && h >= 0 && h <= 100) {
        humedad = h;
        temperatura = t;
        return true; // Éxito
      }
    }
    delay(500); // Espera 0.5s entre reintentos
  }
  // Si llegamos aquí, todos los intentos fallaron
  return false;
}

// --- NUEVO: Función para leer el sensor PMS5003 ---
bool readPmsSensor() {
  // Intentar leer datos del PMS. readUntil esperará hasta recibir un frame
  // completo o hasta que pase el timeout (en milisegundos).
  // El timeout debe ser suficiente para recibir un frame (1-2 segundos suele bastar).
  if (pms.readUntil(pmsData, 2000)) { // Intenta leer durante 2 segundos máx.
    // Lectura exitosa. Guardamos los valores ambientales (AE = Atmospheric Environment)
    pm1_0_env = pmsData.PM_AE_UG_1_0;
    pm2_5_env = pmsData.PM_AE_UG_2_5;
    pm10_0_env = pmsData.PM_AE_UG_10_0;
    return true; // Éxito
  } else {
    // Falló la lectura (timeout o datos corruptos)
    // Esto es normal si el sensor no está conectado.
    // Aseguramos que los valores sean 0 si falla la lectura.
    pm1_0_env = 0;
    pm2_5_env = 0;
    pm10_0_env = 0;
    return false; // Falla
  }
}

// --- Chequea si el ESP8266 está conectado a una red WiFi ---
bool checkWifiConnection() {
  // AT+CWJAP? devuelve el SSID al que está conectado si lo está.
  // Buscamos la respuesta "+CWJAP:" que indica que está conectado a algo.
  // El 'false' final evita que se imprima la respuesta completa de AT+CWJAP? en el log.
  if (sendCommand("AT+CWJAP?", 3000, "+CWJAP:", false)) {
    return true; // Conectado
  } else {
    return false; // No conectado (o ESP no responde)
  }
}

// --- Intenta conectar el ESP8266 a la red WiFi configurada ---
bool attemptAutoConnect() {
  char localSsidBuffer[35]; // Buffer RAM para SSID (ajustar tamaño si SSID es largo)
  char localPassBuffer[65]; // Buffer RAM para Password (ajustar tamaño si es largo)
  char cmdConnectBuffer[110]; // Buffer para comando AT+CWJAP

  Serial.println(F("  1. Verificando respuesta del ESP8266 (AT)..."));
  if (!sendCommand("AT", 2000, "OK", false)) { // Usar 'false' para no imprimir OK si funciona
    Serial.println(F("     ERROR: ESP8266 no responde a comando AT. Verificar conexiones/alimentación/baudios."));
    return false;
  }
  Serial.println(F("     OK. ESP8266 responde."));
  delay(500);

  Serial.println(F("  2. Configurando modo Station (AT+CWMODE=1)..."));
  if (!sendCommand("AT+CWMODE=1", 3000, "OK", false)) {
     // Podría responder "no change", lo cual también es aceptable. Re-chequear.
     if (!sendCommand("AT+CWMODE?", 2000, "+CWMODE:1", false)){
        Serial.println(F("     ADVERTENCIA: No se pudo confirmar modo Station (CWMODE=1)."));
        // Continuar de todos modos, puede que ya estuviera en modo 1.
     } else {
        Serial.println(F("     OK. Modo Station confirmado."));
     }
  } else {
      Serial.println(F("     OK. Modo Station establecido."));
  }
  delay(500);

  // Copiar SSID y Password de PROGMEM a RAM para usarlos en sprintf
  getStringFromProgmem(WIFI_SSID, localSsidBuffer);
  getStringFromProgmem(WIFI_PASS, localPassBuffer);

  Serial.print(F("  3. Intentando conectar a: ")); Serial.println(localSsidBuffer);
  sprintf(cmdConnectBuffer, "AT+CWJAP=\"%s\",\"%s\"", localSsidBuffer, localPassBuffer);

  // Usar un timeout largo para la conexión WiFi (puede tardar 20s o más)
  // Esperamos "WIFI GOT IP" que confirma la conexión y obtención de IP.
  if (!sendCommand(cmdConnectBuffer, 25000, "WIFI GOT IP", true)) { // 'true' para ver logs de conexión
    Serial.println(F("     ERROR: Falló la conexión WiFi (comando AT+CWJAP)."));
    Serial.println(F("     Verificar SSID, contraseña y señal WiFi."));
    // Comprobar si, a pesar del fallo, ya está conectado (quizás conectó antes)
    if (checkWifiConnection()) {
        Serial.println(F("     INFO: A pesar del timeout/error, parece que ya está conectado."));
        return true; // Considerarlo conectado si checkWifiConnection dice que sí
    }
    return false; // Realmente no conectado
  }

  Serial.println(F("     ÉXITO: WiFi conectado y con IP!"));
  delay(1000); // Pequeña pausa tras conectar exitosamente
  return true;
}

// --- Envía los datos de los sensores a Ubidots ---
// MODIFICADO: Acepta valores PMS y los incluye en el JSON
void sendDataToUbidots(float temp, float hum, float co, float o3, int mq135_adc, float nh3, float nox, float co2, int pm1, int pm25, int pm10) {
  char tempStr[8], humStr[8], coStr[8], o3Str[8];
  char nh3Str[8], noxStr[8], co2Str[8];
  // NUEVO: Buffers para PMS (enteros, necesitan unos 6 chars para "65535")
  char pm1Str[6], pm25Str[6], pm10Str[6];
  char localBuffer[100]; // Buffer temporal para copiar strings de PROGMEM

  // Convertir floats a strings (formato: min width, decimal places, buffer)
  dtostrf(temp, 4, 1, tempStr); // ej: "25.3"
  dtostrf(hum, 4, 1, humStr);  // ej: "55.1"
  dtostrf(co, 4, 1, coStr);    // ej: "15.2" ppm (Estimado)
  dtostrf(o3, 4, 2, o3Str);    // ej: "0.05" ppm (Estimado)
  dtostrf(nh3, 4, 1, nh3Str);  // ej: "5.1" ppm (Calculado)
  dtostrf(nox, 4, 1, noxStr);  // ej: "10.5" ppm (Calculado)
  dtostrf(co2, 5, 0, co2Str);  // ej: "650" ppm (Calculado, sin decimales)

  // NUEVO: Convertir enteros PMS a strings (base 10)
  itoa(pm1, pm1Str, 10);   // ej: "15"
  itoa(pm25, pm25Str, 10); // ej: "25"
  itoa(pm10, pm10Str, 10); // ej: "30"

  // Construir el payload JSON. ASEGÚRATE que las claves ("temperatura", "humedad", etc.)
  // coincidan EXACTAMENTE con las Variables creadas en tu Device de Ubidots.
  // El tamaño de payloadBuffer debe ser suficiente.
  strcpy(payloadBuffer, "{");
  sprintf(payloadBuffer + strlen(payloadBuffer), "\"temperatura\":%s,", tempStr); // Clave: "temperatura"
  sprintf(payloadBuffer + strlen(payloadBuffer), "\"humedad\":%s,", humStr);       // Clave: "humedad"
  sprintf(payloadBuffer + strlen(payloadBuffer), "\"co_estimado\":%s,", coStr); // Clave: "co_estimado" (o la que uses)
  sprintf(payloadBuffer + strlen(payloadBuffer), "\"o3_estimado\":%s,", o3Str); // Clave: "o3_estimado" (o la que uses)
  sprintf(payloadBuffer + strlen(payloadBuffer), "\"mq135_adc\":%d,", mq135_adc); // Clave: "mq135_adc" (valor crudo)
  sprintf(payloadBuffer + strlen(payloadBuffer), "\"nh3_calc\":%s,", nh3Str);     // Clave: "nh3_calc" (calculado)
  sprintf(payloadBuffer + strlen(payloadBuffer), "\"nox_calc\":%s,", noxStr);     // Clave: "nox_calc" (calculado)
  sprintf(payloadBuffer + strlen(payloadBuffer), "\"co2_calc\":%s,", co2Str);     // Clave: "co2_calc" (calculado)
  // NUEVO: Añadir datos PMS al JSON (usa las claves que definas en Ubidots)
  sprintf(payloadBuffer + strlen(payloadBuffer), "\"pm1\":%s,", pm1Str);      // Clave: "pm1"
  sprintf(payloadBuffer + strlen(payloadBuffer), "\"pm25\":%s,", pm25Str);     // Clave: "pm25"
  sprintf(payloadBuffer + strlen(payloadBuffer), "\"pm10\":%s", pm10Str);     // Clave: "pm10" (ÚLTIMA VARIABLE SIN COMA)
  strcat(payloadBuffer, "}"); // Cerrar el JSON

  int payloadLen = strlen(payloadBuffer);
  Serial.print(F("    Payload JSON (len=")); Serial.print(payloadLen); Serial.println(F("):"));
  // ***** CORRECCIÓN 1: Cambiado Serial.println(F(...) + String(...)) a dos prints *****
  Serial.print(F("      "));
  Serial.println(payloadBuffer); // Imprimir payload para depuración

  // --- Enviar usando comandos AT ---
  Serial.println(F("    Iniciando conexión TCP a Ubidots..."));
  getStringFromProgmem(UBIDOTS_SERVER, localBuffer);
  sprintf(cmdBuffer, "AT+CIPSTART=\"TCP\",\"%s\",80", localBuffer);

  // Intentar conectar al servidor de Ubidots
  if (!sendCommand(cmdBuffer, 10000, "CONNECT", false)) {
    Serial.println(F("      ERROR: Falló conexión TCP (AT+CIPSTART)."));
    sendCommand("AT+CIPCLOSE", 2000, "OK", false); // Intentar cerrar por si quedó abierta
    return; // Salir de la función si no se pudo conectar
  }
  Serial.println(F("      OK. Conectado a servidor Ubidots."));

  // Construir la petición HTTP POST completa
  getStringFromProgmem(DEVICE_LABEL, localBuffer); // Obtener Device Label de PROGMEM
  sprintf(requestBuffer, "POST /api/v1.6/devices/%s HTTP/1.1\r\n", localBuffer); // Línea POST

  getStringFromProgmem(UBIDOTS_SERVER, localBuffer); // Obtener Host de PROGMEM
  sprintf(requestBuffer + strlen(requestBuffer), "Host: %s\r\n", localBuffer); // Cabecera Host

  strcat(requestBuffer, "Content-Type: application/json\r\n"); // Cabecera Content-Type

  getStringFromProgmem(UBIDOTS_TOKEN, localBuffer); // Obtener Token de PROGMEM
  sprintf(requestBuffer + strlen(requestBuffer), "X-Auth-Token: %s\r\n", localBuffer); // Cabecera Token

  sprintf(requestBuffer + strlen(requestBuffer), "Content-Length: %d\r\n", payloadLen); // Cabecera Content-Length

  strcat(requestBuffer, "Connection: close\r\n"); // Pedir cerrar conexión después de la respuesta

  strcat(requestBuffer, "\r\n"); // Línea en blanco OBLIGATORIA entre cabeceras y cuerpo

  strcat(requestBuffer, payloadBuffer); // Añadir el cuerpo JSON al final

  int requestLen = strlen(requestBuffer);
  // Serial.print(F("    Petición HTTP (len=")); Serial.print(requestLen); Serial.println(F("):"));
  // Serial.println(requestBuffer); // Descomentar para depurar la petición completa

  // Enviar comando CIPSEND para indicar tamaño de datos a enviar
  sprintf(cmdBuffer, "AT+CIPSEND=%d", requestLen);
  // Esperar el prompt '>' que indica que el ESP está listo para recibir los datos
  if (!sendCommand(cmdBuffer, 5000, ">", false)) {
    Serial.println(F("      ERROR: Falló comando AT+CIPSEND (no se recibió '>')."));
    sendCommand("AT+CIPCLOSE", 2000, "OK", false); // Intentar cerrar conexión
    return; // Salir si CIPSEND falla
  }
  Serial.println(F("      OK. Recibido '>'. Enviando datos HTTP..."));

  // Enviar la petición HTTP completa al ESP8266, que la reenviará por TCP
  wifiSerial.print(requestBuffer);

  // Esperar la respuesta del servidor (buscamos "SEND OK" del ESP y "HTTP/1.1 200 OK" o "201 Created" de Ubidots)
  unsigned long responseStartTime = millis();
  bool sendOkReceived = false; // Flag para saber si el ESP confirmó el envío TCP
  bool httpSuccess = false;    // Flag para saber si Ubidots respondió con éxito (2xx)
  int httpStatusCode = 0;      // Para guardar el código HTTP recibido
  // ***** CORRECCIÓN 2: Declaración de responseIdx faltante *****
  byte responseIdx = 0;        // Índice para llenar responseBuffer

  Serial.print(F("      Esperando respuesta de Ubidots (máx 15s)... "));
  responseBuffer[0] = '\0'; // Asegurar que el buffer esté limpio al empezar

  while (millis() - responseStartTime < 15000) { // Timeout de 15 segundos para la respuesta
    while (wifiSerial.available()) {
      char c = wifiSerial.read();
      // Añadir al buffer de respuesta general si hay espacio
      if (responseIdx < sizeof(responseBuffer) - 1) {
          responseBuffer[responseIdx++] = c;
          responseBuffer[responseIdx] = '\0'; // Siempre terminar con null
      } else {
          // Buffer lleno, podríamos rotar o simplemente dejar de añadir
          // Para este propósito, dejar de añadir es suficiente
      }
    } // fin while available

    // Comprobar si ya recibimos SEND OK del ESP
    if (!sendOkReceived && strstr(responseBuffer, "SEND OK") != NULL) {
      sendOkReceived = true;
      // Serial.println(F("DEBUG: SEND OK recibido.")); // Log debug
    }

    // Comprobar si recibimos una respuesta HTTP 200 OK o 201 Created (éxito)
    // Solo lo consideramos válido si ya recibimos SEND OK
    if (sendOkReceived && !httpSuccess) {
       if (strstr(responseBuffer, "HTTP/1.1 200 OK") != NULL) {
         httpSuccess = true;
         httpStatusCode = 200;
       } else if (strstr(responseBuffer, "HTTP/1.1 201 Created") != NULL) {
         httpSuccess = true;
         httpStatusCode = 201;
       }
       // También detectar errores HTTP comunes
       else if (strstr(responseBuffer, "HTTP/1.1 4") != NULL) { // Error 4xx (ej: 400 Bad Request, 401 Unauthorized)
          // Podríamos intentar parsear el código exacto, pero 400 es genérico
          if (strstr(responseBuffer, " 401 ") != NULL) httpStatusCode = 401;
          else httpStatusCode = 400;
          httpSuccess = false; // Error
          break; // Salir del bucle si hay error HTTP
       } else if (strstr(responseBuffer, "HTTP/1.1 5") != NULL) { // Error 5xx (Server error)
          httpStatusCode = 500; // Simplificado
          httpSuccess = false; // Error
          break; // Salir del bucle si hay error HTTP
       }
    }

    // Comprobar si la conexión fue cerrada (esperado después de "Connection: close")
    if (strstr(responseBuffer, "CLOSED") != NULL) {
      // Serial.println(F("DEBUG: CLOSED recibido.")); // Log debug
      break; // Salir del bucle de espera, ya terminó la transacción
    }
    // Comprobar si el ESP reportó un error general ANTES de SEND OK
    if (strstr(responseBuffer, "ERROR") != NULL && !sendOkReceived) {
        Serial.println(F("\n      ERROR: ESP reportó ERROR durante el envío (antes de SEND OK)."));
        httpSuccess = false; // Marcar como fallo
        break;
    }

    // Si ya tenemos éxito HTTP confirmado, podemos salir
    if (httpSuccess) {
        break;
    }

    delay(50); // Pequeña pausa para no saturar CPU
  } // fin while timeout

  // Evaluar resultado final
  if (httpSuccess && sendOkReceived) {
    Serial.println(F(" ÉXITO!"));
    // Serial.print(F("      Respuesta Ubidots: HTTP ")); Serial.println(httpStatusCode); // Opcional log éxito
  } else {
    Serial.println(F(" FALLÓ!"));
    if (!sendOkReceived) Serial.println(F("      Causa: No se confirmó 'SEND OK' del ESP."));
    if (sendOkReceived && !httpSuccess) {
        Serial.print(F("      Causa: No se recibió respuesta HTTP 2xx de Ubidots (recibido: "));
        if (httpStatusCode != 0) Serial.print(httpStatusCode); else Serial.print("Timeout/Desconocido");
        Serial.println(F(")"));
    } else if (!sendOkReceived && !httpSuccess && httpStatusCode != 0) {
        // Caso raro: error HTTP detectado pero sin SEND OK (?)
         Serial.print(F("      Causa: Error HTTP ")); Serial.print(httpStatusCode); Serial.println(F(" detectado sin 'SEND OK' previo."));
    }
    Serial.println(F("      Respuesta parcial/completa recibida:"));
    Serial.println(responseBuffer); // Imprimir lo que se haya recibido para depurar
  }

  // Asegurarse de cerrar la conexión TCP, aunque hayamos pedido "Connection: close"
  // Esto ayuda a liberar recursos en el ESP8266.
  sendCommand("AT+CIPCLOSE", 3000, "OK", false); // No imprimir OK si funciona
}

// --- Envía un comando AT al ESP8266 y espera una respuesta específica ---
// Devuelve true si se encontró la respuesta esperada, false en caso contrario (timeout o error)
bool sendCommand(const char* cmd, unsigned long timeout, const char* expectedResponse, bool printOutput) {
    byte responseIdx = 0;      // Índice actual en responseBuffer
    bool foundResponse = false; // Indica si se encontró expectedResponse
    bool foundError = false;    // Indica si se encontró "ERROR" o "FAIL"

    // 1. Limpiar buffer de recepción serial del WiFi por si hay datos viejos
    while (wifiSerial.available()) {
        wifiSerial.read();
    }

    // 2. Enviar el comando AT al ESP8266 (println añade \r\n)
    wifiSerial.println(cmd);
    // Ocultar contraseña en el log si se imprime el comando de conexión
    if (printOutput && strncmp(cmd, "AT+CWJAP", 8) == 0) {
        Serial.println(F("     CMD>> AT+CWJAP=\"SSID\",\"********\"")); // Ocultar pass
    } else if (printOutput) {
        // Serial.print(F("CMD>> ")); Serial.println(cmd); // Log opcional del comando
    }


    // 3. Esperar la respuesta durante el 'timeout' especificado
    unsigned long startTime = millis();
    responseBuffer[0] = '\0'; // Limpiar buffer de respuesta

    while (millis() - startTime < timeout) {
        while (wifiSerial.available()) {
            char c = wifiSerial.read();
            // Añadir caracter al buffer si hay espacio
            if (responseIdx < sizeof(responseBuffer) - 1) {
                responseBuffer[responseIdx++] = c;
                responseBuffer[responseIdx] = '\0'; // Siempre terminar con null
            }
            // (Opcional) Imprimir la respuesta mientras se recibe para depuración detallada
            // if (printOutput) { Serial.write(c); }
        }

        // Comprobar si la respuesta ESPERADA está en el buffer
        if (strstr(responseBuffer, expectedResponse) != NULL) {
            foundResponse = true;
            break; // Salir del bucle while timeout, ¡encontrado!
        }

        // Comprobar si se recibió "ERROR" o "FAIL" (y no era lo esperado)
        if (strcmp(expectedResponse, "ERROR") != 0 && strcmp(expectedResponse, "FAIL") != 0) {
             if (strstr(responseBuffer, "ERROR") != NULL || strstr(responseBuffer, "FAIL") != NULL) {
                foundError = true;
                foundResponse = false; // Marcar que la respuesta correcta no se encontró
                break; // Salir del bucle while timeout, ¡error encontrado!
             }
        }
    } // Fin while timeout

    // 4. Informar resultado (si printOutput es true y hubo problema o es log específico)
    if (printOutput) {
        if (!foundResponse && !foundError) {
            // No imprimir timeout para comandos muy comunes que pueden fallar/tardar (OK, SEND OK, WIFI GOT IP)
            // Solo imprimir timeout para otros comandos si fallan.
            if (strcmp(expectedResponse,"OK") != 0 && strcmp(expectedResponse,"SEND OK") != 0 && strcmp(expectedResponse, "WIFI GOT IP") != 0 && strcmp(expectedResponse, ">") != 0) {
                 Serial.print(F("     Timeout esperando '")); Serial.print(expectedResponse); Serial.println(F("'."));
                 // Serial.print(F("     Buffer recibido: "));Serial.println(responseBuffer); // Opcional: mostrar buffer en timeout
            }
        } else if (foundError) {
            // No imprimir "ERROR detectado" si el comando era el de conexión (AT+CWJAP),
            // ya que el log de attemptAutoConnect es más específico.
            if (strncmp(cmd, "AT+CWJAP", 8) != 0) {
                Serial.println(F("     ERROR/FAIL detectado en respuesta AT."));
                // Serial.print(F("     Buffer recibido: "));Serial.println(responseBuffer); // Opcional: mostrar buffer en error
            }
        } else {
            // Éxito, no imprimir nada extra aquí generalmente.
            // Los mensajes de éxito específicos se imprimen en las funciones llamadoras (ej: "Modo Station OK.")
        }
    }

    return foundResponse; // Devolver true SOLO si se encontró la respuesta esperada
}
