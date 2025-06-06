# Descrição do Projeto #
Enchentes são desastres naturais que ocorrem frequentemente aqui no Brasil e que sempre ocorre alguma tragédia.
Pensando por esse lado, criamos uma solução que prevê as enchentes e reduz o número de vítimas no processo.

# Funcionalidades principais #
- 📊 Medição contínua do nível da água com sensor ultrassônico
- 🚨 Alerta sonoro (buzzer) quando o nível atinge limite perigoso
- 📶 Transmissão de dados via Wi-Fi para broker MQTT público
- 📱 Visualização dos dados em tempo real via Node-RED
- 📈 Histórico de medições e tendências para previsão de enchentes
- 🖥️ Interface local com display LCD para status atual

# Componentes do Sistema #
## Hardware ##
- ESP32 (com Wi-Fi integrado)
- Placa Half BreadBoard
- Sensor ultrassônico HC-SR04 (para medição de distância/nível da água)
- Display LCD 16x2 com interface I2C
- Buzzer ativo para alertas sonoros
- Fonte de alimentação 5V

## Software ##
- Wokwi para ESP32 (este repositório)
- Broker MQTT público (HiveMQ)
- Dashboard Node-RED para visualização

# Configuração do Sistema #
1º Montagem Física:

- Instale o Sensor Ultrassônico na placa Half Breadboard
- Posicione o display LCD em local visível
- Conecte o buzzer para alertas audíveis

2º Conexões:

Sensor Ultrassônico:

- TRIG → GPIO13
- ECHO → GPIO12
- VCC → 5V
- GND → GND

LCD I2C:

- SDA → GPIO21 (ESP32)
- SCL → GPIO22 (ESP32)
- VCC → 5V
- GND → GND

Buzzer:
- → GPIO14
- → GND

# Configurações do Software #

Após ter feito toda a montagem no wokwi ou com peças compradas, cole o código a seguir (junto com a explicação) na IDE:

```cpp
#include <WiFi.h> // Biblioteca para conectar o ESP32 à rede Wi-Fi
#include <PubSubClient.h> // Biblioteca para comunicação MQTT
#include <Wire.h> // Biblioteca para comunicação I2C (usada pelo LCD)
#include <LiquidCrystal_I2C.h> // Biblioteca para controle do display LCD I2C
#include <ArduinoJson.h> // Biblioteca para criação e manipulação de objetos JSON

#define TRIG_PIN 13 // Pino TRIG do sensor ultrassônico
#define ECHO_PIN 12 // Pino ECHO do sensor ultrassônico
#define BUZZER_PIN 14 // Pino onde o buzzer está conectado
#define NIVEL_PERIGOSO 15 // Distância (em cm) abaixo da qual há risco de enchente

LiquidCrystal_I2C lcd(0x27, 16, 2); // Inicializa o display LCD no endereço 0x27 com 16 colunas e 2 linhas

// Dados da rede Wi-Fi
const char* ssid = "Wokwi-GUEST";
const char* password = "";
const char* mqtt_server = "broker.hivemq.com"; // Servidor MQTT público
const int mqtt_port = 1883; // Porta padrão do MQTT

WiFiClient espClient; // Objeto para conexão Wi-Fi
PubSubClient client(espClient); // Objeto para cliente MQTT usando o Wi-Fi

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);

  // Exibe mensagem no LCD enquanto conecta ao Wi-Fi
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Conectando WiFi");
  lcd.setCursor(0, 1);
  lcd.print(ssid);

  WiFi.begin(ssid, password); // Inicia conexão Wi-Fi

  int tentativas = 0;
  // Aguarda conexão ou até 20 tentativas
  while (WiFi.status() != WL_CONNECTED && tentativas < 20) {
    delay(500);
    Serial.print(".");
    tentativas++;
  }

  // Verifica se conectou com sucesso
  if(WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("WiFi conectado");
    Serial.println("Endereço IP: ");
    Serial.println(WiFi.localIP());
    
    // Mostra IP no LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi Conectado");
    lcd.setCursor(0, 1);
    lcd.print(WiFi.localIP());
  } else {
    // Falha na conexão
    Serial.println("Falha na conexão WiFi");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Erro WiFi");
  }
  delay(2000); // Aguarda antes de seguir
}

void reconnect() {
  // Reconecta ao servidor MQTT caso desconectado
  while (!client.connected()) {
    Serial.print("Tentando conexão MQTT...");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Conectando MQTT");
    
    // Tenta se conectar com ID "ESP32Sensor"
    if (client.connect("ESP32Sensor")) {
      Serial.println("conectado");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("MQTT Conectado");
      delay(1000);
    } else {
      // Exibe erro de conexão
      Serial.print("falha, rc=");
      Serial.print(client.state());
      Serial.println(" tentando novamente em 5 segundos");
      lcd.setCursor(0, 1);
      lcd.print("Erro: ");
      lcd.print(client.state());
      delay(5000); // Aguarda antes de tentar de novo
    }
  }
}

void setup() {
  Serial.begin(115200); // Inicializa a comunicação serial para debug

  pinMode(TRIG_PIN, OUTPUT); // Define TRIG como saída
  pinMode(ECHO_PIN, INPUT); // Define ECHO como entrada
  pinMode(BUZZER_PIN, OUTPUT); // Define pino do buzzer como saída

  lcd.init(); // Inicializa o LCD
  lcd.backlight(); // Liga a luz de fundo do LCD
  
  setup_wifi(); // Conecta ao Wi-Fi
  client.setServer(mqtt_server, 1883); // Define o servidor MQTT
}

void loop() {
  if (!client.connected()) {
    reconnect(); // Reestabelece conexão com MQTT se necessário
  }
  client.loop(); // Mantém a conexão MQTT ativa

  // Envia pulso para ativar o sensor ultrassônico
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Lê o tempo de resposta do sensor
  long duracao = pulseIn(ECHO_PIN, HIGH);
  float distancia = duracao * 0.034 / 2; // Converte o tempo em cm

  // Exibe distância no LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Nivel: ");
  lcd.print(distancia, 1);
  lcd.print(" cm");

  lcd.setCursor(0, 1);
  if (distancia <= NIVEL_PERIGOSO) {
    // Emite alerta se o nível for perigoso
    lcd.print("RISCO DE ENCHENTE");
    tone(BUZZER_PIN, 1000); // Ativa o buzzer com 1000 Hz
  } else {
    // Situação normal
    lcd.print("Nivel Seguro");
    noTone(BUZZER_PIN); // Desativa o buzzer
  }

  // Prepara os dados para envio via MQTT em formato JSON
  StaticJsonDocument<200> doc;
  doc["sensor"] = "nivel_agua";
  doc["distancia_cm"] = distancia;
  doc["status"] = (distancia <= NIVEL_PERIGOSO) ? "perigo" : "seguro";
  doc["timestamp"] = millis(); // Timestamp baseado no tempo de execução

  char jsonBuffer[200];
  serializeJson(doc, jsonBuffer); // Converte JSON para string
  
  client.publish("sensor/nivel", jsonBuffer); // Publica os dados no tópico MQTT
  Serial.println("Dados enviados: " + String(jsonBuffer)); // Debug no serial

  delay(2000); // Aguarda 2 segundos antes da próxima medição
}
```

## Node-Red ##
1 - Importe o fluxo na pasta node-red flows

2 - Ajuste conforme necessário

# Lógica de Alerta #
O sistema considera:
- Posição do sensor 10cm acima do solo
- Nível seguro: Água abaixo de 5cm do solo (15cm do sensor)
- Nível de perigo: Água a 5cm ou menos do solo (15cm ou menos do sensor)

Quando o nível atinge 15cm da posição do sensor (5cm do solo), o sistema:

- Ativa o buzzer
- Mostra alerta no LCD
- Envia mensagem de alerta via MQTT
- Atualiza o dashboard Node-RED

# Dashboard Node-RED #
O dashboard inclui:

- Gráfico temporal do nível da água
- Indicador de status (seguro/perigo)
- Variação do Nivel da Água


# Imagens #


