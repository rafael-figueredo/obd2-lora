/*
 * Heltec WiFi LoRa 32 V3 — NO CARRO: LEITOR OBD (via HC-05) + PAINEL + TRANSMISSOR LoRa
 * ----------------------------------------------------------
 * Substitui o DevKit: o HC-05 conecta no ELM327 por Bluetooth e a Heltec
 * conversa com o ELM pela serial do HC-05.
 *
 * - Lê a velocidade (PID 01 0D) numa tarefa separada (não trava a tela)
 * - OLED animado: carrinho na estrada, velocímetro de ponteiro, alerta com fogo e tela tremendo
 * - LED azul (pino 6): aceso = lendo o carro | piscando = ELM ok, carro não responde
 * - LED vermelho (pino 5) + buzzer (pino 4): acima de 60 km/h
 * - Lê também RPM, temperatura do motor, pressão do coletor e do ar
 * - Calcula na viagem: distância, combustível (estimado), vel. máxima, ultrapassagens
 * - ATUALIZAÇÃO SEM FIO: segure o botão PRG por 3 s -> a placa cria a rede "OBD-CARRO";
 *   conecte o celular/PC nela, abra http://192.168.4.1 e envie o .ino.bin
 * - LoRa: envia "OBD,<viagem>,<seq>,<estado>,<vel>,<temp>,<rpm>,<dist_m>,<comb_ml>,<vmax>,<ultrap>"
 *   a cada 1 s em 915 MHz
 *
 * LIGAÇÃO DO HC-05 (já configurado com o código 1):
 *   HC-05 VCC -> Heltec 5V
 *   HC-05 GND -> Heltec GND
 *   HC-05 TXD -> Heltec 7
 *   HC-05 RXD -> Heltec 2
 *   HC-05 EN e STATE: não ligar
 *
 * NUNCA ligue a placa sem a antena LoRa conectada!
 *
 * Bibliotecas: "U8g2" (olikraus) e "RadioLib" (jgromes)
 * Placa: "Heltec WiFi LoRa 32(V3)" | Serial Monitor: 115200
 */

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <RadioLib.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>

// ======================= CONFIGURAÇÃO =======================
// HC-05
#define PIN_RX_HC05     7      // recebe do TXD do HC-05
#define PIN_TX_HC05     2      // envia para o RXD do HC-05
#define BAUD_HC05       115200 // igual ao AT+UART do configurador (115200 evita o HC-05 picotar os comandos)

// Saídas
#define PIN_LED_CONEXAO 6      // LED azul
#define PIN_LED_LIMITE  5      // LED vermelho
#define PIN_BUZZER      4      // buzzer ATIVO

#define LIMITE_KMH      60
#define HISTERESE_KMH   2

// ---------- Atualização pelo navegador (OTA) ----------
// A cada nova versão, aumente o FW_VERSAO (aparece na tela de abertura).
#define FW_VERSAO        1
#define OTA_SSID         "OBD-CARRO"       // rede que a placa cria no modo atualização
#define OTA_SENHA        "obd2lora"        // senha da rede e da página (mín. 8 letras)
#define OTA_USUARIO      "admin"           // usuário da página
#define BIP_ON_MS       150
#define BIP_OFF_MS      350

#define INTERVALO_LEITURA_MS 200
#define FALHAS_PARA_SEM_ECU  10

// Consumo estimado por "speed-density" (o seu carro não tem sensor MAF).
// Usa pressão do coletor (PID 0B), rotação (0C) e temperatura do ar (0F).
#define CILINDRADA_L        1.6    // Gol G5 1.6
#define EFIC_VOLUMETRICA    0.80   // ajuste fino: veja "calibrar consumo" na planilha
#define COMBUSTIVEL_ETANOL  0      // 0 = gasolina | 1 = etanol

// LoRa (precisa ser IGUAL no receptor)
#define LORA_FREQ_MHZ   915.0
#define LORA_BW_KHZ     125.0
#define LORA_SF         9
#define LORA_CR         7
#define LORA_SYNC       0x12
#define LORA_POWER_DBM  17
#define LORA_INTERVALO_MS 1000

// Pinos internos da Heltec V3 (não mexer)
#define PIN_VEXT        36
#define PIN_OLED_RST    21
#define PIN_OLED_SDA    17
#define PIN_OLED_SCL    18
#define PIN_LED_PLACA   35
#define PIN_BOTAO_PRG   0      // segurar 3 s = modo atualização
#define LORA_NSS   8
#define LORA_SCK   9
#define LORA_MOSI  10
#define LORA_MISO  11
#define LORA_RST   12
#define LORA_BUSY  13
#define LORA_DIO1  14
// ============================================================

// Estados (os números vão no pacote LoRa, iguais aos do receptor)
enum Estado { INICIANDO = 0, SEM_ELM = 1, SEM_ECU = 2, LENDO = 3 };

volatile int estadoObd = INICIANDO;   // escrito pela tarefa OBD
volatile int velObd    = 0;

// Dados da viagem (calculados no carro, enviados acumulados pelo LoRa)
volatile int   tempMotor      = -99;   // °C (-99 = ainda não leu)
volatile int   rpmObd         = 0;
volatile int   mapKpa         = 0;
volatile int   iatC           = 25;
volatile float distM          = 0;     // distância da viagem (m)
volatile float combMl         = 0;     // combustível da viagem (mL)
volatile int   velMax         = 0;
volatile int   ultrapassagens = 0;
uint16_t idViagem = 0;                 // muda a cada vez que a placa liga

U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, PIN_OLED_RST, PIN_OLED_SCL, PIN_OLED_SDA);
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);

bool acimaDoLimite = false;

// =================================================================
//  ELM327 pelo HC-05  (roda numa tarefa própria, pode bloquear à vontade)
// =================================================================
// Alguns clones de ELM327 "picotam" o 1º caractere quando chega pelo HC-05.
// Um prefixo (espaço, que o ELM ignora) absorve esse primeiro pacote.
// O prefixo é escolhido sozinho em escolherPrefixo() (depende do HC-05 e do ELM)
String prefixoCmd = "";     // escolhido sozinho em escolherPrefixo() a cada inicialização

String elmComando(const char *cmd, uint32_t timeoutMs = 2000) {
  while (Serial1.available()) Serial1.read();
  String linha = prefixoCmd + String(cmd) + "\r";
  Serial1.write((const uint8_t *)linha.c_str(), linha.length());

  String resp;
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    while (Serial1.available()) {
      char c = Serial1.read();
      if (c == '>') {
        if (resp.length() == 0) continue;   // prompt atrasado do comando anterior
        resp.toUpperCase();
        return resp;
      }
      if (c != '\r' && c != '\n' && c != ' ' && c != '\0') resp += c;
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  resp.toUpperCase();
  return resp;
}

// Envia um comando AT de configuração e repete se o ELM responder "?" (não entendeu)
String elmConfig(const char *cmd, uint32_t timeoutMs = 3000) {
  String r;
  for (int tent = 0; tent < 3; tent++) {
    r = elmComando(cmd, timeoutMs);
    if (r.indexOf("?") < 0) break;
    Serial.printf("  %-6s -> %s  (repetindo)\n", cmd, r.c_str());
    vTaskDelay(pdMS_TO_TICKS(300));
    while (Serial1.available()) Serial1.read();
  }
  return r;
}

// Testa formas de enviar e fica com a que o ELM entende sem picotar nem atrasar.
// Alterna ATI (responde "ELM327 v2.1") e ATRV (responde a tensão, ex. "12.4V"):
// se a resposta vier trocada, é sinal de resposta atrasada e não conta ponto.
void escolherPrefixo() {
  const char *opcoes[] = {"", "\r", " ", "  "};
  const char *nomes[]  = {"nenhum", "enter antes", "1 espaco", "2 espacos"};
  int melhor = 0, melhorPontos = -1;
  for (int i = 0; i < 4; i++) {
    prefixoCmd = opcoes[i];
    int pontos = 0;
    for (int k = 0; k < 6; k++) {
      bool ati = (k % 2 == 0);
      String r = elmComando(ati ? "ATI" : "ATRV", 2500);
      bool limpo = r.indexOf("?") < 0;
      if (ati) limpo = limpo && r.indexOf("ELM") >= 0 && r.indexOf("V", r.indexOf("ELM") + 3) >= 0 && r.length() < 20;
      else     limpo = limpo && r.indexOf("ELM") < 0 && r.indexOf("V") > 0 && r.length() < 8;
      if (limpo) pontos++;
      vTaskDelay(pdMS_TO_TICKS(150));
    }
    Serial.printf("  prefixo %-12s -> %d/6 respostas limpas\n", nomes[i], pontos);
    if (pontos > melhorPontos) { melhorPontos = pontos; melhor = i; }
    if (pontos == 6) break;                       // perfeito: não precisa testar os outros
  }
  prefixoCmd = opcoes[melhor];
  Serial.printf("  Usando prefixo: %s\n", nomes[melhor]);
}

// Retorna: 0 = ELM não responde | 1 = ELM ok, carro não | 2 = tudo ok
int inicializarELM() {
  Serial.println("Inicializando ELM327 (via HC-05)...");

  // Limpa qualquer comando pela metade que tenha ficado no ELM
  Serial1.write('\r');
  vTaskDelay(pdMS_TO_TICKS(500));
  while (Serial1.available()) Serial1.read();

  String z = elmComando("ATZ", 5000);
  Serial.println("  ATZ    -> " + z);
  vTaskDelay(pdMS_TO_TICKS(2000));                 // clone v2.1 demora pra voltar do reset
  while (Serial1.available()) Serial1.read();

  bool elmVivo = z.length() > 0;                   // qualquer resposta = HC-05 conectado no ELM
  if (elmVivo) escolherPrefixo();
  const char *cfg[] = {"ATE0", "ATL0", "ATS0", "ATH0", "ATSTFF", "ATSP0"};
  for (const char *c : cfg) {
    String r = elmConfig(c);
    Serial.printf("  %-6s -> %s\n", c, r.c_str());
    if (r.length() > 0) elmVivo = true;
    vTaskDelay(pdMS_TO_TICKS(200));                // folga entre comandos
  }
  if (!elmVivo) {
    Serial.println("  ELM327 nao respondeu: HC-05 conectado? (LED do HC-05 deve dar 2 piscadas a cada 2 s)");
    return 0;
  }

  String r;
  for (int tent = 0; tent < 5; tent++) {
    r = elmComando("0100", 15000);
    Serial.println("  0100   -> " + r);
    if (r.indexOf("4100") >= 0) break;
    if (r.indexOf("?") < 0 && r.indexOf("0100") >= 0) break;   // chegou inteiro: a resposta vale
    vTaskDelay(pdMS_TO_TICKS(300));
  }
  if (r.indexOf("4100") >= 0) return 2;
  if (r.indexOf("NODATA") >= 0 || r.indexOf("UNABLE") >= 0)
    Serial.println("  >>> O ELM perguntou e a ECU NAO respondeu: o MOTOR esta ligado?");
  else
    Serial.println("  (resposta picotada pelo HC-05, tentando de novo)");
  return 1;
}

// Lê um PID do modo 01. Devolve n (bytes lidos) ou 0 se falhou.
int lerPID(const char *pid, uint8_t *out, int n) {
  char cmd[8], alvo[8];
  snprintf(cmd,  sizeof(cmd),  "01%s", pid);
  snprintf(alvo, sizeof(alvo), "41%s", pid);
  String r = elmComando(cmd, 4000);
  if (r.indexOf(alvo) < 0 && r.indexOf("?") >= 0) {      // comando chegou picotado: tenta de novo
    vTaskDelay(pdMS_TO_TICKS(100));
    r = elmComando(cmd, 4000);
  }
  int i = r.indexOf(alvo);
  if (i < 0 || (int)r.length() < i + 4 + 2 * n) {
    if (r.length() > 0 && r.indexOf("NODATA") < 0) Serial.printf("  %s falhou -> %s\n", cmd, r.c_str());
    return 0;
  }
  for (int k = 0; k < n; k++)
    out[k] = (uint8_t)strtol(r.substring(i + 4 + 2 * k, i + 6 + 2 * k).c_str(), NULL, 16);
  return n;
}

int lerVelocidade() {
  uint8_t b[1];
  return lerPID("0D", b, 1) ? b[0] : -1;
}

// Combustível em litros por segundo (speed-density)
float consumoLporS() {
  int rpm = rpmObd, map = mapKpa;
  if (rpm < 300 || map <= 0) return 0;
  if (map < 25 && velObd > 20) return 0;                  // freio-motor: injeção cortada
  float T      = iatC + 273.15f;
  float ar_g_s = (map * 1000.0f) * (CILINDRADA_L / 1000.0f) * (rpm / 120.0f)
                 * EFIC_VOLUMETRICA / (287.05f * T) * 1000.0f;
  float afr    = COMBUSTIVEL_ETANOL ? 9.0f   : 13.2f;      // relação ar/combustível
  float dens   = COMBUSTIVEL_ETANOL ? 789.0f : 745.0f;     // g/L
  return (ar_g_s / afr) / dens;
}

// Soma distância e combustível desde a última leitura
void integrarViagem() {
  static uint32_t tAnt = 0;
  uint32_t agora = millis();
  if (tAnt == 0) { tAnt = agora; return; }
  float dt = (agora - tAnt) / 1000.0f;
  tAnt = agora;
  if (dt > 5) return;                                      // ficou muito tempo sem ler: não chuta
  distM  = distM  + velObd / 3.6f * dt;
  combMl = combMl + consumoLporS() * 1000.0f * dt;
  if (velObd > velMax) velMax = velObd;
}

void tarefaOBD(void *) {
  bool pronto = false;
  int falhas = 0;

  for (;;) {
    if (!pronto) {
      int st = inicializarELM();
      if (st == 0)      { estadoObd = SEM_ELM; vTaskDelay(pdMS_TO_TICKS(2000)); continue; }
      else if (st == 1) { estadoObd = SEM_ECU; vTaskDelay(pdMS_TO_TICKS(3000)); continue; }
      Serial.println("ELM327 pronto, lendo velocidade.");
      pronto = true;
      falhas = 0;
    }

    uint32_t t0 = millis();
    int v = lerVelocidade();

    // Filtro: salto de mais de 25 km/h numa leitura só é quase sempre resposta picotada.
    // Só aceita se a leitura seguinte confirmar o novo valor.
    static int pendente = -1;
    if (v >= 0 && estadoObd == LENDO && abs(v - (int)velObd) > 25) {
      if (pendente >= 0 && abs(v - pendente) <= 10) {
        pendente = -1;                         // confirmado: aceita
      } else {
        pendente = v;                          // suspeito: espera confirmar
        Serial.printf("  (leitura %d km/h descartada, aguardando confirmar)\n", v);
        v = -2;                                // não conta como falha
      }
    } else if (v >= 0) {
      pendente = -1;
    }

    if (v == -2) {
      // ignorada pelo filtro
    } else if (v >= 0) {
      velObd = v;
      estadoObd = LENDO;
      falhas = 0;
      Serial.printf("Vel %3d km/h | RPM %4d | MAP %3d kPa | motor %3d C | %.2f km | %.3f L\n",
                    v, (int)rpmObd, (int)mapKpa, (int)tempMotor, distM / 1000.0f, combMl / 1000.0f);
    } else if (++falhas >= FALHAS_PARA_SEM_ECU) {
      estadoObd = SEM_ECU;
      pronto = false;   // reinicializa (cobre queda do Bluetooth também)
    }

    // Dados extras: um PID a mais por ciclo, alternando (velocidade continua em todo ciclo)
    if (pronto && estadoObd == LENDO) {
      static uint8_t ciclo = 0;
      uint8_t b[2];
      switch (ciclo % 4) {
        case 0: case 2:
          if (lerPID("0C", b, 2)) rpmObd = ((b[0] << 8) | b[1]) / 4;      // RPM
          break;
        case 1:
          if (lerPID("0B", b, 1)) mapKpa = b[0];                          // pressão do coletor
          break;
        case 3:
          if ((ciclo / 4) % 2) { if (lerPID("05", b, 1)) tempMotor = (int)b[0] - 40; }  // motor
          else                 { if (lerPID("0F", b, 1)) iatC      = (int)b[0] - 40; }  // ar
          break;
      }
      ciclo++;
      integrarViagem();
    }

    uint32_t gasto = millis() - t0;
    if (gasto < INTERVALO_LEITURA_MS) vTaskDelay(pdMS_TO_TICKS(INTERVALO_LEITURA_MS - gasto));
  }
}

// =================================================================
//  LoRa
// =================================================================
volatile bool loraTxTerminou = false;
bool loraOk = false, loraOcupado = false;
uint32_t loraInicioTx = 0, loraSeq = 0;

void IRAM_ATTR aoTerminarTx() { loraTxTerminou = true; }

void iniciarLoRa() {
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
#if defined(RADIOLIB_VERSION_MAJOR) && RADIOLIB_VERSION_MAJOR >= 8
  // RadioLib 8.x: configuração por estrutura
  radio.tcxoVoltage = RadioLibTCXOVoltage_t::Voltage1V8;   // TCXO da Heltec V3
  ConfigLoRa_t cfg;
  cfg.frequency       = RADIOLIB_UNIT_MEGA_FLT(LORA_FREQ_MHZ);
  cfg.bandwidth       = RADIOLIB_UNIT_KILO_FLT(LORA_BW_KHZ);
  cfg.spreadingFactor = LORA_SF;
  cfg.codingRate      = LORA_CR;
  cfg.syncWord        = LORA_SYNC;
  cfg.power           = LORA_POWER_DBM;
  cfg.preambleLength  = 8;
  int st = radio.begin(cfg);
#else
  // RadioLib 6.x / 7.x
  int st = radio.begin(LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR,
                       LORA_SYNC, LORA_POWER_DBM, 8, 1.8);   // 1.8 V = TCXO da Heltec V3
#endif
  if (st == RADIOLIB_ERR_NONE) {
    radio.setDio1Action(aoTerminarTx);
    loraOk = true;
    Serial.println("LoRa OK (915 MHz)");
  } else {
    Serial.printf("[ERRO] LoRa falhou, codigo %d\n", st);
  }
}

void enviarLoRa(int estado, int vel) {
  if (!loraOk) return;
  if (loraTxTerminou) { loraTxTerminou = false; radio.finishTransmit(); loraOcupado = false; }
  if (loraOcupado && millis() - loraInicioTx > 3000) loraOcupado = false;

  static uint32_t ultimoTx = 0;
  if (loraOcupado || millis() - ultimoTx < LORA_INTERVALO_MS) return;
  ultimoTx = millis();

  // OBD,<viagem>,<seq>,<estado>,<vel>,<temp>,<rpm>,<dist_m>,<comb_ml>,<vmax>,<ultrapassagens>
  char pkt[96];
  snprintf(pkt, sizeof(pkt), "OBD,%04X,%lu,%d,%d,%d,%d,%lu,%lu,%d,%d",
           idViagem, (unsigned long)loraSeq++, estado, vel, (int)tempMotor, (int)rpmObd,
           (unsigned long)distM, (unsigned long)combMl, (int)velMax, (int)ultrapassagens);
  if (radio.startTransmit(pkt) == RADIOLIB_ERR_NONE) { loraOcupado = true; loraInicioTx = millis(); }
}

// =================================================================
//  ANIMAÇÕES DO OLED (128x64)
// =================================================================
#define CHAO_Y     61    // base das rodas do carrinho
#define ESTRADA_Y  47    // linha de cima da estrada

float velSuave   = 0;    // velocidade suavizada (ponteiro, estrada, rodas)
float estradaOff = 0;    // deslocamento da estrada
float rodaAng    = 0;    // ângulo das rodas
float carroX     = 10;   // posição do carrinho (anda pra frente no alerta)

void atualizarAnim(int vel, bool andando, float dt) {
  float alvo = andando ? (float)vel : 0.0f;
  velSuave  += (alvo - velSuave) * min(1.0f, dt * 5.0f);
  estradaOff = fmodf(estradaOff + velSuave * dt * 2.0f, 480.0f);   // 60 km/h = 120 px/s
  rodaAng    = fmodf(rodaAng + velSuave * dt * 0.6f, 2 * PI);
}

void hLinha(int x, int y, int len) {             // drawHLine com recorte na borda esquerda
  if (x < 0) { len += x; x = 0; }
  if (len > 0 && x < 128) oled.drawHLine(x, y, len);
}

void desenharEstrada(int dx, bool postes) {
  int o = (int)estradaOff;
  oled.drawHLine(0, ESTRADA_Y, 128);
  for (int x = -(o % 16); x < 128; x += 16) hLinha(x + dx, ESTRADA_Y + 8, 8);
  oled.drawHLine(0, 63, 128);
  if (postes) {
    for (int x = -(o % 40); x < 128; x += 40) {
      if (x + dx + 19 < 0) continue;
      oled.drawVLine(x + dx + 20, ESTRADA_Y - 4, 4);
      oled.drawPixel(x + dx + 19, ESTRADA_Y - 4);
      oled.drawPixel(x + dx + 21, ESTRADA_Y - 4);
    }
  }
}

// x = traseira do carro, y = chão
void desenharCarro(int x, int y, float ang, bool farol) {
  oled.setDrawColor(0);
  oled.drawBox(x - 1, y - 16, 30, 16);          // limpa o fundo
  oled.setDrawColor(1);
  oled.drawRBox(x, y - 10, 27, 6, 2);            // carroceria
  oled.drawRBox(x + 6, y - 15, 13, 7, 2);        // cabine
  oled.setDrawColor(0);
  oled.drawBox(x + 8,  y - 13, 4, 3);            // janelas
  oled.drawBox(x + 13, y - 13, 4, 3);
  oled.setDrawColor(1);
  const int rx[2] = {x + 6, x + 21};
  for (int i = 0; i < 2; i++) {                  // rodas girando
    oled.drawDisc(rx[i], y - 3, 3);
    oled.setDrawColor(0);
    oled.drawLine(rx[i], y - 3, rx[i] + (int)roundf(2 * cosf(ang)), y - 3 + (int)roundf(2 * sinf(ang)));
    oled.drawLine(rx[i], y - 3, rx[i] - (int)roundf(2 * cosf(ang)), y - 3 - (int)roundf(2 * sinf(ang)));
    oled.setDrawColor(1);
  }
  if (farol) {                                    // farol aceso
    oled.drawLine(x + 27, y - 8, x + 35, y - 11);
    oled.drawLine(x + 27, y - 8, x + 35, y - 5);
  }
}

void desenharFogo(int x, int y) {                 // fogo no escapamento
  for (int i = 0; i < 3; i++) {
    int len = min((int)random(5, 15), x - 2);
    if (len < 2) continue;
    int yy  = y - 9 + i * 2;
    oled.drawTriangle(x - 1, yy - 1, x - 1, yy + 1, x - 1 - len, yy);
  }
  for (int i = 0; i < 4; i++) {
    int px = x - random(8, 24);
    if (px >= 0) oled.drawPixel(px, y - random(4, 13));
  }
}

void desenharRastros(int ateX) {                  // linhas de velocidade
  if (ateX < 8) return;
  for (int i = 0; i < 5; i++) {
    int yy = random(ESTRADA_Y + 2, 62);
    int xx = random(0, ateX - 6);
    hLinha(xx, yy, random(4, 12));
  }
}

void desenharVelocimetro(int cx, int cy, int r, float vel, int dx, int dy) {
  cx += dx; cy += dy;
  oled.drawCircle(cx, cy, r, U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_UPPER_RIGHT);
  for (int v = 0; v <= 160; v += 20) {
    float a  = PI - (v / 160.0f) * PI;
    int   r1 = (v == LIMITE_KMH) ? r - 6 : r - 3;   // marca maior no limite
    oled.drawLine(cx + r1 * cosf(a), cy - r1 * sinf(a), cx + r * cosf(a), cy - r * sinf(a));
  }
  float a = PI - (constrain(vel, 0.0f, 160.0f) / 160.0f) * PI;
  oled.drawLine(cx, cy, cx + (r - 3) * cosf(a), cy - (r - 3) * sinf(a));
  oled.drawDisc(cx, cy, 2);
}

void desenharAviso(int cx, int cy, int dx, int dy) {   // triângulo de alerta
  cx += dx; cy += dy;
  oled.drawTriangle(cx, cy - 18, cx - 17, cy, cx + 17, cy);
  oled.setDrawColor(0);
  oled.drawBox(cx - 1, cy - 13, 3, 7);
  oled.drawBox(cx - 1, cy - 4, 3, 2);
  oled.setDrawColor(1);
}

void desenharNumero(int vel, int dx, int dy, bool invertido) {
  char num[6];
  snprintf(num, sizeof(num), "%d", vel);
  oled.setFont(u8g2_font_logisoso28_tn);
  int w = oled.getStrWidth(num);
  int x = 100 - w + dx, y = 40 + dy;
  if (invertido) {
    oled.drawRBox(x - 3, y - 31, w + 6, 34, 3);
    oled.setDrawColor(0);
  }
  oled.drawStr(x, y, num);
  oled.setDrawColor(1);
  oled.setFont(u8g2_font_5x7_tf);
  oled.drawStr(103 + dx, 40 + dy, "km/h");
}

// ---- Telas ----

void telaVelocidade(int vel, bool alerta, uint32_t t) {
  int dx = 0, dy = 0;
  if (alerta) { dx = random(-2, 3); dy = random(-1, 2); }   // tela tremendo

  float alvoX = alerta ? 44 : 10;                            // carro acelera pra frente
  carroX += (alvoX - carroX) * 0.15f;

  oled.setFont(u8g2_font_5x7_tf);
  if (alerta) {
    if ((t / 250) % 2) oled.drawStr(84 + dx, 7, "DEVAGAR!");
  } else {
    char lim[12];
    snprintf(lim, sizeof(lim), "lim %d", LIMITE_KMH);
    oled.drawStr(127 - oled.getStrWidth(lim), 7, lim);
  }

  if (alerta && (t / 250) % 2) desenharAviso(22, 40, dx, dy);
  else                         desenharVelocimetro(22, 39, 19, velSuave, dx, dy);

  desenharNumero(vel, dx, dy, alerta && (t / 125) % 2);
  desenharEstrada(dx, true);

  int cx  = (int)carroX + dx;
  int bob = (velSuave > 1 && (t / 80) % 2) ? 1 : 0;           // suspensão balançando
  if (alerta) desenharRastros(cx);
  desenharCarro(cx, CHAO_Y - bob, rodaAng, velSuave > 1);
  if (alerta) desenharFogo(cx, CHAO_Y - bob);
}

void telaProcurando(const char *msg, uint32_t t) {
  oled.setFont(u8g2_font_6x10_tf);
  int x = (128 - (oled.getStrWidth(msg) + 18)) / 2;
  char buf[32];
  int pontos = (t / 400) % 4;
  snprintf(buf, sizeof(buf), "%s%.*s", msg, pontos, "...");
  oled.drawStr(x, 10, buf);

  for (int k = 0; k < 3; k++) {                               // ondas de rádio
    int r = ((t / 50) + k * 8) % 24 + 3;
    oled.drawCircle(63, 44, r, U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_UPPER_RIGHT);
  }
  desenharEstrada(0, false);
  desenharCarro(50, CHAO_Y, 0, false);
}

void telaDormindo(uint32_t t) {
  oled.setFont(u8g2_font_6x10_tf);
  const char *m = "LIGUE O CARRO";
  oled.drawStr((128 - oled.getStrWidth(m)) / 2, 10, m);
  oled.setFont(u8g2_font_5x7_tf);
  const char *m2 = "ELM327 conectado";
  oled.drawStr((128 - oled.getStrWidth(m2)) / 2, 20, m2);

  desenharEstrada(0, false);
  desenharCarro(50, CHAO_Y, 0, false);
  for (int k = 0; k < 3; k++) {                               // zZz saindo do carro
    int p  = ((t / 40) + k * 20) % 60;
    int zx = 76 + p / 5, zy = 44 - p / 2;
    if (zy > 24) {
      oled.setFont(p < 30 ? u8g2_font_5x7_tf : u8g2_font_6x10_tf);
      oled.drawStr(zx, zy, "z");
    }
  }
}

void splash(const char *titulo) {                              // carrinho entrando na tela
  uint32_t t0 = millis(), ant = t0;
  while (millis() - t0 < 1800) {
    uint32_t t = millis() - t0;
    float dt = (millis() - ant) / 1000.0f;
    ant = millis();
    int x = (int)(50.0f * min(1.0f, t / 1000.0f));
    bool andando = t < 1000;
    if (andando) estradaOff = fmodf(estradaOff + 100.0f * dt, 480.0f);
    rodaAng = andando ? fmodf(rodaAng + 30.0f * dt, 2 * PI) : rodaAng;

    oled.clearBuffer();
    desenharEstrada(0, true);
    if (andando) desenharRastros(x);
    desenharCarro(x, CHAO_Y, rodaAng, true);
    if (t > 1000) {
      oled.setFont(u8g2_font_6x10_tf);
      oled.drawStr((128 - oled.getStrWidth(titulo)) / 2, 14, titulo);
      oled.setFont(u8g2_font_5x7_tf);
      char s[24];
      snprintf(s, sizeof(s), "OBD2 + LoRa  v%d", FW_VERSAO);
      oled.drawStr((128 - oled.getStrWidth(s)) / 2, 26, s);
    }
    oled.sendBuffer();
    delay(30);
  }
}

void desenharTela(int est, int vel, bool alerta, float dt) {
  uint32_t t = millis();
  atualizarAnim(vel, est == LENDO, dt);
  oled.clearBuffer();
  switch (est) {
    case LENDO:   telaVelocidade(vel, alerta, t);              break;
    case SEM_ECU: telaDormindo(t);                             break;
    case SEM_ELM: telaProcurando("PROCURANDO ELM327", t);      break;
    default:      telaProcurando("INICIANDO", t);              break;
  }
  oled.sendBuffer();
}

// =================================================================
//  MODO ATUALIZAÇÃO (OTA pelo navegador)
//  Segure o botão PRG por 3 s com a placa ligada -> ela reinicia neste modo,
//  cria a rede Wi-Fi OTA_SSID e serve a página http://192.168.4.1
// =================================================================
RTC_NOINIT_ATTR uint32_t pedidoOTA;          // sobrevive ao reinício por software
#define MAGICO_OTA 0x0DA7A0A5

WebServer servidor(80);
volatile uint32_t otaRecebido = 0;
volatile bool     otaRecebendo = false;
bool otaAutorizado = false;
bool otaOk = false;

const char PAGINA_OTA[] PROGMEM = R"rawliteral(
<!doctype html><html lang="pt-br"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Atualizar carro</title>
<style>
body{font-family:system-ui,sans-serif;background:#111;color:#eee;max-width:420px;margin:30px auto;padding:0 16px}
h1{font-size:20px}.c{background:#1d1d1d;border-radius:12px;padding:16px}
input,button{width:100%;margin-top:12px;font-size:16px}
button{padding:12px;border:0;border-radius:8px;background:#2f7de1;color:#fff}
.b{height:12px;background:#333;border-radius:6px;margin-top:14px;overflow:hidden}
.p{height:100%;width:0;background:#2fbf71}#m{margin-top:10px;min-height:20px}
small{color:#999}
</style></head><body>
<h1>Velocimetro OBD2 &mdash; atualizar</h1>
<div class="c">
<div>Versao instalada: <b>v%VERSAO%</b></div>
<input type="file" id="f" accept=".bin">
<button onclick="enviar()">Enviar e atualizar</button>
<div class="b"><div class="p" id="p"></div></div>
<div id="m"></div>
<small>Use o arquivo <b>.ino.bin</b> (nao o .merged.bin) gerado em Sketch &gt; Export Compiled Binary.</small>
</div>
<script>
function enviar(){
  const f=document.getElementById('f').files[0];
  if(!f){m('Escolha o arquivo .bin primeiro');return;}
  if(!f.name.endsWith('.bin')||f.name.includes('merged')){m('Arquivo errado: use o .ino.bin');return;}
  const fd=new FormData();fd.append('firmware',f,f.name);
  const x=new XMLHttpRequest();x.open('POST','/update');
  x.upload.onprogress=e=>{if(e.lengthComputable){const p=Math.round(e.loaded*100/e.total);
    document.getElementById('p').style.width=p+'%';m('Enviando... '+p+'%');}};
  x.onload=()=>m(x.status==200?'Atualizado! A placa esta reiniciando.':'Falhou: '+x.responseText);
  x.onerror=()=>m('Conexao caiu. Se passou de 100%, a placa provavelmente atualizou.');
  x.send(fd);
}
function m(t){document.getElementById('m').textContent=t;}
</script></body></html>
)rawliteral";

void telaOTA(const IPAddress &ip) {
  char b[40];
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);
  oled.drawStr(4, 10, "MODO ATUALIZACAO");
  oled.setFont(u8g2_font_5x7_tf);
  snprintf(b, sizeof(b), "WiFi:  %s", OTA_SSID);   oled.drawStr(0, 24, b);
  snprintf(b, sizeof(b), "Senha: %s", OTA_SENHA);  oled.drawStr(0, 34, b);
  snprintf(b, sizeof(b), "Abra:  http://%s", ip.toString().c_str()); oled.drawStr(0, 44, b);
  if (otaRecebendo) snprintf(b, sizeof(b), "Recebendo: %lu KB", (unsigned long)(otaRecebido / 1024));
  else              snprintf(b, sizeof(b), "v%d  (RST cancela)", FW_VERSAO);
  oled.drawStr(0, 60, b);
  oled.sendBuffer();
}

void modoAtualizacao() {
  Serial.println("\n=== MODO ATUALIZACAO ===");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(OTA_SSID, OTA_SENHA);
  IPAddress ip = WiFi.softAPIP();
  Serial.printf("Conecte na rede \"%s\" (senha %s) e abra http://%s\n",
                OTA_SSID, OTA_SENHA, ip.toString().c_str());

  servidor.on("/", HTTP_GET, []() {
    if (!servidor.authenticate(OTA_USUARIO, OTA_SENHA)) return servidor.requestAuthentication();
    String pag = FPSTR(PAGINA_OTA);
    pag.replace("%VERSAO%", String(FW_VERSAO));
    servidor.send(200, "text/html", pag);
  });

  servidor.on("/update", HTTP_POST,
    []() {                                           // resposta depois do upload
      if (!otaAutorizado) return servidor.requestAuthentication();
      servidor.sendHeader("Connection", "close");
      if (otaOk) {
        servidor.send(200, "text/plain", "OK");
        delay(1000);
        ESP.restart();
      } else {
        servidor.send(500, "text/plain", Update.errorString());
      }
    },
    []() {                                           // recebendo o arquivo em pedaços
      HTTPUpload &up = servidor.upload();
      if (up.status == UPLOAD_FILE_START) {
        otaAutorizado = servidor.authenticate(OTA_USUARIO, OTA_SENHA);
        otaOk = false;
        otaRecebido = 0;
        if (!otaAutorizado) return;
        Serial.printf("Recebendo %s...\n", up.filename.c_str());
        otaRecebendo = true;
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
      } else if (up.status == UPLOAD_FILE_WRITE && otaAutorizado) {
        if (Update.write(up.buf, up.currentSize) != up.currentSize) Update.printError(Serial);
        otaRecebido = otaRecebido + up.currentSize;
      } else if (up.status == UPLOAD_FILE_END && otaAutorizado) {
        otaOk = Update.end(true);
        otaRecebendo = false;
        if (otaOk) Serial.printf("Atualizado: %u bytes. Reiniciando...\n", up.totalSize);
        else       Update.printError(Serial);
      } else if (up.status == UPLOAD_FILE_ABORTED) {
        Update.abort();
        otaRecebendo = false;
      }
    });

  servidor.begin();

  uint32_t ultimaTela = 0;
  for (;;) {                                         // só sai reiniciando
    servidor.handleClient();
    if (millis() - ultimaTela > 200) { ultimaTela = millis(); telaOTA(ip); }
    if (millis() > 10UL * 60 * 1000 && !otaRecebendo) ESP.restart();   // 10 min parado: volta ao normal
    delay(2);
  }
}

// Segurar PRG por 3 s pede o modo atualização (reinicia e entra nele)
void checarBotaoOTA() {
  static uint32_t apertadoDesde = 0;
  if (digitalRead(PIN_BOTAO_PRG) == LOW) {
    if (apertadoDesde == 0) apertadoDesde = millis();
    if (millis() - apertadoDesde > 3000) {
      Serial.println("PRG segurado: reiniciando no modo atualizacao...");
      pedidoOTA = MAGICO_OTA;
      oled.clearBuffer();
      oled.setFont(u8g2_font_6x10_tf);
      oled.drawStr(4, 36, "MODO ATUALIZACAO...");
      oled.sendBuffer();
      delay(300);
      ESP.restart();
    }
  } else {
    apertadoDesde = 0;
  }
}

// =================================================================
//  SETUP / LOOP
// =================================================================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_VEXT, OUTPUT);
  digitalWrite(PIN_VEXT, LOW);   // liga o OLED
  delay(100);

  pinMode(PIN_LED_PLACA, OUTPUT);
  pinMode(PIN_LED_CONEXAO, OUTPUT);
  pinMode(PIN_LED_LIMITE, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  oled.begin();
  oled.setBusClock(400000);
  pinMode(PIN_BOTAO_PRG, INPUT_PULLUP);

  // Foi pedido o modo atualização (PRG segurado 3 s)?
  if (pedidoOTA == MAGICO_OTA) {
    pedidoOTA = 0;
    modoAtualizacao();                               // não volta
  }
  pedidoOTA = 0;

  randomSeed(esp_random());
  idViagem = esp_random() & 0xFFFF;
  splash("VELOCIMETRO");

  // Autoteste: LEDs + buzzer por meio segundo
  digitalWrite(PIN_LED_CONEXAO, HIGH);
  digitalWrite(PIN_LED_LIMITE, HIGH);
  digitalWrite(PIN_BUZZER, HIGH);
  delay(500);
  digitalWrite(PIN_LED_CONEXAO, LOW);
  digitalWrite(PIN_LED_LIMITE, LOW);
  digitalWrite(PIN_BUZZER, LOW);

  iniciarLoRa();

  Serial1.begin(BAUD_HC05, SERIAL_8N1, PIN_RX_HC05, PIN_TX_HC05);
  delay(2000);   // tempo do HC-05 conectar no ELM ao ligar
  xTaskCreatePinnedToCore(tarefaOBD, "obd", 6144, NULL, 1, NULL, 0);

  Serial.printf("Heltec do carro pronta v%d (HC-05 + painel + LoRa). Segure PRG 3 s para atualizar.\n", FW_VERSAO);
}

void loop() {
  static uint32_t ultimoDesenho = 0;

  int est = estadoObd;
  int vel = velObd;

  // Limite com histerese
  if (est != LENDO) acimaDoLimite = false;
  else if (vel > LIMITE_KMH) acimaDoLimite = true;
  else if (vel <= LIMITE_KMH - HISTERESE_KMH) acimaDoLimite = false;

  // Conta cada vez que passa do limite
  static bool acimaAntes = false;
  if (acimaDoLimite && !acimaAntes) ultrapassagens = ultrapassagens + 1;
  acimaAntes = acimaDoLimite;

  // LED azul (e LED branco da placa junto)
  bool ledCon = (est == LENDO) ? true : (est == SEM_ECU) ? ((millis() / 400) % 2) : false;
  digitalWrite(PIN_LED_CONEXAO, ledCon);
  digitalWrite(PIN_LED_PLACA, ledCon);

  // LED vermelho + buzzer intermitente
  digitalWrite(PIN_LED_LIMITE, acimaDoLimite);
  bool bip = acimaDoLimite && ((millis() % (BIP_ON_MS + BIP_OFF_MS)) < BIP_ON_MS);
  digitalWrite(PIN_BUZZER, bip);

  // OLED animado a ~25 fps
  if (millis() - ultimoDesenho >= 40) {
    float dt = min(0.1f, (millis() - ultimoDesenho) / 1000.0f);
    ultimoDesenho = millis();
    desenharTela(est, vel, acimaDoLimite, dt);
  }

  enviarLoRa(est, vel);
  if (vel == 0) checarBotaoOTA();                    // só entra no modo atualização parado
  delay(5);
}
