/*
 * Heltec WiFi LoRa 32 V3 — ESTAÇÃO: RECEPTOR LoRa + GOOGLE SHEETS
 * ----------------------------------------------------------
 * - Recebe por LoRa os pacotes do carro:
 *     OBD,<viagem>,<seq>,<estado>,<vel>,<temp>,<rpm>,<dist_m>,<comb_ml>,<vmax>,<ultrap>
 *     estado: 0 = iniciando | 1 = procurando ELM | 2 = carro não responde | 3 = lendo
 * - GUARDA tudo num arquivo na própria placa (LittleFS) assim que recebe
 * - Quando tem Wi-Fi, envia a fila para o Google Sheets (Apps Script) e apaga o que foi enviado
 *     aba "Leituras": uma linha a cada 10 s (velocidade, temperatura, RPM, distância, consumo...)
 *     aba "Viagens" : uma linha por viagem, atualizada a cada 60 s e ao terminar
 * - Sem Wi-Fi: continua guardando e manda tudo quando a rede voltar
 * - OLED animado (carrinho) + ícone de Wi-Fi e nº de linhas esperando envio
 * - ATUALIZAÇÃO REMOTA (OTA pelo GitHub): a cada 5 min confere o versao.txt do repositório;
 *   se o número for maior que FW_VERSAO, baixa o firmware.bin, grava e reinicia sozinha
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
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <LittleFS.h>
#include <time.h>
#include <HTTPUpdate.h>
#include <Preferences.h>

// ======================= CONFIGURAÇÃO =======================
// Wi-Fi da estação e URL da planilha
// - Preenchidos: na 1ª gravação (USB) são SALVOS na memória da placa (não somem com atualizações)
// - Vazios (""): a placa usa o que já está salvo -> é a versão que vai para o GitHub público
#define WIFI_SSID    ""
#define WIFI_SENHA   ""

// URL da implantação do Apps Script (termina em /exec) — vazio = usa a salva
#define URL_SCRIPT   ""
#define CHAVE        "obd2-lora-2026"     // tem que ser IGUAL à do Apps Script

#define INTERVALO_LEITURA_MS   10000      // 1 linha em "Leituras" a cada 10 s
#define INTERVALO_VIAGEM_MS    60000      // atualiza "Viagens" a cada 60 s
#define FIM_VIAGEM_MS          60000      // 60 s sem pacote = viagem encerrada
#define MAX_FILA_BYTES         400000     // tamanho máximo da fila guardada na placa
#define LINHAS_POR_ENVIO       20

// ---------- Atualização remota (OTA pelo GitHub) ----------
// A cada nova versão: AUMENTE o FW_VERSAO aqui, exporte o .bin e suba no GitHub
// junto com o versao.txt contendo o MESMO número.
#define FW_VERSAO        1
#define URL_VERSAO       "https://raw.githubusercontent.com/rafael-figueredo/obd2-lora/main/estacao/versao.txt"
#define URL_FIRMWARE     "https://raw.githubusercontent.com/rafael-figueredo/obd2-lora/main/estacao/firmware.bin"
#define INTERVALO_OTA_MS 300000            // confere a cada 5 min

// LoRa (precisa ser IGUAL no transmissor do carro)
#define LORA_FREQ_MHZ   915.0
#define LORA_BW_KHZ     125.0
#define LORA_SF         9
#define LORA_CR         7
#define LORA_SYNC       0x12
#define LORA_POWER_DBM  17

#define LIMITE_KMH      60
#define TIMEOUT_LORA_MS 5000   // sem pacote por 5 s = sem sinal

// Pinos internos da Heltec V3 (não mexer)
#define PIN_VEXT        36
#define PIN_OLED_RST    21
#define PIN_OLED_SDA    17
#define PIN_OLED_SCL    18
#define PIN_LED_PLACA   35
#define PIN_BOTAO_PRG   0      // botão PRG da placa (teste da planilha)

#define LORA_NSS   8
#define LORA_SCK   9
#define LORA_MOSI  10
#define LORA_MISO  11
#define LORA_RST   12
#define LORA_BUSY  13
#define LORA_DIO1  14
// ============================================================

#define ARQ_FILA  "/fila.txt"
#define ARQ_TMP   "/fila.tmp"

U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, PIN_OLED_RST, PIN_OLED_SCL, PIN_OLED_SDA);
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);

volatile bool chegouPacote = false;
void IRAM_ATTR aoReceber() { chegouPacote = true; }

// Último pacote válido
uint16_t      rxViagem = 0;
unsigned long rxSeq = 0, rxDistM = 0, rxCombMl = 0;
int   rxEstado = 0, rxVel = 0, rxTemp = -99, rxRpm = 0, rxVmax = 0, rxUltrap = 0;
float rxRssi = 0, rxSnr = 0;
uint32_t ultimoPacoteMs = 0, totalPacotes = 0, pacotesPerdidos = 0;
bool acimaDoLimite = false;

// Viagem acompanhada pela estação
struct Viagem {
  uint16_t id;
  bool     ativa;
  uint32_t inicioMs;
  int      tempMax;
} viagem = {0, false, 0, -99};

// Fila local + Wi-Fi
SemaphoreHandle_t mutexFila;
volatile int  linhasPendentes = 0;
volatile bool wifiOk = false;

// Atualização remota
volatile int otaProgresso  = -1;      // -1 = não está atualizando | 0..100 = %
volatile int otaVersaoNova = 0;

// Configuração em uso (vem da memória da placa)
String cfgSsid, cfgSenha, cfgUrl;

void carregarConfig() {
  Preferences p;
  p.begin("cfg", false);
  if (strlen(WIFI_SSID) > 0 && String(WIFI_SSID) != "NOME_DO_WIFI") {
    p.putString("ssid", WIFI_SSID);
    p.putString("senha", WIFI_SENHA);
  }
  if (strlen(URL_SCRIPT) > 0 && String(URL_SCRIPT).indexOf("COLE_O_ID") < 0) {
    p.putString("url", URL_SCRIPT);
  }
  cfgSsid  = p.getString("ssid", "");
  cfgSenha = p.getString("senha", "");
  cfgUrl   = p.getString("url", "");
  p.end();
  Serial.printf("Config: Wi-Fi \"%s\" | planilha %s\n", cfgSsid.c_str(),
                cfgUrl.length() ? "configurada" : "NAO configurada");
  if (cfgSsid.length() == 0)
    Serial.println("[ERRO] Sem Wi-Fi salvo: grave uma vez pelo USB com WIFI_SSID e URL_SCRIPT preenchidos");
}

// =================================================================
//  FILA LOCAL (LittleFS)
// =================================================================
time_t agoraEpoch() {                     // 0 = relógio ainda não sincronizado
  time_t t = time(nullptr);
  return t > 1700000000 ? t : 0;
}

void enfileirar(const String &json, bool essencial) {
  xSemaphoreTake(mutexFila, portMAX_DELAY);
  File f = LittleFS.open(ARQ_FILA, FILE_APPEND);
  if (f) {
    if (essencial || f.size() < MAX_FILA_BYTES) {   // fila cheia: descarta só leituras
      f.println(json);
      linhasPendentes = linhasPendentes + 1;
    }
    f.close();
  }
  xSemaphoreGive(mutexFila);
}

void contarPendentes() {
  int n = 0;
  File f = LittleFS.open(ARQ_FILA, FILE_READ);
  if (f) {
    while (f.available()) if (f.read() == '\n') n++;
    f.close();
  }
  linhasPendentes = n;
}

// Monta o corpo do envio com até LINHAS_POR_ENVIO linhas. Devolve quantas.
int lerLote(String &corpo) {
  int n = 0;
  corpo = String("{\"chave\":\"") + CHAVE + "\",\"linhas\":[";
  xSemaphoreTake(mutexFila, portMAX_DELAY);
  File f = LittleFS.open(ARQ_FILA, FILE_READ);
  if (f) {
    while (f.available() && n < LINHAS_POR_ENVIO) {
      String l = f.readStringUntil('\n');
      l.trim();
      if (l.length() == 0) continue;
      if (n > 0) corpo += ",";
      corpo += l;
      n++;
    }
    f.close();
  }
  xSemaphoreGive(mutexFila);
  corpo += "]}";
  return n;
}

// Apaga as n primeiras linhas (já enviadas)
void removerLote(int n) {
  xSemaphoreTake(mutexFila, portMAX_DELAY);
  File in  = LittleFS.open(ARQ_FILA, FILE_READ);
  File out = LittleFS.open(ARQ_TMP, FILE_WRITE);
  int pulei = 0, restantes = 0;
  if (in && out) {
    while (in.available()) {
      String l = in.readStringUntil('\n');
      l.trim();
      if (l.length() == 0) continue;
      if (pulei < n) { pulei++; continue; }
      out.println(l);
      restantes++;
    }
  }
  if (in)  in.close();
  if (out) out.close();
  LittleFS.remove(ARQ_FILA);
  LittleFS.rename(ARQ_TMP, ARQ_FILA);
  linhasPendentes = restantes;
  xSemaphoreGive(mutexFila);
}

// =================================================================
//  LINHAS DA PLANILHA
// =================================================================
String numOuNulo(int v) { return v <= -99 ? String("null") : String(v); }

float kmPorLitro() {
  float l = rxCombMl / 1000.0f;
  return l > 0.05f ? (rxDistM / 1000.0f) / l : 0;
}

void enfileirarLeitura() {
  char buf[320];
  snprintf(buf, sizeof(buf),
    "{\"tipo\":\"leitura\",\"epoch\":%ld,\"viagem\":\"%04X\",\"estado\":%d,\"vel\":%d,"
    "\"temp\":%s,\"rpm\":%d,\"dist_km\":%.3f,\"comb_l\":%.3f,\"kml\":%.2f,"
    "\"vmax\":%d,\"ultrap\":%d,\"rssi\":%.0f}",
    (long)agoraEpoch(), rxViagem, rxEstado, rxVel, numOuNulo(rxTemp).c_str(), rxRpm,
    rxDistM / 1000.0f, rxCombMl / 1000.0f, kmPorLitro(), rxVmax, rxUltrap, rxRssi);
  enfileirar(String(buf), false);
}

void enfileirarViagem(bool encerrada) {
  time_t agora  = agoraEpoch();
  time_t inicio = agora ? agora - (millis() - viagem.inicioMs) / 1000 : 0;
  char buf[320];
  snprintf(buf, sizeof(buf),
    "{\"tipo\":\"viagem\",\"viagem\":\"%04X\",\"inicio\":%ld,\"epoch\":%ld,"
    "\"dist_km\":%.3f,\"comb_l\":%.3f,\"kml\":%.2f,\"vmax\":%d,\"ultrap\":%d,"
    "\"temp_max\":%s,\"duracao_min\":%.1f,\"encerrada\":%s}",
    viagem.id, (long)inicio, (long)agora, rxDistM / 1000.0f, rxCombMl / 1000.0f, kmPorLitro(),
    rxVmax, rxUltrap, numOuNulo(viagem.tempMax).c_str(),
    (ultimoPacoteMs - viagem.inicioMs) / 60000.0f, encerrada ? "true" : "false");
  enfileirar(String(buf), true);
  Serial.printf("# viagem %04X %s: %.2f km, %.2f L, vmax %d, %d ultrapassagens\n",
                viagem.id, encerrada ? "ENCERRADA" : "atualizada",
                rxDistM / 1000.0f, rxCombMl / 1000.0f, rxVmax, rxUltrap);
}

// Teste sem o carro: digite "t" + Enter no Serial Monitor
void enfileirarTeste() {
  char buf[320];
  snprintf(buf, sizeof(buf),
    "{\"tipo\":\"leitura\",\"epoch\":%ld,\"viagem\":\"TEST\",\"estado\":3,\"vel\":42,"
    "\"temp\":90,\"rpm\":2000,\"dist_km\":1.000,\"comb_l\":0.080,\"kml\":12.50,"
    "\"vmax\":63,\"ultrap\":1,\"rssi\":0}", (long)agoraEpoch());
  enfileirar(String(buf), true);
  Serial.println("# linha de TESTE colocada na fila (viagem TEST na aba OBD Leituras)");
}

// =================================================================
//  ATUALIZAÇÃO REMOTA PELO GITHUB
// =================================================================
int lerVersaoRemota() {
  WiFiClientSecure cli;
  cli.setInsecure();
  HTTPClient http;
  http.setTimeout(15000);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  String url = String(URL_VERSAO) + "?t=" + String(millis());   // evita cache
  if (!http.begin(cli, url)) return -1;
  int code = http.GET();
  int v = -1;
  if (code == 200) {
    String t = http.getString();
    t.trim();
    v = t.toInt();
  }
  http.end();
  return v;
}

void verificarAtualizacao() {
  int remota = lerVersaoRemota();
  if (remota <= 0) {
    Serial.println("[OTA] nao consegui ler o versao.txt (URL certa? repositorio publico?)");
    return;
  }
  if (remota <= FW_VERSAO) {
    Serial.printf("[OTA] em dia (v%d)\n", FW_VERSAO);
    return;
  }

  // Proteção: se já tentou essa versão e continua antiga, o .bin publicado
  // provavelmente está com FW_VERSAO errado. Não fica reiniciando em loop.
  Preferences pref;
  pref.begin("ota", false);
  int tentou = pref.getInt("tentou", 0);
  if (tentou == remota) {
    Serial.printf("[OTA] v%d ja foi instalada mas a placa continua v%d: confira o FW_VERSAO do .bin publicado\n",
                  remota, FW_VERSAO);
    pref.end();
    return;
  }
  pref.putInt("tentou", remota);
  pref.end();

  Serial.printf("[OTA] nova versao v%d (atual v%d), baixando...\n", remota, FW_VERSAO);
  otaVersaoNova = remota;
  otaProgresso  = 0;

  WiFiClientSecure cli;
  cli.setInsecure();
  httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  httpUpdate.rebootOnUpdate(false);
  httpUpdate.onProgress([](int atual, int total) {
    if (total > 0) otaProgresso = (int)(100LL * atual / total);
  });
  t_httpUpdate_return r = httpUpdate.update(cli, URL_FIRMWARE);

  if (r == HTTP_UPDATE_OK) {
    Serial.println("[OTA] gravado! Reiniciando com a versao nova...");
    otaProgresso = 100;
    delay(1500);
    ESP.restart();
  }
  Serial.printf("[OTA] falhou (%d): %s\n", httpUpdate.getLastError(),
                httpUpdate.getLastErrorString().c_str());
  otaProgresso = -1;
}

// =================================================================
//  ENVIO PARA O GOOGLE SHEETS (tarefa no outro núcleo)
// =================================================================
bool enviarLote(const String &corpo, int n) {
  WiFiClientSecure cli;
  cli.setInsecure();                       // sem validar certificado (simples)
  HTTPClient http;
  http.setTimeout(20000);
  http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
  if (cfgUrl.length() == 0) return false;
  if (!http.begin(cli, cfgUrl)) return false;
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(corpo);
  String resp = (code == 200) ? http.getString() : String("");
  http.end();
  // O Google responde 302 (redirecionamento) depois de gravar: conta como sucesso
  bool ok = (code == 302) || (code == 200 && resp.indexOf("\"ok\":true") >= 0);
  Serial.printf("[Sheets] %d linhas -> HTTP %d %s\n", n, code, ok ? "OK" : "FALHOU");
  if (code == 200 && !ok) Serial.println("[Sheets] resposta: " + resp.substring(0, 200));
  if (!ok && code != 200) Serial.println("[Sheets] Se a rede tem pagina de login (ex.: rede de convidados), use o hotspot do celular.");
  return ok;
}

void tarefaUpload(void *) {
  uint32_t ultTentativa = 0;
  bool ntpIniciado = false;
  WiFi.mode(WIFI_STA);

  for (;;) {
    if (cfgSsid.length() == 0) { vTaskDelay(pdMS_TO_TICKS(5000)); continue; }
    if (WiFi.status() != WL_CONNECTED) {
      if (wifiOk) Serial.println("[WiFi] caiu");
      wifiOk = false;
      if (ultTentativa == 0 || millis() - ultTentativa > 15000) {
        ultTentativa = millis();
        Serial.printf("[WiFi] conectando em \"%s\"...\n", cfgSsid.c_str());
        WiFi.disconnect();
        WiFi.begin(cfgSsid.c_str(), cfgSenha.c_str());
      }
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    if (!wifiOk) {
      wifiOk = true;
      Serial.printf("[WiFi] conectado, IP %s\n", WiFi.localIP().toString().c_str());
    }
    if (!ntpIniciado) {
      configTzTime("<-03>3", "pool.ntp.org", "time.google.com");   // horário de Brasília
      ntpIniciado = true;
    }

    static uint32_t ultimaOTA = 0;
    if (ultimaOTA == 0 || millis() - ultimaOTA >= INTERVALO_OTA_MS) {
      ultimaOTA = millis();
      verificarAtualizacao();
    }

    if (linhasPendentes > 0 && cfgUrl.length() > 0) {
      String corpo;
      int n = lerLote(corpo);
      if (n > 0) {
        if (enviarLote(corpo, n)) { removerLote(n); continue; }   // manda o próximo lote já
        vTaskDelay(pdMS_TO_TICKS(10000));                         // falhou: espera e tenta de novo
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// ---------- LoRa ----------
void iniciarLoRa() {
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
#if defined(RADIOLIB_VERSION_MAJOR) && RADIOLIB_VERSION_MAJOR >= 8
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
  int st = radio.begin(LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR,
                       LORA_SYNC, LORA_POWER_DBM, 8, 1.8);
#endif
  if (st != RADIOLIB_ERR_NONE) {
    Serial.printf("[ERRO] LoRa falhou, codigo %d\n", st);
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0, 30, "ERRO NO LORA");
    oled.sendBuffer();
    while (true) delay(1000);
  }
  radio.setDio1Action(aoReceber);
  radio.startReceive();
  Serial.println("LoRa OK (915 MHz), aguardando o carro...");
  Serial.println("viagem,seq,estado,vel,temp,rpm,dist_m,comb_ml,vmax,ultrap,rssi,snr");
}

void tratarPacote() {
  String s;
  int st = radio.readData(s);
  if (st == RADIOLIB_ERR_NONE) {
    unsigned int via;
    unsigned long seq, dist, comb;
    int est, vel, temp, rpm, vmax, ult;
    if (sscanf(s.c_str(), "OBD,%x,%lu,%d,%d,%d,%d,%lu,%lu,%d,%d",
               &via, &seq, &est, &vel, &temp, &rpm, &dist, &comb, &vmax, &ult) == 10 &&
        est >= 0 && est <= 3 && vel >= 0 && vel <= 255) {

      // Viagem nova (o carro ligou de novo): fecha a anterior e começa outra
      if (!viagem.ativa || via != viagem.id) {
        if (viagem.ativa) enfileirarViagem(true);
        viagem.id       = via;
        viagem.ativa    = true;
        viagem.inicioMs = millis();
        viagem.tempMax  = -99;
        totalPacotes = 0;
        pacotesPerdidos = 0;
        Serial.printf("# nova viagem %04X\n", via);
      }

      if (totalPacotes > 0 && seq > rxSeq + 1) pacotesPerdidos += seq - rxSeq - 1;
      rxViagem = via; rxSeq = seq; rxEstado = est; rxVel = vel; rxTemp = temp; rxRpm = rpm;
      rxDistM = dist; rxCombMl = comb; rxVmax = vmax; rxUltrap = ult;
      rxRssi = radio.getRSSI();
      rxSnr  = radio.getSNR();
      ultimoPacoteMs = millis();
      totalPacotes++;
      if (temp > viagem.tempMax) viagem.tempMax = temp;

      Serial.printf("%04X,%lu,%d,%d,%d,%d,%lu,%lu,%d,%d,%.0f,%.1f\n", via, seq, est, vel, temp, rpm,
                    dist, comb, vmax, ult, rxRssi, rxSnr);
      digitalWrite(PIN_LED_PLACA, HIGH);   // pisca a cada pacote
    }
  } else if (st == RADIOLIB_ERR_CRC_MISMATCH) {
    Serial.println("# pacote corrompido (CRC)");
  }
  radio.startReceive();
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

void telaSemSinal(uint32_t t) {
  oled.setFont(u8g2_font_6x10_tf);
  const char *m = "SEM SINAL DO CARRO";
  oled.drawStr((128 - oled.getStrWidth(m)) / 2, 10, m);

  int cx = 64, cy = 34;                                       // antena
  oled.drawLine(cx, cy, cx, 56);
  oled.drawLine(cx, cy + 6, cx - 6, cy);
  oled.drawLine(cx, cy + 6, cx + 6, cy);
  oled.drawHLine(cx - 10, 56, 21);
  int fase = (t / 300) % 4;                                   // ondas pulsando
  for (int k = 1; k <= 3; k++)
    if (k <= fase) oled.drawCircle(cx, cy, 5 + k * 5, U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_UPPER_RIGHT);
  if ((t / 600) % 2) {                                        // X piscando
    oled.drawLine(cx + 20, 38, cx + 30, 48);
    oled.drawLine(cx + 30, 38, cx + 20, 48);
  }
}

// Ícone de Wi-Fi e linhas esperando envio (canto superior esquerdo)
void desenharStatusRede() {
  if (wifiOk) {
    oled.drawVLine(9, 16, 2);
    oled.drawVLine(11, 14, 4);
    oled.drawVLine(13, 12, 6);
  } else {
    oled.drawLine(9, 12, 14, 17);
    oled.drawLine(14, 12, 9, 17);
  }
  if (linhasPendentes > 0) {
    char b[8];
    int pend = linhasPendentes;
    snprintf(b, sizeof(b), "%d", pend > 999 ? 999 : pend);
    oled.setFont(u8g2_font_4x6_tf);
    oled.drawStr(16, 18, b);
  }
}

void telaAtualizando() {
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);
  const char *m = "ATUALIZANDO...";
  oled.drawStr((128 - oled.getStrWidth(m)) / 2, 12, m);
  char b[32];
  snprintf(b, sizeof(b), "v%d -> v%d", FW_VERSAO, (int)otaVersaoNova);
  oled.drawStr((128 - oled.getStrWidth(b)) / 2, 28, b);
  int p = otaProgresso;
  oled.drawFrame(10, 38, 108, 10);
  oled.drawBox(12, 40, (104 * p) / 100, 6);
  snprintf(b, sizeof(b), "%d%%  nao desligue", p);
  oled.setFont(u8g2_font_5x7_tf);
  oled.drawStr((128 - oled.getStrWidth(b)) / 2, 60, b);
  oled.sendBuffer();
}

// ---------- DESENHO (OLED 128x64) ----------
void desenharTela(float dt) {
  uint32_t t = millis();
  if (otaProgresso >= 0) { telaAtualizando(); return; }
  bool semSinal = (ultimoPacoteMs == 0) || (t - ultimoPacoteMs > TIMEOUT_LORA_MS);

  // Alerta com histerese
  if (semSinal || rxEstado != 3) acimaDoLimite = false;
  else if (rxVel > LIMITE_KMH) acimaDoLimite = true;
  else if (rxVel <= LIMITE_KMH - 2) acimaDoLimite = false;

  atualizarAnim(rxVel, !semSinal && rxEstado == 3, dt);
  oled.clearBuffer();
  if (semSinal) telaSemSinal(t);
  else switch (rxEstado) {
    case 3:  telaVelocidade(rxVel, acimaDoLimite, t);           break;
    case 2:  telaDormindo(t);                                    break;
    case 1:  telaProcurando("PROCURANDO ELM327", t);             break;
    default: telaProcurando("CARRO INICIANDO", t);               break;
  }
  if (!semSinal && t - ultimoPacoteMs < 150) oled.drawDisc(3, 3, 2);   // pontinho: chegou pacote
  desenharStatusRede();
  oled.sendBuffer();
}

// ---------- SETUP / LOOP ----------
void setup() {
  Serial.begin(115200);

  pinMode(PIN_VEXT, OUTPUT);
  digitalWrite(PIN_VEXT, LOW);   // liga o OLED
  delay(100);
  pinMode(PIN_LED_PLACA, OUTPUT);
  pinMode(PIN_BOTAO_PRG, INPUT_PULLUP);

  oled.begin();
  oled.setBusClock(400000);
  randomSeed(esp_random());
  splash("ESTACAO + SHEETS");

  carregarConfig();

  // Fila local
  mutexFila = xSemaphoreCreateMutex();
  if (!LittleFS.begin(true)) Serial.println("[ERRO] LittleFS nao montou");
  contarPendentes();
  Serial.printf("Fila local: %d linhas esperando envio\n", (int)linhasPendentes);
  Serial.printf("Firmware v%d | teste da planilha: digite t ou aperte PRG\n", FW_VERSAO);

  iniciarLoRa();
  xTaskCreatePinnedToCore(tarefaUpload, "upload", 12288, NULL, 1, NULL, 0);
}

void loop() {
  static uint32_t ultimoDesenho = 0, ultimoResumo = 0;
  static uint32_t ultLeitura = 0, ultViagem = 0;

  if (chegouPacote) {
    chegouPacote = false;
    tratarPacote();
  }

  // Teste da planilha: "t" no Serial Monitor OU apertar o botão PRG da placa
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 't' || c == 'T') enfileirarTeste();
  }
  static uint32_t ultimoBotao = 0;
  if (digitalRead(PIN_BOTAO_PRG) == LOW && millis() - ultimoBotao > 1000) {
    ultimoBotao = millis();
    enfileirarTeste();
  }

  // Apaga o LED 50 ms depois do pacote
  if (millis() - ultimoPacoteMs > 50) digitalWrite(PIN_LED_PLACA, LOW);

  // Guarda dados para a planilha
  if (viagem.ativa) {
    uint32_t semPacote = millis() - ultimoPacoteMs;
    if (semPacote > FIM_VIAGEM_MS) {
      enfileirarViagem(true);                                  // viagem acabou
      viagem.ativa = false;
    } else {
      if (semPacote < TIMEOUT_LORA_MS && millis() - ultLeitura >= INTERVALO_LEITURA_MS) {
        ultLeitura = millis();
        enfileirarLeitura();
      }
      if (millis() - ultViagem >= INTERVALO_VIAGEM_MS) {
        ultViagem = millis();
        enfileirarViagem(false);
      }
    }
  }

  // OLED animado a ~25 fps
  if (millis() - ultimoDesenho >= 40) {
    float dt = min(0.1f, (millis() - ultimoDesenho) / 1000.0f);
    ultimoDesenho = millis();
    desenharTela(dt);
  }

  // Resumo de qualidade do link a cada 30 s
  if (millis() - ultimoResumo > 30000 && totalPacotes > 0) {
    ultimoResumo = millis();
    float perda = 100.0 * pacotesPerdidos / (totalPacotes + pacotesPerdidos);
    Serial.printf("# recebidos=%lu perdidos=%lu perda=%.1f%% | fila=%d | wifi=%s\n",
                  (unsigned long)totalPacotes, (unsigned long)pacotesPerdidos, perda,
                  (int)linhasPendentes, wifiOk ? "ok" : "sem");
  }
}
