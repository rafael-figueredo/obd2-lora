/*
 * CONFIGURADOR DO HC-05 (rodar UMA VEZ, na Heltec do carro)
 * ----------------------------------------------------------
 * Deixa o HC-05 como MESTRE, pareado com o seu ELM327 (MAC 00:10:CC:4F:36:03),
 * senha 1234 e serial a 115200. Depois disso ele conecta sozinho no ELM ao ligar.
 *
 * COMO ENTRAR NO MODO AT:
 *   1. Ligue os 4 fios do HC-05 na Heltec (VCC->5V, GND->GND, TXD->7, RXD->2)
 *   2. Tire o fio do VCC do HC-05
 *   3. SEGURE o botãozinho do HC-05 e recoloque o fio do VCC
 *   4. Solte o botão: o LED do HC-05 deve piscar DEVAGAR (1x a cada 2 s)
 *      (sem botão no módulo: ligue o pino EN do HC-05 no 3V3 antes de dar energia)
 *   5. Grave este código e abra o Serial Monitor em 115200
 *
 * Faça isso com o ELM327 na tomada do carro e a ignição LIGADA,
 * assim o pareamento já é testado na hora.
 *
 * Depois que terminar, o Serial Monitor vira um terminal: dá pra digitar
 * comandos AT na mão (configure o Serial Monitor para "Both NL & CR").
 *
 * Placa: "Heltec WiFi LoRa 32(V3)" | Serial Monitor: 115200
 */

#include <Arduino.h>

#define PIN_RX_HC05  7      // recebe do TXD do HC-05
#define PIN_TX_HC05  2      // envia para o RXD do HC-05
#define BAUD_AT      38400  // velocidade fixa do modo AT

// MAC do ELM327 00:10:CC:4F:36:03 no formato do HC-05 (NAP,UAP,LAP)
#define ELM_MAC_HC05 "0010,CC,4F3603"
#define ELM_PIN      "1234"

String enviarAT(const String &cmd, uint32_t timeoutMs = 2000) {
  while (Serial1.available()) Serial1.read();
  Serial1.print(cmd);
  Serial1.print("\r\n");

  String r;
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    while (Serial1.available()) r += (char)Serial1.read();
    if (r.indexOf("OK") >= 0 || r.indexOf("ERROR") >= 0 || r.indexOf("FAIL") >= 0) {
      delay(100);
      while (Serial1.available()) r += (char)Serial1.read();
      break;
    }
    delay(5);
  }
  r.trim();
  r.replace("\r\n", " | ");
  Serial.printf("  %-30s -> %s\n", cmd.c_str(), r.length() ? r.c_str() : "(sem resposta)");
  return r;
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n=== Configurador HC-05 ===");

  // 1. Procura o HC-05: testa velocidades e fios normais/invertidos
  const long bauds[] = {38400, 9600, 115200, 57600, 19200};
  bool achou = false, invertido = false;
  long baudAchado = 0;
  String r;

  for (int inv = 0; inv < 2 && !achou; inv++) {
    for (long b : bauds) {
      Serial1.end();
      delay(50);
      if (inv == 0) Serial1.begin(b, SERIAL_8N1, PIN_RX_HC05, PIN_TX_HC05);
      else          Serial1.begin(b, SERIAL_8N1, PIN_TX_HC05, PIN_RX_HC05);
      delay(100);
      Serial.printf("Testando %ld baud, fios %s...\n", b, inv ? "INVERTIDOS" : "normais");
      r = enviarAT("AT", 1000);
      if (r.indexOf("OK") < 0) r = enviarAT("AT", 1000);
      if (r.indexOf("OK") >= 0) { achou = true; invertido = inv; baudAchado = b; break; }
    }
  }

  if (!achou) {
    Serial.println("\n[ERRO] O HC-05 nao respondeu em nenhuma velocidade.");
    Serial.println("  1. O LED do HC-05 acende? Se nao: confira VCC no 5V e GND no GND.");
    Serial.println("  2. Pisca RAPIDO? Nao entrou no modo AT: tire o VCC, SEGURE o botao,");
    Serial.println("     recoloque o VCC, solte. Tem que piscar DEVAGAR (1x a cada 2 s).");
    Serial.println("  3. Confira se os fios estao nos pinos 7 e 2 (numeros impressos na Heltec).");
    Serial.println("Aperte RST na Heltec para tentar de novo.");
    return;
  }

  Serial.printf("\nHC-05 encontrado: %ld baud, fios %s\n", baudAchado, invertido ? "INVERTIDOS" : "normais");
  if (invertido) {
    Serial.println("  !!! Seus fios TXD/RXD estao trocados. A configuracao vai funcionar agora,");
    Serial.println("  !!! mas DEPOIS inverta: HC-05 TXD -> pino 7 e RXD -> pino 2.");
  }
  if (baudAchado != 38400) {
    Serial.println("  (Nao esta no modo AT completo: alguns comandos podem dar ERROR.");
    Serial.println("   Se der, refaca a entrada no modo AT segurando o botao ao ligar.)");
  }

  // 2. Informações do módulo
  enviarAT("AT+VERSION?");

  // 3. Configuração
  // Firmware 3.0-20170601 sai do modo AT ao mudar o ROLE.
  // Por isso: se já for mestre, pula; se não for, muda e pede pra rodar de novo.
  r = enviarAT("AT+ROLE?");
  if (r.indexOf("ROLE:1") < 0) {
    enviarAT("AT+RMAAD");                     // apaga pareamentos antigos
    enviarAT("AT+ROLE=1");                    // mestre
    delay(1500);
    r = enviarAT("AT");
    if (r.indexOf("OK") < 0) {
      Serial.println("\n>>> ROLE=1 salvo, mas o HC-05 saiu do modo AT (normal nesse firmware).");
      Serial.println(">>> Entre no modo AT DE NOVO (tire o VCC, segure o botao, recoloque, solte)");
      Serial.println(">>> e aperte RST na Heltec. Na 2a vez ele pula esta parte e termina.");
      return;
    }
  } else {
    Serial.println("  Ja esta como MESTRE, pulando ROLE.");
  }
  enviarAT("AT+CMODE=0");                     // conecta só no endereço do BIND

  // Senha do ELM: cada firmware de HC-05 usa um formato, tenta todos
  const String senhas[] = {
    String("AT+PSWD=") + ELM_PIN,                 // firmware 2.0
    String("AT+PSWD=\"") + ELM_PIN + "\"",        // firmware 3.0
    String("AT+PSWD:") + ELM_PIN,                 // alguns clones
    String("AT+PIN=") + ELM_PIN,
    String("AT+PIN") + ELM_PIN                    // clones "_le"
  };
  for (const String &cmd : senhas) {
    r = enviarAT(cmd);
    if (r.indexOf("OK") >= 0) break;
  }

  enviarAT("AT+UART=115200,0,0");             // serial rápida: o HC-05 manda o comando inteiro num pacote só
  enviarAT(String("AT+BIND=") + ELM_MAC_HC05);

  // 4. Conferência (antes de conectar, senão ele sai do modo AT)
  Serial.println("\nConferindo a configuracao:");
  enviarAT("AT+ROLE?");
  enviarAT("AT+CMODE?");
  enviarAT("AT+BIND?");
  enviarAT("AT+UART?");
  enviarAT("AT+PSWD?");

  // 5. Pareia e testa a conexão (precisa do ELM ligado no carro)
  enviarAT("AT+INIT", 3000);                  // "ERROR:(17)" aqui é normal
  Serial.println("  Pareando com o ELM327 (ate 25 s)...");
  r = enviarAT(String("AT+PAIR=") + ELM_MAC_HC05 + ",20", 25000);
  if (r.indexOf("OK") >= 0) {
    Serial.println("  Conectando (ate 15 s)...");
    r = enviarAT(String("AT+LINK=") + ELM_MAC_HC05, 15000);
    if (r.indexOf("FAIL") >= 0) Serial.println("  >>> LINK FAIL: o ELM recusou a conexao (senha? celular conectado nele?)");
    else if (r.length() == 0)   Serial.println("  >>> Sem resposta no LINK: normalmente quer dizer que CONECTOU (ele sai do modo AT). Veja se o LED do HC-05 da 2 piscadas a cada 2 s.");
  } else {
    Serial.println("  (Pareamento falhou: ELM327 ligado no carro com ignicao ligada?");
    Serial.println("   Se o BIND deu OK, ele ainda pode conectar sozinho no modo normal.)");
  }

  Serial.println("\nPronto! Esperado: ROLE:1, CMODE:0, BIND:10:cc:4f3603, UART:115200,0,0.");
  Serial.println("Agora: tire e recoloque o VCC do HC-05 SEM segurar o botao,");
  Serial.println("e grave o codigo 2_heltec_carro_hc05.");
  Serial.println("\n(Terminal livre: digite comandos AT, Serial Monitor em 'Both NL & CR')");
}

void loop() {
  // Ponte manual Serial Monitor <-> HC-05
  while (Serial.available())  Serial1.write(Serial.read());
  while (Serial1.available()) Serial.write(Serial1.read());
}
