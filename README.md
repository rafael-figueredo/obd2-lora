# Velocímetro OBD2 + LoRa

Projeto com duas placas **Heltec WiFi LoRa 32 V3** (ESP32-S3 + SX1262 + OLED):

- **Carro:** lê o ELM327 (Bluetooth) por um módulo **HC-05**, mostra a velocidade animada no OLED, acende LEDs e buzzer acima de 60 km/h e transmite os dados por **LoRa (915 MHz)**.
- **Estação:** recebe por LoRa, guarda numa fila na própria placa e envia pelo Wi-Fi para o **Google Sheets** (abas *OBD Leituras* e *OBD Viagens*).

```
ELM327 ──Bluetooth──> HC-05 ──serial──> Heltec do carro ──LoRa──> Heltec da estação ──Wi-Fi──> Google Sheets
```

## Pastas

| Pasta | Conteúdo |
|---|---|
| `firmware/1_configurar_hc05` | Configura o HC-05 como mestre pareado com o ELM327 (roda uma vez, na Heltec do carro) |
| `firmware/2_heltec_carro_hc05` | Código do carro: leitura OBD, painel, LoRa e atualização pelo navegador |
| `firmware/3_heltec_estacao_receptor` | Código da estação: LoRa, fila local, Google Sheets e atualização pelo GitHub |
| `planilha/apps_script.gs` | Script do Google Sheets (Extensões > Apps Script) |
| `estacao/` | `versao.txt` e `firmware.bin` que a estação baixa sozinha (OTA) |

> **Dados sensíveis ficam fora do GitHub.** O código da estação aqui está com `WIFI_SSID`,
> `WIFI_SENHA` e `URL_SCRIPT` vazios de propósito. Na **primeira gravação pelo USB**, use uma cópia
> com esses dados preenchidos: a placa salva tudo na memória dela (que não é apagada nas
> atualizações). Os `firmware.bin` publicados aqui são compilados **desta versão vazia** e usam
> o que já está salvo na placa.

## Ligações (Heltec do carro)

| Heltec | Liga em |
|---|---|
| 7 | TXD do HC-05 |
| 2 | RXD do HC-05 |
| 5V | VCC do HC-05 |
| 4 | + do buzzer ativo |
| 5 | resistor 330 Ω → LED vermelho |
| 6 | resistor 100 Ω → LED azul |
| GND | GND do HC-05, negativos dos LEDs e do buzzer |

Bibliotecas: **U8g2** (olikraus) e **RadioLib** (jgromes). Placa: *Heltec WiFi LoRa 32(V3)*.

## Atualização sem fio

### Estação (pelo GitHub, de qualquer lugar)

A estação confere `estacao/versao.txt` a cada 5 minutos. Se o número for maior que o `FW_VERSAO` dela, baixa `estacao/firmware.bin`, grava e reinicia.

Para publicar uma versão nova:

1. Abra **`firmware/3_heltec_estacao_receptor`** (a versão SEM Wi-Fi/URL) e aumente `#define FW_VERSAO` (ex.: de `1` para `2`).
2. Arduino IDE: **Sketch > Export Compiled Binary**.
3. Na pasta `build/...` do sketch, pegue o arquivo **`.ino.bin`** (não o `.merged.bin`).
4. Aqui no GitHub, envie esse arquivo para a pasta `estacao/` com o nome **`firmware.bin`**.
5. Edite `estacao/versao.txt` para o **mesmo número** do `FW_VERSAO`.
6. Em até ~5 minutos a estação mostra *ATUALIZANDO...* e reinicia com a versão nova.

### Carro (pelo navegador)

1. Com o carro parado, segure o botão **PRG** por 3 s.
2. Conecte o celular/PC na rede **`OBD-CARRO`** (senha `obd2lora`).
3. Abra **http://192.168.4.1** (usuário `admin`, senha `obd2lora`).
4. Envie o `.ino.bin` do código do carro.

A primeira gravação de cada placa (a que já contém o OTA) é feita pelo USB.
