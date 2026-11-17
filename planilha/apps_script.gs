/**
 * Apps Script — recebe os dados da estação LoRa e grava no Google Sheets
 * --------------------------------------------------------------------
 * Cria sozinho as abas:
 *   "OBD Leituras" -> uma linha a cada 10 s durante a viagem
 *   "OBD Viagens"  -> uma linha por viagem (atualizada enquanto a viagem acontece)
 *
 * COMO INSTALAR:
 *   1. Crie uma planilha NOVA no Google Sheets (só para este projeto)
 *   2. Extensões > Apps Script, apague o que tiver e cole este código
 *   3. Salve, escolha a função "testar" e clique em Executar (autorize o acesso)
 *      -> devem aparecer as abas "OBD Leituras" e "OBD Viagens" com uma linha "TEST"
 *   4. Implantar > Nova implantação > tipo "App da Web"
 *        Executar como: Eu  |  Quem pode acessar: Qualquer pessoa
 *   5. Copie a URL que termina em /exec e cole no URL_SCRIPT da estação
 */

const PLANILHA_ID  = '';    // vazio = a planilha onde este script foi colado
const ABA_LEITURAS = 'OBD Leituras';                 // nomes das abas (não mexe nas outras)
const ABA_VIAGENS  = 'OBD Viagens';

const FUSO   = 'America/Sao_Paulo';
const CHAVE  = 'obd2-lora-2026';          // tem que ser IGUAL ao CHAVE do código da estação
const ESTADOS = ['Iniciando', 'Procurando ELM327', 'Carro desligado', 'Lendo'];

const CAB_LEITURAS = [
  'Data/hora', 'Viagem', 'Estado', 'Velocidade (km/h)', 'Temp. motor (°C)', 'RPM',
  'Distância (km)', 'Combustível (L)', 'Consumo médio (km/L)', 'Vel. máxima (km/h)',
  'Ultrapassagens do limite', 'Sinal LoRa (dBm)'
];
const CAB_VIAGENS = [
  'Viagem', 'Início', 'Última atualização', 'Situação', 'Distância (km)', 'Combustível (L)',
  'Consumo médio (km/L)', 'Vel. máxima (km/h)', 'Ultrapassagens do limite',
  'Temp. máx. motor (°C)', 'Duração (min)'
];

function doPost(e) {
  const lock = LockService.getScriptLock();
  lock.waitLock(30000);
  try {
    const dados = JSON.parse(e.postData.contents);
    if (dados.chave !== CHAVE) return json_({ ok: false, erro: 'chave invalida' });

    const ss = abrirPlanilha_();
    if (!PLANILHA_ID && ss.getSpreadsheetTimeZone() !== FUSO) ss.setSpreadsheetTimeZone(FUSO);
    const shL = aba_(ss, ABA_LEITURAS, CAB_LEITURAS, [1], 2);
    const shV = aba_(ss, ABA_VIAGENS,  CAB_VIAGENS,  [2, 3], 1);

    const linhas = dados.linhas || [];
    const novas = [];
    linhas.forEach(function (l) {
      if (l.tipo === 'leitura')     novas.push(linhaLeitura_(l));
      else if (l.tipo === 'viagem') gravarViagem_(shV, l);
    });
    if (novas.length) {
      shL.getRange(shL.getLastRow() + 1, 1, novas.length, novas[0].length).setValues(novas);
    }
    return json_({ ok: true, n: linhas.length });
  } catch (err) {
    return json_({ ok: false, erro: String(err), onde: String(err.stack || '') });
  } finally {
    lock.releaseLock();
  }
}

// Abrir a URL no navegador mostra se o script está no ar
function doGet() {
  return json_({ ok: true, msg: 'Estacao OBD2 LoRa: script no ar' });
}

// ---------- linhas ----------
function linhaLeitura_(l) {
  return [
    dataDe_(l.epoch), String(l.viagem), ESTADOS[l.estado] || l.estado, l.vel, vazio_(l.temp),
    l.rpm, l.dist_km, l.comb_l, l.kml || '', l.vmax, l.ultrap, l.rssi
  ];
}

function gravarViagem_(sh, l) {
  const linha = [
    String(l.viagem), l.inicio > 1e9 ? new Date(l.inicio * 1000) : '', dataDe_(l.epoch),
    l.encerrada ? 'Encerrada' : 'Em andamento', l.dist_km, l.comb_l, l.kml || '',
    l.vmax, l.ultrap, vazio_(l.temp_max), l.duracao_min
  ];
  const n = sh.getLastRow();
  if (n > 1) {
    const ids = sh.getRange(2, 1, n - 1, 1).getDisplayValues().map(function (r) { return r[0]; });
    const i = ids.lastIndexOf(String(l.viagem));
    if (i >= 0) {                                    // viagem já existe: atualiza a linha
      if (!linha[1]) linha[1] = sh.getRange(i + 2, 2).getValue();
      sh.getRange(i + 2, 1, 1, linha.length).setValues([linha]);
      return;
    }
  }
  sh.appendRow(linha);
}

// ---------- auxiliares ----------
function abrirPlanilha_() {
  const id = (PLANILHA_ID || '').trim();
  if (id && id !== 'COLE_O_ID_DA_SUA_PLANILHA') return SpreadsheetApp.openById(id);
  return SpreadsheetApp.getActiveSpreadsheet();
}

function aba_(ss, nome, cab, colsData, colTexto) {
  let sh = ss.getSheetByName(nome);
  if (!sh) {
    sh = ss.insertSheet(nome);
    sh.getRange(1, 1, 1, cab.length).setValues([cab])
      .setFontWeight('bold').setBackground('#e8eaf6').setWrap(true);
    sh.setFrozenRows(1);
    sh.getRange(2, colTexto, sh.getMaxRows() - 1, 1).setNumberFormat('@');   // ID da viagem como texto
    colsData.forEach(function (c) {
      sh.getRange(2, c, sh.getMaxRows() - 1, 1).setNumberFormat('dd/mm/yyyy hh:mm:ss');
    });
    sh.autoResizeColumns(1, cab.length);
  }
  return sh;
}

function dataDe_(epoch) { return epoch && epoch > 1e9 ? new Date(epoch * 1000) : new Date(); }
// (as datas aparecem no fuso configurado na planilha: Arquivo > Configurações > Fuso horário)
function vazio_(v)      { return (v === null || v === undefined) ? '' : v; }
function json_(o) {
  return ContentService.createTextOutput(JSON.stringify(o)).setMimeType(ContentService.MimeType.JSON);
}

// ---------- teste manual (Executar > testar) ----------
function testar() {
  const e = { postData: { contents: JSON.stringify({
    chave: CHAVE,
    linhas: [
      { tipo: 'leitura', epoch: 0, viagem: 'TEST', estado: 3, vel: 42, temp: 90, rpm: 2100,
        dist_km: 1.234, comb_l: 0.1, kml: 12.34, vmax: 63, ultrap: 1, rssi: -80 },
      { tipo: 'viagem', viagem: 'TEST', inicio: 0, epoch: 0, dist_km: 1.234, comb_l: 0.1,
        kml: 12.34, vmax: 63, ultrap: 1, temp_max: 92, duracao_min: 5.0, encerrada: false }
    ]
  }) } };
  Logger.log(doPost(e).getContent());
}
