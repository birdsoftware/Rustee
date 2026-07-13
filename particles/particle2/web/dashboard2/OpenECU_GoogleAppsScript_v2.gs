const COLUMNS = [
  { owner: "BOTH", header: "timestamp", jsonKey: null },
  { owner: "CAB_ECU", header: "CAB_UnitID", jsonKey: "cabUnitID" },
  { owner: "TRAILER_ECU", header: "TRAILER_UnitID", jsonKey: "trailerUnitID" },
  { owner: "TRAILER_ECU", header: "DischargeChargeCurrLim", jsonKey: "dccLim" },
  { owner: "TRAILER_ECU", header: "PumpLowSpeedRPM", jsonKey: "pumpLowRPM" },
  { owner: "TRAILER_ECU", header: "PumpHighSpeedRPM", jsonKey: "pumpRPM" },
  { owner: "TRAILER_ECU", header: "FlipFlowDirection", jsonKey: "flipFlow" },
  { owner: "TRAILER_ECU", header: "CI_TankerPressure", jsonKey: "tankerPressure" },
  { owner: "TRAILER_ECU", header: "CI_ExternalPressure", jsonKey: "externalPressure" },
  { owner: "TRAILER_ECU", header: "CAN_RectifierTemp", jsonKey: "rectTemp" },
  { owner: "TRAILER_ECU", header: "CAN_McuDcCurrent", jsonKey: "dcCurrent" },
  { owner: "TRAILER_ECU", header: "CAN_McuDcVoltage", jsonKey: "dcVoltage" },
  { owner: "TRAILER_ECU", header: "CAN_McuMotorSpeed", jsonKey: "motorSpeed" },
  { owner: "TRAILER_ECU", header: "CAN_McuMotorTemp", jsonKey: "motorTemp" },
  { owner: "CAB_ECU", header: "CI_RectifierTempDegC", jsonKey: "ciRectTemp" },
  { owner: "CAB_ECU", header: "CI_GeneratorTempDegC", jsonKey: "ciGenTemp" },
  { owner: "BOTH", header: "frames", jsonKey: "frames" },
  { owner: "BOTH", header: "ccpReq", jsonKey: "ccpReq" },
  { owner: "BOTH", header: "ccpResp", jsonKey: "ccpResp" },
  { owner: "BOTH", header: "matched", jsonKey: "matched" },
  { owner: "BOTH", header: "successPct", jsonKey: "successPct" },
  { owner: "BOTH", header: "activeTimeout", jsonKey: "activeTimeout" },
  { owner: "BOTH", header: "txSkip", jsonKey: "txSkip" },
  { owner: "TRAILER_ECU", header: "Log_FatalError", jsonKey: "fatalError" },
  { owner: "TRAILER_ECU", header: "Log_ErrorCode", jsonKey: "errorCode" },
  { owner: "TRAILER_ECU", header: "CAN_GeneratorTemp", jsonKey: "generatorTemp" },
  { owner: "TRAILER_ECU", header: "pdgc_override_service_0a", jsonKey: "pdgcOverride0a" },
  { owner: "TRAILER_ECU", header: "pdgc_override_service_03", jsonKey: "pdgcOverride03" },
  { owner: "TRAILER_ECU", header: "CAN_RectifierSensorError", jsonKey: "rectifierSensorError" },
  { owner: "TRAILER_ECU", header: "CAN_PtoLatchedOpen", jsonKey: "ptoLatchedOpen" },
  { owner: "TRAILER_ECU", header: "CAN_GeneratorSensorError", jsonKey: "generatorSensorError" },
  { owner: "TRAILER_ECU", header: "CAN_DoorInterlock", jsonKey: "doorInterlock" },
  { owner: "TRAILER_ECU", header: "Log_PumpFrozen", jsonKey: "pumpFrozen" },
  { owner: "TRAILER_ECU", header: "Log_PrechargeTimeout", jsonKey: "prechargeTimeout" },
  { owner: "TRAILER_ECU", header: "Log_McuStartupStatus", jsonKey: "mcuStartupStatus" },
  { owner: "TRAILER_ECU", header: "Log_McuHeartbeat", jsonKey: "mcuHeartbeat" },
  { owner: "TRAILER_ECU", header: "Log_McuCanTimeout", jsonKey: "mcuCanTimeout" },
  { owner: "TRAILER_ECU", header: "Log_IgnorePressureSensors", jsonKey: "ignorePressureSensors" },
  { owner: "TRAILER_ECU", header: "Log_IgnoreCabEnclosureTempSensors", jsonKey: "ignoreCabTempSensors" },
  { owner: "TRAILER_ECU", header: "MFSM_WakeInverter", jsonKey: "wakeInverter" },
  { owner: "TRAILER_ECU", header: "MFSM_StartPrecharge", jsonKey: "startPrecharge" },
  { owner: "TRAILER_ECU", header: "MFSM_RestartRequest", jsonKey: "restartRequest" },
  { owner: "TRAILER_ECU", header: "MFSM_IgnoreErrors", jsonKey: "ignoreErrors" },
  { owner: "TRAILER_ECU", header: "MFSM_ClosePosContact", jsonKey: "closePosContact" },
  { owner: "TRAILER_ECU", header: "MFSM_CloseNegContact", jsonKey: "closeNegContact" },
  { owner: "TRAILER_ECU", header: "Log_TorqueRefPercent", jsonKey: "torqueRefPercent" }
];

const LEGACY_FALLBACK_INDEXES = {
  timestamp: 0,
  dccLim: 1,
  pumpRPM: 2,
  rectTemp: 4,
  dcCurrent: 5,
  dcVoltage: 6,
  motorSpeed: 7,
  motorTemp: 8,
  ciRectTemp: 9,
  ciGenTemp: 10,
  frames: 11,
  fatalError: 23,
  errorCode: 24,
  generatorTemp: 25,
  pdgcOverride0a: 26,
  pdgcOverride03: 27,
  rectifierSensorError: 28,
  ptoLatchedOpen: 29,
  generatorSensorError: 30,
  doorInterlock: 31,
  pumpFrozen: 32,
  prechargeTimeout: 33,
  mcuStartupStatus: 34,
  mcuHeartbeat: 35,
  mcuCanTimeout: 36,
  ignorePressureSensors: 37,
  ignoreCabTempSensors: 38,
  wakeInverter: 39,
  startPrecharge: 40,
  restartRequest: 41,
  ignoreErrors: 42,
  closePosContact: 43,
  closeNegContact: 44,
  torqueRefPercent: 45
};

function setupSheet() {
  const sheet = SpreadsheetApp.getActiveSpreadsheet().getActiveSheet();
  sheet.getRange(1, 1, 2, COLUMNS.length).setValues([
    COLUMNS.map(function(column) { return column.owner; }),
    COLUMNS.map(function(column) { return column.header; })
  ]);
  sheet.setFrozenRows(2);
}

function cleanJsonText(text) {
  return String(text || "")
    .replace(/:nan/g, ":null")
    .replace(/:inf/g, ":null")
    .replace(/:-inf/g, ":null")
    .replace(/:NAN/g, ":null")
    .replace(/:INF/g, ":null")
    .replace(/:-INF/g, ":null");
}

function parseParticlePayload(e) {
  let body = "";

  if (e.parameter && e.parameter.data) {
    body = e.parameter.data;
  } else if (e.postData && e.postData.contents) {
    body = e.postData.contents;
  }

  let json = JSON.parse(cleanJsonText(body));

  if (json.data && typeof json.data === "string") {
    json = JSON.parse(cleanJsonText(json.data));
  }

  return json;
}

function value(json, key) {
  return json[key] === undefined || json[key] === null ? "" : json[key];
}

function doPost(e) {
  const sheet = SpreadsheetApp.getActiveSpreadsheet().getActiveSheet();
  const json = parseParticlePayload(e);

  sheet.appendRow(COLUMNS.map(function(column) {
    return column.jsonKey ? value(json, column.jsonKey) : new Date();
  }));

  return ContentService.createTextOutput("OK");
}

function headerIndexes(headers) {
  const indexes = {};
  headers.forEach(function(header, index) {
    if (header) indexes[String(header)] = index;
  });
  return indexes;
}

function getByHeader(row, indexes, column) {
  const index = indexes[column.header];
  if (index !== undefined && row[index] !== undefined && row[index] !== "") {
    return row[index];
  }

  const fallbackIndex = LEGACY_FALLBACK_INDEXES[column.jsonKey || "timestamp"];
  if (fallbackIndex !== undefined && row[fallbackIndex] !== undefined) {
    return row[fallbackIndex];
  }

  return "";
}

function mapSheetRow(row, indexes) {
  const mapped = {};
  COLUMNS.forEach(function(column) {
    const key = column.jsonKey || "timestamp";
    mapped[key] = getByHeader(row, indexes, column);
  });
  return mapped;
}

function doGet(e) {
  const sheet = SpreadsheetApp.getActiveSpreadsheet().getActiveSheet();
  const rows = sheet.getDataRange().getValues();
  const indexes = headerIndexes(rows[1] || []);

  const data = rows.slice(2).filter(function(row) {
    return row[0];
  }).map(function(row) {
    return mapSheetRow(row, indexes);
  });

  const callback = e.parameter.callback || "handleSensorData";

  return ContentService
    .createTextOutput(callback + "(" + JSON.stringify({ rows: data }) + ");")
    .setMimeType(ContentService.MimeType.JAVASCRIPT);
}
