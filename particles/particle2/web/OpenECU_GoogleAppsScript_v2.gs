const HEADERS = [
  [
    "BOTH",
    "CAB_ECU",
    "TRAILER_ECU",
    "TRAILER_ECU",
    "TRAILER_ECU",
    "TRAILER_ECU",
    "TRAILER_ECU",
    "TRAILER_ECU",
    "TRAILER_ECU",
    "TRAILER_ECU",
    "TRAILER_ECU",
    "TRAILER_ECU",
    "TRAILER_ECU",
    "TRAILER_ECU",
    "TRAILER_ECU",
    "CAB_ECU",
    "CAB_ECU",
    "BOTH",
    "BOTH",
    "BOTH",
    "BOTH",
    "BOTH",
    "BOTH",
    "BOTH"
  ],
  [
    "timestamp",
    "CAB_UnitID",
    "TRAILER_UnitID",
    "DischargeChargeCurrLim",
    "PumpLowSpeedRPM",
    "PumpHighSpeedRPM",
    "MotorToPumpGearRatio",
    "FlipFlowDirection",
    "CI_TankerPressure",
    "CI_ExternalPressure",
    "CAN_RectifierTemp",
    "CAN_McuDcCurrent",
    "CAN_McuDcVoltage",
    "CAN_McuMotorSpeed",
    "CAN_McuMotorTemp",
    "CI_RectifierTempDegC",
    "CI_GeneratorTempDegC",
    "frames",
    "ccpReq",
    "ccpResp",
    "matched",
    "successPct",
    "activeTimeout",
    "txSkip"
  ]
];

function setupSheet() {
  const sheet = SpreadsheetApp.getActiveSpreadsheet().getActiveSheet();
  sheet.getRange(1, 1, 2, HEADERS[0].length).setValues(HEADERS);
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

  // Particle webhooks often wrap the event payload as { data: "{...}" }.
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

  sheet.appendRow([
    new Date(),
    value(json, "cabUnitID"),
    value(json, "trailerUnitID"),
    value(json, "dccLim"),
    value(json, "pumpLowRPM"),
    value(json, "pumpRPM"),
    value(json, "gearRatio"),
    value(json, "flipFlow"),
    value(json, "tankerPressure"),
    value(json, "externalPressure"),
    value(json, "rectTemp"),
    value(json, "dcCurrent"),
    value(json, "dcVoltage"),
    value(json, "motorSpeed"),
    value(json, "motorTemp"),
    value(json, "ciRectTemp"),
    value(json, "ciGenTemp"),
    value(json, "frames"),
    value(json, "ccpReq"),
    value(json, "ccpResp"),
    value(json, "matched"),
    value(json, "successPct"),
    value(json, "activeTimeout"),
    value(json, "txSkip")
  ]);

  return ContentService.createTextOutput("OK");
}

function getByHeader(row, indexes, headerName, fallbackIndex) {
  const index = indexes[headerName];
  if (index !== undefined && row[index] !== undefined && row[index] !== "") {
    return row[index];
  }

  if (fallbackIndex !== undefined && row[fallbackIndex] !== undefined) {
    return row[fallbackIndex];
  }

  return "";
}

function doGet(e) {
  const sheet = SpreadsheetApp.getActiveSpreadsheet().getActiveSheet();
  const rows = sheet.getDataRange().getValues();
  const headers = rows[1] || [];
  const indexes = {};

  headers.forEach(function(header, index) {
    indexes[header] = index;
  });

  const data = rows.slice(2).filter(function(row) {
    return row[0];
  }).map(function(row) {
    return {
      timestamp: getByHeader(row, indexes, "timestamp", 0),
      cabUnitID: getByHeader(row, indexes, "CAB_UnitID"),
      trailerUnitID: getByHeader(row, indexes, "TRAILER_UnitID"),
      dccLim: getByHeader(row, indexes, "DischargeChargeCurrLim", 1),
      pumpLowRPM: getByHeader(row, indexes, "PumpLowSpeedRPM"),
      pumpRPM: getByHeader(row, indexes, "PumpHighSpeedRPM", 2),
      gearRatio: getByHeader(row, indexes, "MotorToPumpGearRatio", 3),
      flipFlow: getByHeader(row, indexes, "FlipFlowDirection"),
      tankerPressure: getByHeader(row, indexes, "CI_TankerPressure"),
      externalPressure: getByHeader(row, indexes, "CI_ExternalPressure"),
      rectTemp: getByHeader(row, indexes, "CAN_RectifierTemp", 4),
      dcCurrent: getByHeader(row, indexes, "CAN_McuDcCurrent", 5),
      dcVoltage: getByHeader(row, indexes, "CAN_McuDcVoltage", 6),
      motorSpeed: getByHeader(row, indexes, "CAN_McuMotorSpeed", 7),
      motorTemp: getByHeader(row, indexes, "CAN_McuMotorTemp", 8),
      ciRectTemp: getByHeader(row, indexes, "CI_RectifierTempDegC", 9),
      ciGenTemp: getByHeader(row, indexes, "CI_GeneratorTempDegC", 10),
      frames: getByHeader(row, indexes, "frames", 11),
      ccpReq: getByHeader(row, indexes, "ccpReq"),
      ccpResp: getByHeader(row, indexes, "ccpResp"),
      matched: getByHeader(row, indexes, "matched"),
      successPct: getByHeader(row, indexes, "successPct"),
      activeTimeout: getByHeader(row, indexes, "activeTimeout"),
      txSkip: getByHeader(row, indexes, "txSkip")
    };
  });

  const callback = e.parameter.callback || "handleSensorData";

  return ContentService
    .createTextOutput(callback + "(" + JSON.stringify({ rows: data }) + ");")
    .setMimeType(ContentService.MimeType.JAVASCRIPT);
}
