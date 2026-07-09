const DRIVER_KEYS = new Set([
  "timestamp",
  "cabUnitID",
  "trailerUnitID",
  "dccLim",
  "pumpLowRPM",
  "pumpHighRPM",
  "flipFlow",
  "tankerPressure",
  "externalPressure",
  "rectifierTemp",
  "dcCurrent",
  "dcVoltage",
  "motorSpeed",
  "motorTemp",
  "generatorTemp"
]);

const DEFAULT_TRUCK_ID = process.env.TELEMETRY_TRUCK_ID || "default";
const NORMAL_REFRESH_MS = Number(process.env.TELEMETRY_NORMAL_REFRESH_MS || 55_000);
const SUPPORT_REFRESH_MS = Number(process.env.TELEMETRY_SUPPORT_REFRESH_MS || 15_000);
const MAX_CACHE_ROWS = Number(process.env.TELEMETRY_MAX_ROWS || 2000);

const jsonHeaders = {
  "content-type": "application/json; charset=utf-8",
  "cache-control": "no-store"
};

function response(statusCode, body) {
  return {
    statusCode,
    headers: jsonHeaders,
    body: JSON.stringify(body)
  };
}

function requireEnv(name) {
  const value = process.env[name];
  if (!value) throw new Error(`Missing ${name}`);
  return value;
}

function authApiKey() {
  return process.env.SUPABASE_ANON_KEY
    || process.env.SUPABASE_PUBLISHABLE_KEY
    || process.env.SUPABASE_SERVICE_ROLE_KEY;
}

function supabaseRestHeaders() {
  const key = requireEnv("SUPABASE_SERVICE_ROLE_KEY");
  return {
    apikey: key,
    authorization: `Bearer ${key}`,
    "content-type": "application/json"
  };
}

async function verifyUser(accessToken) {
  const baseUrl = requireEnv("SUPABASE_URL");
  const key = authApiKey();
  if (!key) throw new Error("Missing SUPABASE_ANON_KEY or SUPABASE_PUBLISHABLE_KEY");

  const result = await fetch(`${baseUrl}/auth/v1/user`, {
    headers: {
      apikey: key,
      authorization: `Bearer ${accessToken}`
    }
  });

  if (!result.ok) return null;
  return result.json();
}

async function supabaseRest(path, options = {}) {
  const baseUrl = requireEnv("SUPABASE_URL");
  const result = await fetch(`${baseUrl}/rest/v1/${path}`, {
    ...options,
    headers: {
      ...supabaseRestHeaders(),
      ...(options.headers || {})
    }
  });

  if (!result.ok) {
    const message = await result.text();
    throw new Error(`Supabase REST ${result.status}: ${message}`);
  }

  if (result.status === 204) return null;
  return result.json();
}

async function loadUserRole(userId) {
  const rows = await supabaseRest(`profiles?id=eq.${encodeURIComponent(userId)}&select=role,email&limit=1`);
  return rows?.[0]?.role || "driver";
}

async function readTelemetryCache(truckId) {
  const rows = await supabaseRest(
    `truck_telemetry_cache?truck_id=eq.${encodeURIComponent(truckId)}&select=*&limit=1`
  );
  return rows?.[0] || null;
}

async function writeTelemetryCache(truckId, rows, refreshError = null) {
  const latest = rows[rows.length - 1] || null;
  const cachedRows = rows.slice(-MAX_CACHE_ROWS);
  const sourceUpdatedAt = latest?.timestamp || null;
  const payload = {
    truck_id: truckId,
    latest,
    rows: cachedRows,
    row_count: rows.length,
    source_updated_at: sourceUpdatedAt,
    fetched_at: new Date().toISOString(),
    source: "google_sheets",
    refresh_error: refreshError
  };

  const result = await supabaseRest("truck_telemetry_cache?on_conflict=truck_id", {
    method: "POST",
    headers: {
      prefer: "resolution=merge-duplicates,return=representation"
    },
    body: JSON.stringify(payload)
  });

  return result?.[0] || payload;
}

function parseSheetTimestamp(value) {
  if (!value && value !== 0) return null;
  if (typeof value === "number") {
    if (value > 1_000_000_000_000) return new Date(value).toISOString();
    if (value > 20000) return new Date(Math.round((value - 25569) * 86400000)).toISOString();
  }

  const parsed = new Date(value);
  return Number.isFinite(parsed.getTime()) ? parsed.toISOString() : null;
}

function firstValue(row, keys) {
  for (const key of keys) {
    const value = row[key];
    if (value !== undefined && value !== null && value !== "") return value;
  }
  return undefined;
}

function numberValue(row, keys, fallback = null) {
  const value = firstValue(row, keys);
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

function stringValue(row, keys, fallback = "") {
  const value = firstValue(row, keys);
  return value === undefined ? fallback : String(value);
}

function normalizeSheetRow(row) {
  return {
    timestamp: parseSheetTimestamp(firstValue(row, ["timestamp", "Timestamp"])),
    cabUnitID: stringValue(row, ["cabUnitID", "CAB_UnitID", "unitID", "UnitID"]),
    trailerUnitID: stringValue(row, ["trailerUnitID", "TRAILER_UnitID"], "0"),
    dccLim: numberValue(row, ["dccLim", "DischargeChargeCurrLim"]),
    pumpLowRPM: numberValue(row, ["pumpLowRPM", "PumpLowSpeedRPM"]),
    pumpHighRPM: numberValue(row, ["pumpRPM", "pumpHighRPM", "PumpHighSpeedRPM"]),
    flipFlow: numberValue(row, ["flipFlow", "FlipFlowDirection"]),
    tankerPressure: numberValue(row, ["tankerPressure", "CI_TankerPressure"]),
    externalPressure: numberValue(row, ["externalPressure", "CI_ExternalPressure"]),
    rectifierTemp: numberValue(row, ["rectTemp", "rectifierTemp", "CAN_RectifierTemp"]),
    dcCurrent: numberValue(row, ["dcCurrent", "CAN_McuDcCurrent"]),
    dcVoltage: numberValue(row, ["dcVoltage", "CAN_McuDcVoltage"]),
    motorSpeed: numberValue(row, ["motorSpeed", "CAN_McuMotorSpeed"]),
    motorTemp: numberValue(row, ["motorTemp", "CAN_McuMotorTemp"]),
    generatorTemp: numberValue(row, ["generatorTemp", "CAN_GeneratorTemp", "ciGenTemp", "CI_GeneratorTempDegC"]),
    frames: numberValue(row, ["frames"]),
    ccpReq: numberValue(row, ["ccpReq"]),
    ccpResp: numberValue(row, ["ccpResp"]),
    matched: numberValue(row, ["matched"]),
    ccpSuccessPct: numberValue(row, ["successPct", "ccpSuccessPct"]),
    activeTimeout: numberValue(row, ["activeTimeout"]),
    txSkip: numberValue(row, ["txSkip"])
  };
}

function parseJsonp(text) {
  const trimmed = String(text || "").trim();
  if (trimmed.startsWith("{")) return JSON.parse(trimmed);

  const start = trimmed.indexOf("(");
  const end = trimmed.lastIndexOf(")");
  if (start === -1 || end === -1 || end <= start) {
    throw new Error("Google Apps Script returned an unexpected format");
  }

  return JSON.parse(trimmed.slice(start + 1, end));
}

async function fetchGoogleSheetRows() {
  const apiUrl = new URL(requireEnv("GOOGLE_SHEETS_API_URL"));
  apiUrl.searchParams.set("callback", "serverTelemetry");
  apiUrl.searchParams.set("cacheBust", String(Date.now()));

  const result = await fetch(apiUrl);
  if (!result.ok) {
    throw new Error(`Google Apps Script ${result.status}`);
  }

  const json = parseJsonp(await result.text());
  return (json.rows || [])
    .filter(row => row && row.timestamp)
    .map(normalizeSheetRow)
    .filter(row => row.timestamp);
}

function cacheIsFresh(cache, maxAgeMs) {
  if (!cache?.fetched_at) return false;
  const fetchedAt = new Date(cache.fetched_at).getTime();
  return Number.isFinite(fetchedAt) && Date.now() - fetchedAt < maxAgeMs;
}

function filterRowForRole(row, role) {
  if (role === "tech" || role === "admin") return row;
  return Object.fromEntries(
    Object.entries(row || {}).filter(([key]) => DRIVER_KEYS.has(key))
  );
}

exports.handler = async event => {
  if (event.httpMethod === "OPTIONS") return response(204, {});
  if (event.httpMethod !== "GET") return response(405, { error: "Method not allowed" });

  try {
    const authHeader = event.headers.authorization || event.headers.Authorization || "";
    const accessToken = authHeader.replace(/^Bearer\s+/i, "");
    if (!accessToken) return response(401, { error: "Missing bearer token" });

    const user = await verifyUser(accessToken);
    if (!user?.id) return response(401, { error: "Invalid or expired login" });

    const role = await loadUserRole(user.id);
    const params = event.queryStringParameters || {};
    const truckId = params.truckId || DEFAULT_TRUCK_ID;
    const forceRefresh = params.refresh === "1";
    const supportMode = params.support === "1";
    const maxAgeMs = supportMode ? SUPPORT_REFRESH_MS : NORMAL_REFRESH_MS;

    let cache = await readTelemetryCache(truckId);
    let source = "cache";
    let refreshError = null;

    if (forceRefresh || !cache || !cacheIsFresh(cache, maxAgeMs)) {
      try {
        const rows = await fetchGoogleSheetRows();
        if (rows.length) {
          cache = await writeTelemetryCache(truckId, rows);
          source = "live";
        } else {
          refreshError = "Google Sheets returned no telemetry rows";
        }
      } catch (error) {
        refreshError = error.message || "Telemetry refresh failed";
      }
    }

    if (!cache) {
      return response(503, {
        error: refreshError || "No telemetry cache is available yet",
        role
      });
    }

    const rows = Array.isArray(cache.rows) ? cache.rows : [];
    const latest = cache.latest || rows[rows.length - 1] || null;

    return response(200, {
      role,
      truckId,
      source,
      supportMode,
      refreshError,
      fetchedAt: cache.fetched_at,
      sourceUpdatedAt: cache.source_updated_at,
      rowCount: cache.row_count || rows.length,
      latest: filterRowForRole(latest, role),
      rows: rows.map(row => filterRowForRole(row, role))
    });
  } catch (error) {
    return response(500, {
      error: error.message || "Telemetry function failed"
    });
  }
};
