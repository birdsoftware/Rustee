// =====================================================
// OpenECU CCP Symbol Verification Tool
// Particle Argon + MCP2515/HW-184 + TXS0108E
//
// Purpose:
// - Keep all 185 extracted OpenECU symbols in firmware.
// - Toggle ONE group of about 15 symbols at a time.
// - With OpenECU Calibrator running, this listens for CCP memory-read
//   request/response pairs and prints only the active group.
// - This does NOT transmit CCP requests yet. It is passive verification.
//
// Wiring:
// HW-184 VCC -> external 5V
// HW-184 GND -> common GND
// CS -> D5, SO -> D11/MISO, SI -> D12/MOSI, SCK -> D13/SCK, INT -> D2
// SPI lines through TXS0108E. UA/OE -> Argon 3V3, UB -> external 5V.
// CAN bus: 250 kbps, MCP2515 crystal: 8 MHz
// =====================================================

#include "Particle.h"
#include <mcp_can.h>
#include <SPI.h>
#include <math.h>

SYSTEM_MODE(AUTOMATIC);

#define CAN_CS_PIN  D5
#define CAN_INT_PIN D2

MCP_CAN CAN0(CAN_CS_PIN);

// ----------------------
// GROUP TOGGLES
// Set exactly ONE group true at a time.
// GROUP_01 = symbols 1-15, GROUP_02 = symbols 16-30, etc.
// ----------------------
const bool GROUP_01 = true;
const bool GROUP_02 = false;
const bool GROUP_03 = false;
const bool GROUP_04 = false;
const bool GROUP_05 = false;
const bool GROUP_06 = false;
const bool GROUP_07 = false;
const bool GROUP_08 = false;
const bool GROUP_09 = false;
const bool GROUP_10 = false;
const bool GROUP_11 = false;
const bool GROUP_12 = false;
const bool GROUP_13 = false;

enum SymbolType {
    SYM_FLOAT,
    SYM_U8,
    SYM_U16,
    SYM_S16,
    SYM_U32,
    SYM_S32,
    SYM_BYTES
};

struct SymbolDef {
    uint32_t addr;
    const char *name;
    SymbolType type;
    uint8_t sizeBytes;
};

struct ActiveRange {
    int start;
    int endExclusive;
};

const SymbolDef symbols[] = {
    {0x00040400, "speedtoggledebouncems", SYM_U16, 2},
    {0x00040404, "UnitID", SYM_U32, 4},
    {0x00040408, "TankerCriticalVacuum", SYM_FLOAT, 4},
    {0x0004040C, "TankerCriticalPressure", SYM_FLOAT, 4},
    {0x00040410, "TankFullDebouncems", SYM_U16, 2},
    {0x00040414, "SwitchSettingPeriodicTransRate", SYM_FLOAT, 4},
    {0x00040418, "SwitchDisableMsgDelaymsec", SYM_U16, 2},
    {0x0004041C, "StopReqSlewRate", SYM_FLOAT, 4},
    {0x00040420, "StartupCANTimeout", SYM_FLOAT, 4},
    {0x00040424, "StartingTorqueLimitPercent", SYM_S16, 2},
    {0x00040428, "SpeedReqSlewRate", SYM_FLOAT, 4},
    {0x0004042C, "SoftwareDisplayLightsOffDelaymsec", SYM_FLOAT, 4},
    {0x00040430, "RectifierCritTempDegC", SYM_FLOAT, 4},
    {0x00040434, "PumpPreStartLightDelaymsec", SYM_FLOAT, 4},
    {0x00040438, "PumpPostStartLightDelaymsec", SYM_FLOAT, 4},
    {0x0004043C, "PumpMotionSpeedThreshRPM", SYM_S16, 2},
    {0x00040440, "PumpFrozenDelaymsec", SYM_FLOAT, 4},
    {0x00040444, "PressureSensorDebounceMsec", SYM_FLOAT, 4},
    {0x00040448, "PrechargeTimeout_s", SYM_FLOAT, 4},
    {0x0004044C, "PSensorDisconnectedVThresh", SYM_FLOAT, 4},
    {0x00040450, "OperationalTorqueLimitPercent", SYM_S16, 2},
    {0x00040452, "MotorTempCoolingOnThreshold", SYM_U16, 2},
    {0x00040454, "MotorTempCoolingOffThreshold", SYM_U16, 2},
    {0x00040458, "MotorStoppedSpeedThresholdRpm", SYM_FLOAT, 4},
    {0x0004045C, "MotorPhaseDisconnectedCurrLimRms", SYM_FLOAT, 4},
    {0x00040460, "MotorFanPWMFrequency", SYM_FLOAT, 4},
    {0x00040464, "MotorFanDutyCycleRampRate", SYM_FLOAT, 4},
    {0x00040468, "MotorFanDutyCyclePercent", SYM_FLOAT, 4},
    {0x0004046C, "KeepAliveTimeoutmsec", SYM_FLOAT, 4},
    {0x00040470, "IgnorePressureSensorNumLightCycles", SYM_FLOAT, 4},
    {0x00040474, "IgnitionLatchedErrorFlashDelayMsec", SYM_U16, 2},
    {0x00040478, "GeneratorCritTempDegC", SYM_FLOAT, 4},
    {0x0004047C, "FrozenPumpTimeoutmsec", SYM_S16, 2},
    {0x00040480, "ExternalCriticalPressure", SYM_FLOAT, 4},
    {0x00040484, "EnclosureSensorVoltThreshold", SYM_FLOAT, 4},
    {0x00040488, "EnclosureCritTempDegC", SYM_FLOAT, 4},
    {0x0004048C, "DischargeChargeCurrLim", SYM_S16, 2},
    {0x00040490, "DirUnselectedWarnTogglemsec", SYM_FLOAT, 4},
    {0x00040494, "DirUnselectedWarnDurationmsec", SYM_FLOAT, 4},
    {0x00040498, "ContactCloseToPumpEnDelay_msec", SYM_FLOAT, 4},
    {0x0004049C, "ChargeCurrLim", SYM_S16, 2},
    {0x0004049E, "CabCoolingRequestDebounceMsec", SYM_S16, 2},
    {0x000404A0, "ActiveCANTimeout", SYM_FLOAT, 4},
    {0x000404A4, "TorqueModeTorqueToggleIntervalPercent", SYM_FLOAT, 4},
    {0x000404A8, "TorqueModeStartTorqueUnloadPercent", SYM_FLOAT, 4},
    {0x000404AC, "TorqueModeStartTorqueLoadPercent", SYM_FLOAT, 4},
    {0x000404B0, "TorqueModeRateTransition", SYM_FLOAT, 4},
    {0x000404B4, "TorqueModeMinTorquePercent", SYM_FLOAT, 4},
    {0x000404B8, "TorqueModeMaxTorquePercent", SYM_FLOAT, 4},
    {0x000404BC, "TorqueModeDroopRatePercent", SYM_FLOAT, 4},
    {0x000404C0, "PumpStartSpeedUnloadRPM", SYM_FLOAT, 4},
    {0x000404C4, "PumpStartSpeedLoadRPM", SYM_FLOAT, 4},
    {0x000404C8, "PumpSpeedToggleInterval", SYM_FLOAT, 4},
    {0x000404CC, "PumpLowSpeedRPM", SYM_FLOAT, 4},
    {0x000404D0, "PumpHighSpeedRPM", SYM_FLOAT, 4},
    {0x000404D4, "PressureSensorModelTankerSide", SYM_U8, 4},
    {0x000404D8, "PressureSensorModelExternalSide", SYM_U8, 4},
    {0x000404DC, "MotorToPumpGearRatio", SYM_FLOAT, 4},
    {0x000404E0, "MotorInvPairingErrorDelayS", SYM_FLOAT, 4},
    {0x000404E4, "MaskEnclosureTempSensorFaults", SYM_U8, 4},
    {0x000404E8, "IgnoreErrorsForMotorInvPair", SYM_U8, 4},
    {0x000404EC, "FlipFlowDirection", SYM_U8, 4},
    {0x000404F0, "BackDriveDetectedSpeedThresh", SYM_FLOAT, 4},
    {0x000404F4, "BackDriveDetectedDebounceMsec", SYM_FLOAT, 4},
    {0x000404F8, "pdgc_override_service_0a", SYM_U32, 4},
    {0x000404FC, "pdgc_override_service_07", SYM_U32, 4},
    {0x00040500, "pdgc_override_service_03", SYM_U32, 4},
    {0x00040504, "pdgc_func_can_rx_id", SYM_U32, 4},
    {0x00040508, "pdgc_emissions_report_min_sev", SYM_U32, 4},
    {0x0004050C, "pdgc_can_bus_id", SYM_U32, 1},
    {0x0004050D, "pdgc_can_rx_id_extd", SYM_U8, 1},
    {0x00040510, "pdgc_can_rx_id", SYM_U32, 4},
    {0x00040514, "pdgc_can_tx_id_extd", SYM_U8, 1},
    {0x00040518, "pdgc_can_tx_id", SYM_U32, 4},
    {0x0004051C, "svcc_ecu_reprog_seedkey_cal", SYM_U32, 4},
    {0x00040520, "svcc_ecu_config_seedkey_cal", SYM_U32, 4},
    {0x00040524, "pj1939c_node_addr_0", SYM_BYTES, 2},
    {0x00040526, "mplc_tcr1_scalar", SYM_U16, 2},
    {0x00040528, "pioc_time_dmin_sample_default_us", SYM_FLOAT, 4},
    {0x0004052C, "pioc_rate_spot_max_hz", SYM_FLOAT, 4},
    {0x00040530, "pioc_rate_pot_max_hz", SYM_FLOAT, 4},
    {0x00040534, "pscl_build_time_str", SYM_BYTES, 32},
    {0x00040554, "pscl_target_str", SYM_BYTES, 8},
    {0x0004055C, "pscl_copyright_str", SYM_BYTES, 32},
    {0x0004057C, "pscl_ver_str", SYM_BYTES, 1073488824},
    {0x40002934, "Main_State_Machinestate", SYM_U8, 4},
    {0x40002934, "__diab_bss_start", SYM_U8, 4},
    {0x40002934, "__diab_data_rw_end", SYM_U8, 4},
    {0x40002938, "MFSM_WakeInverter", SYM_U8, 4},
    {0x4000293C, "MFSM_TorqueMode", SYM_U8, 4},
    {0x40002940, "MFSM_StartPrecharge", SYM_U8, 4},
    {0x40002944, "MFSM_SpeedEnable", SYM_U8, 4},
    {0x40002948, "MFSM_SolidRed", SYM_U8, 4},
    {0x4000294C, "MFSM_ShutdownInitiated", SYM_U8, 4},
    {0x40002950, "MFSM_RestartRequest", SYM_U8, 4},
    {0x40002954, "MFSM_PumpEnabled", SYM_U8, 4},
    {0x40002958, "MFSM_KeepCoolingAlive", SYM_U8, 4},
    {0x4000295C, "MFSM_KeepAlive", SYM_U8, 4},
    {0x40002960, "MFSM_InverterStartup", SYM_U8, 4},
    {0x40002964, "MFSM_IgnoreErrors", SYM_U8, 4},
    {0x40002968, "MFSM_E_Stop", SYM_U8, 4},
    {0x4000296C, "MFSM_CommsAllowed", SYM_U8, 4},
    {0x40002970, "MFSM_ClosePosContact", SYM_U8, 4},
    {0x40002974, "MFSM_CloseNegContact", SYM_U8, 4},
    {0x40002978, "MFSM_BackDriveDetected", SYM_U8, 4},
    {0x4000297C, "Log_WrongFlowDirectionError", SYM_U8, 4},
    {0x40002980, "Log_WakeMotorFan", SYM_U8, 4},
    {0x40002984, "Log_TotalOperationalTime", SYM_U32, 4},
    {0x40002988, "Log_TotalOnTime", SYM_U32, 4},
    {0x4000298C, "Log_TorqueRefPercent", SYM_FLOAT, 4},
    {0x40002990, "Log_TorqueLimPercent", SYM_S16, 2},
    {0x40002994, "Log_SwitchCanTimeout", SYM_U8, 4},
    {0x40002998, "Log_SpeedRequest", SYM_FLOAT, 4},
    {0x4000299C, "Log_SpeedRef", SYM_FLOAT, 4},
    {0x400029A0, "Log_RedPulses", SYM_U8, 1},
    {0x400029A4, "Log_RedLightSig", SYM_U8, 4},
    {0x400029A8, "Log_PumpOk", SYM_U8, 4},
    {0x400029AC, "Log_PumpFrozen", SYM_U8, 4},
    {0x400029B0, "Log_PumpConfigModeEn", SYM_U8, 4},
    {0x400029B4, "Log_PrechargeTimeout", SYM_U8, 4},
    {0x400029B8, "Log_OperationalTime", SYM_U32, 4},
    {0x400029BC, "Log_OnTime", SYM_U32, 4},
    {0x400029C0, "Log_McuStartupStatus", SYM_FLOAT, 4},
    {0x400029C4, "Log_McuHeartbeat", SYM_U8, 1},
    {0x400029C8, "Log_McuCanTimeout", SYM_U8, 4},
    {0x400029CC, "Log_LowestTorqueSelected", SYM_U8, 4},
    {0x400029D0, "Log_LowestSpeedSelected", SYM_U8, 4},
    {0x400029D4, "Log_LightControlStateMachine", SYM_U8, 4},
    {0x400029D8, "Log_InverterResetCmd", SYM_U8, 4},
    {0x400029DC, "Log_InverterAwake", SYM_U8, 4},
    {0x400029E0, "Log_IgnorePressureSensors", SYM_U8, 4},
    {0x400029E4, "Log_IgnoreCabEnclosureTempSensors", SYM_U8, 4},
    {0x400029E8, "Log_HighestTorqueSelected", SYM_U8, 4},
    {0x400029EC, "Log_HighestSpeedSelected", SYM_U8, 4},
    {0x400029F0, "Log_GreenPulses", SYM_U8, 1},
    {0x400029F4, "Log_GreenLightSig", SYM_U8, 4},
    {0x400029F8, "Log_FlashAllLights", SYM_U8, 4},
    {0x400029FC, "Log_FatalError", SYM_U8, 4},
    {0x40002A00, "Log_ErrorCode", SYM_U8, 4},
    {0x40002A04, "Log_DisplayFirmware", SYM_U8, 4},
    {0x40002A08, "Log_CabCanTimeout", SYM_U8, 4},
    {0x40002A0C, "Log_CANTimeout", SYM_U8, 4},
    {0x40002A10, "Log_CANCommOk", SYM_U8, 4},
    {0x40002A14, "DI_TankLevelPumpEn", SYM_U8, 4},
    {0x40002A18, "CI_TankerPressure", SYM_FLOAT, 4},
    {0x40002A1C, "CI_ExternalPressure", SYM_FLOAT, 4},
    {0x40002A20, "CI_EnclosureTemp", SYM_FLOAT, 4},
    {0x40002A24, "CAN_Welded", SYM_U8, 4},
    {0x40002A28, "CAN_SwitchStartButtton", SYM_U8, 4},
    {0x40002A2C, "CAN_SwitchHighspdButton", SYM_U8, 4},
    {0x40002A30, "CAN_SwitchFwdButton", SYM_U8, 4},
    {0x40002A34, "CAN_RectifierTemp", SYM_FLOAT, 4},
    {0x40002A38, "CAN_RectifierSensorError", SYM_U8, 4},
    {0x40002A3C, "CAN_RawStopButton", SYM_U8, 4},
    {0x40002A40, "CAN_RawStartButton", SYM_U8, 4},
    {0x40002A44, "CAN_RawRevButton", SYM_U8, 4},
    {0x40002A48, "CAN_RawLowButton", SYM_U8, 4},
    {0x40002A4C, "CAN_RawHighButton", SYM_U8, 4},
    {0x40002A50, "CAN_RawFwdButton", SYM_U8, 4},
    {0x40002A54, "CAN_PtoLatchedOpen", SYM_U8, 4},
    {0x40002A58, "CAN_McuMotorTemp", SYM_FLOAT, 4},
    {0x40002A5C, "CAN_McuMotorSpeed", SYM_FLOAT, 4},
    {0x40002A60, "CAN_McuMainsState", SYM_U32, 4},
    {0x40002A64, "CAN_McuFaultCode", SYM_U32, 4},
    {0x40002A68, "CAN_McuDcVoltage", SYM_FLOAT, 4},
    {0x40002A6C, "CAN_McuDcCurrent", SYM_FLOAT, 4},
    {0x40002A70, "CAN_ImdOk", SYM_FLOAT, 4},
    {0x40002A74, "CAN_Ignition", SYM_U8, 4},
    {0x40002A78, "CAN_GeneratorTemp", SYM_FLOAT, 4},
    {0x40002A7C, "CAN_GeneratorSensorError", SYM_U8, 4},
    {0x40002A80, "CAN_DoorInterlock", SYM_U8, 4},
    {0x40002A84, "CAN_CabSideEn", SYM_U8, 4},
    {0x40002A88, "CAN_ActiveFault8", SYM_U8, 1},
    {0x40002A89, "CAN_ActiveFault7", SYM_U8, 1},
    {0x40002A8A, "CAN_ActiveFault6", SYM_U8, 1},
    {0x40002A8B, "CAN_ActiveFault5", SYM_U8, 1},
    {0x40002A8C, "CAN_ActiveFault4", SYM_U8, 1},
    {0x40002A8D, "CAN_ActiveFault3", SYM_U8, 1},
    {0x40002A8E, "CAN_ActiveFault2", SYM_U8, 1},
    {0x40002A8F, "CAN_ActiveFault1", SYM_U8, 1},
    {0x40002A90, "AI_VrefDiodePinB4", SYM_FLOAT, 4},
    {0x40002A94, "AI_TankerPressureV", SYM_FLOAT, 4},
    {0x40002A98, "AI_ExternalPressureV", SYM_FLOAT, 4},
    {0x40002A9C, "AI_EnclosureTempV", SYM_FLOAT, 4},
    {0x40002AA0, "rtNaNF", SYM_FLOAT, 4},
};

const int SYMBOL_COUNT = sizeof(symbols) / sizeof(symbols[0]);
const int MAX_ACTIVE = 15;

uint32_t pendingAddr[256];

unsigned long frameCount = 0;
unsigned long ccpRequestCount = 0;
unsigned long ccpResponseCount = 0;
unsigned long matchedResponseCount = 0;
unsigned long lastHeartbeatMs = 0;
unsigned long lastPrintMs = 0;

bool slotSeen[MAX_ACTIVE];
uint32_t slotLastRaw[MAX_ACTIVE];
float slotLastFloat[MAX_ACTIVE];
unsigned long slotLastSeenMs[MAX_ACTIVE];

ActiveRange activeRange() {
    if (GROUP_01) return ActiveRange(0, 15);
    if (GROUP_02) return ActiveRange(15, 30);
    if (GROUP_03) return ActiveRange(30, 45);
    if (GROUP_04) return ActiveRange(45, 60);
    if (GROUP_05) return ActiveRange(60, 75);
    if (GROUP_06) return ActiveRange(75, 90);
    if (GROUP_07) return ActiveRange(90, 105);
    if (GROUP_08) return ActiveRange(105, 120);
    if (GROUP_09) return ActiveRange(120, 135);
    if (GROUP_10) return ActiveRange(135, 150);
    if (GROUP_11) return ActiveRange(150, 165);
    if (GROUP_12) return ActiveRange(165, 180);
    if (GROUP_13) return ActiveRange(180, 185);
    return ActiveRange(0, 15);
}

uint32_t readU32BE(const byte *b) {
    return ((uint32_t)b[0] << 24) |
           ((uint32_t)b[1] << 16) |
           ((uint32_t)b[2] << 8)  |
           ((uint32_t)b[3]);
}

int32_t readS32BE(const byte *b) {
    return (int32_t)readU32BE(b);
}

uint16_t readU16BE(const byte *b) {
    return ((uint16_t)b[0] << 8) | b[1];
}

int16_t readS16BE(const byte *b) {
    return (int16_t)readU16BE(b);
}

float readFloatBE(const byte *b) {
    union {
        uint32_t u;
        float f;
    } val;
    val.u = readU32BE(b);
    return val.f;
}

uint32_t readAddrFromRequest(const byte *buf) {
    return ((uint32_t)buf[4] << 24) |
           ((uint32_t)buf[5] << 16) |
           ((uint32_t)buf[6] << 8)  |
           ((uint32_t)buf[7]);
}

void printHexByte(byte v) {
    if (v < 0x10) Serial.print("0");
    Serial.print(v, HEX);
}

int activeIndexForAddress(uint32_t addr) {
    ActiveRange r = activeRange();
    for (int i = r.start; i < r.endExclusive; i++) {
        if (symbols[i].addr == addr) {
            return i - r.start;
        }
    }
    return -1;
}

void decodeAndStore(int slot, const SymbolDef &sym, const byte *data) {
    slotSeen[slot] = true;
    slotLastSeenMs[slot] = millis();

    switch (sym.type) {
        case SYM_FLOAT:
            slotLastFloat[slot] = readFloatBE(data);
            slotLastRaw[slot] = readU32BE(data);
            break;

        case SYM_U8:
        case SYM_BYTES:
            slotLastRaw[slot] = data[0];
            slotLastFloat[slot] = NAN;
            break;

        case SYM_U16:
            slotLastRaw[slot] = readU16BE(data);
            slotLastFloat[slot] = NAN;
            break;

        case SYM_S16:
            slotLastRaw[slot] = (uint32_t)(int32_t)readS16BE(data);
            slotLastFloat[slot] = NAN;
            break;

        case SYM_U32:
            slotLastRaw[slot] = readU32BE(data);
            slotLastFloat[slot] = NAN;
            break;

        case SYM_S32:
            slotLastRaw[slot] = (uint32_t)readS32BE(data);
            slotLastFloat[slot] = NAN;
            break;
    }
}

void printDecodedValue(const SymbolDef &sym, int slot) {
    if (!slotSeen[slot]) {
        Serial.print("--");
        return;
    }

    switch (sym.type) {
        case SYM_FLOAT:
            if (isnan(slotLastFloat[slot])) Serial.print("nan");
            else if (isinf(slotLastFloat[slot])) Serial.print("inf");
            else Serial.print(slotLastFloat[slot], 6);
            break;

        case SYM_S16:
            Serial.print((int16_t)slotLastRaw[slot]);
            break;

        case SYM_S32:
            Serial.print((int32_t)slotLastRaw[slot]);
            break;

        case SYM_U8:
        case SYM_U16:
        case SYM_U32:
        case SYM_BYTES:
            Serial.print(slotLastRaw[slot]);
            break;
    }
}

void printActiveGroupHeader() {
    ActiveRange r = activeRange();
    int groupNum = (r.start / 15) + 1;

    Serial.println();
    Serial.println("============================================================");
    Serial.printlnf("OpenECU CCP Verification - GROUP_%02d symbols %d-%d of %d",
                    groupNum, r.start + 1, r.endExclusive, SYMBOL_COUNT);
    Serial.println("Compare these against OpenECU Calibrator values.");
    Serial.println("If a value is '--', Calibrator has not requested that address yet.");
    Serial.println("============================================================");

    for (int i = r.start; i < r.endExclusive; i++) {
        Serial.printlnf("%2d) 0x%08lX  %-44s",
                        i - r.start + 1,
                        symbols[i].addr,
                        symbols[i].name);
    }
    Serial.println("============================================================");
}

void printActiveGroupTable() {
    ActiveRange r = activeRange();

    Serial.println();
    Serial.println("===== ACTIVE SYMBOL VALUES =====");
    Serial.printlnf("frames=%lu ccpReq=%lu ccpResp=%lu matched=%lu WiFi=%d Cloud=%d",
                    frameCount,
                    ccpRequestCount,
                    ccpResponseCount,
                    matchedResponseCount,
                    WiFi.ready(),
                    Particle.connected());

    for (int i = r.start; i < r.endExclusive; i++) {
        int slot = i - r.start;
        const SymbolDef &sym = symbols[i];

        Serial.print("0x");
        Serial.print(sym.addr, HEX);
        Serial.print("  ");
        Serial.print(sym.name);
        Serial.print(": ");

        printDecodedValue(sym, slot);

        Serial.print("   raw=0x");
        if (slotSeen[slot]) {
            Serial.print(slotLastRaw[slot], HEX);
            Serial.print("   ageMs=");
            Serial.print(millis() - slotLastSeenMs[slot]);
        } else {
            Serial.print("--");
        }

        Serial.println();
    }

    Serial.println("===============================");
}

void setup() {
    Serial.begin(115200);
    delay(3000);

    Serial.println();
    Serial.println("BOOT");
    Serial.println("Starting OpenECU CCP symbol verifier...");

    pinMode(CAN_INT_PIN, INPUT_PULLUP);

    SPI.begin();
    SPI.setClockSpeed(1, MHZ);

    while (CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) != CAN_OK) {
        Serial.println("CAN INIT FAIL");
        delay(1000);
    }

    CAN0.setMode(MCP_LISTENONLY);

    for (int i = 0; i < 256; i++) pendingAddr[i] = 0;
    for (int i = 0; i < MAX_ACTIVE; i++) {
        slotSeen[i] = false;
        slotLastRaw[i] = 0;
        slotLastFloat[i] = NAN;
        slotLastSeenMs[i] = 0;
    }

    Serial.println("CAN init OK");
    Serial.println("Passive CCP verification mode. Start OpenECU Calibrator polling.");
    printActiveGroupHeader();
}

void loop() {
    unsigned long now = millis();

    if (CAN0.checkReceive() == CAN_MSGAVAIL) {
        unsigned long rxId;
        byte len = 0;
        byte buf[8];

        if (CAN0.readMsgBuf(&rxId, &len, buf) != CAN_OK) return;

        frameCount++;
        rxId &= 0x1FFFFFFF;

        // OpenECU CCP request:
        // 0x6F9 / 0x6EF, len 8, first byte 0x0F, address in bytes 4-7
        if ((rxId == 0x6F9 || rxId == 0x6EF) &&
            len == 8 &&
            buf[0] == 0x0F) {

            byte tx = buf[1];
            uint32_t addr = readAddrFromRequest(buf);
            pendingAddr[tx] = addr;
            ccpRequestCount++;
        }

        // OpenECU CCP response:
        // 0x6F8 / 0x6EE, len 8, FF 00, tx in byte 2, value bytes 3-6
        if ((rxId == 0x6F8 || rxId == 0x6EE) &&
            len == 8 &&
            buf[0] == 0xFF &&
            buf[1] == 0x00) {

            byte tx = buf[2];
            uint32_t addr = pendingAddr[tx];
            ccpResponseCount++;

            int slot = activeIndexForAddress(addr);
            if (slot >= 0) {
                ActiveRange r = activeRange();
                const SymbolDef &sym = symbols[r.start + slot];

                decodeAndStore(slot, sym, &buf[3]);
                matchedResponseCount++;

                Serial.print("MATCH ");
                Serial.print(sym.name);
                Serial.print(" addr=0x");
                Serial.print(addr, HEX);
                Serial.print(" bytes=");
                for (int i = 3; i <= 6; i++) {
                    printHexByte(buf[i]);
                    Serial.print(" ");
                }
                Serial.print(" decoded=");
                printDecodedValue(sym, slot);
                Serial.println();
            }
        }
    }

    if (now - lastHeartbeatMs >= 2000) {
        lastHeartbeatMs = now;
        Serial.printlnf("alive frames=%lu ccpReq=%lu ccpResp=%lu matched=%lu",
                        frameCount,
                        ccpRequestCount,
                        ccpResponseCount,
                        matchedResponseCount);
    }

    if (now - lastPrintMs >= 5000) {
        lastPrintMs = now;
        printActiveGroupTable();
    }
}
