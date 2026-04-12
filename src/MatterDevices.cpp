#include "MatterDevices.h"
#include "Config.h"
#include <esp_matter.h>
#include <esp_matter_cluster.h>
#include <esp_matter_endpoint.h>
#include <esp_log.h>
#include <lib/core/TLV.h>
#include <cstring>

// Forward-Deklarationen: nicht direkt inkludieren (LWIP-Konflikt)
namespace SolarLogic {
    enum class Mode : uint8_t { AUTO = 0, MANUAL_POOL = 1, MANUAL_BOILER = 2 };
    void setMode(Mode mode);
}
namespace CirculationLogic {
    bool requestRun();
    void stop();
}

static const char* TAG = "MatterDevices";

// ─────────────────────────────────────────────────────────────────────────────
// Cluster-IDs (Kurzschreibweise)
// ─────────────────────────────────────────────────────────────────────────────
static constexpr uint32_t kClusterModeSelect = chip::app::Clusters::ModeSelect::Id;
static constexpr uint32_t kAttrCurrentMode   = chip::app::Clusters::ModeSelect::Attributes::CurrentMode::Id;
static constexpr uint32_t kAttrSupportedModes= chip::app::Clusters::ModeSelect::Attributes::SupportedModes::Id;
static constexpr uint32_t kClusterOnOff      = chip::app::Clusters::OnOff::Id;
static constexpr uint32_t kAttrOnOff         = chip::app::Clusters::OnOff::Attributes::OnOff::Id;
static constexpr uint32_t kClusterTempMeas   = chip::app::Clusters::TemperatureMeasurement::Id;
static constexpr uint32_t kAttrMeasuredValue = chip::app::Clusters::TemperatureMeasurement::Attributes::MeasuredValue::Id;

namespace MatterDevices {

// ── Endpoint Handles ─────────────────────────────────────────────────────────
esp_matter::endpoint_t* epControlMode   = nullptr;  // EP1
esp_matter::endpoint_t* epPump          = nullptr;  // EP2
esp_matter::endpoint_t* epValveFeedback = nullptr;  // EP3
esp_matter::endpoint_t* epRoofTemp      = nullptr;  // EP4
esp_matter::endpoint_t* epBoilerTemp    = nullptr;  // EP5
esp_matter::endpoint_t* epCirculation   = nullptr;  // EP6

// ─────────────────────────────────────────────────────────────────────────────
// Hilfsfunktion: SupportedModes-Liste als CHIP-TLV kodieren
//
// Format pro ModeOptionStruct (Matter spec 1.0 §1.8.9):
//   Structure {
//     ContextTag(0) → Label    : UTF8String
//     ContextTag(1) → Mode     : UInt8
//     ContextTag(2) → ModeTags : Array<ModeTagStruct>  (leer = vendor-spezifisch)
//   }
//
// buf     : Ausgabe-Puffer (muss persistent sein – static in caller)
// bufSize : Größe des Puffers in Bytes
// labels  : Array aus null-terminierten Label-Strings
// modes   : Array der entsprechenden Modus-Werte (uint8_t)
// count   : Anzahl der Einträge
// Rückgabe: geschriebene Bytes (0 = Fehler)
// ─────────────────────────────────────────────────────────────────────────────
static uint16_t encodeSupportedModes(
    uint8_t*      buf,
    uint16_t      bufSize,
    const char**  labels,
    const uint8_t* modes,
    uint8_t       count)
{
    chip::TLV::TLVWriter writer;
    writer.Init(buf, bufSize);

    chip::TLV::TLVType arrayType;
    if (writer.StartContainer(chip::TLV::AnonymousTag(),
                              chip::TLV::kTLVType_Array,
                              arrayType) != CHIP_NO_ERROR)
        return 0;

    for (uint8_t i = 0; i < count; i++) {
        chip::TLV::TLVType structType;
        if (writer.StartContainer(chip::TLV::AnonymousTag(),
                                  chip::TLV::kTLVType_Structure,
                                  structType) != CHIP_NO_ERROR)
            return 0;

        if (writer.PutString(chip::TLV::ContextTag(0), labels[i]) != CHIP_NO_ERROR)
            return 0;
        if (writer.Put(chip::TLV::ContextTag(1), modes[i]) != CHIP_NO_ERROR)
            return 0;

        // Leere ModeTags-Liste (vendor-spezifisch, kein semantisches Tag)
        chip::TLV::TLVType tagsType;
        if (writer.StartContainer(chip::TLV::ContextTag(2),
                                  chip::TLV::kTLVType_Array,
                                  tagsType) != CHIP_NO_ERROR)
            return 0;
        if (writer.EndContainer(tagsType) != CHIP_NO_ERROR)
            return 0;

        if (writer.EndContainer(structType) != CHIP_NO_ERROR)
            return 0;
    }

    if (writer.EndContainer(arrayType) != CHIP_NO_ERROR) return 0;
    if (writer.Finalize()              != CHIP_NO_ERROR) return 0;

    return static_cast<uint16_t>(writer.GetLengthWritten());
}

// ─────────────────────────────────────────────────────────────────────────────
// Hilfsfunktion: Mode Select Endpoint erstellen und SupportedModes setzen
// ─────────────────────────────────────────────────────────────────────────────
static esp_matter::endpoint_t* createModeSelectEP(
    esp_matter::node_t* node,
    const char*         description,
    uint8_t             initialMode,
    const char**        labels,
    const uint8_t*      modeVals,
    uint8_t             modeCount,
    uint8_t*            tlvBuf,       // persistent buffer, min. 256 Bytes
    uint16_t            tlvBufSize)
{
    esp_matter::endpoint::mode_select_device::config_t cfg;
    cfg.mode_select.current_mode  = initialMode;
    // mode_select_description ist ein char[65]-Array, kein Pointer
    strncpy(cfg.mode_select.mode_select_description,
            description,
            sizeof(cfg.mode_select.mode_select_description) - 1);
    cfg.mode_select.mode_select_description[
        sizeof(cfg.mode_select.mode_select_description) - 1] = '\0';
    // standard_namespace: default (nullable, leer) → vendor-spezifisch

    esp_matter::endpoint_t* ep = esp_matter::endpoint::mode_select_device::create(
        node, &cfg, esp_matter::ENDPOINT_FLAG_NONE, nullptr);

    if (!ep) {
        ESP_LOGE(TAG, "Mode Select EP '%s' erstellen fehlgeschlagen", description);
        return nullptr;
    }

    uint16_t ep_id = esp_matter::endpoint::get_id(ep);
    ESP_LOGI(TAG, "Mode Select EP '%s' EP-ID: %d", description, ep_id);

    // SupportedModes als TLV kodieren und schreiben
    uint16_t len = encodeSupportedModes(tlvBuf, tlvBufSize, labels, modeVals, modeCount);
    if (len == 0) {
        ESP_LOGE(TAG, "SupportedModes TLV Kodierung fehlgeschlagen für '%s'", description);
        return ep;  // EP bleibt nutzbar, SupportedModes ist leer
    }

    esp_matter_attr_val_t val;
    val.type     = ESP_MATTER_VAL_TYPE_ARRAY;
    val.val.a.b  = tlvBuf;
    val.val.a.s  = len;
    val.val.a.n  = modeCount;
    val.val.a.t  = 0;  // nicht verwendet bei Struct-Arrays

    esp_err_t err = esp_matter::attribute::update(
        ep_id, kClusterModeSelect, kAttrSupportedModes, &val);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SupportedModes Update für '%s' fehlgeschlagen: %s",
                 description, esp_err_to_name(err));
    }

    return ep;
}

// ─────────────────────────────────────────────────────────────────────────────
// Hilfsfunktion: Temperature Sensor Endpoint
// ─────────────────────────────────────────────────────────────────────────────
static esp_matter::endpoint_t* createTempSensor(
    esp_matter::node_t* node, const char* label)
{
    esp_matter::endpoint::temperature_sensor::config_t cfg;
    cfg.temperature_measurement.measured_value     = static_cast<int16_t>(0x8000);
    cfg.temperature_measurement.min_measured_value = static_cast<int16_t>(0x8000);
    cfg.temperature_measurement.max_measured_value = static_cast<int16_t>(0x8000);

    esp_matter::endpoint_t* ep = esp_matter::endpoint::temperature_sensor::create(
        node, &cfg, esp_matter::ENDPOINT_FLAG_NONE, nullptr);

    if (!ep) {
        ESP_LOGE(TAG, "Temp Sensor '%s' erstellen fehlgeschlagen", label);
    } else {
        ESP_LOGI(TAG, "Temp Sensor '%s' EP-ID: %d",
                 label, esp_matter::endpoint::get_id(ep));
    }
    return ep;
}

// ─────────────────────────────────────────────────────────────────────────────
// init()
// Erstellt die 5 Endpoints (EP1–EP5) gemäß Datenmodell.
// EP0 (Root Node) wird automatisch von esp_matter::node::create() erstellt.
// ─────────────────────────────────────────────────────────────────────────────
esp_err_t init(esp_matter::node_t* node)
{
    if (!node) {
        ESP_LOGE(TAG, "Node ist nullptr!");
        return ESP_ERR_INVALID_ARG;
    }

    // ── EP1: Betriebsmodus ────────────────────────────────────────────────────
    // Modi: AUTO(0), POOL(1), BOILER(2)
    // Persistent: TLV-Puffer muss die Lifetime des Endpoints überleben
    static uint8_t s_tlvControl[256];
    static const char* kControlLabels[] = { "AUTO", "POOL", "BOILER" };
    static const uint8_t kControlModes[] = { MODE_AUTO, MODE_POOL, MODE_BOILER };

    epControlMode = createModeSelectEP(
        node,
        "Betriebsmodus",
        MODE_AUTO,              // Startwert: AUTO
        kControlLabels,
        kControlModes,
        3,
        s_tlvControl,
        sizeof(s_tlvControl));

    if (!epControlMode) return ESP_FAIL;

    // ── EP2: Solarpumpe ───────────────────────────────────────────────────────
    // On/Off Plugin Unit – wird nur von SolarLogic geschrieben, nicht von Matter
    {
        esp_matter::endpoint::on_off_plugin_unit::config_t cfg;
        cfg.on_off.on_off = false;
        epPump = esp_matter::endpoint::on_off_plugin_unit::create(
            node, &cfg, esp_matter::ENDPOINT_FLAG_NONE, nullptr);
        if (!epPump) {
            ESP_LOGE(TAG, "EP2 Pumpe erstellen fehlgeschlagen");
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "On/Off 'Solarpumpe' EP-ID: %d",
                 esp_matter::endpoint::get_id(epPump));
    }

    // ── EP3: Ventil-Rückmeldung ───────────────────────────────────────────────
    // Modi: BOILER(0), POOL(1) – CurrentMode ist read-only (Feedback-Pin)
    // ChangeToMode-Befehle werden im attributeChangeCallback abgelehnt.
    static uint8_t s_tlvValve[128];
    static const char* kValveLabels[] = { "BOILER", "POOL" };
    static const uint8_t kValveModes[] = { VALVE_BOILER, VALVE_POOL };

    epValveFeedback = createModeSelectEP(
        node,
        "Ventil-Feedback",
        VALVE_BOILER,           // Startwert: BOILER
        kValveLabels,
        kValveModes,
        2,
        s_tlvValve,
        sizeof(s_tlvValve));

    if (!epValveFeedback) return ESP_FAIL;

    // ── EP4: Dach-Kollektor Temperatur ────────────────────────────────────────
    epRoofTemp = createTempSensor(node, "Dach-Kollektor");
    if (!epRoofTemp) return ESP_FAIL;

    // ── EP5: Boiler Temperatur ────────────────────────────────────────────────
    epBoilerTemp = createTempSensor(node, "Boiler");
    if (!epBoilerTemp) return ESP_FAIL;

    // ── EP6: Zirkulationspumpe ────────────────────────────────────────────────
    // On/Off Plugin Unit – per Matter schaltbar (SmartHome-Regel-Trigger).
    // CirculationLogic entscheidet ob der Lauf tatsächlich startet (Sperrzeit,
    // Präsenzfenster). Read-back über updateCirculationState() nach jedem Lauf.
    {
        esp_matter::endpoint::on_off_plugin_unit::config_t cfg;
        cfg.on_off.on_off = false;
        epCirculation = esp_matter::endpoint::on_off_plugin_unit::create(
            node, &cfg, esp_matter::ENDPOINT_FLAG_NONE, nullptr);
        if (!epCirculation) {
            ESP_LOGE(TAG, "EP6 Zirkulation erstellen fehlgeschlagen");
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "On/Off 'Zirkulationspumpe' EP-ID: %d",
                 esp_matter::endpoint::get_id(epCirculation));
    }

    ESP_LOGI(TAG, "6 Matter Endpoints initialisiert (EP1-EP6)");
    return ESP_OK;
}

// ─────────────────────────────────────────────────────────────────────────────
// updateControlMode – EP1 CurrentMode publizieren
// Wird aufgerufen wenn SolarLogic den Modus intern ändert (z.B. nach NVS-Restore)
// damit der Matter-Controller den aktuellen Zustand sieht.
// ─────────────────────────────────────────────────────────────────────────────
esp_err_t updateControlMode(uint8_t mode)
{
    if (!epControlMode) return ESP_ERR_INVALID_STATE;
    uint16_t ep_id = esp_matter::endpoint::get_id(epControlMode);
    esp_matter_attr_val_t val = esp_matter_uint8(mode);
    return esp_matter::attribute::update(ep_id, kClusterModeSelect, kAttrCurrentMode, &val);
}

// ─────────────────────────────────────────────────────────────────────────────
// updatePumpState – EP2 OnOff publizieren (read-only aus Controller-Sicht)
// ─────────────────────────────────────────────────────────────────────────────
esp_err_t updatePumpState(bool running)
{
    if (!epPump) return ESP_ERR_INVALID_STATE;
    uint16_t ep_id = esp_matter::endpoint::get_id(epPump);
    esp_matter_attr_val_t val = esp_matter_bool(running);
    return esp_matter::attribute::update(ep_id, kClusterOnOff, kAttrOnOff, &val);
}

// ─────────────────────────────────────────────────────────────────────────────
// updateValveFeedback – EP3 CurrentMode aus Hardware-Pin aktualisieren
// mode: VALVE_BOILER(0) oder VALVE_POOL(1)
// Entkoppelt von EP1: spiegelt nur den physischen Endschalter wider.
// ─────────────────────────────────────────────────────────────────────────────
esp_err_t updateValveFeedback(uint8_t mode)
{
    if (!epValveFeedback) return ESP_ERR_INVALID_STATE;
    uint16_t ep_id = esp_matter::endpoint::get_id(epValveFeedback);
    esp_matter_attr_val_t val = esp_matter_uint8(mode);
    return esp_matter::attribute::update(ep_id, kClusterModeSelect, kAttrCurrentMode, &val);
}

// ─────────────────────────────────────────────────────────────────────────────
// updateCirculationState – EP6 OnOff Zustand spiegeln
// Wird von CirculationLogic nach jedem Start/Stop aufgerufen.
// ─────────────────────────────────────────────────────────────────────────────
esp_err_t updateCirculationState(bool running)
{
    if (!epCirculation) return ESP_ERR_INVALID_STATE;
    uint16_t ep_id = esp_matter::endpoint::get_id(epCirculation);
    esp_matter_attr_val_t val = esp_matter_bool(running);
    return esp_matter::attribute::update(ep_id, kClusterOnOff, kAttrOnOff, &val);
}

// ─────────────────────────────────────────────────────────────────────────────
// updateTemperature – EP4 oder EP5, Skalierung: °C × 100 → int16
// ─────────────────────────────────────────────────────────────────────────────
esp_err_t updateTemperature(esp_matter::endpoint_t* ep, float tempCelsius)
{
    if (!ep) return ESP_ERR_INVALID_ARG;
    if (tempCelsius < -40.0f || tempCelsius > 150.0f) return ESP_ERR_INVALID_ARG;

    int16_t matterVal = static_cast<int16_t>(tempCelsius * 100.0f);
    uint16_t ep_id    = esp_matter::endpoint::get_id(ep);
    esp_matter_attr_val_t val = esp_matter_int16(matterVal);

    return esp_matter::attribute::update(ep_id, kClusterTempMeas, kAttrMeasuredValue, &val);
}

// ─────────────────────────────────────────────────────────────────────────────
// attributeChangeCallback
//
// EP1 (Betriebsmodus) – ChangeToMode → SolarLogic::setMode()
//   Matter-Controller schreibt CurrentMode via ChangeToMode-Command:
//   0 → AUTO, 1 → MANUAL_POOL, 2 → MANUAL_BOILER
//
// EP2 (Pumpe) – OnOff-Schreibzugriff ablehnen (read-only)
//   Pumpe wird ausschließlich von der Steuerlogik geschaltet.
//
// EP3 (Ventil-Feedback) – ChangeToMode ablehnen (read-only)
//   Ventilposition spiegelt den Hardware-Pin wider, ist nicht schaltbar.
// ─────────────────────────────────────────────────────────────────────────────
esp_err_t attributeChangeCallback(
    const esp_matter::attribute::callback_type_t type,
    uint16_t endpoint_id,
    uint32_t cluster_id,
    uint32_t attribute_id,
    esp_matter_attr_val_t* val)
{
    if (type != esp_matter::attribute::PRE_UPDATE) return ESP_OK;

    // ── EP1: Betriebsmodus ────────────────────────────────────────────────────
    if (epControlMode &&
        endpoint_id == esp_matter::endpoint::get_id(epControlMode) &&
        cluster_id  == kClusterModeSelect &&
        attribute_id == kAttrCurrentMode)
    {
        uint8_t newMode = val->val.u8;
        ESP_LOGI(TAG, "Matter → Betriebsmodus: %s",
                 newMode == MODE_AUTO   ? "AUTO"   :
                 newMode == MODE_POOL   ? "POOL"   :
                 newMode == MODE_BOILER ? "BOILER" : "?");

        SolarLogic::Mode logicMode;
        switch (newMode) {
            case MODE_POOL:   logicMode = SolarLogic::Mode::MANUAL_POOL;   break;
            case MODE_BOILER: logicMode = SolarLogic::Mode::MANUAL_BOILER; break;
            default:          logicMode = SolarLogic::Mode::AUTO;          break;
        }
        SolarLogic::setMode(logicMode);
        return ESP_OK;
    }

    // ── EP2: Pumpe – Schreibzugriff ablehnen ──────────────────────────────────
    if (epPump &&
        endpoint_id == esp_matter::endpoint::get_id(epPump) &&
        cluster_id  == kClusterOnOff &&
        attribute_id == kAttrOnOff)
    {
        ESP_LOGW(TAG, "Matter → Pumpe: read-only, Schreibzugriff abgelehnt");
        return ESP_FAIL;
    }

    // ── EP3: Ventil-Feedback – ChangeToMode ablehnen ─────────────────────────
    if (epValveFeedback &&
        endpoint_id == esp_matter::endpoint::get_id(epValveFeedback) &&
        cluster_id  == kClusterModeSelect &&
        attribute_id == kAttrCurrentMode)
    {
        ESP_LOGW(TAG, "Matter → Ventil-Feedback: read-only, ChangeToMode abgelehnt");
        return ESP_FAIL;
    }

    // ── EP6: Zirkulationspumpe – OnOff-Schreibzugriff ────────────────────────
    // OnOff=true  → CirculationLogic::requestRun() prüft Sperrzeit/Fenster
    // OnOff=false → CirculationLogic::stop() bricht laufenden Zyklus ab
    if (epCirculation &&
        endpoint_id == esp_matter::endpoint::get_id(epCirculation) &&
        cluster_id  == kClusterOnOff &&
        attribute_id == kAttrOnOff)
    {
        bool requested = val->val.b;
        if (requested) {
            if (!CirculationLogic::requestRun()) {
                // Gesperrt: Anforderung ablehnen, Attribut bleibt false
                return ESP_FAIL;
            }
            ESP_LOGI(TAG, "Matter → Zirkulation: Lauf angefordert");
        } else {
            CirculationLogic::stop();
            ESP_LOGI(TAG, "Matter → Zirkulation: Stop angefordert");
        }
        return ESP_OK;
    }

    return ESP_OK;
}

// ─────────────────────────────────────────────────────────────────────────────
// subscribeToPoolMaster – unverändert
// ─────────────────────────────────────────────────────────────────────────────
esp_err_t subscribeToPoolMaster(uint64_t poolNodeId,
                                 uint16_t epPoolTemp,
                                 uint16_t epPoolSolltemp,
                                 uint16_t epSolarMode)
{
    if (poolNodeId == 0) {
        ESP_LOGW(TAG, "PoolMaster NodeID = 0 → noch nicht konfiguriert");
        return ESP_ERR_INVALID_ARG;
    }
    ESP_LOGI(TAG, "Subscription zu PoolMaster NodeID: 0x%016llX", poolNodeId);
    ESP_LOGI(TAG, "  EP%d: Pool_Temp  EP%d: Pool_Soll  EP%d: Solar_Mode",
             epPoolTemp, epPoolSolltemp, epSolarMode);
    return ESP_OK;
}

} // namespace MatterDevices
