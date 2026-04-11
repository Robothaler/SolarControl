#include "MatterDevices.h"
#include "Config.h"
#include <esp_matter.h>
#include <esp_matter_cluster.h>
#include <esp_matter_endpoint.h>
#include <esp_log.h>

// Vorwärts-Deklaration um LWIP-Konflikt zu vermeiden
// SolarLogic wird NICHT includiert – nur Callbacks über Funktionszeiger
namespace SolarLogic {
    void setCirculation(bool on);
    void setIllumination(bool on);
    void setValve(bool poolMode);
}

static const char* TAG = "MatterDevices";

namespace MatterDevices {

// Endpoint Handles
esp_matter::endpoint_t* epRoofTemp     = nullptr;
esp_matter::endpoint_t* epBoilerTemp   = nullptr;
esp_matter::endpoint_t* epStorageTemp  = nullptr;
esp_matter::endpoint_t* epBackflowTemp = nullptr;
esp_matter::endpoint_t* epPump         = nullptr;
esp_matter::endpoint_t* epValve        = nullptr;
esp_matter::endpoint_t* epCirculation  = nullptr;
esp_matter::endpoint_t* epIllumination = nullptr;
esp_matter::endpoint_t* epValveStatus  = nullptr;
esp_matter::endpoint_t* epMotion       = nullptr;
esp_matter::endpoint_t* epLevelWarn    = nullptr;

// ── Hilfsfunktion: Temperature Sensor ────────────────────────────────────────
static esp_matter::endpoint_t* createTempSensor(
    esp_matter::node_t* node, const char* label)
{
    esp_matter::endpoint::temperature_sensor::config_t cfg;
    // Korrigiert: esp_matter_invalid → direkt mit 0x8000 (Matter "invalid" Wert)
    cfg.temperature_measurement.measured_value     = static_cast<int16_t>(0x8000);
    cfg.temperature_measurement.min_measured_value = static_cast<int16_t>(0x8000);
    cfg.temperature_measurement.max_measured_value = static_cast<int16_t>(0x8000);

    esp_matter::endpoint_t* ep = esp_matter::endpoint::temperature_sensor::create(
        node, &cfg,
        esp_matter::ENDPOINT_FLAG_NONE,   // ← korrigiert: mit Namespace
        nullptr);

    if (!ep) {
        ESP_LOGE(TAG, "Temp Sensor '%s' erstellen fehlgeschlagen!", label);
    } else {
        ESP_LOGI(TAG, "Temp Sensor '%s' EP-ID: %d",
                 label, esp_matter::endpoint::get_id(ep));
    }
    return ep;
}

// ── Hilfsfunktion: On/Off Plugin Unit ────────────────────────────────────────
static esp_matter::endpoint_t* createOnOffUnit(
    esp_matter::node_t* node, bool initialState, const char* label)
{
    esp_matter::endpoint::on_off_plugin_unit::config_t cfg;
    cfg.on_off.on_off = initialState;

    esp_matter::endpoint_t* ep = esp_matter::endpoint::on_off_plugin_unit::create(
        node, &cfg,
        esp_matter::ENDPOINT_FLAG_NONE,   // ← korrigiert: mit Namespace
        nullptr);

    if (!ep) {
        ESP_LOGE(TAG, "On/Off Unit '%s' erstellen fehlgeschlagen!", label);
    } else {
        ESP_LOGI(TAG, "On/Off Unit '%s' EP-ID: %d",
                 label, esp_matter::endpoint::get_id(ep));
    }
    return ep;
}

// ── Hilfsfunktion: Contact Sensor ────────────────────────────────────────────
static esp_matter::endpoint_t* createContactSensor(
    esp_matter::node_t* node, bool initialState, const char* label)
{
    esp_matter::endpoint::contact_sensor::config_t cfg;
    cfg.boolean_state.state_value = initialState;

    esp_matter::endpoint_t* ep = esp_matter::endpoint::contact_sensor::create(
        node, &cfg,
        esp_matter::ENDPOINT_FLAG_NONE,   // ← korrigiert: mit Namespace
        nullptr);

    if (!ep) {
        ESP_LOGE(TAG, "Contact Sensor '%s' erstellen fehlgeschlagen!", label);
    } else {
        ESP_LOGI(TAG, "Contact Sensor '%s' EP-ID: %d",
                 label, esp_matter::endpoint::get_id(ep));
    }
    return ep;
}

// ── init() ───────────────────────────────────────────────────────────────────
esp_err_t init(esp_matter::node_t* node)
{
    if (!node) {
        ESP_LOGE(TAG, "Node ist nullptr!");
        return ESP_ERR_INVALID_ARG;
    }

    epRoofTemp     = createTempSensor(node, "Dach-Kollektor");
    epBoilerTemp   = createTempSensor(node, "Boiler");
    epStorageTemp  = createTempSensor(node, "Pufferspeicher");
    epBackflowTemp = createTempSensor(node, "Ruecklauf");

    if (!epRoofTemp || !epBoilerTemp || !epStorageTemp || !epBackflowTemp)
        return ESP_FAIL;

    epPump         = createOnOffUnit(node, false, "Solarpumpe");
    epValve        = createOnOffUnit(node, false, "Ventil");
    epCirculation  = createOnOffUnit(node, false, "Zirkulation");
    epIllumination = createOnOffUnit(node, false, "Beleuchtung");

    if (!epPump || !epValve || !epCirculation || !epIllumination)
        return ESP_FAIL;

    epValveStatus  = createContactSensor(node, false, "Ventilstatus");
    epLevelWarn    = createContactSensor(node, false, "Pegel-Warnung");

    {
        esp_matter::endpoint::occupancy_sensor::config_t cfg;
        cfg.occupancy_sensing.occupancy                  = 0;
        cfg.occupancy_sensing.occupancy_sensor_type      = 0; // PIR
        cfg.occupancy_sensing.occupancy_sensor_type_bitmap = 1; // PIR bit – mandatory per Matter spec
        epMotion = esp_matter::endpoint::occupancy_sensor::create(
            node, &cfg,
            esp_matter::ENDPOINT_FLAG_NONE,  // ← korrigiert
            nullptr);
    }

    if (!epValveStatus || !epLevelWarn || !epMotion)
        return ESP_FAIL;

    ESP_LOGI(TAG, "Alle 11 Matter Endpoints initialisiert");
    return ESP_OK;
}

// ── updateTemperature ─────────────────────────────────────────────────────────
esp_err_t updateTemperature(esp_matter::endpoint_t* ep, float tempCelsius)
{
    if (!ep) return ESP_ERR_INVALID_ARG;
    if (tempCelsius < -40.0f || tempCelsius > 150.0f) return ESP_ERR_INVALID_ARG;

    int16_t  matterVal = static_cast<int16_t>(tempCelsius * 100.0f);
    uint16_t ep_id     = esp_matter::endpoint::get_id(ep);
    esp_matter_attr_val_t val = esp_matter_int16(matterVal);

    return esp_matter::attribute::update(
        ep_id,
        chip::app::Clusters::TemperatureMeasurement::Id,
        chip::app::Clusters::TemperatureMeasurement::Attributes::MeasuredValue::Id,
        &val);
}

// ── Interne Hilfsfunktion OnOff Update ───────────────────────────────────────
static esp_err_t updateOnOff(esp_matter::endpoint_t* ep,
                              bool state, const char* name)
{
    if (!ep) return ESP_ERR_INVALID_STATE;
    uint16_t ep_id = esp_matter::endpoint::get_id(ep);
    esp_matter_attr_val_t val = esp_matter_bool(state);
    return esp_matter::attribute::update(
        ep_id,
        chip::app::Clusters::OnOff::Id,
        chip::app::Clusters::OnOff::Attributes::OnOff::Id,
        &val);
}

// ── Interne Hilfsfunktion BooleanState Update ─────────────────────────────────
static esp_err_t updateBoolState(esp_matter::endpoint_t* ep,
                                  bool state, const char* name)
{
    if (!ep) return ESP_ERR_INVALID_STATE;
    uint16_t ep_id = esp_matter::endpoint::get_id(ep);
    esp_matter_attr_val_t val = esp_matter_bool(state);
    return esp_matter::attribute::update(
        ep_id,
        chip::app::Clusters::BooleanState::Id,
        chip::app::Clusters::BooleanState::Attributes::StateValue::Id,
        &val);
}

esp_err_t updatePumpState(bool r)        { return updateOnOff(epPump,         r,  "Pumpe");       }
esp_err_t updateValveState(bool p)       { return updateOnOff(epValve,         p,  "Ventil");      }
esp_err_t updateCirculationState(bool o) { return updateOnOff(epCirculation,   o,  "Zirkulation"); }
esp_err_t updateIlluminationState(bool o){ return updateOnOff(epIllumination,  o,  "Beleuchtung"); }
esp_err_t updateValveStatus(bool ok)     { return updateBoolState(epValveStatus,ok, "VentilSt.");  }
esp_err_t updateLevelWarning(bool warn)  { return updateBoolState(epLevelWarn, warn,"Pegel");      }

esp_err_t updateMotionDetected(bool detected)
{
    if (!epMotion) return ESP_ERR_INVALID_STATE;
    uint16_t ep_id = esp_matter::endpoint::get_id(epMotion);
    uint8_t occ = detected ? 1 : 0;
    esp_matter_attr_val_t val = esp_matter_uint8(occ);
    return esp_matter::attribute::update(
        ep_id,
        chip::app::Clusters::OccupancySensing::Id,
        chip::app::Clusters::OccupancySensing::Attributes::Occupancy::Id,
        &val);
}

// ── Attribute Change Callback ─────────────────────────────────────────────────
esp_err_t attributeChangeCallback(
    const esp_matter::attribute::callback_type_t type,
    uint16_t endpoint_id,
    uint32_t cluster_id,
    uint32_t attribute_id,
    esp_matter_attr_val_t* val)
{
    if (type != esp_matter::attribute::PRE_UPDATE) return ESP_OK;
    if (cluster_id != chip::app::Clusters::OnOff::Id) return ESP_OK;
    if (attribute_id != chip::app::Clusters::OnOff::Attributes::OnOff::Id) return ESP_OK;

    bool newState = val->val.b;

    if (epCirculation &&
        endpoint_id == esp_matter::endpoint::get_id(epCirculation)) {
        ESP_LOGI(TAG, "Matter → Zirkulation: %s", newState ? "EIN" : "AUS");
        SolarLogic::setCirculation(newState);
        return ESP_OK;
    }
    if (epIllumination &&
        endpoint_id == esp_matter::endpoint::get_id(epIllumination)) {
        ESP_LOGI(TAG, "Matter → Beleuchtung: %s", newState ? "EIN" : "AUS");
        SolarLogic::setIllumination(newState);
        return ESP_OK;
    }
    if (epValve &&
        endpoint_id == esp_matter::endpoint::get_id(epValve)) {
        ESP_LOGI(TAG, "Matter → Ventil: %s", newState ? "POOL" : "BOILER");
        SolarLogic::setValve(newState);
        return ESP_OK;
    }
    if (epPump &&
        endpoint_id == esp_matter::endpoint::get_id(epPump)) {
        ESP_LOGW(TAG, "Matter → Pumpe: read-only, Schreibzugriff abgelehnt");
        return ESP_FAIL;
    }
    return ESP_OK;
}

// ── subscribeToPoolMaster ─────────────────────────────────────────────────────
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

