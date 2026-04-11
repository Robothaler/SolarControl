#pragma once

#include <esp_matter.h>
#include <esp_matter_cluster.h>
#include "Config.h"

namespace MatterDevices {

// Endpoint Handles (extern damit andere .cpp darauf zugreifen können)
extern esp_matter::endpoint_t* epRoofTemp;
extern esp_matter::endpoint_t* epBoilerTemp;
extern esp_matter::endpoint_t* epStorageTemp;
extern esp_matter::endpoint_t* epBackflowTemp;
extern esp_matter::endpoint_t* epPump;
extern esp_matter::endpoint_t* epValve;
extern esp_matter::endpoint_t* epCirculation;
extern esp_matter::endpoint_t* epIllumination;
extern esp_matter::endpoint_t* epValveStatus;
extern esp_matter::endpoint_t* epMotion;
extern esp_matter::endpoint_t* epLevelWarn;

// Initialisierung
esp_err_t init(esp_matter::node_t* node);

// Temperatur-Update
esp_err_t updateTemperature(esp_matter::endpoint_t* ep, float tempCelsius);

// Aktor-Updates
esp_err_t updatePumpState(bool running);
esp_err_t updateValveState(bool poolMode);
esp_err_t updateCirculationState(bool on);
esp_err_t updateIlluminationState(bool on);
esp_err_t updateValveStatus(bool ok);
esp_err_t updateLevelWarning(bool warn);    // ← war fehlend
esp_err_t updateMotionDetected(bool detected);

// Attribute Change Callback
esp_err_t attributeChangeCallback(
    const esp_matter::attribute::callback_type_t type,
    uint16_t endpoint_id,
    uint32_t cluster_id,
    uint32_t attribute_id,
    esp_matter_attr_val_t* val);

// PoolMaster Subscription
esp_err_t subscribeToPoolMaster(uint64_t poolNodeId,
                                 uint16_t epPoolTemp,
                                 uint16_t epPoolSolltemp,
                                 uint16_t epSolarMode);

} // namespace MatterDevices
