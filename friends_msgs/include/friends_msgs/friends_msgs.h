#ifndef MSGS_EIGEN_MSGS_H
#define MSGS_EIGEN_MSGS_H

#include <iostream>
#include <jsoncpp/json/json.h>
#include <std_msgs/String.h>
#include <initializer_list>

#include "friends_msgs/common.h"

namespace friends_msgs {

Json::Reader reader;
Json::StreamWriterBuilder builder;

inline std::string create_cmd(std::string cmd, std::string droneID) {
    Json::Value root;
    root["droneId"] = droneID;
    root["mode"] = "action";
    root["cmd"] = cmd;
    std::string cmd_str = Json::writeString(builder,root);
    // Create Ros1 message with cmd_str and send
    return cmd_str;
} 

/**
* brief Battery type.
*/
struct Battery {
  double voltage_v; /**< @brief Voltage in volts. */
  double remaining_percent; /**< @brief Estimated battery percentage remaining (range: 0.0 to 1.0). */
  Battery()
      : voltage_v(0.0),
        remaining_percent(0.0) {}

  inline static const Battery zero(){
    static const Battery battery;
    return battery;
  }
  
};

/**
* brief Position type in global coordinates.
*/
struct Position {
  double latitude_deg; /**< @brief Latitude in degrees (range: -90 to +90) */
  double longitude_deg; /**< @brief Longitude in degrees (range: -180 to 180) */
  double heading;
  double relative_altitude_m; /**< @brief Altitude relative to takeoff altitude in metres */
};

inline std::string create_cmd_goto(std::string droneID, Position pos) {
    Json::Value root;
    builder.settings_["indentation"] = "";
    builder.settings_["enableYAMLCompatibility"] = true;
    root["droneId"] = droneID;
    root["mode"] = "action";
    root["cmd"] = "goto";
    root["lat"] = pos.latitude_deg;
    root["lon"] = pos.longitude_deg;
    root["alt"] = pos.relative_altitude_m;
    root["yaw"] = pos.heading;
    std::string cmd_str = Json::writeString(builder,root);
    // Create Ros1 message with cmd_str and send
    return cmd_str;
}

struct Telemetry {
  Telemetry()
      : armed(false),
        battery(Battery::zero()),
        droneID(""),
        flight_mode(""),
        landed_state(""),
        healthFail({"init"}),
        position(),
        timestamp_ns(-1){}

  Telemetry(bool _armed,
                Battery _battery,
                std::string _droneID,
                std::string _flight_mode,
                std::string _landed_state,
                std::vector<std::string> _healthFail,
                Position _position,
                int64_t _timestamp_ns )
      : armed(_armed),
        battery(_battery),
        droneID(_droneID),
        flight_mode(_flight_mode),
        landed_state(_landed_state),
        healthFail(_healthFail),
        position(_position),
        timestamp_ns(_timestamp_ns){}

  bool armed;
  Battery battery;
  std::string droneID;
  std::string flight_mode;
  std::string landed_state;
  std::vector<std::string> healthFail;
  Position position;
  int64_t timestamp_ns;  // Time since epoch, negative value = invalid timestamp.

  std::string toString() const {
    std::stringstream ss;
    ss << "armed: " << armed << std::endl
       << "battery remaining: " << battery.remaining_percent << std::endl
       << "battery voltage: " << battery.voltage_v << std::endl
       << "droneID: " << droneID << std::endl
       << "flight mode: " << flight_mode << std::endl
       << "landed state: " << landed_state << std::endl
       << "healthFail: [ ";
    size_t last = healthFail.size() - 1;
    for(size_t i = 0; i < healthFail.size(); ++i) {
        ss << healthFail[i];
        if (i != last) 
            ss << ", ";
    }
    ss << "]" << std::endl
       << "heading: " << position.heading << std::endl
       << "Altitude: " << position.relative_altitude_m << std::endl
       << "Latitude: " << position.latitude_deg << std::endl
       << "Longitude: " << position.longitude_deg << std::endl
       << "Time(ns): " << timestamp_ns << std::endl;

    return ss.str();
  }

  inline void fromString(const std_msgs::String::ConstPtr& msg) {
    Json::Value root;
    if (!reader.parse(msg->data, root)) {
        std::cout << "Can't be parsed!" << std::endl;
    }
    armed = root["armed"].asBool();
    droneID = root["droneId"].asString();
    timestamp_ns = root["timestamp"].asInt64();
    flight_mode.clear();
    flight_mode = root["flight_mode"].asString();
    landed_state = root["landed_state"].asString();
    healthFail.clear();
    for(Json::Value::ArrayIndex i = 0; i < root["healthFail"].size(); ++i) {
        healthFail.push_back(root["healthFail"][i].asString());
    }
    position.latitude_deg = root["position"]["lat"].asDouble();
    position.longitude_deg = root["position"]["lon"].asDouble();
    position.heading = root["heading"].asDouble();
    position.relative_altitude_m = root["position"]["alt"].asDouble();
    battery.remaining_percent = root["battery"]["remaining_percent"].asDouble();
    battery.voltage_v = root["battery"]["voltage"].asDouble();
  }

  inline bool health_all_ok(){
    return healthFail.empty();
  }

  inline bool on_air(){
    if(landed_state.compare("in_air") == 0) return true;
    return false;
  }

};

}

#endif
