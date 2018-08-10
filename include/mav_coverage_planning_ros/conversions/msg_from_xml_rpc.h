#ifndef MAV_COVERAGE_PLANNING_ROS_CONVERSIONS_MSG_FROM_XML_RPC_H
#define MAV_COVERAGE_PLANNING_ROS_CONVERSIONS_MSG_FROM_XML_RPC_H

#include <exception>
#include <string>

#include <glog/logging.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>
#include <std_msgs/UInt32.h>

#include <planning_msgs/Point2D.h>
#include <planning_msgs/Polygon2D.h>
#include <planning_msgs/PolygonWithHoles.h>
#include <planning_msgs/PolygonWithHolesStamped.h>

namespace mav_coverage_planning {

// Check if a XML RPC member exists and return an error message if not.
inline bool hasMember(const XmlRpc::XmlRpcValue& xml_rpc,
                      const std::string& key) {
  if (xml_rpc.hasMember(key)) {
    return true;
  } else {
    ROS_WARN_STREAM("XML RPC has no key: " << key.c_str());
    return false;
  }
}

// Check if a XML RPC value has the correct type and return an error message if
// not.
inline bool checkType(XmlRpc::XmlRpcValue& xml_rpc, const std::string& key,
                      const XmlRpc::XmlRpcValue::Type& expected_type) {
  if (xml_rpc[key].getType() == expected_type) {
    return true;
  } else {
    ROS_ERROR_STREAM("XML RPC expected type: " << expected_type << " is type: "
                                               << xml_rpc[key].getType());
    return false;
  }
}

// Read an elementary type from an XML RPC.
template <typename T>
inline bool readElementaryTypeFromXmlRpc(
    XmlRpc::XmlRpcValue& xml_rpc, const std::string& key,
    const XmlRpc::XmlRpcValue::Type& expected_type, T* result) {
  CHECK_NOTNULL(result);

  const bool has_member = hasMember(xml_rpc, key);
  const bool is_correct_type = checkType(xml_rpc, key, expected_type);
  if (has_member && is_correct_type) {
    if (expected_type == XmlRpc::XmlRpcValue::TypeBoolean ||
        expected_type == XmlRpc::XmlRpcValue::TypeInt ||
        expected_type == XmlRpc::XmlRpcValue::TypeDouble ||
        expected_type == XmlRpc::XmlRpcValue::TypeString) {
      // Catch special cases.
      *result = static_cast<T>(xml_rpc[key]);
      return true;
    } else {
      ROS_ERROR_STREAM("Method not implemented for type " << expected_type
                                                          << ".");
      return false;
    }
  } else {
    ROS_WARN_STREAM("Failed reading elementary XML RPC member.");
    ROS_WARN_STREAM_COND(!has_member,
                         "Member with key " << key << " does not exist.");
    ROS_WARN_STREAM_COND(!is_correct_type,
                         "Member with key " << key
                                            << " does not have expected type "
                                            << expected_type << ".");
    return false;  // XML RPC lacks member or member has wrong data type.
  }
}

// Catch special cases that cannot be casted.
// Unsigned int.
inline bool readElementaryTypeFromXmlRpc(
    XmlRpc::XmlRpcValue& xml_rpc, const std::string& key,
    const XmlRpc::XmlRpcValue::Type& expected_type, unsigned int* result) {
  CHECK_NOTNULL(result);

  int temp_result = -1;
  if (readElementaryTypeFromXmlRpc(xml_rpc, key, expected_type, &temp_result)) {
    *result = static_cast<unsigned int>(temp_result);
    return true;
  } else {
    return false;
  }
}

// Float.
inline bool readElementaryTypeFromXmlRpc(
    XmlRpc::XmlRpcValue& xml_rpc, const std::string& key,
    const XmlRpc::XmlRpcValue::Type& expected_type, float* result) {
  CHECK_NOTNULL(result);

  double temp_result = 0.0;
  if (readElementaryTypeFromXmlRpc(xml_rpc, key, expected_type, &temp_result)) {
    *result = static_cast<float>(temp_result);
    return true;
  } else {
    return false;
  }
}

// Stamp from XML RPC.
inline bool timeFromXmlRpc(XmlRpc::XmlRpcValue& xml_rpc, ros::Time* t) {
  CHECK_NOTNULL(t);

  const std::string kSecsKey = "secs";
  const std::string kNsecsKey = "nsecs";

  try {
    // Get seconds.
    int secs = 0;
    if (!readElementaryTypeFromXmlRpc(xml_rpc, kSecsKey,
                                      XmlRpc::XmlRpcValue::TypeInt, &secs)) {
      ROS_WARN_STREAM("Resetting key " << kSecsKey << " to default.");
      secs = std_msgs::Int32().data;
    }
    // Get nano seconds.
    int nsecs = 0;
    if (!readElementaryTypeFromXmlRpc(xml_rpc, kNsecsKey,
                                      XmlRpc::XmlRpcValue::TypeInt, &nsecs)) {
      ROS_WARN_STREAM("Resetting key " << kNsecsKey << " to default.");
      nsecs = std_msgs::Int32().data;
    }
    *t = ros::Time(secs, nsecs);
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("XML RPC client threw error: " << e.what());
    return false;  // XML RPC client exception.
  }
  return true;
}

// Header from XML RPC.
inline bool headerMsgFromXmlRpc(XmlRpc::XmlRpcValue& xml_rpc,
                                std_msgs::Header* msg) {
  CHECK_NOTNULL(msg);

  const std::string kSeqKey = "seq";
  const std::string kStampKey = "stamp";
  const std::string kFrameIdKey = "frame_id";

  try {
    // Get sequence.
    if (!readElementaryTypeFromXmlRpc(
            xml_rpc, kSeqKey, XmlRpc::XmlRpcValue::TypeInt, &msg->seq)) {
      ROS_WARN_STREAM("Resetting key " << kSeqKey << " to default.");
      msg->seq = std_msgs::UInt32().data;
    }

    // Get time stamp.
    if (!hasMember(xml_rpc, kStampKey) ||
        !timeFromXmlRpc(xml_rpc[kStampKey], &(msg->stamp))) {
      ROS_WARN_STREAM("Missing "
                      << kStampKey
                      << " key or failed reading time. Resetting to default");
      msg->stamp = std_msgs::Time().data;
    }

    // Get frame id.
    if (!readElementaryTypeFromXmlRpc(xml_rpc, kFrameIdKey,
                                      XmlRpc::XmlRpcValue::TypeString,
                                      &msg->frame_id)) {
      ROS_WARN_STREAM("Resetting key " << kFrameIdKey << " to default.");
      msg->frame_id = std_msgs::String().data;
    }

  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("XML RPC client threw error: " << e.what());
    return false;  // XML RPC client exception.
  }
  return true;
}

// Point2D from XML RPC.
inline bool point2DMsgFromXmlRpc(XmlRpc::XmlRpcValue& xml_rpc,
                                 planning_msgs::Point2D* msg) {
  CHECK_NOTNULL(msg);

  const std::string kXKey = "x";
  const std::string kYKey = "y";

  try {
    // Get x.
    if (!readElementaryTypeFromXmlRpc(
            xml_rpc, kXKey, XmlRpc::XmlRpcValue::TypeDouble, &msg->x)) {
      ROS_WARN_STREAM("Resetting key " << kXKey << " to default.");
      msg->x = std_msgs::Float64().data;
    }

    // Get y.
    if (!readElementaryTypeFromXmlRpc(
            xml_rpc, kYKey, XmlRpc::XmlRpcValue::TypeDouble, &msg->y)) {
      ROS_WARN_STREAM("Resetting key " << kYKey << " to default.");
      msg->y = std_msgs::Float64().data;
    }
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("XML RPC client threw error: " << e.what());
    return false;  // XML RPC client exception.
  }
  return true;
}

// Polygon2D from XML RPC.
inline bool polygon2DMsgFromXmlRpc(XmlRpc::XmlRpcValue& xml_rpc,
                                   planning_msgs::Polygon2D* msg) {
  CHECK_NOTNULL(msg);

  const std::string kPointsKey = "points";

  try {
    // Get polygon vertices.
    if (hasMember(xml_rpc, kPointsKey) &&
        checkType(xml_rpc, kPointsKey, XmlRpc::XmlRpcValue::TypeArray)) {
      msg->points.resize(xml_rpc[kPointsKey].size());
      for (int i = 0; i < xml_rpc[kPointsKey].size(); ++i) {
        planning_msgs::Point2D p;
        if (!point2DMsgFromXmlRpc(xml_rpc[kPointsKey][i], &p)) {
          ROS_WARN_STREAM("Loading point " << i
                                           << " failed. Resetting to default.");
          p = planning_msgs::Point2D();
        }
        msg->points[i] = p;
      }
    } else {
      ROS_WARN_STREAM("Missing or non-array "
                      << kPointsKey << " member. Resetting to default");
      msg->points = std::vector<planning_msgs::Point2D>();
    }
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("XML RPC client threw error: " << e.what());
    return false;  // XML RPC client exception.
  }
  return true;
}

// PolygonWithHoles from XML RPC.
inline bool polygonWithHolesMsgFromXmlRpc(
    XmlRpc::XmlRpcValue& xml_rpc, planning_msgs::PolygonWithHoles* msg) {
  CHECK_NOTNULL(msg);

  const std::string kHullKey = "hull";
  const std::string kHolesKey = "holes";

  try {
    // Get hull.
    if (!hasMember(xml_rpc, kHullKey) ||
        !polygon2DMsgFromXmlRpc(xml_rpc[kHullKey], &msg->hull)) {
      ROS_WARN_STREAM(
          "Missing " << kHullKey
                     << " key or failed loading polygon. Resetting to default");
      msg->hull = planning_msgs::Polygon2D();
    }

    // Get holes.
    if (hasMember(xml_rpc, kHolesKey) &&
        checkType(xml_rpc, kHolesKey, XmlRpc::XmlRpcValue::TypeArray)) {
      msg->holes.resize(xml_rpc[kHolesKey].size());
      for (int i = 0; i < xml_rpc[kHolesKey].size(); ++i) {
        planning_msgs::Polygon2D p;
        if (!polygon2DMsgFromXmlRpc(xml_rpc[kHolesKey][i], &p)) {
          ROS_WARN_STREAM("Loading hole " << i
                                          << " failed. Resetting to default.");
          p = planning_msgs::Polygon2D();
        }
        msg->holes[i] = p;
      }
    } else {
      ROS_WARN_STREAM("Missing or non-array "
                      << kHolesKey << " member. Resetting to default");
      msg->holes = std::vector<planning_msgs::Polygon2D>();
    }

  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("XML RPC client threw error: " << e.what());
    return false;  // XML RPC client exception.
  }
  return true;
}

// PolygonWithHolesStamped from XML RPC.
inline bool PolygonWithHolesStampedMsgFromXmlRpc(
    XmlRpc::XmlRpcValue& xml_rpc, planning_msgs::PolygonWithHolesStamped* msg) {
  CHECK_NOTNULL(msg);

  const std::string kHeaderKey = "header";
  const std::string kPolygonWithHolesKey = "polygon";
  const std::string kAltitudeKey = "altitude";

  try {
    // Get header.
    if (!hasMember(xml_rpc, kHeaderKey) ||
        !headerMsgFromXmlRpc(xml_rpc[kHeaderKey], &msg->header)) {
      ROS_WARN_STREAM(
          "Missing "
          << kHeaderKey
          << " member or loading header failed. Resetting to default");
      msg->header = std_msgs::Header();
    }

    // Get polygon with holes.
    if (!hasMember(xml_rpc, kPolygonWithHolesKey) ||
        !polygonWithHolesMsgFromXmlRpc(xml_rpc[kPolygonWithHolesKey],
                                       &msg->polygon)) {
      ROS_WARN_STREAM("Missing " << kPolygonWithHolesKey
                                 << " member or loading polygon with holes "
                                    "failed. Resetting to default");
      msg->polygon = planning_msgs::PolygonWithHoles();
    }

    // Get altitude from ground.
    if (!readElementaryTypeFromXmlRpc(xml_rpc, kAltitudeKey,
                                      XmlRpc::XmlRpcValue::TypeDouble,
                                      &msg->altitude)) {
      ROS_WARN_STREAM("Resetting key " << kAltitudeKey << " to default.");
      msg->altitude = std_msgs::Float64().data;
    }

  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("XML RPC client threw error: " << e.what());
    return false;  // XML RPC client exception.
  }
  return true;
}

}  // namespace mav_coverage_planning

#endif  // MAV_COVERAGE_PLANNING_ROS_CONVERSIONS_MSG_FROM_XML_RPC_H
