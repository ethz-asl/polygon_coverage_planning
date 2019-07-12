/*
 * polygon_coverage_planning implements algorithms for coverage planning in
 * general polygons with holes. Copyright (C) 2019, Rik Bähnemann, Autonomous
 * Systems Lab, ETH Zürich
 *
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "polygon_coverage_msgs/msg_from_xml_rpc.h"

#include <exception>
#include <string>

#include <geometry_msgs/Point32.h>
#include <ros/assert.h>
#include <ros/console.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>
#include <std_msgs/UInt32.h>

#include <polygon_coverage_msgs/PolygonWithHoles.h>
#include <polygon_coverage_msgs/PolygonWithHolesStamped.h>

namespace polygon_coverage_planning {

// Check if a XML RPC member exists and return an error message if not.
bool hasMember(const XmlRpc::XmlRpcValue& xml_rpc, const std::string& key) {
  if (xml_rpc.hasMember(key)) {
    return true;
  } else {
    ROS_WARN_STREAM("XML RPC has no key: " << key.c_str());
    return false;
  }
}

// Check if a XML RPC value has the correct type and return an error message if
// not.
bool checkType(XmlRpc::XmlRpcValue& xml_rpc, const std::string& key,
               const XmlRpc::XmlRpcValue::Type& expected_type) {
  if (xml_rpc[key].getType() == expected_type) {
    return true;
  } else {
    ROS_ERROR_STREAM("XML RPC expected type: " << expected_type << " is type: "
                                               << xml_rpc[key].getType());
    return false;
  }
}

// Catch special cases that cannot be casted.
// Unsigned int.
bool readElementaryTypeFromXmlRpc(
    XmlRpc::XmlRpcValue& xml_rpc, const std::string& key,
    const XmlRpc::XmlRpcValue::Type& expected_type, unsigned int* result) {
  ROS_ASSERT(result);

  int temp_result = -1;
  if (readElementaryTypeFromXmlRpc(xml_rpc, key, expected_type, &temp_result)) {
    *result = static_cast<unsigned int>(temp_result);
    return true;
  } else {
    return false;
  }
}

// Float.
bool readElementaryTypeFromXmlRpc(
    XmlRpc::XmlRpcValue& xml_rpc, const std::string& key,
    const XmlRpc::XmlRpcValue::Type& expected_type, float* result) {
  ROS_ASSERT(result);

  double temp_result = 0.0;
  if (readElementaryTypeFromXmlRpc(xml_rpc, key, expected_type, &temp_result)) {
    *result = static_cast<float>(temp_result);
    return true;
  } else {
    return false;
  }
}

// Stamp from XML RPC.
bool timeFromXmlRpc(XmlRpc::XmlRpcValue& xml_rpc, ros::Time* t) {
  ROS_ASSERT(t);

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
bool headerMsgFromXmlRpc(XmlRpc::XmlRpcValue& xml_rpc, std_msgs::Header* msg) {
  ROS_ASSERT(msg);

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

// Point32 from XML RPC.
bool point32MsgFromXmlRpc(XmlRpc::XmlRpcValue& xml_rpc,
                          geometry_msgs::Point32* msg) {
  ROS_ASSERT(msg);

  const std::string kXKey = "x";
  const std::string kYKey = "y";
  const std::string kZKey = "z";

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

    // Get z.
    if (!readElementaryTypeFromXmlRpc(
            xml_rpc, kZKey, XmlRpc::XmlRpcValue::TypeDouble, &msg->z)) {
      ROS_WARN_STREAM("Resetting key " << kZKey << " to default.");
      msg->z = std_msgs::Float64().data;
    }
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("XML RPC client threw error: " << e.what());
    return false;  // XML RPC client exception.
  }
  return true;
}

// Polygon from XML RPC.
bool polygonMsgFromXmlRpc(XmlRpc::XmlRpcValue& xml_rpc,
                          geometry_msgs::Polygon* msg) {
  ROS_ASSERT(msg);

  const std::string kPointsKey = "points";

  try {
    // Get polygon vertices.
    if (hasMember(xml_rpc, kPointsKey) &&
        checkType(xml_rpc, kPointsKey, XmlRpc::XmlRpcValue::TypeArray)) {
      msg->points.resize(xml_rpc[kPointsKey].size());
      for (int i = 0; i < xml_rpc[kPointsKey].size(); ++i) {
        geometry_msgs::Point32 p;
        if (!point32MsgFromXmlRpc(xml_rpc[kPointsKey][i], &p)) {
          ROS_WARN_STREAM("Loading point " << i
                                           << " failed. Resetting to default.");
          p = geometry_msgs::Point32();
        }
        msg->points[i] = p;
      }
    } else {
      ROS_WARN_STREAM("Missing or non-array "
                      << kPointsKey << " member. Resetting to default");
      msg->points = std::vector<geometry_msgs::Point32>();
    }
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("XML RPC client threw error: " << e.what());
    return false;  // XML RPC client exception.
  }
  return true;
}

// PolygonWithHoles from XML RPC.
bool polygonWithHolesMsgFromXmlRpc(
    XmlRpc::XmlRpcValue& xml_rpc,
    polygon_coverage_msgs::PolygonWithHoles* msg) {
  ROS_ASSERT(msg);

  const std::string kHullKey = "hull";
  const std::string kHolesKey = "holes";

  try {
    // Get hull.
    if (!hasMember(xml_rpc, kHullKey) ||
        !polygonMsgFromXmlRpc(xml_rpc[kHullKey], &msg->hull)) {
      ROS_WARN_STREAM(
          "Missing " << kHullKey
                     << " key or failed loading polygon. Resetting to default");
      msg->hull = geometry_msgs::Polygon();
    }

    // Get holes.
    if (hasMember(xml_rpc, kHolesKey) &&
        checkType(xml_rpc, kHolesKey, XmlRpc::XmlRpcValue::TypeArray)) {
      msg->holes.resize(xml_rpc[kHolesKey].size());
      for (int i = 0; i < xml_rpc[kHolesKey].size(); ++i) {
        geometry_msgs::Polygon p;
        if (!polygonMsgFromXmlRpc(xml_rpc[kHolesKey][i], &p)) {
          ROS_WARN_STREAM("Loading hole " << i
                                          << " failed. Resetting to default.");
          p = geometry_msgs::Polygon();
        }
        msg->holes[i] = p;
      }
    } else {
      ROS_WARN_STREAM("Missing or non-array "
                      << kHolesKey << " member. Resetting to default");
      msg->holes = std::vector<geometry_msgs::Polygon>();
    }

  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("XML RPC client threw error: " << e.what());
    return false;  // XML RPC client exception.
  }
  return true;
}

// PolygonWithHolesStamped from XML RPC.
bool PolygonWithHolesStampedMsgFromXmlRpc(
    XmlRpc::XmlRpcValue& xml_rpc,
    polygon_coverage_msgs::PolygonWithHolesStamped* msg) {
  ROS_ASSERT(msg);

  const std::string kHeaderKey = "header";
  const std::string kPolygonWithHolesKey = "polygon";

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
      msg->polygon = polygon_coverage_msgs::PolygonWithHoles();
    }

  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("XML RPC client threw error: " << e.what());
    return false;  // XML RPC client exception.
  }
  return true;
}

}  // namespace polygon_coverage_planning
