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

#ifndef POLYGON_COVERAGE_MSGS_MSG_FROM_XML_RPC_H_
#define POLYGON_COVERAGE_MSGS_MSG_FROM_XML_RPC_H_
#include <string>

#include <geometry_msgs/Point32.h>
#include <ros/assert.h>
#include <ros/console.h>
#include <std_msgs/Header.h>
#include <std_msgs/Time.h>
#include <xmlrpcpp/XmlRpc.h>

#include <polygon_coverage_msgs/PolygonWithHoles.h>
#include <polygon_coverage_msgs/PolygonWithHolesStamped.h>

namespace polygon_coverage_planning {

// Check if a XML RPC member exists and return an error message if not.
bool hasMember(const XmlRpc::XmlRpcValue& xml_rpc, const std::string& key);

// Check if a XML RPC value has the correct type and return an error message if
// not.
bool checkType(XmlRpc::XmlRpcValue& xml_rpc, const std::string& key,
               const XmlRpc::XmlRpcValue::Type& expected_type);

// Read an elementary type from an XML RPC.
template <typename T>
inline bool readElementaryTypeFromXmlRpc(
    XmlRpc::XmlRpcValue& xml_rpc, const std::string& key,
    const XmlRpc::XmlRpcValue::Type& expected_type, T* result) {
  ROS_ASSERT(result);

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
bool readElementaryTypeFromXmlRpc(
    XmlRpc::XmlRpcValue& xml_rpc, const std::string& key,
    const XmlRpc::XmlRpcValue::Type& expected_type, unsigned int* result);

// Float.
bool readElementaryTypeFromXmlRpc(
    XmlRpc::XmlRpcValue& xml_rpc, const std::string& key,
    const XmlRpc::XmlRpcValue::Type& expected_type, float* result);

// Stamp from XML RPC.
bool timeFromXmlRpc(XmlRpc::XmlRpcValue& xml_rpc, ros::Time* t);

// Header from XML RPC.
bool headerMsgFromXmlRpc(XmlRpc::XmlRpcValue& xml_rpc, std_msgs::Header* msg);

// Point32 from XML RPC.
bool point32MsgFromXmlRpc(XmlRpc::XmlRpcValue& xml_rpc,
                          geometry_msgs::Point32* msg);

// Polygon from XML RPC.
bool polygonMsgFromXmlRpc(XmlRpc::XmlRpcValue& xml_rpc,
                          geometry_msgs::Polygon* msg);

// PolygonWithHoles from XML RPC.
bool polygonWithHolesMsgFromXmlRpc(
    XmlRpc::XmlRpcValue& xml_rpc, polygon_coverage_msgs::PolygonWithHoles* msg);

// PolygonWithHolesStamped from XML RPC.
bool PolygonWithHolesStampedMsgFromXmlRpc(
    XmlRpc::XmlRpcValue& xml_rpc,
    polygon_coverage_msgs::PolygonWithHolesStamped* msg);

}  // namespace polygon_coverage_planning

#endif  // POLYGON_COVERAGE_MSGS_MSG_FROM_XML_RPC_H_
