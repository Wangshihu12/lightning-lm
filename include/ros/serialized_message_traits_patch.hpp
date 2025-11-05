// 修补：为 ros::SerializedMessage 提供 message_traits 特化，避免要求 __s_getMD5Sum 等成员。
#pragma once
#include <ros/serialized_message.h>
#include <ros/message_traits.h>

namespace ros {
namespace message_traits {

template<>
struct MD5Sum<ros::SerializedMessage> {
    static const char* value() { return "*"; }
    static const char* value(const ros::SerializedMessage&) { return value(); }
};

template<>
struct DataType<ros::SerializedMessage> {
    static const char* value() { return "*"; }
    static const char* value(const ros::SerializedMessage&) { return value(); }
};

template<>
struct Definition<ros::SerializedMessage> {
    static const char* value() { return "serialized message"; }
    static const char* value(const ros::SerializedMessage&) { return value(); }
};

// SerializedMessage 本身不是标准消息，不带 Header
template<>
struct HasHeader<ros::SerializedMessage> : FalseType {};

} // namespace message_traits
} // namespace ros


// --- 追加：为 SerializedMessage 提供 Serializer 特化 ---
// --- 适配 IStream: getData() / getLength() / advance() ---
// include/ros/serialized_message_traits_patch.hpp
#include <ros/serialization.h>
#include <cstring>

namespace ros {
namespace serialization {

template<>
struct Serializer<ros::SerializedMessage> {
  template<typename Stream>
  inline static void read(Stream& stream, ros::SerializedMessage& m) {
    const uint32_t n = stream.getLength();  // 这里的 stream 已是“限长子流”
    boost::shared_array<uint8_t> buf(new uint8_t[n]);
    std::memcpy(buf.get(), stream.getData(), n);
    // ★ 不在这里 advance，上层已经用 msg_len 推进了父流
    m.buf       = buf;
    m.num_bytes = n;
    // 如有 message_start 字段则设置：m.message_start = m.buf.get();
  }

  template<typename Stream>
  inline static void write(Stream&, const ros::SerializedMessage&) {}
  inline static uint32_t serializedLength(const ros::SerializedMessage& m) {
    return static_cast<uint32_t>(m.num_bytes);
  }
};

} // namespace serialization
} // namespace ros
