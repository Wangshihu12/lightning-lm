// SPDX-License-Identifier: BSD-3-Clause
// Minimal, MSVC-friendly implementation of ros::Header methods.
// Matches the interface in <ros/header.h>.

#include <ros/header.h>
#include <boost/make_shared.hpp>
#include <boost/shared_array.hpp>
#include <cstdint>
#include <cstring>
#include <string>

namespace ros {

Header::Header()
{
  // 上游是一个 shared_ptr 指向 map，用于存储解析结果
  read_map_ = boost::make_shared<M_string>();
}

Header::~Header() = default;

// 共享数组版本委托到裸指针版本
bool Header::parse(const boost::shared_array<uint8_t>& buffer, uint32_t size, std::string& error_msg)
{
  return parse(const_cast<uint8_t*>(buffer.get()), size, error_msg);
}

// 解析按 ROS 头格式： [u32 len][bytes "key=value"] 重复直到耗尽
bool Header::parse(uint8_t* buffer, uint32_t size, std::string& error_msg)
{
  read_map_->clear();

  uint32_t pos = 0;
  while (pos < size)
  {
    if (size - pos < 4) {
      error_msg = "Header parse error: not enough bytes for field length";
      return false;
    }

    uint32_t field_len = 0;
    // 小端读取 4 字节长度（ROS1 头是小端）
    std::memcpy(&field_len, buffer + pos, 4);
    pos += 4;

    if (field_len == 0 || field_len > (size - pos)) {
      error_msg = "Header parse error: invalid field length";
      return false;
    }

    const char* data = reinterpret_cast<const char*>(buffer + pos);
    // field 是不带终止符的一段字节，内容为 "key=value"
    std::string line(data, data + field_len);
    pos += field_len;

    std::string::size_type eq = line.find('=');
    if (eq == std::string::npos) {
      error_msg = "Header parse error: missing '=' in field";
      return false;
    }

    std::string key   = line.substr(0, eq);
    std::string value = line.substr(eq + 1);

    // 存入字典
    (*read_map_)[key] = value;
  }

  return true;
}

// 将 key_vals 写为 [u32 len]["key=value"]... 并返回 buffer 与 size
/*static*/ void Header::write(const M_string& key_vals,
                              boost::shared_array<uint8_t>& buffer,
                              uint32_t& size)
{
  // 先计算总长度
  size = 0;
  for (const auto& kv : key_vals) {
    const uint32_t field_len = static_cast<uint32_t>(kv.first.size() + 1 /* '=' */ + kv.second.size());
    size += 4 /*len*/ + field_len;
  }

  buffer.reset(new uint8_t[size]);

  uint32_t pos = 0;
  for (const auto& kv : key_vals) {
    const std::string line = kv.first + "=" + kv.second;
    const uint32_t field_len = static_cast<uint32_t>(line.size());

    std::memcpy(buffer.get() + pos, &field_len, 4);
    pos += 4;

    std::memcpy(buffer.get() + pos, line.data(), field_len);
    pos += field_len;
  }
}

} // namespace ros
