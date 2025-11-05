#include <ros/serialization.h>
#include <ros/time.h>
#include <ros/message_traits.h>

namespace ros {
namespace serialization {

// 辅助：读取 string / vector / array 会自动调用 Serializer 特化

template<>
struct Serializer<msgs::Time> {
    template<typename Stream>
    inline static void read(Stream& stream, msgs::Time& v) {
        stream.next(v.sec);
        stream.next(v.nsec);
    }
    inline static uint32_t serializedLength(const msgs::Time&) { return 8; }
    template<typename Stream>
    inline static void write(Stream& stream, const msgs::Time& v) {
        stream.next(v.sec); stream.next(v.nsec);
    }
};

template<>
struct Serializer<msgs::Header> {
    template<typename Stream>
    inline static void read(Stream& stream, msgs::Header& v) {
        stream.next(v.seq);
        stream.next(v.stamp);
        stream.next(v.frame_id); // string: uint32 len + bytes
    }
    inline static uint32_t serializedLength(const msgs::Header& v) {
        return 4 + 8 + 4 + (uint32_t)v.frame_id.size();
    }
    template<typename Stream>
    inline static void write(Stream& stream, const msgs::Header& v) {
        stream.next(v.seq);
        stream.next(v.stamp);
        stream.next(v.frame_id);
    }
};

template<>
struct Serializer<msgs::CustomPoint> {
    template<typename Stream>
    inline static void read(Stream& s, msgs::CustomPoint& p) {
        s.next(p.offset_time);
        s.next(p.x); s.next(p.y); s.next(p.z);
        s.next(p.reflectivity);
        s.next(p.tag);
        s.next(p.line);
    }
    inline static uint32_t serializedLength(const msgs::CustomPoint&) { return 4+4*3+1+1+1; }
    template<typename Stream>
    inline static void write(Stream& s, const msgs::CustomPoint& p) {
        s.next(p.offset_time);
        s.next(p.x); s.next(p.y); s.next(p.z);
        s.next(p.reflectivity); s.next(p.tag); s.next(p.line);
    }
};

template<>
struct Serializer<msgs::CustomMsg> {
    template<typename Stream>
    inline static void read(Stream& s, msgs::CustomMsg& m) {
        s.next(m.header);
        s.next(m.timebase);
        s.next(m.point_num);
        s.next(m.lidar_id);
        for (int i = 0; i < 3; ++i) {
            s.next(m.rsvd[i]);
        }
        s.next(m.points);
    }
    inline static uint32_t serializedLength(const msgs::CustomMsg& m) {
        return 0; 
    }
    template<typename Stream>
    inline static void write(Stream& s, const msgs::CustomMsg& m) {
        s.next(m.header);
        s.next(m.timebase);
        s.next(m.point_num);
        s.next(m.lidar_id);
        for (int i = 0; i < 3; ++i) {
            s.next(m.rsvd[i]);
        }
        s.next(m.points);
    }
};

template<>
struct Serializer<msgs::Quaterniond> {
    template<typename Stream>
    inline static void read(Stream& s, msgs::Quaterniond& q) {
        s.next(q.x); s.next(q.y); s.next(q.z); s.next(q.w);
    }
    inline static uint32_t serializedLength(const msgs::Quaterniond&) { return 8*4; }
    template<typename Stream>
    inline static void write(Stream& s, const msgs::Quaterniond& q) {
        s.next(q.x); s.next(q.y); s.next(q.z); s.next(q.w);
    }
};

template<>
struct Serializer<msgs::Vector3d> {
    template<typename Stream>
    inline static void read(Stream& s, msgs::Vector3d& v) {
        s.next(v.x); s.next(v.y); s.next(v.z);
    }
    inline static uint32_t serializedLength(const msgs::Vector3d&) { return 8*3; }
    template<typename Stream>
    inline static void write(Stream& s, const msgs::Vector3d& v) {
        s.next(v.x); s.next(v.y); s.next(v.z);
    }
};

template<>
struct Serializer<msgs::Imu> {
    template<typename Stream>
    inline static void read(Stream& s, msgs::Imu& m) {
        s.next(m.header);
        s.next(m.orientation);
        for (double& d : m.orientation_covariance) s.next(d);
        s.next(m.angular_velocity);
        for (double& d : m.angular_velocity_covariance) s.next(d);
        s.next(m.linear_acceleration);
        for (double& d : m.linear_acceleration_covariance) s.next(d);
    }
    inline static uint32_t serializedLength(const msgs::Imu&) { return 0; }
    template<typename Stream>
    inline static void write(Stream& s, const msgs::Imu& m) {
        s.next(m.header);
        s.next(m.orientation);
        for (const double& d : m.orientation_covariance) s.next(d);
        s.next(m.angular_velocity);
        for (const double& d : m.angular_velocity_covariance) s.next(d);
        s.next(m.linear_acceleration);
        for (const double& d : m.linear_acceleration_covariance) s.next(d);
    }
};

} // namespace serialization
} // namespace ros
