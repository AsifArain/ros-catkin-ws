
/* Auto-generated by genmsg_cpp for file /home/victor/ros_workspace/localized_RMLD/msg/rmld_msg.msg */
#ifndef LOCALIZED_RMLD_MESSAGE_RMLD_MSG_H
#define LOCALIZED_RMLD_MESSAGE_RMLD_MSG_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"


namespace localized_RMLD
{
template <class ContainerAllocator>
struct rmld_msg_ {
  typedef rmld_msg_<ContainerAllocator> Type;

  rmld_msg_()
  : concentration_ppmm(0.0)
  , origin_x(0.0)
  , origin_y(0.0)
  , origin_z(0.0)
  , end_x(0.0)
  , end_y(0.0)
  , end_z(0.0)
  , rmld_data_string()
  {
  }

  rmld_msg_(const ContainerAllocator& _alloc)
  : concentration_ppmm(0.0)
  , origin_x(0.0)
  , origin_y(0.0)
  , origin_z(0.0)
  , end_x(0.0)
  , end_y(0.0)
  , end_z(0.0)
  , rmld_data_string(_alloc)
  {
  }

  typedef float _concentration_ppmm_type;
  float concentration_ppmm;

  typedef float _origin_x_type;
  float origin_x;

  typedef float _origin_y_type;
  float origin_y;

  typedef float _origin_z_type;
  float origin_z;

  typedef float _end_x_type;
  float end_x;

  typedef float _end_y_type;
  float end_y;

  typedef float _end_z_type;
  float end_z;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _rmld_data_string_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  rmld_data_string;


private:
  static const char* __s_getDataType_() { return "localized_RMLD/rmld_msg"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "2d0ddf2bb2d78ac356fd7ceb54e858a8"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "float32 concentration_ppmm\n\
float32 origin_x\n\
float32 origin_y\n\
float32 origin_z\n\
float32 end_x\n\
float32 end_y\n\
float32 end_z\n\
string rmld_data_string\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, concentration_ppmm);
    ros::serialization::serialize(stream, origin_x);
    ros::serialization::serialize(stream, origin_y);
    ros::serialization::serialize(stream, origin_z);
    ros::serialization::serialize(stream, end_x);
    ros::serialization::serialize(stream, end_y);
    ros::serialization::serialize(stream, end_z);
    ros::serialization::serialize(stream, rmld_data_string);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, concentration_ppmm);
    ros::serialization::deserialize(stream, origin_x);
    ros::serialization::deserialize(stream, origin_y);
    ros::serialization::deserialize(stream, origin_z);
    ros::serialization::deserialize(stream, end_x);
    ros::serialization::deserialize(stream, end_y);
    ros::serialization::deserialize(stream, end_z);
    ros::serialization::deserialize(stream, rmld_data_string);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(concentration_ppmm);
    size += ros::serialization::serializationLength(origin_x);
    size += ros::serialization::serializationLength(origin_y);
    size += ros::serialization::serializationLength(origin_z);
    size += ros::serialization::serializationLength(end_x);
    size += ros::serialization::serializationLength(end_y);
    size += ros::serialization::serializationLength(end_z);
    size += ros::serialization::serializationLength(rmld_data_string);
    return size;
  }

  typedef boost::shared_ptr< ::localized_RMLD::rmld_msg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::localized_RMLD::rmld_msg_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct rmld_msg
typedef  ::localized_RMLD::rmld_msg_<std::allocator<void> > rmld_msg;

typedef boost::shared_ptr< ::localized_RMLD::rmld_msg> rmld_msgPtr;
typedef boost::shared_ptr< ::localized_RMLD::rmld_msg const> rmld_msgConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::localized_RMLD::rmld_msg_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::localized_RMLD::rmld_msg_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace localized_RMLD

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::localized_RMLD::rmld_msg_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::localized_RMLD::rmld_msg_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::localized_RMLD::rmld_msg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "2d0ddf2bb2d78ac356fd7ceb54e858a8";
  }

  static const char* value(const  ::localized_RMLD::rmld_msg_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x2d0ddf2bb2d78ac3ULL;
  static const uint64_t static_value2 = 0x56fd7ceb54e858a8ULL;
};

template<class ContainerAllocator>
struct DataType< ::localized_RMLD::rmld_msg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "localized_RMLD/rmld_msg";
  }

  static const char* value(const  ::localized_RMLD::rmld_msg_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::localized_RMLD::rmld_msg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float32 concentration_ppmm\n\
float32 origin_x\n\
float32 origin_y\n\
float32 origin_z\n\
float32 end_x\n\
float32 end_y\n\
float32 end_z\n\
string rmld_data_string\n\
\n\
";
  }

  static const char* value(const  ::localized_RMLD::rmld_msg_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::localized_RMLD::rmld_msg_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.concentration_ppmm);
    stream.next(m.origin_x);
    stream.next(m.origin_y);
    stream.next(m.origin_z);
    stream.next(m.end_x);
    stream.next(m.end_y);
    stream.next(m.end_z);
    stream.next(m.rmld_data_string);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct rmld_msg_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::localized_RMLD::rmld_msg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::localized_RMLD::rmld_msg_<ContainerAllocator> & v) 
  {
    s << indent << "concentration_ppmm: ";
    Printer<float>::stream(s, indent + "  ", v.concentration_ppmm);
    s << indent << "origin_x: ";
    Printer<float>::stream(s, indent + "  ", v.origin_x);
    s << indent << "origin_y: ";
    Printer<float>::stream(s, indent + "  ", v.origin_y);
    s << indent << "origin_z: ";
    Printer<float>::stream(s, indent + "  ", v.origin_z);
    s << indent << "end_x: ";
    Printer<float>::stream(s, indent + "  ", v.end_x);
    s << indent << "end_y: ";
    Printer<float>::stream(s, indent + "  ", v.end_y);
    s << indent << "end_z: ";
    Printer<float>::stream(s, indent + "  ", v.end_z);
    s << indent << "rmld_data_string: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.rmld_data_string);
  }
};


} // namespace message_operations
} // namespace ros

#endif // LOCALIZED_RMLD_MESSAGE_RMLD_MSG_H

