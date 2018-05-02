/* Auto-generated by genmsg_cpp for file /home/administrator/ros/chem_nodes/rmld_node/msg/rmld_msg.msg */
#ifndef RMLD_NODE_MESSAGE_RMLD_MSG_H
#define RMLD_NODE_MESSAGE_RMLD_MSG_H
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


namespace rmld_node
{
template <class ContainerAllocator>
struct rmld_msg_ {
  typedef rmld_msg_<ContainerAllocator> Type;

  rmld_msg_()
  : concentration_ppmm(0.0)
  , rmld_data_string()
  {
  }

  rmld_msg_(const ContainerAllocator& _alloc)
  : concentration_ppmm(0.0)
  , rmld_data_string(_alloc)
  {
  }

  typedef float _concentration_ppmm_type;
  float concentration_ppmm;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _rmld_data_string_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  rmld_data_string;


  typedef boost::shared_ptr< ::rmld_node::rmld_msg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rmld_node::rmld_msg_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct rmld_msg
typedef  ::rmld_node::rmld_msg_<std::allocator<void> > rmld_msg;

typedef boost::shared_ptr< ::rmld_node::rmld_msg> rmld_msgPtr;
typedef boost::shared_ptr< ::rmld_node::rmld_msg const> rmld_msgConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::rmld_node::rmld_msg_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::rmld_node::rmld_msg_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace rmld_node

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::rmld_node::rmld_msg_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::rmld_node::rmld_msg_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::rmld_node::rmld_msg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "1e11ef0acc09eeffee4c91b316534192";
  }

  static const char* value(const  ::rmld_node::rmld_msg_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x1e11ef0acc09eeffULL;
  static const uint64_t static_value2 = 0xee4c91b316534192ULL;
};

template<class ContainerAllocator>
struct DataType< ::rmld_node::rmld_msg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "rmld_node/rmld_msg";
  }

  static const char* value(const  ::rmld_node::rmld_msg_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::rmld_node::rmld_msg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float32 concentration_ppmm\n\
string rmld_data_string\n\
\n\
";
  }

  static const char* value(const  ::rmld_node::rmld_msg_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::rmld_node::rmld_msg_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.concentration_ppmm);
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
struct Printer< ::rmld_node::rmld_msg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::rmld_node::rmld_msg_<ContainerAllocator> & v) 
  {
    s << indent << "concentration_ppmm: ";
    Printer<float>::stream(s, indent + "  ", v.concentration_ppmm);
    s << indent << "rmld_data_string: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.rmld_data_string);
  }
};


} // namespace message_operations
} // namespace ros

#endif // RMLD_NODE_MESSAGE_RMLD_MSG_H

