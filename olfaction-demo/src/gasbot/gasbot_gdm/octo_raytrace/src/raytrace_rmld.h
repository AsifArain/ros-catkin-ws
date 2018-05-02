/* Auto-generated by genmsg_cpp for file /home/victor/groovy_workspace/sandbox/octo_raytrace/msg/raytrace_rmld.msg */
#ifndef OCTO_RAYTRACE_MESSAGE_RAYTRACE_RMLD_H
#define OCTO_RAYTRACE_MESSAGE_RAYTRACE_RMLD_H
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


namespace octo_raytrace
{
template <class ContainerAllocator>
struct raytrace_rmld_ {
  typedef raytrace_rmld_<ContainerAllocator> Type;

  raytrace_rmld_()
  : startX(0.0)
  , startY(0.0)
  , startZ(0.0)
  , endX(0.0)
  , endY(0.0)
  , endZ(0.0)
  , ppmm(0.0)
  {
  }

  raytrace_rmld_(const ContainerAllocator& _alloc)
  : startX(0.0)
  , startY(0.0)
  , startZ(0.0)
  , endX(0.0)
  , endY(0.0)
  , endZ(0.0)
  , ppmm(0.0)
  {
  }

  typedef float _startX_type;
  float startX;

  typedef float _startY_type;
  float startY;

  typedef float _startZ_type;
  float startZ;

  typedef float _endX_type;
  float endX;

  typedef float _endY_type;
  float endY;

  typedef float _endZ_type;
  float endZ;

  typedef float _ppmm_type;
  float ppmm;


  typedef boost::shared_ptr< ::octo_raytrace::raytrace_rmld_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::octo_raytrace::raytrace_rmld_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct raytrace_rmld
typedef  ::octo_raytrace::raytrace_rmld_<std::allocator<void> > raytrace_rmld;

typedef boost::shared_ptr< ::octo_raytrace::raytrace_rmld> raytrace_rmldPtr;
typedef boost::shared_ptr< ::octo_raytrace::raytrace_rmld const> raytrace_rmldConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::octo_raytrace::raytrace_rmld_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::octo_raytrace::raytrace_rmld_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace octo_raytrace

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::octo_raytrace::raytrace_rmld_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::octo_raytrace::raytrace_rmld_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::octo_raytrace::raytrace_rmld_<ContainerAllocator> > {
  static const char* value() 
  {
    return "fc2bde0ec68eb095a15b4032843ae8dc";
  }

  static const char* value(const  ::octo_raytrace::raytrace_rmld_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xfc2bde0ec68eb095ULL;
  static const uint64_t static_value2 = 0xa15b4032843ae8dcULL;
};

template<class ContainerAllocator>
struct DataType< ::octo_raytrace::raytrace_rmld_<ContainerAllocator> > {
  static const char* value() 
  {
    return "octo_raytrace/raytrace_rmld";
  }

  static const char* value(const  ::octo_raytrace::raytrace_rmld_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::octo_raytrace::raytrace_rmld_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float32 startX\n\
float32 startY\n\
float32 startZ\n\
float32 endX\n\
float32 endY\n\
float32 endZ\n\
float32 ppmm\n\
\n\
\n\
\n\
";
  }

  static const char* value(const  ::octo_raytrace::raytrace_rmld_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::octo_raytrace::raytrace_rmld_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::octo_raytrace::raytrace_rmld_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.startX);
    stream.next(m.startY);
    stream.next(m.startZ);
    stream.next(m.endX);
    stream.next(m.endY);
    stream.next(m.endZ);
    stream.next(m.ppmm);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct raytrace_rmld_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::octo_raytrace::raytrace_rmld_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::octo_raytrace::raytrace_rmld_<ContainerAllocator> & v) 
  {
    s << indent << "startX: ";
    Printer<float>::stream(s, indent + "  ", v.startX);
    s << indent << "startY: ";
    Printer<float>::stream(s, indent + "  ", v.startY);
    s << indent << "startZ: ";
    Printer<float>::stream(s, indent + "  ", v.startZ);
    s << indent << "endX: ";
    Printer<float>::stream(s, indent + "  ", v.endX);
    s << indent << "endY: ";
    Printer<float>::stream(s, indent + "  ", v.endY);
    s << indent << "endZ: ";
    Printer<float>::stream(s, indent + "  ", v.endZ);
    s << indent << "ppmm: ";
    Printer<float>::stream(s, indent + "  ", v.ppmm);
  }
};


} // namespace message_operations
} // namespace ros

#endif // OCTO_RAYTRACE_MESSAGE_RAYTRACE_RMLD_H

