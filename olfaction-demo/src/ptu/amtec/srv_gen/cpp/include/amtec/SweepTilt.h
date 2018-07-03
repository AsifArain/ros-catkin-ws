/* Auto-generated by genmsg_cpp for file /home/administrator/ros/amtec/srv/SweepTilt.srv */
#ifndef AMTEC_SERVICE_SWEEPTILT_H
#define AMTEC_SERVICE_SWEEPTILT_H
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

#include "ros/service_traits.h"




namespace amtec
{
template <class ContainerAllocator>
struct SweepTiltRequest_ {
  typedef SweepTiltRequest_<ContainerAllocator> Type;

  SweepTiltRequest_()
  : sweep_amplitude(0.0)
  , sweep_period(0.0)
  {
  }

  SweepTiltRequest_(const ContainerAllocator& _alloc)
  : sweep_amplitude(0.0)
  , sweep_period(0.0)
  {
  }

  typedef double _sweep_amplitude_type;
  double sweep_amplitude;

  typedef double _sweep_period_type;
  double sweep_period;


  typedef boost::shared_ptr< ::amtec::SweepTiltRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::amtec::SweepTiltRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct SweepTiltRequest
typedef  ::amtec::SweepTiltRequest_<std::allocator<void> > SweepTiltRequest;

typedef boost::shared_ptr< ::amtec::SweepTiltRequest> SweepTiltRequestPtr;
typedef boost::shared_ptr< ::amtec::SweepTiltRequest const> SweepTiltRequestConstPtr;


template <class ContainerAllocator>
struct SweepTiltResponse_ {
  typedef SweepTiltResponse_<ContainerAllocator> Type;

  SweepTiltResponse_()
  {
  }

  SweepTiltResponse_(const ContainerAllocator& _alloc)
  {
  }


  typedef boost::shared_ptr< ::amtec::SweepTiltResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::amtec::SweepTiltResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct SweepTiltResponse
typedef  ::amtec::SweepTiltResponse_<std::allocator<void> > SweepTiltResponse;

typedef boost::shared_ptr< ::amtec::SweepTiltResponse> SweepTiltResponsePtr;
typedef boost::shared_ptr< ::amtec::SweepTiltResponse const> SweepTiltResponseConstPtr;

struct SweepTilt
{

typedef SweepTiltRequest Request;
typedef SweepTiltResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct SweepTilt
} // namespace amtec

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::amtec::SweepTiltRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::amtec::SweepTiltRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::amtec::SweepTiltRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "5ecc0a29ab6ceff25a8c7df356aada72";
  }

  static const char* value(const  ::amtec::SweepTiltRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x5ecc0a29ab6ceff2ULL;
  static const uint64_t static_value2 = 0x5a8c7df356aada72ULL;
};

template<class ContainerAllocator>
struct DataType< ::amtec::SweepTiltRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "amtec/SweepTiltRequest";
  }

  static const char* value(const  ::amtec::SweepTiltRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::amtec::SweepTiltRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float64 sweep_amplitude\n\
float64 sweep_period\n\
\n\
";
  }

  static const char* value(const  ::amtec::SweepTiltRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::amtec::SweepTiltRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::amtec::SweepTiltResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::amtec::SweepTiltResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::amtec::SweepTiltResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::amtec::SweepTiltResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::amtec::SweepTiltResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "amtec/SweepTiltResponse";
  }

  static const char* value(const  ::amtec::SweepTiltResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::amtec::SweepTiltResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
";
  }

  static const char* value(const  ::amtec::SweepTiltResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::amtec::SweepTiltResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::amtec::SweepTiltRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.sweep_amplitude);
    stream.next(m.sweep_period);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SweepTiltRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::amtec::SweepTiltResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SweepTiltResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<amtec::SweepTilt> {
  static const char* value() 
  {
    return "5ecc0a29ab6ceff25a8c7df356aada72";
  }

  static const char* value(const amtec::SweepTilt&) { return value(); } 
};

template<>
struct DataType<amtec::SweepTilt> {
  static const char* value() 
  {
    return "amtec/SweepTilt";
  }

  static const char* value(const amtec::SweepTilt&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<amtec::SweepTiltRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "5ecc0a29ab6ceff25a8c7df356aada72";
  }

  static const char* value(const amtec::SweepTiltRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<amtec::SweepTiltRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "amtec/SweepTilt";
  }

  static const char* value(const amtec::SweepTiltRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<amtec::SweepTiltResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "5ecc0a29ab6ceff25a8c7df356aada72";
  }

  static const char* value(const amtec::SweepTiltResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<amtec::SweepTiltResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "amtec/SweepTilt";
  }

  static const char* value(const amtec::SweepTiltResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // AMTEC_SERVICE_SWEEPTILT_H
