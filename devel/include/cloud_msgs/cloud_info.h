// Generated by gencpp from file cloud_msgs/cloud_info.msg
// DO NOT EDIT!


#ifndef CLOUD_MSGS_MESSAGE_CLOUD_INFO_H
#define CLOUD_MSGS_MESSAGE_CLOUD_INFO_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace cloud_msgs
{
template <class ContainerAllocator>
struct cloud_info_
{
  typedef cloud_info_<ContainerAllocator> Type;

  cloud_info_()
    : header()
    , startRingIndex()
    , endRingIndex()
    , startOrientation(0.0)
    , endOrientation(0.0)
    , orientationDiff(0.0)
    , segmentedCloudGroundFlag()
    , segmentedCloudColInd()
    , segmentedCloudRange()  {
    }
  cloud_info_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , startRingIndex(_alloc)
    , endRingIndex(_alloc)
    , startOrientation(0.0)
    , endOrientation(0.0)
    , orientationDiff(0.0)
    , segmentedCloudGroundFlag(_alloc)
    , segmentedCloudColInd(_alloc)
    , segmentedCloudRange(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _startRingIndex_type;
  _startRingIndex_type startRingIndex;

   typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _endRingIndex_type;
  _endRingIndex_type endRingIndex;

   typedef float _startOrientation_type;
  _startOrientation_type startOrientation;

   typedef float _endOrientation_type;
  _endOrientation_type endOrientation;

   typedef float _orientationDiff_type;
  _orientationDiff_type orientationDiff;

   typedef std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  _segmentedCloudGroundFlag_type;
  _segmentedCloudGroundFlag_type segmentedCloudGroundFlag;

   typedef std::vector<uint32_t, typename ContainerAllocator::template rebind<uint32_t>::other >  _segmentedCloudColInd_type;
  _segmentedCloudColInd_type segmentedCloudColInd;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _segmentedCloudRange_type;
  _segmentedCloudRange_type segmentedCloudRange;





  typedef boost::shared_ptr< ::cloud_msgs::cloud_info_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cloud_msgs::cloud_info_<ContainerAllocator> const> ConstPtr;

}; // struct cloud_info_

typedef ::cloud_msgs::cloud_info_<std::allocator<void> > cloud_info;

typedef boost::shared_ptr< ::cloud_msgs::cloud_info > cloud_infoPtr;
typedef boost::shared_ptr< ::cloud_msgs::cloud_info const> cloud_infoConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cloud_msgs::cloud_info_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cloud_msgs::cloud_info_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace cloud_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'nav_msgs': ['/opt/ros/kinetic/share/nav_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'cloud_msgs': ['/home/kang/dev/VI_Loam/src/LeGO-LOAM/cloud_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::cloud_msgs::cloud_info_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cloud_msgs::cloud_info_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cloud_msgs::cloud_info_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cloud_msgs::cloud_info_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cloud_msgs::cloud_info_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cloud_msgs::cloud_info_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cloud_msgs::cloud_info_<ContainerAllocator> >
{
  static const char* value()
  {
    return "af8fdf3af62b4ae75761d0e92aa4cf43";
  }

  static const char* value(const ::cloud_msgs::cloud_info_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xaf8fdf3af62b4ae7ULL;
  static const uint64_t static_value2 = 0x5761d0e92aa4cf43ULL;
};

template<class ContainerAllocator>
struct DataType< ::cloud_msgs::cloud_info_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cloud_msgs/cloud_info";
  }

  static const char* value(const ::cloud_msgs::cloud_info_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cloud_msgs::cloud_info_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header \n\
\n\
int32[] startRingIndex\n\
int32[] endRingIndex\n\
\n\
float32 startOrientation\n\
float32 endOrientation\n\
float32 orientationDiff\n\
\n\
bool[]    segmentedCloudGroundFlag\n\
uint32[]  segmentedCloudColInd \n\
float32[] segmentedCloudRange\n\
\n\
\n\
\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::cloud_msgs::cloud_info_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cloud_msgs::cloud_info_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.startRingIndex);
      stream.next(m.endRingIndex);
      stream.next(m.startOrientation);
      stream.next(m.endOrientation);
      stream.next(m.orientationDiff);
      stream.next(m.segmentedCloudGroundFlag);
      stream.next(m.segmentedCloudColInd);
      stream.next(m.segmentedCloudRange);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct cloud_info_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cloud_msgs::cloud_info_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cloud_msgs::cloud_info_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "startRingIndex[]" << std::endl;
    for (size_t i = 0; i < v.startRingIndex.size(); ++i)
    {
      s << indent << "  startRingIndex[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.startRingIndex[i]);
    }
    s << indent << "endRingIndex[]" << std::endl;
    for (size_t i = 0; i < v.endRingIndex.size(); ++i)
    {
      s << indent << "  endRingIndex[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.endRingIndex[i]);
    }
    s << indent << "startOrientation: ";
    Printer<float>::stream(s, indent + "  ", v.startOrientation);
    s << indent << "endOrientation: ";
    Printer<float>::stream(s, indent + "  ", v.endOrientation);
    s << indent << "orientationDiff: ";
    Printer<float>::stream(s, indent + "  ", v.orientationDiff);
    s << indent << "segmentedCloudGroundFlag[]" << std::endl;
    for (size_t i = 0; i < v.segmentedCloudGroundFlag.size(); ++i)
    {
      s << indent << "  segmentedCloudGroundFlag[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.segmentedCloudGroundFlag[i]);
    }
    s << indent << "segmentedCloudColInd[]" << std::endl;
    for (size_t i = 0; i < v.segmentedCloudColInd.size(); ++i)
    {
      s << indent << "  segmentedCloudColInd[" << i << "]: ";
      Printer<uint32_t>::stream(s, indent + "  ", v.segmentedCloudColInd[i]);
    }
    s << indent << "segmentedCloudRange[]" << std::endl;
    for (size_t i = 0; i < v.segmentedCloudRange.size(); ++i)
    {
      s << indent << "  segmentedCloudRange[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.segmentedCloudRange[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // CLOUD_MSGS_MESSAGE_CLOUD_INFO_H
