 /* Auto-generated by genmsg_cpp for file /home/rosbuild/hudson/workspace/doc-electric-object_manipulation/doc_stacks/2013-03-01_16-13-18.345538/object_manipulation/compressed_pointcloud_transport/msg/CompressedPointCloud.msg */
#ifndef COMPRESSED_POINTCLOUD_TRANSPORT_MESSAGE_COMPRESSEDPOINTCLOUD_H
#define COMPRESSED_POINTCLOUD_TRANSPORT_MESSAGE_COMPRESSEDPOINTCLOUD_H
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

#include "std_msgs/Header.h"
 
namespace compressed_pointcloud_transport
{
template <class ContainerAllocator>
 struct CompressedPointCloud_ {
   typedef CompressedPointCloud_<ContainerAllocator> Type;
 
   CompressedPointCloud_()
   : header()
   , data()
   {
   }
 
   CompressedPointCloud_(const ContainerAllocator& _alloc)
   : header(_alloc)
   , data(_alloc)
   {
   }
 
   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
    ::std_msgs::Header_<ContainerAllocator>  header;
 
   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _data_type;
   std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  data;
 
 
 private:
   static const char* __s_getDataType_() { return "compressed_pointcloud_transport/CompressedPointCloud"; }
 public:
   ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }
 
   ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }
 
 private:
   static const char* __s_getMD5Sum_() { return "c99a9440709e4d4a9716d55b8270d5e7"; }
 public:
   ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }
 
   ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }
 
 private:
   static const char* __s_getMessageDefinition_() { return "Header header\n\
 string data\n\
 \n\
 \n\
 ================================================================================\n\
00064 MSG: std_msgs/Header\n\
00065 # Standard metadata for higher-level stamped data types.\n\
00066 # This is generally used to communicate timestamped data \n\
00067 # in a particular coordinate frame.\n\
00068 # \n\
00069 # sequence ID: consecutively increasing ID \n\
00070 uint32 seq\n\
00071 #Two-integer timestamp that is expressed as:\n\
00072 # * stamp.secs: seconds (stamp_secs) since epoch\n\
00073 # * stamp.nsecs: nanoseconds since stamp_secs\n\
00074 # time-handling sugar is provided by the client library\n\
00075 time stamp\n\
00076 #Frame this data is associated with\n\
00077 # 0: no frame\n\
00078 # 1: global frame\n\
00079 string frame_id\n\
00080 \n\
00081 "; }
 public:
   ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }
 
   ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }
 
   ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
   {
     ros::serialization::OStream stream(write_ptr, 1000000000);
     ros::serialization::serialize(stream, header);
     ros::serialization::serialize(stream, data);
     return stream.getData();
   }
 
   ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
   {
     ros::serialization::IStream stream(read_ptr, 1000000000);
     ros::serialization::deserialize(stream, header);
     ros::serialization::deserialize(stream, data);
     return stream.getData();
   }
 
   ROS_DEPRECATED virtual uint32_t serializationLength() const
   {
     uint32_t size = 0;
     size += ros::serialization::serializationLength(header);
     size += ros::serialization::serializationLength(data);
     return size;
   }
 
   typedef boost::shared_ptr< ::compressed_pointcloud_transport::CompressedPointCloud_<ContainerAllocator> > Ptr;
   typedef boost::shared_ptr< ::compressed_pointcloud_transport::CompressedPointCloud_<ContainerAllocator>  const> ConstPtr;
   boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
 }; // struct CompressedPointCloud
 typedef  ::compressed_pointcloud_transport::CompressedPointCloud_<std::allocator<void> > CompressedPointCloud;
 
 typedef boost::shared_ptr< ::compressed_pointcloud_transport::CompressedPointCloud> CompressedPointCloudPtr;
 typedef boost::shared_ptr< ::compressed_pointcloud_transport::CompressedPointCloud const> CompressedPointCloudConstPtr;
 
 
 template<typename ContainerAllocator>
 std::ostream& operator<<(std::ostream& s, const  ::compressed_pointcloud_transport::CompressedPointCloud_<ContainerAllocator> & v)
 {
   ros::message_operations::Printer< ::compressed_pointcloud_transport::CompressedPointCloud_<ContainerAllocator> >::stream(s, "", v);
   return s;}
 
 } // namespace compressed_pointcloud_transport
 
 namespace ros
 {
 namespace message_traits
 {
 template<class ContainerAllocator> struct IsMessage< ::compressed_pointcloud_transport::CompressedPointCloud_<ContainerAllocator> > : public TrueType {};
 template<class ContainerAllocator> struct IsMessage< ::compressed_pointcloud_transport::CompressedPointCloud_<ContainerAllocator>  const> : public TrueType {};
 template<class ContainerAllocator>
 struct MD5Sum< ::compressed_pointcloud_transport::CompressedPointCloud_<ContainerAllocator> > {
   static const char* value() 
   {
     return "c99a9440709e4d4a9716d55b8270d5e7";
   }
 
   static const char* value(const  ::compressed_pointcloud_transport::CompressedPointCloud_<ContainerAllocator> &) { return value(); } 
   static const uint64_t static_value1 = 0xc99a9440709e4d4aULL;
   static const uint64_t static_value2 = 0x9716d55b8270d5e7ULL;
 };
 
 template<class ContainerAllocator>
 struct DataType< ::compressed_pointcloud_transport::CompressedPointCloud_<ContainerAllocator> > {
   static const char* value() 
   {
     return "compressed_pointcloud_transport/CompressedPointCloud";
   }
 
   static const char* value(const  ::compressed_pointcloud_transport::CompressedPointCloud_<ContainerAllocator> &) { return value(); } 
 };
 
 template<class ContainerAllocator>
 struct Definition< ::compressed_pointcloud_transport::CompressedPointCloud_<ContainerAllocator> > {
   static const char* value() 
   {
     return "Header header\n\
 string data\n\
 \n\
 \n\
 ================================================================================\n\
00166 MSG: std_msgs/Header\n\
00167 # Standard metadata for higher-level stamped data types.\n\
00168 # This is generally used to communicate timestamped data \n\
00169 # in a particular coordinate frame.\n\
00170 # \n\
00171 # sequence ID: consecutively increasing ID \n\
00172 uint32 seq\n\
00173 #Two-integer timestamp that is expressed as:\n\
00174 # * stamp.secs: seconds (stamp_secs) since epoch\n\
00175 # * stamp.nsecs: nanoseconds since stamp_secs\n\
00176 # time-handling sugar is provided by the client library\n\
00177 time stamp\n\
00178 #Frame this data is associated with\n\
00179 # 0: no frame\n\
00180 # 1: global frame\n\
00181 string frame_id\n\
00182 \n\
00183 ";
   }
 
   static const char* value(const  ::compressed_pointcloud_transport::CompressedPointCloud_<ContainerAllocator> &) { return value(); } 
 };
 
 template<class ContainerAllocator> struct HasHeader< ::compressed_pointcloud_transport::CompressedPointCloud_<ContainerAllocator> > : public TrueType {};
 template<class ContainerAllocator> struct HasHeader< const ::compressed_pointcloud_transport::CompressedPointCloud_<ContainerAllocator> > : public TrueType {};
 } // namespace message_traits
 } // namespace ros
 
 namespace ros
 {
 namespace serialization
 {
 
 template<class ContainerAllocator> struct Serializer< ::compressed_pointcloud_transport::CompressedPointCloud_<ContainerAllocator> >
 {
   template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
   {
     stream.next(m.header);
     stream.next(m.data);
   }
 
   ROS_DECLARE_ALLINONE_SERIALIZER;
 }; // struct CompressedPointCloud_
 } // namespace serialization
 } // namespace ros
 
 namespace ros
 {
 namespace message_operations
 {
 
 template<class ContainerAllocator>
 struct Printer< ::compressed_pointcloud_transport::CompressedPointCloud_<ContainerAllocator> >
 {
   template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::compressed_pointcloud_transport::CompressedPointCloud_<ContainerAllocator> & v) 
   {
     s << indent << "header: ";
 s << std::endl;
     Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
     s << indent << "data: ";
     Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.data);
   }
 };
 
 
 } // namespace message_operations
 } // namespace ros
 
 #endif // COMPRESSED_POINTCLOUD_TRANSPORT_MESSAGE_COMPRESSEDPOINTCLOUD_H
