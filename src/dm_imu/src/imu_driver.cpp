#include "dm_imu/imu_driver.h"
#include <chrono>

using namespace std::chrono_literals;

namespace dmbot_serial
{
// 初始化父类 Node，并声明节点名称为 dm_imu_node
DmImu::DmImu() : Node("dm_imu_node")
{
  // 声明并获取参数
  this->declare_parameter<std::string>("port", "/dev/dm_imu");
  this->declare_parameter<int>("baud", 921600);
  
  imu_serial_port = this->get_parameter("port").as_string();
  imu_seial_baud = this->get_parameter("baud").as_int();
                 
  imu_msgs.header.frame_id = "imu_link";

  init_imu_serial();

  // 创建发布者
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 2);
  imu_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 100);
  euler2_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("dm/imu", 9);

  // 替换延时函数
  enter_setting_mode();
  rclcpp::sleep_for(10ms);

  turn_on_accel();
  rclcpp::sleep_for(10ms);

  turn_on_gyro();
  rclcpp::sleep_for(10ms);

  turn_on_euler();
  rclcpp::sleep_for(10ms);

  turn_off_quat();
  rclcpp::sleep_for(10ms);

  set_output_1000HZ();
  rclcpp::sleep_for(10ms);

  save_imu_para();
  rclcpp::sleep_for(10ms);
  
  exit_setting_mode();
  rclcpp::sleep_for(100ms);

  rec_thread = std::thread(&DmImu::get_imu_data_thread, this);

  RCLCPP_INFO(this->get_logger(), "imu init complete");
}

DmImu::~DmImu()
{ 
  RCLCPP_INFO(this->get_logger(), "enter ~DmImu()");
  
  stop_thread_ = true;
  
  if(rec_thread.joinable())
  {
    rec_thread.join(); 
  }

  if (serial_imu.isOpen())
  {
    serial_imu.close(); 
  } 
}

void DmImu::init_imu_serial()
{         
    try
    {
      serial_imu.setPort(imu_serial_port);
      serial_imu.setBaudrate(imu_seial_baud);
      serial_imu.setFlowcontrol(serial::flowcontrol_none);
      serial_imu.setParity(serial::parity_none);
      serial_imu.setStopbits(serial::stopbits_one);
      serial_imu.setBytesize(serial::eightbits);
      serial::Timeout time_out = serial::Timeout::simpleTimeout(20);
      serial_imu.setTimeout(time_out);
      serial_imu.open();
    } 
    catch (serial::IOException &e) 
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "In initialization, Unable to open imu serial port");
        exit(0);
    }
    if (serial_imu.isOpen())
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "In initialization, Imu Serial Port initialized");
    }
    else
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "In initialization, Unable to open imu serial port");
        exit(0);
    }
}

// === 下面这部分发送指令的代码只需替换延时 ===
void DmImu::enter_setting_mode() {
  uint8_t txbuf[4]={0xAA,0x06,0x01,0x0D};
  for(int i=0;i<5;i++){ serial_imu.write(txbuf,sizeof(txbuf)); rclcpp::sleep_for(10ms); }
}
void DmImu::turn_on_accel() {
  uint8_t txbuf[4]={0xAA,0x01,0x14,0x0D};
  for(int i=0;i<5;i++){ serial_imu.write(txbuf,sizeof(txbuf)); rclcpp::sleep_for(10ms); }
}
void DmImu::turn_on_gyro() {
  uint8_t txbuf[4]={0xAA,0x01,0x15,0x0D};
  for(int i=0;i<5;i++){ serial_imu.write(txbuf,sizeof(txbuf)); rclcpp::sleep_for(10ms); }
}
void DmImu::turn_on_euler() {
  uint8_t txbuf[4]={0xAA,0x01,0x16,0x0D};
  for(int i=0;i<5;i++){ serial_imu.write(txbuf,sizeof(txbuf)); rclcpp::sleep_for(10ms); }
}
void DmImu::turn_off_quat() {
  uint8_t txbuf[4]={0xAA,0x01,0x07,0x0D};
  for(int i=0;i<5;i++){ serial_imu.write(txbuf,sizeof(txbuf)); rclcpp::sleep_for(10ms); }
}
void DmImu::set_output_1000HZ() {
  uint8_t txbuf[5]={0xAA,0x02,0x01,0x00,0x0D};
  for(int i=0;i<5;i++){ serial_imu.write(txbuf,sizeof(txbuf)); rclcpp::sleep_for(10ms); }
}
void DmImu::save_imu_para() {
  uint8_t txbuf[4]={0xAA,0x03,0x01,0x0D};
  for(int i=0;i<5;i++){ serial_imu.write(txbuf,sizeof(txbuf)); rclcpp::sleep_for(10ms); }
}
void DmImu::exit_setting_mode() {
  uint8_t txbuf[4]={0xAA,0x06,0x00,0x0D};
  for(int i=0;i<5;i++){ serial_imu.write(txbuf,sizeof(txbuf)); rclcpp::sleep_for(10ms); }
}
void DmImu::restart_imu() {
  uint8_t txbuf[4]={0xAA,0x00,0x00,0x0D};
  for(int i=0;i<5;i++){ serial_imu.write(txbuf,sizeof(txbuf)); rclcpp::sleep_for(10ms); }
}

void DmImu::get_imu_data_thread()
{ 
  int error_num=0;
  // ros::ok() 改为 rclcpp::ok()
  while (rclcpp::ok() && !stop_thread_)
  {    
    if (!serial_imu.isOpen())
    {
      RCLCPP_WARN(this->get_logger(), "In get_imu_data_thread, imu serial port unopen");
    }       

    serial_imu.read((uint8_t*)(&receive_data.FrameHeader1),4);
    if(receive_data.FrameHeader1==0x55&&receive_data.flag1==0xAA&&receive_data.slave_id1==0x01&&receive_data.reg_acc==0x01)
    {
      serial_imu.read((uint8_t*)(&receive_data.accx_u32),57-4); 

      if(Get_CRC16((uint8_t*)(&receive_data.FrameHeader1), 16)==receive_data.crc1)
      {
        data.accx =*((float *)(&receive_data.accx_u32));
        data.accy =*((float *)(&receive_data.accy_u32));
        data.accz =*((float *)(&receive_data.accz_u32));
      }
      if(Get_CRC16((uint8_t*)(&receive_data.FrameHeader2), 16)==receive_data.crc2)
      {
        data.gyrox =*((float *)(&receive_data.gyrox_u32));
        data.gyroy =*((float *)(&receive_data.gyroy_u32));
        data.gyroz =*((float *)(&receive_data.gyroz_u32));
      }
      if(Get_CRC16((uint8_t*)(&receive_data.FrameHeader3), 16)==receive_data.crc3)
      { 
        data.roll =*((float *)(&receive_data.roll_u32));
        data.pitch =*((float *)(&receive_data.pitch_u32));
        data.yaw =*((float *)(&receive_data.yaw_u32));
      }
      
        // 获取系统当前时间
        imu_msgs.header.stamp = this->get_clock()->now();
  
        // ROS 2 中使用 tf2 将欧拉角转为四元数
        tf2::Quaternion q;
        q.setRPY((data.roll)/ 180.0 * M_PI, data.pitch/ 180.0 * M_PI, data.yaw/ 180.0 * M_PI);
        imu_msgs.orientation.w = q.w();
        imu_msgs.orientation.x = q.x();
        imu_msgs.orientation.y = q.y();
        imu_msgs.orientation.z = q.z();

        imu_msgs.angular_velocity.x = data.gyrox;
        imu_msgs.angular_velocity.y = data.gyroy;
        imu_msgs.angular_velocity.z = data.gyroz;
    
        imu_msgs.linear_acceleration.x = data.accx;
        imu_msgs.linear_acceleration.y = data.accy;
        imu_msgs.linear_acceleration.z = data.accz;
    
        imu_pub_->publish(imu_msgs);

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "imu_link";
        pose.header.stamp = imu_msgs.header.stamp;
        pose.pose.position.x = 0.0;
        pose.pose.position.y = 0.0;
        pose.pose.position.z = 0.0;
        pose.pose.orientation = imu_msgs.orientation; // 直接赋值转换好的四元数
        
        imu_pose_pub_->publish(pose);

        std_msgs::msg::Float64MultiArray euler2_msg;
        euler2_msg.data.resize(9);
        euler2_msg.data[0]=data.gyrox;
        euler2_msg.data[1]=data.gyroy;
        euler2_msg.data[2]=data.gyroz;
        euler2_msg.data[3]=data.accx;
        euler2_msg.data[4]=data.accy;
        euler2_msg.data[5]=data.accz;
        euler2_msg.data[6]=data.roll/ 180.0 * M_PI;
        euler2_msg.data[7]=data.pitch/ 180.0 * M_PI;
        euler2_msg.data[8]=data.yaw/ 180.0 * M_PI;
        //euler2_pub_->publish(euler2_msg);
    }
    else
    { 
      error_num++;
      if(error_num>1200)
      {
        std::cerr<<"fail to get the correct imu data, finding header 0x55"<<std::endl;
      }
    }
  }
}

}
