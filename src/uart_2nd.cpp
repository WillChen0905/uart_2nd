#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <math.h>

#define rBUFFERSIZE 30
#define sBUFFERSIZE 6
unsigned char r_buffer[rBUFFERSIZE];
unsigned char s_buffer[sBUFFERSIZE];
unsigned char ret=0;

serial::Serial ser;
ros::Publisher uart_pub;
ros::Subscriber uart_sub;

void data_analysis(unsigned char *buffer){
    std::vector<uint8_t> csum;
    float Acc_x=0,Acc_y=0,Acc_z=0,GX=0,GY=0,GZ=0;
    float WH1=0,WH2=0,WH3=0,WH4=0;
    sensor_msgs::Imu rec;
    for(int i=0; i<rBUFFERSIZE; i++){
        if(buffer[i]==0xff && buffer[i+17]==0xfe){
            csum.clear();
            for(int j=0; j<18; j++){
                csum.push_back(buffer[i+j]);
            }
            if(csum[1]*256+csum[2] > 32768){
                Acc_x = (float)(csum[1]*256+csum[2]-65536);
            }else{
                Acc_x = (float)(csum[1]*256+csum[2]);
            }
            if(csum[3]*256+csum[4] > 32768){
                Acc_y= (float)(csum[3]*256+csum[4]-65536);
            }else{
                Acc_y = (float)(csum[3]*256+csum[4]);
            }
            if(csum[5]*256+csum[6] > 32768){
                Acc_z = (float)(csum[5]*256+csum[6]-65536);
            }else{
                Acc_z = (float)(csum[5]*256+csum[6]);
            }
	    if(csum[7]*256+csum[8] > 32768){
                GX = (float)(csum[7]*256+csum[8]-65536);
            }else{
                GX = (float)(csum[7]*256+csum[8]);
            }
            if(csum[9]*256+csum[10] > 32768){
                GY = (float)(csum[9]*256+csum[10]-65536);
            }else{
                GY = (float)(csum[9]*256+csum[10]);
            }
            if(csum[11]*256+csum[12] > 32768){
                GZ = (float)(csum[11]*256+csum[12]-65536);
            }else{
                GZ = (float)(csum[11]*256+csum[12]);
            }

            WH1=csum[13]/10;
            WH2=csum[14]/10;
            WH3=csum[15]/10;
            WH4=csum[16]/10;
            rec.angular_velocity.x = GX*3.14159/1800;
            rec.angular_velocity.y = GY*3.14159/1800;
            rec.angular_velocity.z = GZ*3.14159/1800;
            rec.linear_acceleration.x = Acc_x/1000;
            rec.linear_acceleration.y = Acc_y/1000;
            rec.linear_acceleration.z = Acc_z/1000;
            rec.header.stamp = ros::Time::now();
            rec.header.frame_id = "IMU_link";
            
            float ang_cov[9] = {1e5, 0, 0,
                                0, 1e5, 0,
                                0, 0, 1e-5};
            float acc_cov[9] = {1e-5, 0, 0,
                                0, 1e-5, 0,
                                0, 0, 1e-5};
            rec.orientation_covariance[0] = -1;
            for(int h=0; h<9; h++){
                rec.angular_velocity_covariance[h] = ang_cov[h];
                rec.linear_acceleration_covariance[h] = acc_cov[h];
            }
            uart_pub.publish(rec);
            i=i+18;
        }
    }
}

void data_pack(const geometry_msgs::Twist cmd_vel){
    float  Vx,Vy,Ang_v;
    uint8_t Ox,Oy,Oz;
    Vx = cmd_vel.linear.x*100;
    Vy = cmd_vel.linear.y*100;
    Ang_v = cmd_vel.angular.z*100;
    if(fabs(Vx)<256 && fabs(Vy)<256 && fabs(Ang_v)<256){
        memset(s_buffer,0,sBUFFERSIZE);
        Vx < 0 ? Ox = 0x04 : Ox = 0x00;
        Vy < 0 ? Oy = 0x02 : Oy = 0x00;
        Ang_v < 0 ? Oz = 0x01 : Oz = 0x00;
        s_buffer[0] = 0xff;
        s_buffer[1] = fabs(Vx);
        s_buffer[2] = fabs(Vy);
        s_buffer[3] = fabs(Ang_v);
        s_buffer[4] = Ox+Oy+Oz;
        s_buffer[5] = 0xfe;
        ser.write(s_buffer,sBUFFERSIZE);
    }
}

void cmd_vel_CB(const geometry_msgs::Twist cmd_vel){
    ROS_INFO("linear velocity: x-[%f],y-[%f]",cmd_vel.linear.x,cmd_vel.linear.y);
    ROS_INFO("angular velocity: yaw-[%f]",cmd_vel.angular.z);
    std::cout << "Twist Received" << std::endl;
    data_pack(cmd_vel);
}


int main(int argc, char **argv){
    ros::init(argc, argv, "uart_2nd");
    ros::NodeHandle nh;
    uart_pub = nh.advertise<sensor_msgs::Imu>("imu", 1000);
    uart_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1000,cmd_vel_CB);
    try
        {
            ser.setPort("/dev/ttyACM0");
            ser.setBaudrate(57600);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser.setTimeout(to);
            ser.open();
        }
        catch (serial::IOException& e)
        {
            ROS_ERROR_STREAM("Unable to open port ");
            return -1;
        }

        if(ser.isOpen()){
            ROS_INFO_STREAM("Serial Port initialized");
        }else{
            return -1;
        }

    ros::Rate loop_rate(10);
    while (ros::ok()){
        ros::spinOnce();
        if(ser.available()){
           ROS_INFO_STREAM("Reading from serial port");
            ser.read(r_buffer,rBUFFERSIZE);
            data_analysis(r_buffer);
        }
    }
}
