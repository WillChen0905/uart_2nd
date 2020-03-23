#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <math.h>

#define sBUFFERSIZE 7
unsigned char s_buffer[sBUFFERSIZE];
std::vector<uint8_t> buf;
std::vector<uint8_t> r_buf;

serial::Serial ser;
ros::Publisher uart_pub;
ros::Publisher odom_pub;
ros::Subscriber uart_sub;

int count = 0;

ros::Time current_time, last_time;

float x=0,y=0,th=0;


void data_analysis(std::vector<uint8_t> csum){

    float Acc_x=0,Acc_y=0,Acc_z=0,GX=0,GY=0,GZ=0;
    float Vx=0,Vy=0,W=0;
    float R=0.232,a=0.26,b=0.15;
    float WH1=0,WH2=0,WH3=0,WH4=0;

    sensor_msgs::Imu rec;
    nav_msgs::Odometry odom;
    static tf::TransformBroadcaster odom_broadcaster;


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
            if( csum[13]*256+csum[14] > 32768 ){
                WH1= (float)(csum[13]*256+csum[14]-65536)*6.28*20/352000;
            }else{
                WH1= (float)(csum[13]*256+csum[14])*6.28*20/352000;
           }
            if( csum[15]*256+csum[16] > 32768 ){
                WH2= (float)(csum[15]*256+csum[16]-65536)*6.28*20/352000;
            }else{
                WH2= (float)(csum[15]*256+csum[16])*6.28*20/352000;
            }
            if( csum[17]*256+csum[18] > 32768 ){
                WH3= (float)(csum[17]*256+csum[18]-65536)*6.28*20/352000;
            }else{
                WH3= (float)(csum[17]*256+csum[18])*6.28*20/352000;
            }
            if( csum[19]*256+csum[20] > 32768 ){
                WH4= (float)(csum[19]*256+csum[20]-65536)*6.28*20/352000;
            }else{
                WH4= (float)(csum[19]*256+csum[20])*6.28*20/352000;
            }
            WH1 = WH1 * 1.05;
            WH2 = WH2 * 1.05;
            WH3 = WH3 * 1.05;
            WH4 = WH4 * 1.05;
//            if(fabs(WH1)<0.3) {WH1 = 0;}
//            if(fabs(WH2)<0.3) {WH2 = 0;}
//            if(fabs(WH3)<0.3) {WH3 = 0;}
//            if(fabs(WH4)<0.3) {WH4 = 0;}
//            std::cout << WH1  <<  "    "<< WH2  <<  "    "<< WH3  <<  "    "<< WH4  <<  std::endl;



            ////////////////// IMU //////////////////

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
//            float acc_cov[9] = {1e-5, 0, 0,
//                                0, 1e-5, 0,
//                                0, 0, 1e-5};
            rec.orientation_covariance[0] = -1;
            rec.linear_acceleration_covariance[0] = -1;
            for(int h=0; h<9; h++){
                rec.angular_velocity_covariance[h] = ang_cov[h];
//                rec.linear_acceleration_covariance[h] = acc_cov[h];
            }

            //////////// Odom //////////////

            Vx = 0.25*R*(WH1+WH2+WH3+WH4);
            Vy = 0.25*R*(WH1-WH2-WH3+WH4);
            W = 0.25*R*(WH1-WH2+WH3-WH4)/(a+b);

            current_time = ros::Time::now();

            double dt = (current_time - last_time).toSec();;
            double delta_x = (Vx * cos(th) - Vy * sin(th)) * dt;
            double delta_y = (Vx * sin(th) + Vy * cos(th)) * dt;
            double delta_th = W * dt;




            x += delta_x;
            y += delta_y;
            th += delta_th;

            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = current_time;
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base_footprint";

            odom_trans.transform.translation.x = x;
            odom_trans.transform.translation.y = y;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = odom_quat;

//            odom_broadcaster.sendTransform(odom_trans);

            odom.header.stamp = current_time;
            odom.header.frame_id = "odom";

            //set the position
            odom.pose.pose.position.x = x;
            odom.pose.pose.position.y = y;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation = odom_quat;

            //set the velocity
            odom.child_frame_id = "base_footprint";
            odom.twist.twist.linear.x = Vx;
            odom.twist.twist.linear.y = Vy;
            odom.twist.twist.angular.z = W;


            uart_pub.publish(rec);
            odom_pub.publish(odom);
//            count = count + 1;
//            std::cout << "dt: " << dt << "  dx: " << delta_x << "  Vx: " << Vx << std::endl;
//            std::cout << "round:" << count << std::endl;
            last_time = current_time;
        
  
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
        s_buffer[6] = 0xfe;
        ser.write(s_buffer,sBUFFERSIZE);
    }
}

void cmd_vel_CB(const geometry_msgs::Twist cmd_vel){
//    ROS_INFO("linear velocity: x-[%f],y-[%f]",cmd_vel.linear.x,cmd_vel.linear.y);
//    ROS_INFO("angular velocity: yaw-[%f]",cmd_vel.angular.z);
//    std::cout << "Twist Received" << std::endl;
    data_pack(cmd_vel);
}


int main(int argc, char **argv){
    ros::init(argc, argv, "uart_2nd");

    ros::NodeHandle nh;
    tf::TransformBroadcaster odom_broadcaster;
    uart_pub = nh.advertise<sensor_msgs::Imu>("imu", 1000);
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom",1000);
    uart_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1000,cmd_vel_CB);
    try
        {
            ser.setPort("/dev/ttyACM0");
            ser.setBaudrate(115200);
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
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::Rate loop_rate(10);
//    ros::Duration(5).sleep();
    while (ros::ok()){
        if(ser.available()){
           std::string str = ser.read(ser.available());
           for(int i = 0; i < str.length(); i++)
           {
             buf.push_back(str[i]);
//             printf("%x, ", *buf.rbegin());
             if(*buf.rbegin() == 0x58 && *(buf.rbegin()+1) == 0x55){
                if(buf.size() == 23){
                    if(*(buf.rbegin() + 22) == 0xbb){
                       for(int l=0; l<buf.size(); l++){
                          r_buf.push_back(buf[l]);
                       }
                       data_analysis(r_buf);
                       r_buf.clear();

                    }
                }
//               std::cout << std::endl;
               buf.clear();
             }
           }
        }
    }
}
