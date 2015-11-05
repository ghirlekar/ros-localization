#include "ros.h"
#include "ros/time.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/Vector3Stamped.h"

#include "Adafruit_BNO055.h"
#include "Adafruit_Sensor.h"
#include "utility/imumaths.h"

SYSTEM_MODE(SEMI_AUTOMATIC)

Adafruit_BNO055 bno = Adafruit_BNO055();
Servo speed_esc, steering_servo;
ros::NodeHandle nh;
uint32_t timestamp = 0, now;

geometry_msgs::QuaternionStamped orientation_msg;
geometry_msgs::Vector3Stamped angular_vel_msg, linear_accel_msg;
ros::Time msg_timestamp;
uint32_t msg_seq = 0;
const char msg_frame_id[] = "base_link";
ros::Publisher pub_orientation("crawler0/orientation", &orientation_msg);
ros::Publisher pub_angular_vel("crawler0/angular_vel", &angular_vel_msg);
ros::Publisher pub_linear_accel("crawler0/linear_accel", &linear_accel_msg);
imu::Quaternion orientation;
imu::Vector<3> linear_accel, angular_vel;

geometry_msgs::Twist cmd_vel_msg;
void cmd_vel_cb(const geometry_msgs::Twist &cmd_vel_msg) {
  speed_esc.write(90 - 90 * cmd_vel_msg.linear.x);
  steering_servo.write(90 + 90 * cmd_vel_msg.angular.z);
}
ros::Subscriber<geometry_msgs::Twist> sub_vel("crawler0/cmd_vel", cmd_vel_cb);

void calibrate_servo(Servo &servo, int minVal, int maxVal) {
  const int calibration_delay = 1000;
  const int calibration_angles[] = {maxVal, minVal, (minVal + maxVal) / 2, (minVal + maxVal) / 2};
  for (int i = 0; i < 4 ; i++) {
    servo.write(calibration_angles[i]);
    delay(calibration_delay);
  }
}


void setup(){
	nh.initNode();
	nh.advertise(pub_orientation);
	nh.advertise(pub_angular_vel);
	nh.advertise(pub_linear_accel);
	nh.subscribe(sub_vel);

	orientation_msg.header.frame_id = msg_frame_id;
	angular_vel_msg.header.frame_id = msg_frame_id;
	linear_accel_msg.header.frame_id = msg_frame_id;
	if(!bno.begin())	nh.logerror("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
	else	nh.loginfo("Connected to BNO055");

	delay(1000);
	bno.setExtCrystalUse(true);

	speed_esc.attach(D2);
	speed_esc.write(90);
	steering_servo.attach(D3);
	calibrate_servo(steering_servo, 0, 180);
	nh.loginfo("Calibrated steering servo");
}

void loop(){
	now = millis();
	if(now - timestamp > 10){
		timestamp = now;
		msg_timestamp = nh.now();

		orientation = bno.getQuat();
		orientation_msg.header.seq = msg_seq;
		orientation_msg.header.stamp = msg_timestamp;
		orientation_msg.quaternion.x = orientation.x();
		orientation_msg.quaternion.y = orientation.y();
		orientation_msg.quaternion.z = orientation.z();
		orientation_msg.quaternion.w = orientation.w();
		pub_orientation.publish(&orientation_msg);

		angular_vel = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
		angular_vel_msg.header.seq = msg_seq;
		angular_vel_msg.header.stamp = msg_timestamp;
		angular_vel_msg.vector.x = angular_vel.x();
		angular_vel_msg.vector.y = angular_vel.y();
		angular_vel_msg.vector.z = angular_vel.z();
		pub_angular_vel.publish(&angular_vel_msg);

		linear_accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
		linear_accel_msg.header.seq = msg_seq;
		linear_accel_msg.header.stamp = msg_timestamp;
		linear_accel_msg.vector.x = linear_accel.x();
		linear_accel_msg.vector.y = linear_accel.y();
		linear_accel_msg.vector.z = linear_accel.z();
		pub_linear_accel.publish(&linear_accel_msg);

		msg_seq++;
	}
	nh.spinOnce();
	delay(1);
}

// #include "ros.h"
// #include "ros/time.h"
// #include "geometry_msgs/Twist.h"
// #include "sensor_msgs/Imu.h"
// #include "std_msgs/String.h"

// #include "Adafruit_BNO055.h"
// #include "Adafruit_Sensor.h"
// #include "utility/imumaths.h"

// SYSTEM_MODE(SEMI_AUTOMATIC)

// Adafruit_BNO055 bno = Adafruit_BNO055();
// // Servo speed_esc, steering_servo;
// ros::NodeHandle nh;
// uint32_t timestamp = 0, now;

// sensor_msgs::Imu imu_msg;
// uint32_t imu_msg_seq = 0;
// const char imu_msg_frame_id[] = "base_link";
// ros::Publisher pub_imu("crawler0/imu", &imu_msg);
// imu::Quaternion quat;
// imu::Vector<3> vec;

// // geometry_msgs::Twist cmd_vel_msg;
// // void cmd_vel_cb(const geometry_msgs::Twist &cmd_vel_msg) {
// //   speed_esc.write(90 - 90 * cmd_vel_msg.linear.x);
// //   steering_servo.write(90 + 90 * cmd_vel_msg.angular.z);
// // }
// // ros::Subscriber<geometry_msgs::Twist> sub_vel("crawler0/cmd_vel", cmd_vel_cb);

// // void calibrate_servo(Servo &servo, int minVal, int maxVal) {
// //   const int calibration_delay = 1000;
// //   const int calibration_angles[] = {maxVal, minVal, (minVal + maxVal) / 2, (minVal + maxVal) / 2};
// //   for (int i = 0; i < 4 ; i++) {
// //     servo.write(calibration_angles[i]);
// //     delay(calibration_delay);
// //   }
// // }


// void setup(){
// 	nh.getHardware()->setBaud(1000000);
// 	nh.initNode();
// 	nh.advertise(pub_imu);
// 	// nh.subscribe(sub_vel);

// 	imu_msg.header.frame_id = imu_msg_frame_id;
// 	if(!bno.begin())	nh.logerror("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
// 	else	nh.loginfo("Connected to BNO055");

// 	delay(1000);
// 	bno.setExtCrystalUse(true);

// 	// speed_esc.attach(D2);
// 	// steering_servo.attach(D3);
// 	// calibrate_servo(steering_servo, 0, 180);
// 	// nh.loginfo("Calibrated steering servo");

// 	while (!nh.connected())	nh.spinOnce();
// }

// void loop(){
// 	now = millis();
// 	if(now - timestamp > 10){
// 		timestamp = now;

// 		imu_msg.header.stamp = nh.now();
// 		imu_msg.header.seq = imu_msg_seq++;

// 		quat = bno.getQuat();
// 		imu_msg.orientation.x = quat.x();
// 		imu_msg.orientation.y = quat.y();
// 		imu_msg.orientation.z = quat.z();
// 		imu_msg.orientation.w = quat.w();

// 		vec = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
// 		imu_msg.linear_acceleration.x = vec.x();
// 		imu_msg.linear_acceleration.y = vec.y();
// 		imu_msg.linear_acceleration.z = vec.z();

// 		vec = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
// 		imu_msg.angular_velocity.x = vec.x();
// 		imu_msg.angular_velocity.y = vec.y();
// 		imu_msg.angular_velocity.z = vec.z();

// 		pub_imu.publish(&imu_msg);
// 	}
// 	nh.spinOnce();
// 	delay(1);
// }