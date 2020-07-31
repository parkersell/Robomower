#include "Arduino.h"
#include "Subscriber.h"
#include <math.h>
#define USE_TEENSY_HW_SERIAL
#define HWSERIAL Serial1

Subscriber::Subscriber(): pose_subscriber("/orb_slam2_stereo/pose", & positionCallback, this), laser_subscriber("/distance", & laserCallback, this) {
  bno = Adafruit_BNO055(55);
}

ros::NodeHandle nh;
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("/imu_data", & imu_msg); //Publisher for IMU topic
unsigned long previousMillis = 0;
unsigned long counter = 0;
const long interval = 10;

//unsigned long lastStreamTime = 0;
//const int streamPeriod = 10;

char odom[] = "/camera/imu";

geometry_msgs::PoseStamped pos;
geometry_msgs::Pose p;
std_msgs::Float32 msg;

std_msgs::String cmd;
double toast = 0; //x
double eggs = 0; //y
//orientation
double fruit = 0; //w
double cereal = 0; //x
double pancakes = 0; //y
double syrup = 0; //z
double heading = 0;

double imuyaw;

float scandistance = 0;

unsigned long settTimer = millis();
unsigned long trackTimer = millis();
double oldtoast = 0;
double oldeggs = 0;
boolean state;

void Subscriber::positionCallback(const geometry_msgs::PoseStamped & pos) {
  toast = pos.pose.position.x;
  eggs = pos.pose.position.y;
  toast = toast * 39.37;
  eggs = eggs * 39.37;
  fruit = pos.pose.orientation.w;
  cereal = pos.pose.orientation.x;
  pancakes = pos.pose.orientation.y;
  syrup = pos.pose.orientation.z;
  heading = getYaw();
}
bool Subscriber::tracking() {

  if (millis() - settTimer > 1000) {
    if (oldtoast == toast && oldeggs == eggs) {
      state = false;
      //HWSERIAL.println("false");
    } else {
      oldtoast = toast;
      oldeggs = eggs;
      state = true;
      // HWSERIAL.println("true");
    }
    settTimer = millis();
    return state;
  } else {
    return state;
  }

}

void Subscriber::laserCallback(const std_msgs::Float32 & msg) {

  //scandistance = 39.37 * (msg.ranges[360]);
  scandistance = msg.data;
  /*
    float[] regions = {
    39.37 * min(msg.ranges[0:143]),
    39.37 * min(msg.ranges[144:287]),
    39.37 * min(msg.ranges[288:431]),
    39.37 * min(msg.ranges[432:575]),
    39.37 * min(msg.ranges[576:713]),

    }*/
  //rospy.loginfo(regions)
}

double Subscriber::getLaser() {
  double laser = scandistance;
  return laser;
}
Point Subscriber::getPosition() {
  Point location = Point(toast, eggs);
  return location;
}

double Subscriber::getRoll() {
  double sinr_cosp = 2 * (fruit * cereal + pancakes * syrup);
  double cosr_cosp = 1 - 2 * (cereal * cereal + pancakes * pancakes);
  double roll = atan2(sinr_cosp, cosr_cosp);
  return roll;
}

double Subscriber::getYaw() {
  double siny_cosp = 2 * (fruit * syrup + cereal * pancakes);
  double cosy_cosp = 1 - 2 * (pancakes * pancakes + syrup * syrup);
  double yaw = atan2(siny_cosp, cosy_cosp);
  yaw = to_degrees(yaw);
  return yaw;
}

void Subscriber::initSLAM() {

  nh.initNode();
  setupIMU(); //Uncomment for IMU topic
  nh.subscribe(pose_subscriber);
  nh.subscribe(laser_subscriber);

}

void Subscriber::spinOnceS() {
  nh.spinOnce();
  //delay(50);//with no delay we get position, but no motor control
  //nh.spin();
}

void Subscriber::setupIMU() {
  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH);
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial1.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");

  }
  bno.setExtCrystalUse(true);
  nh.advertise(imu_pub);
}
double Subscriber::constrainAngle(double x) {
  x = fmod(x + 180, 360);
  if (x < 0)
    x += 360;
  return -(x - 180);
}
double Subscriber::getIMU() {
  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent( & event);
  imuyaw = event.orientation.x;
  imuyaw = constrainAngle(imuyaw);
  return imuyaw;
}

void Subscriber::sendIMU() {
  imu_msg.header.frame_id = odom;
  imu_msg.header.seq = counter;
  imu_msg.header.stamp = nh.now();

  imu::Quaternion quat = bno.getQuat();
  imu::Vector < 3 > angular = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector < 3 > accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  imu_msg.orientation.x = quat.x();
  imu_msg.orientation.y = quat.y();
  imu_msg.orientation.z = quat.z();
  imu_msg.orientation.w = quat.w();

  imu_msg.angular_velocity.x = angular.x();
  imu_msg.angular_velocity.y = angular.y();
  imu_msg.angular_velocity.z = angular.z();

  imu_msg.linear_acceleration.x = accel.x();
  imu_msg.linear_acceleration.y = accel.y();
  imu_msg.linear_acceleration.z = accel.z();

  counter++;

  imu_pub.publish( & imu_msg);
}

void Subscriber::resetIMU() {
  digitalWrite(7, LOW);

  delayMicroseconds(30);

  digitalWrite(7, HIGH);
  delayMicroseconds(30);
  bno.begin();
}
