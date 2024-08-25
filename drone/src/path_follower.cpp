#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <algorithm>
#include <iostream>
#include <string>
#include <sstream>

using namespace std;
using namespace cv;

//---------------------  Object Tracking  ---------------------------------------

//static const std::string OPENCV_WINDOW = "Image window";

ros::Publisher topictakeoff;
ros::Publisher topiclanding;
ros::ServiceClient serviceflattrim;
ros::Publisher cmd_vel;
ros::Publisher topicinfo;
float longitud, latitud, altitud;


geometry_msgs::Twist changeTwist(float x, float y, float z, float turn){
    geometry_msgs::Twist msg_vel;
    msg_vel.angular.x=0;
    msg_vel.angular.y=0;
    msg_vel.angular.z=turn;
    msg_vel.linear.x=x;
    msg_vel.linear.y=y;
    msg_vel.linear.z=z;
    return(msg_vel);
}

void ajuste (void){
    std_srvs::Empty srvflattrim;
    serviceflattrim.call(srvflattrim);
}

void takeoff (void){
    std_msgs::Empty empty;
    geometry_msgs::Twist msg_vel;
    topictakeoff.publish(empty);
    printf("Starting...\n");
    usleep(250000);
    printf("hovering..\n");
    msg_vel=changeTwist(0,0,0,0);
    cmd_vel.publish(msg_vel);
}

void land (void){
    std_msgs::Empty empty;
    topiclanding.publish(empty);
}

//forward
void forwardx (void){
    geometry_msgs::Twist msg_vel;
    msg_vel=changeTwist(0.7,0,0,0);
    cmd_vel.publish(msg_vel);

}
//left
void forwardy (void){
    geometry_msgs::Twist msg_vel;
    msg_vel=changeTwist(0,0.7,0,0);
    cmd_vel.publish(msg_vel);
}
//backward
void backx (void){
    geometry_msgs::Twist msg_vel;
    msg_vel=changeTwist(-0.7,0,0,0);
    cmd_vel.publish(msg_vel);
}
//right
void backy (void){
    geometry_msgs::Twist msg_vel;
    msg_vel=changeTwist(0,-0.7,0,0);
    cmd_vel.publish(msg_vel);
}
//up
void up (void){
    geometry_msgs::Twist msg_vel;
    msg_vel=changeTwist(0,0,0.7,0);
    cmd_vel.publish(msg_vel);
}
//down
void down (void){
    geometry_msgs::Twist msg_vel;
    msg_vel=changeTwist(0,0,-0.7,0);
    cmd_vel.publish(msg_vel);
}
void stop (void){
    geometry_msgs::Twist msg_vel;
    msg_vel=changeTwist(0,0,0,0);
    cmd_vel.publish(msg_vel);
}

//Rotate Clock Wise
void turnRight (void){
    geometry_msgs::Twist msg_vel;
    msg_vel=changeTwist(0,0,0,0.7);
    cmd_vel.publish(msg_vel);
}

//Totate Counter Clock Wise
void turnLeft (void){
    geometry_msgs::Twist msg_vel;
    msg_vel=changeTwist(0,0,0,-0.7);
    cmd_vel.publish(msg_vel);
}

void manual_control (int ascii_num){
    if (ascii_num == 105){ forwardx(); }        //i -> forward
    else if (ascii_num == 106){ forwardy(); }   //j -> left
    else if (ascii_num == 107){ backx(); }      //k ->  back
    else if (ascii_num == 108){ backy();}       //l -> right
    else if (ascii_num == 121){ up();}          //y -> up
    else if (ascii_num == 104){ down();}        //h -> down
    else if (ascii_num == 100){ takeoff();}     //d -> takeoff
    else if (ascii_num == 32){ stop();}         //space -> stop
    else if (ascii_num == 115) { land();}       //s -> land
    else if (ascii_num == 117) { turnRight();}  //u -> Turn RIght
    else if (ascii_num == 111) { turnLeft();}   //o -> Turn Left
    else{
        //cout << 'Key Not Found' << endl;
    }
    ajuste();
}

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Subscriber image_sub_depth;
    image_transport::Subscriber image_sub_threshold;
    image_transport::Publisher image_pub_;
    image_transport::Publisher image_pub_depth;
    image_transport::Publisher image_pub_threshold;


public:
    int estado = 0;
    double angulo = 0.0;
    int cam_w = 640;
    int cam_h = 480;

public:
    ImageConverter()
            : it_(nh_)
    {
        image_sub_=it_.subscribe("/ardrone/front/image_raw",1,&ImageConverter::imageCb,this); //subscribe to webcam image
        //cv::namedWindow(OPENCV_WINDOW);
        image_pub_=it_.advertise("/image_converter/output_video",1);
    }

    ~ImageConverter()
    {
        //cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }


        if (estado == 1) {
            cv::putText(cv_ptr->image, "Armado", cvPoint(20, 30), CV_FONT_HERSHEY_SIMPLEX, 1, cvScalar(0, 255, 0), 3);

        }
        else{
            cv::putText(cv_ptr->image, "Desarmado", cvPoint(20, 30), CV_FONT_HERSHEY_SIMPLEX, 1,cvScalar(0,0,255),3);
        }
        //in my case, the resolution of camera is 640x480, the range of radius of cicle detected and the region of left, right, up and down sides should be adjusted according to your camera properties
        imshow("detect",cv_ptr->image);
    }


    void horizontalCallback(const std_msgs::Float64::ConstPtr& msg) {
        double dist_horiz = msg->data;
        cout << dist_horiz << endl;
        if (estado == 1){
            if (dist_horiz < (cam_w/2) - 100){
                //Left
                cout << "Go LEFT" << endl;
                forwardy();
            }
            else if(dist_horiz > (cam_w/2) + 100){
                //Right
                cout << "Go Right" << endl;
                backy();
            }
            else{
                cout << "Piola" << endl;
            }
        }
    }

    void angularCallback(const std_msgs::Float64::ConstPtr& msg){
        double angle = msg->data;
        //ROS_INFO("I heard: [%s]", msg->data.c_str());
        //cout << msg->data.c_str() << endl;
        //std::stringstream ss(angle);
        cout << angle << endl;
        if (estado == 1){
            //Angulo en Rango Aceptable
            if (abs(angle) > 80.0 && abs(angle) <90.0){
                forwardx();
            }
            //Angulo Negativo -> Rotar derecha
            else if (angle < 0){
                cout << "Turn Right" << endl;
                turnRight();

            }
                //SI el ANgulo es Positivo, girar a la izquierda
            else {
                cout << "Turn LEFT" << endl;
                turnLeft();
            }
        }
    }
};

int main (int argc, char** argv)
{
    ros::init(argc, argv, "drone");
    ros::NodeHandle n;

    ros::init(argc,argv,"image_converter");
    ImageConverter ic;

    //Utilizar una función miembro de la clase ImageConverter como callback. Esto me permite colocar la callback function DENTRO de la clase IC
    ros::Subscriber angle_sub = n.subscribe("angulo", 10, &ImageConverter::angularCallback, &ic);
    ros::Subscriber horiz_sub = n.subscribe("align_horiz", 10, &ImageConverter::horizontalCallback, &ic);

    topictakeoff=n.advertise<std_msgs::Empty>("/ardrone/takeoff",1,true);
    topiclanding=n.advertise<std_msgs::Empty>("/ardrone/land",1,true);
    cmd_vel=n.advertise<geometry_msgs::Twist>("/cmd_vel",1,true);
    serviceflattrim=n.serviceClient<std_srvs::Empty>("/ardrone/flattrim");

    ajuste();
    printf("Calibration\n");
    //takeoff();

    int state = 0;

    while(1)
    {
        ros::spinOnce();

        if (waitKey(1)==27)
        {
            cout<<"esc key is pressed by user" << endl;
            land();
            break;
        }
        else if(waitKey(1)==97){ // a
            ic.estado = 1;
            state = 1;
            cout<<"Sistema Armado" << endl;
            //takeoff();
        }
        else if(waitKey(1)==113){ // q
            ic.estado = 0;
            state = 0;
            cout<<"DesArmado" << endl;
            //takeoff();
        }

        else if(waitKey(1)) { //El valor por default de waitkey es -1
            //cout<< waitKey(1) << endl;
            //ic.estado = 0;
            manual_control(waitKey(1));
        }

    }

    return 0;

}


//
// Created by erick on 14/04/24.
//

