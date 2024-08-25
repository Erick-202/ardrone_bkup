#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <algorithm>

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

public:
    ImageConverter()
        : it_(nh_)
    {
        image_sub_=it_.subscribe("/cv_camera/image_raw",1,&ImageConverter::imageCb,this); //subscribe to webcam image
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



    cv::Mat HSVImage,gray,dst;
    cv::cvtColor(cv_ptr->image,HSVImage,CV_BGR2HSV); //HSV
    cv::cvtColor(cv_ptr->image,gray,CV_BGR2GRAY);
    cv::Size size = HSVImage.size();
    CvMat * A = cvCreateMat(size.height,size.width,CV_8UC1);
    cv::Mat mask = cv::cvarrToMat(A);

//the color in this tutorial is bright yellow (tennis ball), the color range should be adjusted according to your object
    cv::inRange(HSVImage,cv::Scalar(20,100,100),cv::Scalar(30, 255, 255),mask); //detecting colors

    //morphological closing (fill small holes in the foreground
    dilate(mask,mask,getStructuringElement(MORPH_RECT,Size(21,21)));
    erode(mask, mask, getStructuringElement(MORPH_RECT,Size(5,5)));

    //opening
    erode(mask,mask,getStructuringElement(MORPH_RECT,Size(11,11)));
    dilate(mask,mask,getStructuringElement(MORPH_RECT,Size(5,5)));

    GaussianBlur(mask, mask, Size(15,15), 2, 2);
    std::vector<Vec3f> circles;
    imshow("filter",mask);

    HoughCircles(mask,circles,CV_HOUGH_GRADIENT,2.5,20,100);

    float x,y,r;
    for(size_t i=0;i<circles.size();i++)
        {
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius=cvRound(circles[i][2]);
            circle(cv_ptr->image, center, 3, Scalar(0,255,0), -1, 8, 0);
            circle(cv_ptr->image, center, radius, Scalar(0,0,255), 3, 8, 0);
            x=circles[i][0];
            y=circles[i][1];
            r=circles[i][2];
        }


        if (estado == 1) {
            bool temp1, temp2; //help variables
            //cvInitFont(CV_FONT_HERSHEY_SIMPLEX, 1.0 ,1.0, 0.0, 1, 20);
            cv::putText(cv_ptr->image, "Armado", cvPoint(20, 30), CV_FONT_HERSHEY_SIMPLEX, 1,cvScalar(0,255,0),3);
            cout << "Width : " << cv_ptr->image.size().width << endl;
            cout << "Height: " << cv_ptr->image.size().height << endl;

            if (circles.size() > 0) {
                //printf("detect \n");

                if (r < 40 && circles.size() > 0) {
                    forwardx();
                    //printf("forward \n");
                    temp1 = true;
                }
                if (r > 60) {
                    backx();
                    //printf("back \n");
                    temp1 = true;
                }
                if (r < 60 && r > 40) {
                    if (temp1 == true) {
                        stop();
                        //printf("stop \n");
                        temp1 = false;
                    }
                    if (y < 200 && circles.size() > 0) //go up
                    {
                        up();
                        //printf("up \n");
                        temp2 = true;
                    }
                    if (y > 280 && circles.size() > 0) //go down
                    {
                        down();
                        //printf("down \n");
                        temp2 = true;
                    }
                    if (y < 280 && y > 200) {
                        if (temp2 == true) {
                            stop();
                            //printf("stop \n");
                            temp2 = false;
                        }
                        if (x > 360 && circles.size() > 0) {
                            backy();
                            //printf("right \n");
                        }
                        if (x < 280 && circles.size() > 0) {
                            forwardy();
                            //printf("left \n");
                        }
                        if (x < 360 && x > 280) {
                            stop();
                            //printf("stop \n");
                        }
                    }


                }
            } else if (circles.size() == 0) {
                stop();
            }

        }

        else{
            cv::putText(cv_ptr->image, "DesArmado", cvPoint(20, 30), CV_FONT_HERSHEY_SIMPLEX, 1,cvScalar(0,0,255),3);
        }
        //in my case, the resolution of camera is 640x480, the range of radius of cicle detected and the region of left, right, up and down sides should be adjusted according to your camera properties
        imshow("detect",cv_ptr->image);
    }
};

int main (int argc, char** argv)
{
    ros::init(argc, argv, "drone");
    ros::NodeHandle n;

    ros::init(argc,argv,"image_converter");
    ImageConverter ic;

    topictakeoff=n.advertise<std_msgs::Empty>("/ardrone/takeoff",1,true);
    topiclanding=n.advertise<std_msgs::Empty>("/ardrone/land",1,true);
    cmd_vel=n.advertise<geometry_msgs::Twist>("/cmd_vel",1,true);
    serviceflattrim=n.serviceClient<std_srvs::Empty>("/ardrone/flattrim");
    ajuste();
    printf("Calibration\n");
    //takeoff();

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
                cout<<"Sistema Armado" << endl;
                //takeoff();
            }
            else if(waitKey(1)==113){ // q
                ic.estado = 0;
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


                
    
