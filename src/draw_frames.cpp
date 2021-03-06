#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include  <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>
#include  <radaresr15/radaresr15DataArray.h>
#include  <beginner_tutorials/Num.h>
#include  <radaresr15/radaresr15Data.h>
#include <geometry_msgs/PointStamped.h>
#include <opencv2/highgui/highgui.hpp>
#include  <string>
#include  <iostream>
#include <fstream>

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

class FrameDrawer {
    //    CvHaarClassifierCascade *cascade;
    //    CvMemStorage *storage;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::CameraSubscriber sub_;
    ros::Subscriber subradar_;
    radaresr15::radaresr15Data data_;
    radaresr15::radaresr15DataArray::ConstPtr radarDataArray_;
    image_transport::Publisher pub_;

    tf::TransformListener tf_listener_;
    image_geometry::PinholeCameraModel cam_model_;
    //    std::vector<std::string> frame_ids_;
    int i;

    std::string s;

    CvFont font_;
    int radarMsgLength_;

public:

    FrameDrawer(const std::vector<std::string>& frame_ids)
    : it_(nh_)//, frame_ids_(frame_ids) 
    {
        subradar_ = nh_.subscribe("radar15_msg_array", 1000, &FrameDrawer::radarCallback, this);
        std::string image_topic = nh_.resolveName("image");
        sub_ = it_.subscribeCamera(image_topic, 1, &FrameDrawer::imageCb, this);
        pub_ = it_.advertise("image_out", 1);
        cvInitFont(&font_, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);
        i = 0;




    }

    void imageCb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg) {

        cv::Mat image;

        cv_bridge::CvImagePtr input_bridge;
        //        cv_bridge::CvImagePtr input_bridge2;
        std::ofstream myfile;
        try {
            input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
            //            input_bridge2 = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
            image = input_bridge->image;

            std::stringstream out;
            out << i;
            s = out.str();




            char filename[255]; // final file name
            char* namePt1 = "/home/luis/catkin_ws/imageswrite/image";
            //            int num = 5;
            sprintf(filename, "%s%d.txt", namePt1, i);
            std::ofstream fileLabyrinth(filename);




            myfile.open(filename);
            //            myfile.open("/home/luis/catkin_ws/imageswrite/image" + (unsigned char) filename + ".txt");


            cv::imwrite("/home/luis/catkin_ws/imageswrite/image" + s + ".png", image);

            i++;

            //            image2 = input_bridge2->image;
        } catch (cv_bridge::Exception& ex) {
            ROS_ERROR("[draw_frames] Failed to convert image");
            return;
        }

        cam_model_.fromCameraInfo(info_msg);


        for (int i = 0; i < radarDataArray_->tracks.size(); ++i) {
            //
            data_ = radarDataArray_->tracks[i];
            //we'll create a point in the base_radar(viene de radaresr(podria usarse radar esr envez de base_radar)) frame that we'd like to transform to the /bumblebee frame
            geometry_msgs::PointStamped radar_point;
            radar_point.header.frame_id = "radaresr";

            //we'll just use the most recent transform available for our simple example
            radar_point.header.stamp = ros::Time();

            radar_point.point.x = data_.y_track;
            radar_point.point.y = -data_.x_track;
            radar_point.point.z = 0.0;


            geometry_msgs::PointStamped bumblebee_point;
            try {

                tf_listener_.transformPoint(cam_model_.tfFrame(), radar_point, bumblebee_point);

            } catch (tf::TransformException& ex) {
                ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
            }

            cv::Point3d pt_cv(bumblebee_point.point.x, bumblebee_point.point.y, bumblebee_point.point.z);

            cv::Point2d uv;
            //            uv = cam_model_.project3dToPixel(bumblebee_point);
            uv = cam_model_.project3dToPixel(pt_cv);

            //            ROS_INFO("ind cords 2d:  %f , %f ", uv.x, uv.y);

            //            x=distancia radar            y= tmanho ventana 
            //            p1(x0,y0), p2(x1,y1) 
            //            p1(80,2), p2(10,100)       80 metros --> 2 pixels          10 metros 100 pixels

            //             x-80      y-7
            //           -------- = --------  equação da função linear, utilizando dois pontos
            //            10-80     100-7

            //            y = (7930 - 93 x)/70

            double w = (7930 - 93 * data_.range_track) / 70;
            if (w < 0) {
                w = 0;
            }
            double h = w;

            if (uv.x > 0 && uv.y > 0) {

                if (radar_point.point.x != 0.0) {
                    std::string u;
                    std::stringstream out;
                    out << uv.x;
                    u = out.str();

                    std::string v;
                    std::stringstream out1;
                    out1 << uv.y;
                    v = out1.str();

                    std::string range_tr;
                    std::stringstream out2;
                    out2 << data_.range_track;
                    range_tr = out2.str();

                    std::string w_s;
                    std::stringstream out3;
                    out3 << w;
                    w_s = out3.str();
                    
                    std::string range_rate_tr;
                    std::stringstream out4;
                    out4 << data_.range_rate_track;
                    range_rate_tr = out4.str();

                    if (myfile.is_open()) {
                        myfile << "" + u + " " + v + " " + range_tr + " " + w_s + " " + range_rate_tr + "\n";
                    } else ROS_ERROR("Unable to open file");
                }
            }


            static const int RADIUS = 3;
            cv::circle(image, uv, RADIUS, CV_RGB(0, 255, 0), -1);
            if (radar_point.point.x != 0.0) {
                if (data_.range_rate_track < -130) {
                    cv::rectangle(image, cvPoint(uv.x - w / 2, uv.y - h / 2), cvPoint(uv.x + w / 2, uv.y + h / 2), CV_RGB(0, 255, 0), 1, 8);

                } else {
                    cv::rectangle(image, cvPoint(uv.x - w / 2, uv.y - h / 2), cvPoint(uv.x + w / 2, uv.y + h / 2), CV_RGB(0, 0, 255), 1, 8);

                }

            }
            CvPoint origin = cvPoint(uv.x, uv.y);

            std::string s;
            // convert double b to string s

            std::ostringstream ss;
            //            ss << data_.range_track;
            //            s = ss.str();

            ss << data_.range_rate_track;
            s = ss.str();
            cv::putText(image, s, origin, cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 0, 0));
        }
        myfile.close();
        cv::imwrite("/home/luis/catkin_ws/imageswrite/image" + s + "_radar.png", image);


        pub_.publish(input_bridge->toImageMsg());
    }

    void radarCallback(const radaresr15::radaresr15DataArray::ConstPtr& msgradar) {
        radarDataArray_ = msgradar;

    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "draw_frames");
    std::vector<std::string> frame_ids(argv + 1, argv + argc);
    FrameDrawer drawer(frame_ids);
    ros::spin();
}