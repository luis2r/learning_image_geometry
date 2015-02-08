#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>
#include "radaresr15/radaresr15Data.h"
#include "radaresr15/radaresr15DataArray.h"

class FrameDrawer {
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::CameraSubscriber sub;
    image_transport::Publisher pub;
    tf::TransformListener tf_listener;
    image_geometry::PinholeCameraModel cam_model;
    ros::Subscriber subr;
    std::vector<std::string> frame_ids;
    CvFont font;
    int indice;
    double X[64];
    double Y[64];

public:

    FrameDrawer(const std::vector<std::string>& frame_ids)
    : it(nh), frame_ids(frame_ids) {
        std::string image_topic = nh.resolveName("image");
        sub = it.subscribeCamera(image_topic, 1, &FrameDrawer::imageCb, this);
        subr = nh.subscribe("radar15_msg_array", 1000, &FrameDrawer::chatterCallback, this);

        pub = it.advertise("image_out", 1);
        cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);
    }

    void chatterCallback(const radaresr15::radaresr15DataArray::ConstPtr& msg) {

        ros::NodeHandle nh;


        for (int i = 0; i < msg->tracks.size(); ++i) {

            const radaresr15::radaresr15Data &data = msg->tracks[i];
            //double angle = msg->tracks[i].angle_track;
            //indice=data.id_track;
            X[data.id_track] = data.y_track;
            Y[data.id_track] = -data.x_track;
            //            ROS_INFO("ind %d cords  %f , %f", indice, X, Y);
            //marker_array_msg.markers[indice].id = indice;
        }
    }

    void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
            const sensor_msgs::CameraInfoConstPtr& info_msg) {
        cv::Mat image;
        cv_bridge::CvImagePtr input_bridge;
        try {
            input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
            image = input_bridge->image;
        } catch (cv_bridge::Exception& ex) {
            ROS_ERROR("[draw_frames] Failed to convert image");
            return;
        }

        cam_model.fromCameraInfo(info_msg);

        BOOST_FOREACH(const std::string& frame_id, frame_ids) {
            tf::StampedTransform transform;
            try {
                ros::Time acquisition_time = info_msg->header.stamp;
                //ros::Duration timeout(1.0 / 30);
                ros::Duration timeout(1.0);
                tf_listener.waitForTransform(cam_model.tfFrame(), frame_id,
                        acquisition_time, timeout);
                tf_listener.lookupTransform(cam_model.tfFrame(), frame_id,
                        acquisition_time, transform);
            } catch (tf::TransformException& ex) {
                ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
                return;
            }
            for (int i = 0; i < 64; i++) {



                tf::Point pt = transform.getOrigin();
                cv::Point3d pt_cv(X[i], Y[i], 0.4);
//                ROS_INFO("ind %d cords  %f , %f", i, X[i], Y[i]);
                cv::Point2d uv;
                uv = cam_model.project3dToPixel(pt_cv);

                static const int RADIUS = 5;
                cv::circle(image, uv, RADIUS, CV_RGB(255, 0, 0), -1);
                CvSize text_size;
                int baseline;
                cvGetTextSize(frame_id.c_str(), &font, &text_size, &baseline);
                CvPoint origin = cvPoint(uv.x - text_size.width / 2,
                        uv.y - RADIUS - baseline - 3);
                ROS_INFO("ind %d cords3d  %f , %f", i, X[i], X[i]);
                ROS_INFO("ind %d cords2d  %f , %f", i, uv.x, uv.y);
cv:
                putText(image, frame_id.c_str(), origin, cv::FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(255, 0, 0));

            }
            //tf::Point pt = transform.getOrigin();
            //            cv::Point3d pt_cv(X[i], Y[i], 0.4);
            //            ROS_INFO("ind %d cords  %f , %f", indice, X, Y);
            //            cv::Point2d uv;
            //            uv = cam_model.project3dToPixel(pt_cv);
            //
            //            static const int RADIUS = 5;
            //            cv::circle(image, uv, RADIUS, CV_RGB(255, 0, 0), -1);
            //            CvSize text_size;
            //            int baseline;
            //            cvGetTextSize(frame_id.c_str(), &font, &text_size, &baseline);
            //            CvPoint origin = cvPoint(uv.x - text_size.width / 2,
            //                    uv.y - RADIUS - baseline - 3);
            //cv:
            //            putText(image, frame_id.c_str(), origin, cv::FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(255, 0, 0));
        }

        pub.publish(input_bridge->toImageMsg());
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "draw_frames");
    std::vector<std::string> frame_ids(argv + 1, argv + argc);
    FrameDrawer drawer(frame_ids);
    ros::spin();
}
