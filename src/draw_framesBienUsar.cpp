#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>
#include  <radaresr15/radaresr15DataArray.h>
#include  <beginner_tutorials/Num.h>
#include  <radaresr15/radaresr15Data.h>

class FrameDrawer {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::CameraSubscriber sub_;
    ros::Subscriber subradar_;

    image_transport::Publisher pub_;
    tf::TransformListener tf_listener_;
    image_geometry::PinholeCameraModel cam_model_;
    std::vector<std::string> frame_ids_;
    radaresr15::radaresr15Data data_;
    radaresr15::radaresr15DataArray::ConstPtr radarDataArray_;
    CvFont font_;
    int radarMsgLength_;

public:

    FrameDrawer(const std::vector<std::string>& frame_ids)
    : it_(nh_), frame_ids_(frame_ids) {
        std::string image_topic = nh_.resolveName("image");
        subradar_ = nh_.subscribe("radar15_msg_array", 1000, &FrameDrawer::radarCallback, this);
        sub_ = it_.subscribeCamera(image_topic, 1, &FrameDrawer::imageCb, this);
        pub_ = it_.advertise("image_out", 1);
        cvInitFont(&font_, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg) {
        cv::Mat image;
        cv_bridge::CvImagePtr input_bridge;
        try {
            input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
            image = input_bridge->image;
        } catch (cv_bridge::Exception& ex) {
            ROS_ERROR("[draw_frames] Failed to convert image");
            return;
        }

        cam_model_.fromCameraInfo(info_msg);

        BOOST_FOREACH(const std::string& frame_id, frame_ids_) {
            tf::StampedTransform transform;
            try {
                ROS_INFO_STREAM("Frame cam  " << cam_model_.tfFrame());
                ROS_INFO_STREAM("Frame id new  " << frame_id);
//                ros::Time acquisition_time = info_msg->header.stamp;
                ros::Duration timeout(1.0 / 20);
                tf_listener_.waitForTransform(cam_model_.tfFrame(), frame_id,ros::Time(0), timeout);
                        //                        acquisition_time, timeout);
                        
                tf_listener_.lookupTransform(cam_model_.tfFrame(), frame_id, ros::Time(0), transform);
                        //                        acquisition_time, transform);
                        
            } catch (tf::TransformException& ex) {
                ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
                return;
            }


            for (int i = 0; i < radarMsgLength_; ++i) {

                data_ = radarDataArray_->tracks[i];
                //double angle = msg->tracks[i].angle_track;
                int indice = data_.id_track;
                double X = data_.x_track;
                double Y = data_.y_track;
                ROS_INFO("id im: [%d]", (int) data_.id_track);
                ROS_INFO("x im: [%f]", (double) data_.x_track);
                ROS_INFO("y im: [%f]", (double) data_.y_track);

                if (X == 0 && Y == 0) {

                } else {

                }
            }

            tf::Point pt = transform.getOrigin();
            cv::Point3d pt_cv(pt.x(), pt.y(), pt.z());
            ROS_INFO("ind %d cords:  %f , %f, %f", i, pt.x(), pt.x(), pt.z());
            cv::Point2d uv;
            uv = cam_model_.project3dToPixel(pt_cv);

            static const int RADIUS = 3;
            cv::circle(image, uv, RADIUS, CV_RGB(255, 0, 0), -1);
            CvSize text_size;
            int baseline;
            cvGetTextSize(frame_id.c_str(), &font_, &text_size, &baseline);
            CvPoint origin = cvPoint(uv.x - text_size.width / 2,
                    uv.y - RADIUS - baseline - 3);
            cv::putText(image, frame_id.c_str(), origin, cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(255, 0, 0));
        }

        pub_.publish(input_bridge->toImageMsg());
    }

    void radarCallback(const radaresr15::radaresr15DataArray::ConstPtr& msgradar) {
        //int indice = 0;
        radarDataArray_ = msgradar;

        }

    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "draw_frames");
    std::vector<std::string> frame_ids(argv + 1, argv + argc);
    FrameDrawer drawer(frame_ids);
    ros::spin();
}
