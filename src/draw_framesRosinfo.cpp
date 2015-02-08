#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>
#include <ros/console.h>

static int count = 0;
class FrameDrawer
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber sub_;
  image_transport::Publisher pub_;
  tf::TransformListener tf_listener_;
  image_geometry::PinholeCameraModel cam_model_;
  std::vector<std::string> frame_ids_;
  CvFont font_;
  

public:
  FrameDrawer(const std::vector<std::string>& frame_ids)
    : it_(nh_), frame_ids_(frame_ids)
  {
    std::string image_topic = nh_.resolveName("image");
    sub_ = it_.subscribeCamera(image_topic, 1, &FrameDrawer::imageCb, this);
    pub_ = it_.advertise("image_out", 1);
    cvInitFont(&font_, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    cv::Mat image;
    cv::Mat image_rec;
    cv_bridge::CvImagePtr input_bridge;
    try {
      input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
      image = input_bridge->image;
    }
    catch (cv_bridge::Exception& ex){
      ROS_ERROR("[draw_frames00] Failed to convert image");
      return;
    }

    cam_model_.fromCameraInfo(info_msg);
    cam_model_.rectifyImage(image,image_rec);
    //image=image_rec;

//    BOOST_FOREACH(const std::string& frame_id, frame_ids_) {
      const std::string& frame_id = frame_ids_.at(1);

      tf::StampedTransform transform;
      count++;
      try {
        ros::Time acquisition_time = info_msg->header.stamp;
        ros::Duration timeout(1.0 / 40);
        ROS_INFO_STREAM("acquisition_time  " << ros::Time(0));
        ROS_INFO_STREAM("timeout  " << timeout);
        ROS_INFO_STREAM("Frame cam  " << cam_model_.tfFrame());
        ROS_INFO_STREAM("Frame id  " << frame_id);
        tf_listener_.waitForTransform(cam_model_.tfFrame(), frame_id,
                                      ros::Time(0), timeout);
        tf_listener_.lookupTransform(cam_model_.tfFrame(), frame_id,
                                     ros::Time(0), transform);
//        ROS_INFO("Frame cam %s", (std::string)cam_model_.tfFrame());
//        ROS_INFO("Frame id %s", (std::string)frame_id);
        
        
//        ROS_INFO("Frame acquisition_time %f", acquisition_time);
//        ROS_INFO("Frame timeout %f", timeout);
      }
      catch (tf::TransformException& ex) {
        ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
        return;
      }

      tf::Point pt = transform.getOrigin();
      cv::Point3d pt_cv(pt.x()-1, pt.y()-2, pt.z());

      //cv::Point3d pt_cv(-103.0+count, 0.0, -4.0);
      cv::Point2d uv;
      uv = cam_model_.project3dToPixel(pt_cv);
      //ROS_INFO("pt2d %f , %f", uv.x,uv.y);
      //uv = cam_model_.rectifyPoint(uv);
      ROS_INFO("pt2d rec %f , %f", uv.x,uv.y);
      ROS_INFO("Pt3d %f , %f, %f", pt_cv.x, pt_cv.y, pt_cv.z);
//      uv.y=uv.y-120;
      /*eliminar esto*/
//      uv.x = 320;
//      uv.y = 240;
      static const int RADIUS = 3;
      cv::circle(image, uv, RADIUS, CV_RGB(255,0,0), -1);
      CvSize text_size;
      int baseline;
      cvGetTextSize(frame_id.c_str(), &font_, &text_size, &baseline);
      

      CvPoint origin = cvPoint( uv.x - text_size.width / 2,
                                uv.y - RADIUS - baseline - 3);


    cv::putText(image, frame_id.c_str(), origin, cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(255,0,0));
    
//    }

    pub_.publish(input_bridge->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "draw_frames");
  std::vector<std::string> frame_ids(argv + 1, argv + argc);
  FrameDrawer drawer(frame_ids);
  ros::spin();
}
