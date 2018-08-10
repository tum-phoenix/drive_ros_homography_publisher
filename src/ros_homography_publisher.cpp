#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <drive_ros_msgs/Homography.h>

int main(int argc, char ** argv) {
    ros::init(argc, argv, "homography_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::Publisher pub = nh.advertise<drive_ros_msgs::Homography>("homography_out", 1, true);
    std::string param_file_path;
    if(!pnh.getParam("param_file_path", param_file_path))
    {
      ROS_ERROR_STREAM("'param_file_path' is empty");
      return 1;
    }

    ROS_INFO_STREAM("'param_file_path' is: " << param_file_path);

    cv::FileStorage fs(param_file_path, cv::FileStorage::READ);

    cv::Mat w2c, c2w;
    fs["world2cam"] >> w2c;
    fs["cam2world"] >> c2w;
    fs.release();

    ROS_INFO_STREAM("world2cam" << w2c);
    ROS_INFO_STREAM("cam2world" << c2w);

    drive_ros_msgs::Homography msg;
    msg.cam2world.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.cam2world.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.cam2world.layout.dim[0].label = "height";
    msg.cam2world.layout.dim[1].label = "width";
    msg.cam2world.layout.dim[0].size = c2w.size().height;
    msg.cam2world.layout.dim[1].size = c2w.size().width;
    msg.cam2world.layout.dim[0].stride = c2w.size().height*c2w.size().width;
    msg.cam2world.layout.dim[1].stride = c2w.size().width;
    msg.cam2world.layout.data_offset = 0;

    for (int i=0; i<c2w.size().height; i++){
        for (int j=0; j<c2w.size().width; j++){
            msg.cam2world.data.push_back(c2w.at<double>(i,j));
        }
    }

    msg.world2cam.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.world2cam.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.world2cam.layout.dim[0].label = "height";
    msg.world2cam.layout.dim[1].label = "width";
    msg.world2cam.layout.dim[0].size = w2c.size().height;
    msg.world2cam.layout.dim[1].size = w2c.size().width;
    msg.world2cam.layout.dim[0].stride = w2c.size().height*w2c.size().width;
    msg.world2cam.layout.dim[1].stride = w2c.size().width;
    msg.world2cam.layout.data_offset = 0;

    for (int i=0; i<w2c.size().height; i++){
        for (int j=0; j<w2c.size().width; j++){
            msg.world2cam.data.push_back(w2c.at<double>(i,j));
        }
    }

    // since the topic is latched we publish only once
    pub.publish(msg);
    ros::spin();
}
