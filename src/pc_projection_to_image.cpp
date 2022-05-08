#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

class PcProjectionToImage{
	private:
		/*node handle*/
		ros::NodeHandle nh_;
		ros::NodeHandle nh_private_;
		image_transport::ImageTransport it_;
		/*subscriber*/
		ros::Subscriber pc_sub_;
		// ros::Subscriber img_sub_;
		image_transport::CameraSubscriber camera_sub_;
		/*publisher*/
		ros::Publisher pub_pc_;
		ros::Publisher pub_image_;
		/*buffer*/
		pcl::PointCloud<pcl::PointXYZI>::Ptr pc_ {new pcl::PointCloud<pcl::PointXYZI>};
		image_geometry::PinholeCameraModel camera_model_;
		/*parameter*/

	public:
		PcProjectionToImage();
		void callbackPc(const sensor_msgs::PointCloud2ConstPtr& msg);
		void callbackImage(const sensor_msgs::ImageConstPtr& img_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
		void project(cv_bridge::CvImagePtr cv_bridge_ptr);
		// void publication(pcl::PointCloud<pcl::PointXYZI>::Ptr pc);
};

PcProjectionToImage::PcProjectionToImage()
	: it_(nh_), nh_private_("~")
{
	std::cout << "--- pc_projection_to_image ---" << std::endl;
	/*parameter*/
	// nh_private_.param("publish_frame", publish_frame_, std::string(""));
	// std::cout << "publish_frame_ = " << publish_frame_ << std::endl;
	/*subscriber*/
	pc_sub_ = nh_.subscribe("/point_cloud", 1, &PcProjectionToImage::callbackPc, this);
	camera_sub_ = it_.subscribeCamera("/image", 1, &PcProjectionToImage::callbackImage, this);
	/*publisher*/
	pub_pc_ = nh_.advertise<sensor_msgs::PointCloud2>("/point_cloud/colored", 1);
	pub_image_ = nh_.advertise<sensor_msgs::Image>("/image/projected", 1);
}

void PcProjectionToImage::callbackPc(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::fromROSMsg(*msg, *pc_);
}

void PcProjectionToImage::callbackImage(const sensor_msgs::ImageConstPtr& img_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
	std::cout << "--- callback ---" << std::endl;
	if(!pc_->points.empty()){
		camera_model_.fromCameraInfo(info_msg);
		try{
			cv_bridge::CvImagePtr cv_bridge_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
			project(cv_bridge_ptr);
			// publication(pc);
		}
		catch(cv_bridge::Exception& exc){
			ROS_ERROR("cv_bridge exception: %s", exc.what());
		}
	}
}

void PcProjectionToImage::project(cv_bridge::CvImagePtr cv_bridge_ptr)
{
	std::cout << "--- project ---" << std::endl;
	for(size_t i = 0; i < pc_->points.size(); ++i){
		cv::Point3d p3d(
			pc_->points[i].x,
			pc_->points[i].y,
			pc_->points[i].z
		);
	}
}

// void PcProjectionToImage::publication(pcl::PointCloud<pcl::PointXYZI>::Ptr pc)
// {
//     sensor_msgs::PointCloud2 ros_pc;
//     pcl::toROSMsg(*pc, ros_pc);
// 	if(publish_frame_ != "")	ros_pc.header.frame_id = publish_frame_;
//     pub_.publish(ros_pc);
// }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pc_projection_to_image");
	
	PcProjectionToImage pc_projection_to_image;

	ros::spin();
}
