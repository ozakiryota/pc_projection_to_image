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
		cv::Mat image_;
		/*parameter*/
		double max_range_;

	public:
		PcProjectionToImage();
		void callbackPc(const sensor_msgs::PointCloud2ConstPtr& msg);
		void callbackImage(const sensor_msgs::ImageConstPtr& img_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
		void project(cv_bridge::CvImagePtr cv_bridge_ptr);
		void rangeToRGB(double range, int& r, int& g, int& b);
		void publication(cv_bridge::CvImagePtr cv_bridge_ptr);
};

PcProjectionToImage::PcProjectionToImage()
	: it_(nh_), nh_private_("~")
{
	std::cout << "--- pc_projection_to_image ---" << std::endl;
	/*parameter*/
	nh_private_.param("max_range", max_range_, 100.0);
	std::cout << "max_range_ = " << max_range_ << std::endl;
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
	if(!pc_->points.empty()){
		camera_model_.fromCameraInfo(info_msg);
		try{
			cv_bridge::CvImagePtr cv_bridge_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
			project(cv_bridge_ptr);
			publication(cv_bridge_ptr);
		}
		catch(cv_bridge::Exception& exc){
			ROS_ERROR("cv_bridge exception: %s", exc.what());
		}
	}
}

void PcProjectionToImage::project(cv_bridge::CvImagePtr cv_bridge_ptr)
{
	for(size_t i = 0; i < pc_->points.size(); ++i){
		/*depth > 0*/
		if(pc_->points[i].z > 0){
			cv::Point3d p3d(
				pc_->points[i].x,
				pc_->points[i].y,
				pc_->points[i].z
			);
			cv::Point2d p2d = camera_model_.project3dToPixel(p3d);
			double range = sqrt(
				pc_->points[i].x * pc_->points[i].x 
				+ pc_->points[i].y * pc_->points[i].y 
				+ pc_->points[i].z * pc_->points[i].z
			);
			int r, g, b;
			rangeToRGB(range, r, g, b);

			if(p2d.x >= 0 && p2d.x < cv_bridge_ptr->image.cols && p2d.y >= 0 && p2d.y < cv_bridge_ptr->image.rows){
				cv::circle(cv_bridge_ptr->image, p2d, 1, CV_RGB(r, g, b), -1);
			}
		}
	}
}

void PcProjectionToImage::rangeToRGB(double range, int& r, int& g, int& b)
{
	const int max_scale = 255;
	range = std::min(range, max_range_);
	int scale = int(range / max_range_ * 3 * max_scale);
	
	r = std::min(scale, max_scale);
	g = std::min(scale - r, max_scale);
	b = std::min(scale - r - g, max_scale);
}

void PcProjectionToImage::publication(cv_bridge::CvImagePtr cv_bridge_ptr)
{
	pub_image_.publish(cv_bridge_ptr->toImageMsg());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pc_projection_to_image");
	
	PcProjectionToImage pc_projection_to_image;

	ros::spin();
}
