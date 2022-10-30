#include <filesystem>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class PcProjectionToImageOffline{
	private:
		/*node handle*/
		ros::NodeHandle nh_;
		ros::NodeHandle nh_private_;
        /*publisher*/
		ros::Publisher image_debug_pub_;
        /*buffer*/
		image_geometry::PinholeCameraModel camera_model_;
        struct PcTopic{
            std::string topic_name;
            sensor_msgs::PointCloud2ConstPtr pc_ptr;
            bool is_buffered = false;
            ros::Publisher debug_pub;
        };
        std::vector<PcTopic> pc_topic_list_;
        rosbag::Bag save_bag_;
		/*parameter*/
		std::string load_rosbag_path_;
		std::string save_rosbag_path_;
		std::string camera_info_name_;
		std::string load_image_name_;
		std::string save_image_name_;
		std::string save_depth_childname_;
        float max_range_;
        float debug_hz_;
        /*function*/
        void openRosBag(rosbag::Bag& bag, const std::string& rosbag_path, int mode);
        void getCameraModel();
        void bufferFirstPc();
        void projectPc(const sensor_msgs::PointCloud2ConstPtr& ros_pc_ptr, cv::Mat& image, bool range_to_rgb);
        void rangeToRGB(float range, int& r, int& g, int& b);

	public:
		PcProjectionToImageOffline();
        void execute();
};

PcProjectionToImageOffline::PcProjectionToImageOffline()
	: nh_private_("~")
{
	std::cout << "----- pc_projection_to_image_offline -----" << std::endl;

	/*parameter*/
    if(!nh_private_.getParam("load_rosbag_path", load_rosbag_path_)){
        std::cerr << "Set load_rosbag_path." << std::endl; 
        exit(true);
    }
	std::cout << "load_rosbag_path_ = " << load_rosbag_path_ << std::endl;
    nh_private_.param("save_rosbag_path", save_rosbag_path_, std::string(load_rosbag_path_.substr(0, load_rosbag_path_.length() - 4) + "_projected.bag"));
	std::cout << "save_rosbag_path_ = " << save_rosbag_path_ << std::endl;

    if(!nh_private_.getParam("camera_info_name", camera_info_name_)){
        std::cerr << "Set camera_info_name." << std::endl; 
        exit(true);
    }
	std::cout << "camera_info_name_ = " << camera_info_name_ << std::endl;
    if(!nh_private_.getParam("load_image_name", load_image_name_)){
        std::cerr << "Set load_image_name." << std::endl; 
        exit(true);
    }
	std::cout << "load_image_name_ = " << load_image_name_ << std::endl;
    nh_private_.param("save_image_name", save_image_name_, load_image_name_ + "/projected");
	std::cout << "save_image_name_ = " << save_image_name_ << std::endl;
    nh_private_.param("save_depth_childname", save_depth_childname_, std::string("depth"));
	std::cout << "save_depth_childname_ = " << save_depth_childname_ << std::endl;
    nh_private_.param("max_range", max_range_, float(100));
	std::cout << "max_range_ = " << max_range_ << std::endl;

    nh_private_.param("debug_hz", debug_hz_, float(-1));
	std::cout << "debug_hz_ = " << debug_hz_ << std::endl;

    for(size_t i = 0; ; i++){
        PcTopic tmp_pc_topic;
        if(!nh_private_.getParam("pc_" + std::to_string(i), tmp_pc_topic.topic_name))  break;
        pc_topic_list_.push_back(tmp_pc_topic);
        std::cout << "pc_topic_list_[" << i << "].topic_name = " << pc_topic_list_[i].topic_name << std::endl;
    }

    /*publisher*/
	image_debug_pub_ = nh_.advertise<sensor_msgs::Image>(save_image_name_, 1);
    for(PcTopic& pc_topic : pc_topic_list_) pc_topic.debug_pub = nh_.advertise<sensor_msgs::PointCloud2>(pc_topic.topic_name, 1);

    /*initialize*/
    getCameraModel();
    bufferFirstPc();
    std::filesystem::copy(load_rosbag_path_, save_rosbag_path_, std::filesystem::copy_options::overwrite_existing);
    openRosBag(save_bag_, save_rosbag_path_, rosbag::bagmode::Append);
}

void PcProjectionToImageOffline::openRosBag(rosbag::Bag& bag, const std::string& rosbag_path, int mode)
{
    try{
        bag.open(rosbag_path, mode);
    }
    catch(rosbag::BagException const&){
        std::cerr << "Cannot open " << rosbag_path << std::endl;
        exit(true);
    }
}

void PcProjectionToImageOffline::getCameraModel()
{
    rosbag::Bag load_bag;
    openRosBag(load_bag, load_rosbag_path_, rosbag::bagmode::Read);

    rosbag::View view;
    rosbag::View::iterator view_itr;
    view.addQuery(load_bag, rosbag::TypeQuery("sensor_msgs/CameraInfo"));
    view_itr = view.begin();

    while(view_itr != view.end()){
        if(view_itr->getTopic() == camera_info_name_){
    		camera_model_.fromCameraInfo(*view_itr->instantiate<sensor_msgs::CameraInfo>());
            load_bag.close();
            return;
        }
        view_itr++;
    }
    load_bag.close();

    std::cerr << camera_info_name_ << " does not exist in " << load_rosbag_path_ << "." << std::endl;
    exit(true);
}

void PcProjectionToImageOffline::bufferFirstPc()
{
    rosbag::Bag load_bag;
    openRosBag(load_bag, load_rosbag_path_, rosbag::bagmode::Read);

    rosbag::View view;
    rosbag::View::iterator view_itr;
    view.addQuery(load_bag, rosbag::TypeQuery("sensor_msgs/PointCloud2"));
    view_itr = view.begin();

    while(view_itr != view.end()){
        bool every_topic_is_buffered = true;
        for(PcTopic& pc_topic : pc_topic_list_){
            if(!pc_topic.is_buffered){
                if(view_itr->getTopic() == pc_topic.topic_name){
                    pc_topic.pc_ptr = view_itr->instantiate<sensor_msgs::PointCloud2>();
                    pc_topic.is_buffered = true;
                }
                else    every_topic_is_buffered = false;
            }
        }
        if(every_topic_is_buffered){
            load_bag.close();
            return;
        }
        view_itr++;
    }
    load_bag.close();
}

void PcProjectionToImageOffline::execute()
{
    rosbag::Bag load_bag;
    openRosBag(load_bag, load_rosbag_path_, rosbag::bagmode::Read);

    std::vector<std::string> query_topic_list;
    for(const PcTopic& pc_topic : pc_topic_list_)   query_topic_list.push_back(pc_topic.topic_name);
    query_topic_list.push_back(camera_info_name_);
    query_topic_list.push_back(load_image_name_);

    rosbag::View view(load_bag, rosbag::TopicQuery(query_topic_list));
    rosbag::View::iterator view_itr;
    view_itr = view.begin();

    ros::Rate loop_rate(debug_hz_);
    while(view_itr != view.end()){
        if(view_itr->getDataType() == "sensor_msgs/CameraInfo"){
    		if(view_itr->getTopic() == camera_info_name_)   camera_model_.fromCameraInfo(*view_itr->instantiate<sensor_msgs::CameraInfo>());
        }
        else if(view_itr->getDataType() == "sensor_msgs/PointCloud2"){
            for(PcTopic& pc_topic : pc_topic_list_){
                if(view_itr->getTopic() == pc_topic.topic_name){
                    pc_topic.pc_ptr = view_itr->instantiate<sensor_msgs::PointCloud2>();
                    pc_topic.debug_pub.publish(*pc_topic.pc_ptr);
                    cv_bridge::CvImage cv_64fc1(
                        pc_topic.pc_ptr->header,
                        "64FC1",
                        cv::Mat(camera_model_.cameraInfo().height, camera_model_.cameraInfo().width, CV_64FC1, cv::Scalar(-1))
                    );
                    projectPc(pc_topic.pc_ptr, cv_64fc1.image, false);
                    save_bag_.write(pc_topic.topic_name + "/" + save_depth_childname_, cv_64fc1.toImageMsg()->header.stamp, cv_64fc1.toImageMsg());
                    pc_topic.is_buffered = true;
                    break;
                }
            }
        }
        else if(view_itr->getDataType() == "sensor_msgs/CompressedImage"){
            if(view_itr->getTopic() == load_image_name_){
                sensor_msgs::CompressedImageConstPtr image_ptr = view_itr->instantiate<sensor_msgs::CompressedImage>();
                cv_bridge::CvImagePtr cv_bgr8_ptr = cv_bridge::toCvCopy(image_ptr, sensor_msgs::image_encodings::BGR8);
                cv_bridge::CvImage cv_64fc1(
                    image_ptr->header,
                    "64FC1",
                    cv::Mat(camera_model_.cameraInfo().height, camera_model_.cameraInfo().width, CV_64FC1, cv::Scalar(-1))
                );
                bool has_projected = false;
                for(const PcTopic& pc_topic : pc_topic_list_){
                    if(pc_topic.is_buffered){
                        projectPc(pc_topic.pc_ptr, cv_bgr8_ptr->image, true);
                        projectPc(pc_topic.pc_ptr, cv_64fc1.image, false);
                        has_projected = true;
                    }
                }
                if(has_projected){
                    save_bag_.write(save_image_name_, cv_bgr8_ptr->toImageMsg()->header.stamp, cv_bgr8_ptr->toImageMsg());
                    save_bag_.write(load_image_name_ + "/" + save_depth_childname_, cv_64fc1.toImageMsg()->header.stamp, cv_64fc1.toImageMsg());
                    image_debug_pub_.publish(cv_bgr8_ptr->toImageMsg());
                }
            }
        }
        if(debug_hz_ > 0)    loop_rate.sleep();
        view_itr++;
    }

    load_bag.close();
    save_bag_.close();
}

void PcProjectionToImageOffline::projectPc(const sensor_msgs::PointCloud2ConstPtr& ros_pc_ptr, cv::Mat& image, bool range_to_rgb)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*ros_pc_ptr, *pcl_pc);
	for(const pcl::PointXYZ& point : pcl_pc->points){
		/*depth > 0*/
		if(point.z > 0){
			cv::Point3d p3d(
				point.x,
				point.y,
				point.z
			);
			cv::Point2d p2d = camera_model_.project3dToPixel(p3d);
			float range = sqrt(
				point.x * point.x 
				+ point.y * point.y 
				+ point.z * point.z
			);
			if(p2d.x >= 0 && p2d.x < image.cols && p2d.y >= 0 && p2d.y < image.rows){
                if(range_to_rgb){
                    int r, g, b;
                    rangeToRGB(range, r, g, b);
                    cv::circle(image, p2d, 1, CV_RGB(r, g, b), -1);
                }
                else{
                    image.at<float>(p2d.y, p2d.x) = range;
                }
			}
		}
	}
}

void PcProjectionToImageOffline::rangeToRGB(float range, int& r, int& g, int& b)
{
	const int max_scale = 255;
	range = std::min(range, max_range_);
	int scale = int(range / max_range_ * 3 * max_scale);
	
	r = std::min(scale, max_scale);
	g = std::min(scale - r, max_scale);
	b = std::min(scale - r - g, max_scale);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pc_projection_to_image_offline");
	
	PcProjectionToImageOffline pc_projection_to_image_offline;
    pc_projection_to_image_offline.execute();
}