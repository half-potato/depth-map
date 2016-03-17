#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <math.h>

static const std::string OPENCV_WINDOW = "Depth Map";

class DepthMapNode {
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber camera_sub_r;
	image_transport::Subscriber camera_sub_l;
	image_transport::Publisher depthmap_pub;
	sensor_msgs::ImageConstPtr left;
	sensor_msgs::ImageConstPtr right;
	int counter;
	int iPreFilterCap;
	int iNumOfDisparity;
	int iUniquenessRatio;
	int iSADWindowSize;

	public:
		void collectLeft(const sensor_msgs::ImageConstPtr& msg) {
			this->left = msg;
			//Horrible hack
			if (this->right) {
				this->update();
			}
		}

		void collectRight(const sensor_msgs::ImageConstPtr& msg) {
			this->right = msg;
			if (this->left) {
				this->update();
			}
		}

		DepthMapNode(std::string cam_r, std::string cam_l) : it_(nh_) {
			camera_sub_l = it_.subscribe(cam_l, 1, &DepthMapNode::collectLeft, this);
			camera_sub_r = it_.subscribe(cam_r, 1, &DepthMapNode::collectRight, this);
			depthmap_pub = it_.advertise("/depthmap/image_raw", 1);
			cv::namedWindow(OPENCV_WINDOW);
			counter = 0;
			iPreFilterCap = 9;
			iNumOfDisparity = 0;
			iUniquenessRatio = 0;
			iSADWindowSize = 9;
			cv::createTrackbar("Pre Filter Cap", OPENCV_WINDOW, &iPreFilterCap, 62);
			cv::createTrackbar("Num of Disparity", OPENCV_WINDOW, &iNumOfDisparity, 100);
			cv::createTrackbar("Uniqueness Ratio", OPENCV_WINDOW, &iUniquenessRatio, 100);
			cv::createTrackbar("SADWindowSize", OPENCV_WINDOW, &iSADWindowSize, 250);
		}

		~DepthMapNode() {
			cv::destroyWindow(OPENCV_WINDOW);
		}

		void update() {
			cv_bridge::CvImagePtr ptr_l;
			cv_bridge::CvImagePtr ptr_r;
			try {
				ptr_r = cv_bridge::toCvCopy(this->right, sensor_msgs::image_encodings::BGR8);
				ptr_l = cv_bridge::toCvCopy(this->left, sensor_msgs::image_encodings::BGR8);
			} catch (cv_bridge::Exception &e) {
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}
			cv::StereoBM stereo;
			stereo.state->SADWindowSize = (iSADWindowSize + 5) + (iSADWindowSize + 4) % 2 ;
			stereo.state->numberOfDisparities = 16;
			stereo.state->preFilterSize = 9;
			stereo.state->preFilterCap = iPreFilterCap + 1;
			stereo.state->minDisparity = iNumOfDisparity + 100;
			stereo.state->textureThreshold = 507; //507
			stereo.state->uniquenessRatio = iUniquenessRatio;
			stereo.state->speckleWindowSize = 0;
			stereo.state->speckleRange = 0;
			stereo.state->disp12MaxDiff = 1;
			cv::Mat g_l, g_r, disp, disp8;
			cvtColor(ptr_l->image, g_l, CV_BGR2GRAY);
			cvtColor(ptr_r->image, g_r, CV_BGR2GRAY);
			stereo(g_l, g_r, disp);
			cv::normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);

			cv::imshow(OPENCV_WINDOW, disp8);
			cv::waitKey(3); 
			std_msgs::Header header;
			header.seq = this->counter;
			counter++;
			header.stamp = ros::Time::now();
			cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, disp8);
			sensor_msgs::Image img_msg;
			img_bridge.toImageMsg(img_msg);
			depthmap_pub.publish(img_msg);
		}
};

int main(int argc, char ** argv) {
	ros::init(argc, argv, "depth_map");
	for (int i = 0; i < argc; i++) {
		std::cout << argv[i] << std::endl;
	}
	DepthMapNode dmn("/ps3_stereo/right/image_rect", "/ps3_stereo/left/image_rect");
	ros::spin();
	return 0;
}
