
#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>

#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/normal_coherence.h>

#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/io/openni2/openni.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>
#include <pcl/common/angles.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/surface/convex_hull.h>

#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>

#include <boost/format.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <WinSock2.h>
#include <iostream>


// Variables for tracking
cv::Mat image;
bool selectObject = false;
int trackObject = 0;
bool showHist = true;
cv::Point origin;
cv::Rect selection;
int LINE_AA = 16;
cv::Mat frame, hsv, hue, mask, hist, histimg, backproj;
// ?? 
cv::Rect trackWindow;
// The size of histogram (bins) 
int hsize = 16;
// The range of histogram
float hranges[] = { 0,180 };
// Pointer to the range of histogram 
const float* phranges = hranges;

// Flag for tracking object
bool trackingMode = false;

// Mouse callback function
void onMouse(int event, int x, int y, int, void*) {
	if (selectObject) {
		selection.x = MIN(x, origin.x);
		selection.y = MIN(y, origin.y);
		selection.width = std::abs(x - origin.x);
		selection.height = std::abs(y - origin.y);
		selection &= cv::Rect(0, 0, image.cols, image.rows);
	}

	// Click process
	switch (event) {
		// When left button downed
	case cv::EVENT_LBUTTONDOWN:
		// Set cursor position as origin
		origin = cv::Point(x, y);
		// 
		selection = cv::Rect(x, y, 0, 0);
		selectObject = true;
		break;
	case cv::EVENT_LBUTTONUP:
		selectObject = false;
		if (selection.width > 0 && selection.height > 0)
			trackObject = -1;
		break;
	}
}

#define FPS_CALC_BEGIN                          \
    static double duration = 0;                 \
    double start_time = pcl::getTime ();        \

#define FPS_CALC_END(_WHAT_)                    \
  {                                             \
    double end_time = pcl::getTime ();          \
    static unsigned count = 0;                  \
    if (++count == 10)                          \
    {                                           \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(duration) << " Hz" <<  std::endl; \
      count = 0;                                                        \
      duration = 0.0;                                                   \
    }                                           \
    else                                        \
    {                                           \
      duration += end_time - start_time;        \
    }                                           \
  }

using namespace pcl::tracking;

template <typename PointType>
class OpenNISegmentTracking
{
public:
	//typedef pcl::PointXYZRGBANormal RefPointType;
	typedef pcl::PointXYZRGBA RefPointType;
	//typedef pcl::PointXYZ RefPointType;
	typedef ParticleXYZRPY ParticleT;

	typedef pcl::PointCloud<PointType> Cloud;
	typedef pcl::PointCloud<RefPointType> RefCloud;
	typedef typename RefCloud::Ptr RefCloudPtr;
	typedef typename RefCloud::ConstPtr RefCloudConstPtr;
	typedef typename Cloud::Ptr CloudPtr;
	typedef typename Cloud::ConstPtr CloudConstPtr;
	//typedef KLDAdaptiveParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;
	//typedef KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> ParticleFilter;
	//typedef ParticleFilterOMPTracker<RefPointType, ParticleT> ParticleFilter;
	typedef ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;
	typedef typename ParticleFilter::CoherencePtr CoherencePtr;
	typedef typename pcl::search::KdTree<PointType> KdTree;
	typedef typename KdTree::Ptr KdTreePtr;
	// CONSTRUCTOR
	OpenNISegmentTracking(const std::string& device_id, int thread_nr, double downsampling_grid_size,
		bool use_convex_hull,
		bool visualize_non_downsample, bool visualize_particles,
		bool use_fixed)
		: viewer_("PCL OpenNI Tracking Viewer")
		, device_id_(device_id)
		, new_cloud_(false)
		, ne_(thread_nr)
		, counter_(0)
		, use_convex_hull_(use_convex_hull)
		, visualize_non_downsample_(visualize_non_downsample)
		, visualize_particles_(visualize_particles)
		, downsampling_grid_size_(downsampling_grid_size)
	{
		// INITIALIZE
		KdTreePtr tree(new KdTree(false));
		ne_.setSearchMethod(tree);
		ne_.setRadiusSearch(0.03);

		std::vector<double> default_step_covariance = std::vector<double>(6, 0.015 * 0.015);
		default_step_covariance[3] *= 40.0;
		default_step_covariance[4] *= 40.0;
		default_step_covariance[5] *= 40.0;

		std::vector<double> initial_noise_covariance = std::vector<double>(6, 0.00001);
		std::vector<double> default_initial_mean = std::vector<double>(6, 0.0);
		if (use_fixed)
		{
			boost::shared_ptr<ParticleFilterOMPTracker<RefPointType, ParticleT> > tracker
			(new ParticleFilterOMPTracker<RefPointType, ParticleT>(thread_nr));
			tracker_ = tracker;
		}
		else
		{
			boost::shared_ptr<KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> > tracker
			(new KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT>(thread_nr));
			tracker->setMaximumParticleNum(500);
			tracker->setDelta(0.99);
			tracker->setEpsilon(0.2);
			ParticleT bin_size;
			bin_size.x = 0.1f;
			bin_size.y = 0.1f;
			bin_size.z = 0.1f;
			bin_size.roll = 0.1f;
			bin_size.pitch = 0.1f;
			bin_size.yaw = 0.1f;
			tracker->setBinSize(bin_size);
			tracker_ = tracker;
		}

		tracker_->setTrans(Eigen::Affine3f::Identity());
		tracker_->setStepNoiseCovariance(default_step_covariance);
		tracker_->setInitialNoiseCovariance(initial_noise_covariance);
		tracker_->setInitialNoiseMean(default_initial_mean);
		tracker_->setIterationNum(1);

		tracker_->setParticleNum(400);
		tracker_->setResampleLikelihoodThr(0.00);
		tracker_->setUseNormal(false);
		// setup coherences
		ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence = ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr
		(new ApproxNearestPairPointCloudCoherence<RefPointType>());
		// NearestPairPointCloudCoherence<RefPointType>::Ptr coherence = NearestPairPointCloudCoherence<RefPointType>::Ptr
		//   (new NearestPairPointCloudCoherence<RefPointType> ());

		boost::shared_ptr<DistanceCoherence<RefPointType> > distance_coherence
			= boost::shared_ptr<DistanceCoherence<RefPointType> >(new DistanceCoherence<RefPointType>());
		coherence->addPointCoherence(distance_coherence);

		boost::shared_ptr<HSVColorCoherence<RefPointType> > color_coherence
			= boost::shared_ptr<HSVColorCoherence<RefPointType> >(new HSVColorCoherence<RefPointType>());
		color_coherence->setWeight(0.1);
		coherence->addPointCoherence(color_coherence);

		//boost::shared_ptr<pcl::search::KdTree<RefPointType> > search (new pcl::search::KdTree<RefPointType> (false));
		boost::shared_ptr<pcl::search::Octree<RefPointType> > search(new pcl::search::Octree<RefPointType>(0.01));
		//boost::shared_ptr<pcl::search::OrganizedNeighbor<RefPointType> > search (new pcl::search::OrganizedNeighbor<RefPointType>);
		coherence->setSearchMethod(search);
		coherence->setMaximumDistance(0.01);
		tracker_->setCloudCoherence(coherence);

		// Initialize socket communication
		// Make socket object
		sock_recv_ = socket(AF_INET, SOCK_DGRAM, 0);
		sock_send_ = socket(AF_INET, SOCK_DGRAM, 0);

		addr_recv_.sin_family = AF_INET;
		addr_send_.sin_family = AF_INET;

		addr_recv_.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
		addr_send_.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");

		// Set different port number to each socket
		addr_recv_.sin_port = htons(11111);
		addr_send_.sin_port = htons(22222);

		// Bind receiver socket 
		bind(sock_recv_, (struct sockaddr *)&addr_recv_, sizeof(addr_recv_));

		// fd_setの初期化します
		FD_ZERO(&readfds_);

		// selectで待つ読み込みソケットとしてsock1を登録します
		FD_SET(sock_recv_, &readfds_);

		tv_.tv_sec = 0;
		tv_.tv_usec = 0;


	}

	bool
		drawParticles(pcl::visualization::PCLVisualizer& viz)
	{
		ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles();
		if (particles)
		{
			if (visualize_particles_)
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud(new pcl::PointCloud<pcl::PointXYZ>());
				for (size_t i = 0; i < particles->points.size(); i++)
				{
					pcl::PointXYZ point;

					point.x = particles->points[i].x;
					point.y = particles->points[i].y;
					point.z = particles->points[i].z;
					particle_cloud->points.push_back(point);
				}

				{
					pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue_color(particle_cloud, 250, 99, 71);
					if (!viz.updatePointCloud(particle_cloud, blue_color, "particle cloud"))
						viz.addPointCloud(particle_cloud, blue_color, "particle cloud");
				}
			}
			return true;
		}
		else
		{
			PCL_WARN("no particles\n");
			return false;
		}
	}

	void
		drawResult(pcl::visualization::PCLVisualizer& viz)
	{
		ParticleXYZRPY result = tracker_->getResult();
		Eigen::Affine3f transformation = tracker_->toEigenMatrix(result);
		// move a little bit for better visualization
		transformation.translation() += Eigen::Vector3f(0.0f, 0.0f, -0.005f);
		RefCloudPtr result_cloud(new RefCloud());

		if (!visualize_non_downsample_)
			pcl::transformPointCloud<RefPointType>(*(tracker_->getReferenceCloud()), *result_cloud, transformation);
		else
			pcl::transformPointCloud<RefPointType>(*reference_, *result_cloud, transformation);

		{
			pcl::visualization::PointCloudColorHandlerCustom<RefPointType> red_color(result_cloud, 0, 0, 255);
			if (!viz.updatePointCloud(result_cloud, red_color, "resultcloud"))
				viz.addPointCloud(result_cloud, red_color, "resultcloud");
		}

	}

	// Function to visualize point cloud
	void
		viz_cb(pcl::visualization::PCLVisualizer& viz)
	{
		// Lock mtx_ object
		boost::mutex::scoped_lock lock(mtx_);

		// Cheack if cloud_pass_ is empty
		if (!cloud_pass_)
		{
			boost::this_thread::sleep(boost::posix_time::seconds(1));
			return;
		}

		// Cheack if new_cloud is true and cloud_pass_downsampled_ is not empty
		if (new_cloud_ && cloud_pass_downsampled_)
		{
			CloudPtr cloud_pass;
			// Decide whether downsampled cloud is used or not
			if (!visualize_non_downsample_)
				cloud_pass = cloud_pass_downsampled_;
			else
				cloud_pass = cloud_pass_;

			//	Draw cloud_pass to display 
			if (!viz.updatePointCloud(cloud_pass, "cloudpass"))
			{
				viz.addPointCloud(cloud_pass, "cloudpass");
				viz.resetCameraViewpoint("cloudpass");
				viz.setCameraPosition(
					0, 0, 0,	// Position
					0, 0, 1,	// ViewPoint
					0, -1, 0);	// Up
			}

		}

		// Cheack if new_cloud is true and cloud_pass_downsampled_ is not empty
		if (new_cloud_ && drawOBB_)
		{
			viz.removeAllShapes();
			Eigen::Vector3f position(position_OBB_.x, position_OBB_.y, position_OBB_.z);
			Eigen::Quaternionf quat(rotational_matrix_OBB_);

			// Calculate the position and the pose of object
			pcl::PointXYZ center(mass_center_(0), mass_center_(1), mass_center_(2));
			pcl::PointXYZ x_axis(major_vector_(0) + mass_center_(0), major_vector_(1) + mass_center_(1), major_vector_(2) + mass_center_(2));
			pcl::PointXYZ y_axis(middle_vector_(0) + mass_center_(0), middle_vector_(1) + mass_center_(1), middle_vector_(2) + mass_center_(2));
			pcl::PointXYZ z_axis(minor_vector_(0) + mass_center_(0), minor_vector_(1) + mass_center_(1), minor_vector_(2) + mass_center_(2));

			Eigen::Vector3f p1_(min_point_OBB_.x, min_point_OBB_.y, min_point_OBB_.z);
			Eigen::Vector3f p2_(min_point_OBB_.x, min_point_OBB_.y, max_point_OBB_.z);
			Eigen::Vector3f p3_(max_point_OBB_.x, min_point_OBB_.y, max_point_OBB_.z);
			Eigen::Vector3f p4_(max_point_OBB_.x, min_point_OBB_.y, min_point_OBB_.z);
			Eigen::Vector3f p5_(min_point_OBB_.x, max_point_OBB_.y, min_point_OBB_.z);
			Eigen::Vector3f p6_(min_point_OBB_.x, max_point_OBB_.y, max_point_OBB_.z);
			Eigen::Vector3f p7_(max_point_OBB_.x, max_point_OBB_.y, max_point_OBB_.z);
			Eigen::Vector3f p8_(max_point_OBB_.x, max_point_OBB_.y, min_point_OBB_.z);

			p1_ = rotational_matrix_OBB_ * p1_ + position;
			p2_ = rotational_matrix_OBB_ * p2_ + position;
			p3_ = rotational_matrix_OBB_ * p3_ + position;
			p4_ = rotational_matrix_OBB_ * p4_ + position;
			p5_ = rotational_matrix_OBB_ * p5_ + position;
			p6_ = rotational_matrix_OBB_ * p6_ + position;
			p7_ = rotational_matrix_OBB_ * p7_ + position;
			p8_ = rotational_matrix_OBB_ * p8_ + position;

			pcl::PointXYZ pt1(p1_(0), p1_(1), p1_(2));
			pcl::PointXYZ pt2(p2_(0), p2_(1), p2_(2));
			pcl::PointXYZ pt3(p3_(0), p3_(1), p3_(2));
			pcl::PointXYZ pt4(p4_(0), p4_(1), p4_(2));
			pcl::PointXYZ pt5(p5_(0), p5_(1), p5_(2));
			pcl::PointXYZ pt6(p6_(0), p6_(1), p6_(2));
			pcl::PointXYZ pt7(p7_(0), p7_(1), p7_(2));
			pcl::PointXYZ pt8(p8_(0), p8_(1), p8_(2));

			viz.addLine(pt1, pt2, 1.0, 1.0, 0.0, "1 edge");
			viz.addLine(pt1, pt4, 1.0, 1.0, 0.0, "2 edge");
			viz.addLine(pt1, pt5, 1.0, 1.0, 0.0, "3 edge");
			
			viz.addLine(pt5, pt6, 1.0, 1.0, 0.0, "4 edge");
			viz.addLine(pt5, pt8, 1.0, 1.0, 0.0, "5 edge");
			viz.addLine(pt2, pt6, 1.0, 1.0, 0.0, "6 edge");
			viz.addLine(pt6, pt7, 1.0, 1.0, 0.0, "7 edge");
			viz.addLine(pt7, pt8, 1.0, 1.0, 0.0, "8 edge");
			viz.addLine(pt2, pt3, 1.0, 1.0, 0.0, "9 edge");
			viz.addLine(pt4, pt8, 1.0, 1.0, 0.0, "10 edge");
			viz.addLine(pt3, pt4, 1.0, 1.0, 0.0, "11 edge");
			viz.addLine(pt3, pt7, 1.0, 1.0, 0.0, "12 edge");

			// Draw major axis
			viz.addLine(center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
			viz.addLine(center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
			viz.addLine(center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");
		}

		// Cloud is not new
		new_cloud_ = false;

		// Draw images
		if (!color_img_.empty()) {
			//cv::imshow("Color Image", color_img_);
			cv::imshow("Depth Image", depth_img_);

			//// Canny edge detection
			//cv::Mat gray, edge, draw;
			//cv::cvtColor(color_img_, gray, CV_BGR2GRAY);
			//cv::Canny(gray, edge, 50, 150, 3);
			//edge.convertTo(draw, CV_8UC1);
			//cv::imshow("Canny edge", draw);


			// Create window
			cv::namedWindow("Camera", 0);
			// Set mouse callback function
			cv::setMouseCallback("Camera", onMouse, 0);
			// Test Camshift tracking
			// Variables for tracking
			histimg = cv::Mat::zeros(200, 320, CV_8UC3);
			// Copy frame to image
			//image = cv::Mat(color_img_.rows, color_img_.cols, CV_8UC3);
			color_img_.copyTo(image);
			// Transform captured image from BGR space to HSV space
			cv::cvtColor(image, hsv, CV_BGR2HSV);
			// Check trackObject exists 
			if (trackObject) {
				// Ignore dark region in image
				// mask is set 255 if hsv is between lowerbound and upperbound and 0 otherwise
				cv::inRange(hsv, cv::Scalar(0, 60, 32), cv::Scalar(180, 255, 255), mask);

				// Array for mixChannels() to specify which channels are copied 
				int ch[] = { 0, 0 };
				// Create hue image
				hue.create(hsv.size(), hsv.depth());
				// Extract hue values from hsv image 
				cv::mixChannels(&hsv, 1, &hue, 1, ch, 1);

				// Create histgram of the object to track
				// Execute only once
				if (trackObject < 0) {
					// Extract hue values from hue in selsection size
					// roi is hue values of the object to track
					cv::Mat roi(hue, selection);
					//	Extract 1 or 0 values from mask in selection 
					cv::Mat maskroi(mask, selection);

					// Calculate the histogram of roi 
					// Source image: roi
					// The number of source image: 1
					// the dimention of hannels used to compute: 0
					// Mask image: maskroi
					// The number of bins: hist
					// Histogram dimentionality: 1
					// Array of histogram size: hsize
					// The range of histogram: phranges
					cv::calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);

					// Normalize histogram
					cv::normalize(hist, hist, 0, 255, cv::NORM_MINMAX);
					// Set specified rectangle to trackWindow
					trackWindow = selection;
					// Increament trackObject to execute this code block once
					trackObject = 1;
				}

				// Calculate backprojection image backproj using hist
				cv::calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
				// Set 0 to pixels of dark region 
				backproj &= mask;

				// Calculate rotated rectangle using camshift
				cv::RotatedRect trackBox = cv::CamShift(backproj, trackWindow, cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 10, 1));

				cv::ellipse(image, trackBox, cv::Scalar(0, 0, 255), 3, LINE_AA);
				cv::circle(image, cv::Point(trackBox.center.x, trackBox.center.y), 2, cv::Scalar(0, 0, 255), 3);
				//cout << "Distance between the center of object and image: " << cv::norm(cv::Point(trackBox.center.x, trackBox.center.y) - cv::Point(320, 240)) << endl;
				//cout << "Move vector: (" << trackBox.center.x - 320 << "," << trackBox.center.y - 240 << ")" << endl;
				if (trackingMode) {
					// Sending data
					memset(buf_, 0, sizeof(buf_));
					sprintf(buf_, "%f %f %f %f %f %f %f %f", mass_center_.x(), mass_center_.y(), mass_center_.z(), 0.0f, trackBox.center.x - 320, trackBox.center.y - 240, 0.0f, 0.0f);
					sendto(sock_send_,
						buf_, strlen(buf_), 0, (struct sockaddr *)&addr_send_, sizeof(addr_send_));
				}
			}

			//選択オブジェクトがある時に色を変える。
			if (selectObject && selection.width > 0 && selection.height > 0) {
				cv::Mat roi(image, selection);
				cv::bitwise_not(roi, roi);
			}
			cv::circle(image, cv::Point(320, 240), 2, cv::Scalar(255, 255, 0), 1);

			cv::imshow("Camera", image);


			cv::waitKey(30);
		}
	}

	// Filter point cloud by distance
	void filterPassThrough(const CloudConstPtr &cloud, Cloud &result)
	{
		CloudPtr cloud_tmp(new Cloud);
		//FPS_CALC_BEGIN;
		pcl::PassThrough<PointType> pass;
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.0, 1.0);
		//pass.setFilterLimits (0.0, 1.5);
		//pass.setFilterLimits (0.0, 0.6);
		pass.setKeepOrganized(false);
		pass.setInputCloud(cloud);
		pass.filter(*cloud_tmp);
		pass.setFilterFieldName("x");
		pass.setFilterLimits(-0.5, 0.2);
		pass.setKeepOrganized(false);
		pass.setInputCloud(cloud_tmp);
		pass.filter(result);

		//FPS_CALC_END("filterPassThrough");
	}

	// Remove outliers using a statisticaloutlierremoval filter
	void statisticalRemoval(const CloudConstPtr &cloud,
		Cloud &result, int numOfNeighbors, double sd)
	{
		pcl::StatisticalOutlierRemoval<PointType> sor;
		sor.setInputCloud(cloud);
		sor.setKeepOrganized(false);
		// Set the number of neighbors to analyze for each point 
		sor.setMeanK(numOfNeighbors);
		// Remove sd standard deviation far from mean 
		sor.setStddevMulThresh(sd);
		sor.filter(result);
	}

	// Segmentate each object by euclidean cluster
	void euclideanSegment(const CloudConstPtr &cloud,
		std::vector<pcl::PointIndices> &cluster_indices)
	{
		// Begin calculation of FPS
		//FPS_CALC_BEGIN;
		// Make euclideancluster object
		pcl::EuclideanClusterExtraction<PointType> ec;
		// Use kdtree for clustering
		KdTreePtr tree(new KdTree());

		// Set cluster threshold distance from other clusters
		// 0.05meters
		ec.setClusterTolerance(0.05); // 2cm
		ec.setMinClusterSize(50);
		ec.setMaxClusterSize(25000);
		ec.setSearchMethod(tree);
		ec.setInputCloud(cloud);
		ec.extract(cluster_indices);
		// End calculation of FPS
		//FPS_CALC_END("euclideanSegmentation");
	}


	// Filter point cloud by voxel grid
	void gridSample(const CloudConstPtr &cloud, Cloud &result, double leaf_size = 0.01)
	{
		// Begin calculation of FPS
		//FPS_CALC_BEGIN;
		// Get the current time for FPS
		double start = pcl::getTime();
		// Make voxelgrid object for filtering
		pcl::VoxelGrid<PointType> grid;

		// Why is this line commented out?
		//pcl::ApproximateVoxelGrid<PointType> grid;

		// Set voxel size to leaf_size (default: 0.01meter in each dimension)  6
		grid.setLeafSize(float(leaf_size), float(leaf_size), float(leaf_size));
		// Set input point cloud 
		grid.setInputCloud(cloud);
		// Get filtered point cloud
		grid.filter(result);
		// Get the current time for FPS
		double end = pcl::getTime();
		// Calculate the computation time
		downsampling_time_ = end - start;
		// Print FPS calculate time
		//FPS_CALC_END("gridSample");
	}

	void gridSampleApprox(const CloudConstPtr &cloud, Cloud &result, double leaf_size = 0.01)
	{
		//FPS_CALC_BEGIN;
		double start = pcl::getTime();
		//pcl::VoxelGrid<PointType> grid;
		pcl::ApproximateVoxelGrid<PointType> grid;
		grid.setLeafSize(static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
		grid.setInputCloud(cloud);
		grid.filter(result);
		//result = *cloud;
		double end = pcl::getTime();
		downsampling_time_ = end - start;
		//FPS_CALC_END("gridSample");
	}

	// Function to segment plane from point cloud
	void planeSegmentation(const CloudConstPtr &cloud,
		pcl::ModelCoefficients &coefficients,
		pcl::PointIndices &inliers)
	{
		// Begin calculation for fps
		//FPS_CALC_BEGIN;
		// Make sacsegmentation object
		pcl::SACSegmentation<PointType> seg;
		// Optimize the estimated plane coefficients to reduce the mean-squared-error
		seg.setOptimizeCoefficients(true);
		// Use plane model
		seg.setModelType(pcl::SACMODEL_PLANE);
		// Use RANSAC to estimate plane
		seg.setMethodType(pcl::SAC_RANSAC);
		// Set max iteration to 1000
		seg.setMaxIterations(1000);
		// Set distance threshold to 0.03 meters (distance from estimated plane)
		seg.setDistanceThreshold(0.003);
		// Set input cloud
		seg.setInputCloud(cloud);
		// Get the result (inliers: indices for plane point cloud, coefficients: plane coefficents)
		seg.segment(inliers, coefficients);
		// End calcuation for FPS
		//FPS_CALC_END("planeSegmentation");
	}

	// Project points onto plane
	void planeProjection(const CloudConstPtr &cloud,
		Cloud &result,
		const pcl::ModelCoefficients::ConstPtr &coefficients)
	{
		// Begin calculation of FPS
		//FPS_CALC_BEGIN;
		// Make projectinliers object
		pcl::ProjectInliers<PointType> proj;
		// Use plane model
		proj.setModelType(pcl::SACMODEL_PLANE);
		// Set input cloud
		proj.setInputCloud(cloud);

		// Set plane coefficients
		proj.setModelCoefficients(coefficients);
		// Get filtered cloud
		proj.filter(result);
		// End calculation of FPS
		//FPS_CALC_END("planeProjection");
	}

	// Calculate convex hull of projected plane
	void convexHull(const CloudConstPtr &cloud,
		Cloud &,
		std::vector<pcl::Vertices> &hull_vertices)
	{
		// Begin calculation of FPS
		//FPS_CALC_BEGIN;
		// Make convex hull object
		pcl::ConvexHull<PointType> chull;
		// Set input cloud
		chull.setInputCloud(cloud);
		// Get convex hull
		chull.reconstruct(*cloud_hull_, hull_vertices);
		// End calculation of FPS
		//FPS_CALC_END("convexHull");
	}

	void normalEstimation(const CloudConstPtr &cloud,
		pcl::PointCloud<pcl::Normal> &result)
	{
		//FPS_CALC_BEGIN;
		ne_.setInputCloud(cloud);
		ne_.compute(result);
		//FPS_CALC_END("normalEstimation");
	}

	void tracking(const RefCloudConstPtr &cloud)
	{
		double start = pcl::getTime();
		//FPS_CALC_BEGIN;
		tracker_->setInputCloud(cloud);
		tracker_->compute();
		double end = pcl::getTime();
		//FPS_CALC_END("tracking");
		tracking_time_ = end - start;
	}

	void addNormalToCloud(const CloudConstPtr &cloud,
		const pcl::PointCloud<pcl::Normal>::ConstPtr &,
		RefCloud &result)
	{
		result.width = cloud->width;
		result.height = cloud->height;
		result.is_dense = cloud->is_dense;
		for (size_t i = 0; i < cloud->points.size(); i++)
		{
			RefPointType point;
			point.x = cloud->points[i].x;
			point.y = cloud->points[i].y;
			point.z = cloud->points[i].z;
			point.rgba = cloud->points[i].rgba;
			// point.normal[0] = normals->points[i].normal[0];
			// point.normal[1] = normals->points[i].normal[1];
			// point.normal[2] = normals->points[i].normal[2];
			result.points.push_back(point);
		}
	}

	// Extract non-plane points
	void extractNonPlanePoints(const CloudConstPtr &cloud,
		const CloudConstPtr &cloud_hull,
		Cloud &result)
	{
		// Make	 
		pcl::ExtractPolygonalPrismData<PointType> polygon_extract;
		// Indices for points lying inside polygonal prism
		pcl::PointIndices::Ptr inliers_polygon(new pcl::PointIndices());
		// Set height parameters (0.01m to 10m from the plane) 
		polygon_extract.setHeightLimits(0.005, 10.0);
		// Set convex hull of plane
		polygon_extract.setInputPlanarHull(cloud_hull);
		// Set input cloud
		polygon_extract.setInputCloud(cloud);
		// Get indices inside prism
		polygon_extract.segment(*inliers_polygon);
		{
			pcl::ExtractIndices<PointType> extract_positive;
			extract_positive.setNegative(false);
			extract_positive.setInputCloud(cloud);
			extract_positive.setIndices(inliers_polygon);
			extract_positive.filter(result);
		}
	}

	void removeZeroPoints(const CloudConstPtr &cloud,
		Cloud &result)
	{
		for (size_t i = 0; i < cloud->points.size(); i++)
		{
			PointType point = cloud->points[i];
			if (!(fabs(point.x) < 0.01 &&
				fabs(point.y) < 0.01 &&
				fabs(point.z) < 0.01) &&
				!pcl_isnan(point.x) &&
				!pcl_isnan(point.y) &&
				!pcl_isnan(point.z))
				result.points.push_back(point);
		}

		result.width = static_cast<pcl::uint32_t> (result.points.size());
		result.height = 1;
		result.is_dense = true;
	}

	// Extract 1 cluster from segmentated clusters
	void extractSegmentCluster(const CloudConstPtr &cloud,
		const std::vector<pcl::PointIndices> cluster_indices,
		const int segment_index,
		Cloud &result)
	{
		// Indices given cluster number
		pcl::PointIndices segmented_indices = cluster_indices[segment_index];
		for (size_t i = 0; i < segmented_indices.indices.size(); i++)
		{
			// Extract indexed points from point cloud
			PointType point = cloud->points[segmented_indices.indices[i]];
			result.points.push_back(point);
		}
		// Set width and height
		result.width = pcl::uint32_t(result.points.size());
		result.height = 1;
		result.is_dense = true;
	}

	// Cheack if request has come from robot
	void listen()
	{
		// Initialize fds_
		memcpy(&fds_, &readfds_, sizeof(fd_set));

		// Wait untile fds gets readable in timeout time
		select(0, &fds_, NULL, NULL, &tv_);

		// Cheack readable data is in sock_recv_ 
		if (FD_ISSET(sock_recv_, &fds_)) {
			//	Extract message from sock to buf_
			memset(buf_, 0, sizeof(buf_));
			recv(sock_recv_, buf_, sizeof(buf_), 0);

			// Parse the message
			if (strcmp(buf_, "Scanning") == 0) {
				// Scan objects on a plane
				cout << "Scanning start..." << endl;
				scanning_ = true;
			}
			else if (strcmp(buf_, "TrackingON") == 0) {
				cout << "Tracking start..." << endl;
				trackingMode = true;
			}
			else if (strcmp(buf_, "TrackingOFF") == 0) {
				cout << "Tracking end..." << endl;
				trackingMode = false;
			}
			// Print data
			//printf("%s\n", buf_);
		}
	}

	// Cheack if request has come from robot
	bool isRequestCome()
	{
		// Initialize fds_
		memcpy(&fds_, &readfds_, sizeof(fd_set));

		// Wait untile fds gets readable in timeout time
		select(0, &fds_, NULL, NULL, &tv_);

		// Cheack readable data is in sock_recv_ 
		if (FD_ISSET(sock_recv_, &fds_)) {
			request_ = true;
			//	Recieve data from sock
			memset(buf_, 0, sizeof(buf_));
			recv(sock_recv_, buf_, sizeof(buf_), 0);

			// Check if request is traking mode or not
			if (strcmp(buf_, "ON") == 0) {
				cout << "Tracking start" << endl;
				trackingMode = true;
			}
			else if (strcmp(buf_, "OFF") == 0) {
				cout << "Tracking end" << endl;
				trackingMode = false;
			}

			// Print data
			//printf("%s\n", buf_);
			cout << "Now request from robot has come." << endl;
			cout << "Ready to calculate object to grasp...";
			return true;
		}
		return false;
	}

	// Extract color data and depth data from pointcloud
	void extractColorImage(const CloudConstPtr &cloud)
	{
		// Initialize color_img_ with the size of point cloud
		color_img_ = cv::Mat(cloud->height, cloud->width, CV_8UC3);
		// Extract colored point in each loop
		for (int h = 0; h < color_img_.rows; h++) {
			for (int w = 0; w < color_img_.cols; w++) {
				// Get point that contains RGB data and depth data
				PointType point = cloud->at(w, h);
				// Get RGB data from point
				Eigen::Vector3i rgb = point.getRGBVector3i();

				// Store RGB data to color_img_
				color_img_.at<cv::Vec3b>(h, w)[0] = rgb[2];
				color_img_.at<cv::Vec3b>(h, w)[1] = rgb[1];
				color_img_.at<cv::Vec3b>(h, w)[2] = rgb[0];
			}
		}
	}


	// Callback function when updating point cloud
	// This function is called when the point cloud is accuired from camera
	void
		cloud_cb(const CloudConstPtr &cloud)
	{
		// Lock mtx_ (why?) 
		boost::mutex::scoped_lock lock(mtx_);
		// Reset cloud_pass (why?)
		cloud_pass_.reset(new Cloud);
		// Reset cloud_pass_downsampled_ (why?)
		cloud_pass_downsampled_.reset(new Cloud);
		// Variables for detected plane
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

		// Extract color data and depth data from pointcloud
		// Create color image and depth image
		// Initialize color_img_ with the size of point cloud
		color_img_ = cv::Mat(cloud->height, cloud->width, CV_8UC3);
		// Initialize depth32F_img_ with the size of point cloud
		depth32F_img_ = cv::Mat(cloud->height, cloud->width, CV_32FC1);
		// Initialize depth_img_ with the size of point cloud
		depth_img_ = cv::Mat(cloud->height, cloud->width, CV_8UC1);

		// Extract colored point in each loop
		for (int h = 0; h < color_img_.rows; h++) {
			for (int w = 0; w < color_img_.cols; w++) {
				// Get point that contains RGB data and depth data
				PointType point = cloud->at(w, h);
				// Get RGB data from point
				Eigen::Vector3i rgb = point.getRGBVector3i();

				// Store RGB data to color_img_
				color_img_.at<cv::Vec3b>(h, w)[0] = rgb[2];
				color_img_.at<cv::Vec3b>(h, w)[1] = rgb[1];
				color_img_.at<cv::Vec3b>(h, w)[2] = rgb[0];
				// Store Depth data to depth32F_img_
				depth32F_img_.at<float>(h, w) = cloud->at(w, h).z;
			}
		}
		// Normalize depth image from 32bit float to 8bit unsinged char to make it easy to process
		cv::normalize(depth32F_img_, depth_img_, 0, 255, CV_MINMAX, CV_8UC1);

		// Filter point cloud accuired from camera
		filterPassThrough(cloud, *cloud_pass_);
		// Cheack if request has come from robot
		//isRequestCome();
		listen();
		// Mode to calculate the centroid of object
		if (calc_object_ || scanning_) {
			// Reset flags
			calc_object_ = false;
			scanning_ = false;

			// Do statisticalremoval and Set filtered cloud to downsampled cloud
			statisticalRemoval(cloud_pass_, *cloud_pass_downsampled_, 50, 1.0);

			// Pointer for object's point cloud 
			CloudPtr target_cloud;
			// Segmentate plane from downsampled point cloud
			// Set plane indices to inliers
			planeSegmentation(cloud_pass_downsampled_, *coefficients, *inliers);
			// Cheack whether plane is detected or not
			// I don'nt know why 3.
			if (inliers->indices.size() > 3)
			{
				cout << "A plane was detected..." << endl;
				// Variable for projected cloud 
				CloudPtr cloud_projected(new Cloud);
				// ???? 
				cloud_hull_.reset(new Cloud);
				nonplane_cloud_.reset(new Cloud);

				// Project downsampled cloud onto plane
				planeProjection(cloud_pass_downsampled_, *cloud_projected, coefficients);
				// Get convex hull of projected cloud
				convexHull(cloud_projected, *cloud_hull_, hull_vertices_);
				// Extract objects supported by a plane
				extractNonPlanePoints(cloud_pass_downsampled_, cloud_hull_, *nonplane_cloud_);
				// Objects point cloud
				target_cloud = nonplane_cloud_;
			}
			else
			{
				PCL_WARN("Cannot segment plane...\n");
			}

			// Cheack if target_cloud is not empty
			if (target_cloud != NULL)
			{
				cout << "Start clustering point cloud on the plane..." << endl;
				// Segmentate each object by euclidan cluster
				PCL_INFO("Clustering, please wait...\n");
				// Get indices of each segmentated object
				std::vector<pcl::PointIndices> cluster_indices;
				euclideanSegment(target_cloud, cluster_indices);
				// Cheack if any objects exist
				if (cluster_indices.size() > 0)
				{
					cout << "The number of clusters: " << cluster_indices.size() << endl;
					// Select the cluster to track
					CloudPtr temp_cloud(new Cloud);
					// Extract 0th object cluster (the biggest cluster) from point cloud
					extractSegmentCluster(target_cloud, cluster_indices, 0, *temp_cloud);
					cout << "The number of points in a cluster: " << temp_cloud->width << endl;
					// Calculate OBB of the object
					drawOBB_ = true;
					pcl::MomentOfInertiaEstimation <PointType> feature_extractor;
					feature_extractor.setInputCloud(temp_cloud);
					feature_extractor.compute();
					feature_extractor.getMomentOfInertia(moment_of_inertia_);
					feature_extractor.getEccentricity(eccentricity_);
					feature_extractor.getOBB(min_point_OBB_, max_point_OBB_, position_OBB_, rotational_matrix_OBB_);
					// Calculate eigen vectors of the object point cloud
					feature_extractor.getEigenValues(major_value_, middle_value_, minor_value_);
					// Calculate eigen values of the object point cloud
					feature_extractor.getEigenVectors(major_vector_, middle_vector_, minor_vector_);
					// Calculate centroid
					feature_extractor.getMassCenter(mass_center_);
					// Calculate width of object
					Eigen::Vector3f p1_(min_point_OBB_.x, min_point_OBB_.y, min_point_OBB_.z);
					Eigen::Vector3f p5_(min_point_OBB_.x, max_point_OBB_.y, min_point_OBB_.z);
					Eigen::Vector3f distance = p1_ - p5_;
					cout << "Width of object: " << distance.norm() * 1000 << " mm" <<endl;
					// Calculate direction of the object 
					Eigen::Vector2f xAxis(1.0f, 0.0f);
					Eigen::Vector2f objectAxis;
					if (major_vector_.x() > 0 && major_vector_.z() < 0) {
						objectAxis.x() = -major_vector_.x();
						objectAxis.y() = -major_vector_.z();
					}
					else if (major_vector_.x() < 0 && major_vector_.z() < 0) {
						objectAxis.x() = -major_vector_.x();
						objectAxis.y() = -major_vector_.z();
					}
					else {
						objectAxis.x() = major_vector_.x();
						objectAxis.y() = major_vector_.z();
					}
					xAxis.normalize();
					objectAxis.normalize();
					// Show the orientation angle of object
					cout << "Angle of the object: " << acos(objectAxis.dot(xAxis)) * 180.0f / 3.1415 << " degree" << endl;
					float angle = acos(objectAxis.dot(xAxis));
					// Scaling the object vector to fit OBB
					major_vector_ *= 0.2;
					middle_vector_ *= 0.2;
					minor_vector_ *= 0.2;

					// Sending data
					memset(buf_, 0, sizeof(buf_));
					sprintf(buf_, "%f %f %f %f %f %f %f %f", mass_center_.x(), mass_center_.y(), mass_center_.z(), angle, 0, 0, 0, 0);
					sendto(sock_send_,
						buf_, strlen(buf_), 0, (struct sockaddr *)&addr_send_, sizeof(addr_send_));

				}
				else
				{
					PCL_WARN("euclidean segmentation failed\n");
				}
			}
		}

		new_cloud_ = true;
		counter_++;
	}

	// Keyboard callback function
	void
		keyboard_callback(const pcl::visualization::KeyboardEvent& event, void*)
	{
		if (event.getKeySym() == "t" && event.keyDown())
			calc_object_ = true;
	}

	// Function to run openNI2Grabber
	void
		run()
	{
		//  Make OpenNI2Grabber object
		pcl::Grabber* interface = new pcl::io::OpenNI2Grabber(device_id_);
		//	Callable entity for cloud_cb 
		boost::function<void(const CloudConstPtr&)> f = boost::bind(&OpenNISegmentTracking::cloud_cb, this, _1);
		// Register cloud callback function with grabber interface
		interface->registerCallback(f);
		// Run a viz_cb function on th UI thread
		viewer_.runOnVisualizationThread(boost::bind(&OpenNISegmentTracking::viz_cb, this, _1), "viz_cb");
		viewer_.registerKeyboardCallback(&OpenNISegmentTracking::keyboard_callback, *this);
		// Start the streams
		interface->start();
		//	 
		while (!viewer_.wasStopped()) {
			// 
			boost::this_thread::sleep(boost::posix_time::seconds(1));
		}
		// Stop OpenNI2Grabber
		interface->stop();
	}

	pcl::visualization::CloudViewer viewer_;

	pcl::PointCloud<pcl::Normal>::Ptr normals_;
	CloudPtr cloud_pass_;
	CloudPtr cloud_pass_downsampled_;
	CloudPtr plane_cloud_;
	CloudPtr nonplane_cloud_;
	CloudPtr cloud_hull_;
	CloudPtr segmented_cloud_;
	CloudPtr reference_;
	std::vector<pcl::Vertices> hull_vertices_;

	std::string device_id_;
	boost::mutex mtx_;
	bool new_cloud_;
	pcl::NormalEstimationOMP<PointType, pcl::Normal> ne_; // to store threadpool
	boost::shared_ptr<ParticleFilter> tracker_;
	int counter_;
	bool use_convex_hull_;
	bool visualize_non_downsample_;
	bool visualize_particles_;
	double tracking_time_;
	double computation_time_;
	double downsampling_time_;
	double downsampling_grid_size_;
	bool calc_object_ = false;
	bool request_ = false;
	bool scanning_ = false;

	// Variable for object OBB
	bool drawOBB_ = false;
	std::vector <float> moment_of_inertia_;
	std::vector <float> eccentricity_;
	RefPointType min_point_AABB_;
	RefPointType max_point_AABB_;
	RefPointType min_point_OBB_;
	RefPointType max_point_OBB_;
	RefPointType position_OBB_;
	Eigen::Matrix3f rotational_matrix_OBB_;
	float major_value_, middle_value_, minor_value_;
	Eigen::Vector3f major_vector_, middle_vector_, minor_vector_;
	Eigen::Vector3f mass_center_;

	// Socket for receiving
	SOCKET sock_recv_;
	// Socket for sending
	SOCKET sock_send_;
	// Structure to store socket adress
	struct sockaddr_in addr_recv_, addr_send_;
	// Variable to use select()
	fd_set fds_, readfds_;
	// Variable for text
	char buf_[2048];
	// Structure to store Window socket information
	WSADATA wsaData_;
	// Structure for timeout
	struct timeval tv_;

	// Variables for image
	cv::Mat color_img_;
	cv::Mat depth32F_img_;
	cv::Mat depth_img_;
};

void
usage(char** argv)
{
	std::cout << "usage: " << argv[0] << " <device_id> [-C] [-g]\n\n";
	std::cout << "  -C:  initialize the pointcloud to track without plane segmentation"
		<< std::endl;
	std::cout << "  -D: visualizing with non-downsampled pointclouds."
		<< std::endl;
	std::cout << "  -P: not visualizing particle cloud."
		<< std::endl;
	std::cout << "  -fixed: use the fixed number of the particles."
		<< std::endl;
	std::cout << "  -d <value>: specify the grid size of downsampling (defaults to 0.01)."
		<< std::endl;
}

// MAIN FUNCTION
int
main(int argc, char** argv)
{
	// Variable for 
	bool use_convex_hull = true;
	bool visualize_non_downsample = false;
	bool visualize_particles = true;
	bool use_fixed = false;

	double downsampling_grid_size = 0.01;

	if (pcl::console::find_argument(argc, argv, "-C") > 0)
		use_convex_hull = false;
	if (pcl::console::find_argument(argc, argv, "-D") > 0)
		visualize_non_downsample = true;
	if (pcl::console::find_argument(argc, argv, "-P") > 0)
		visualize_particles = false;
	if (pcl::console::find_argument(argc, argv, "-fixed") > 0)
		use_fixed = true;
	pcl::console::parse_argument(argc, argv, "-d", downsampling_grid_size);
	if (argc < 2)
	{
		usage(argv);
		exit(1);
	}


	std::string device_id = std::string(argv[1]);

	if (device_id == "--help" || device_id == "-h")
	{
		usage(argv);
		exit(1);
	}


	// Create openi object
	OpenNISegmentTracking<pcl::PointXYZRGBA> v(device_id, 8, downsampling_grid_size,
		use_convex_hull,
		visualize_non_downsample, visualize_particles,
		use_fixed);
	v.run();
}