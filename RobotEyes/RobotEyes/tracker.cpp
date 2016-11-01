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
#include <pcl/common/angles.h>>

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
			viz.removeShape("OBB");
			Eigen::Vector3f position(position_OBB_.x, position_OBB_.y, position_OBB_.z);
			Eigen::Quaternionf quat(rotational_matrix_OBB_);
			viz.addCube(position, quat, max_point_OBB_.x - min_point_OBB_.x, max_point_OBB_.y - min_point_OBB_.y, max_point_OBB_.z - min_point_OBB_.z, "OBB");
		}

		// When tracking is available 
		if (new_cloud_ && reference_)
		{
			bool ret = drawParticles(viz);
			if (ret)
			{
				drawResult(viz);

				// draw some texts
				viz.removeShape("N");
				viz.addText((boost::format("number of Reference PointClouds: %d") % tracker_->getReferenceCloud()->points.size()).str(),
					10, 20, 20, 1.0, 1.0, 1.0, "N");

				viz.removeShape("M");
				viz.addText((boost::format("number of Measured PointClouds:  %d") % cloud_pass_downsampled_->points.size()).str(),
					10, 40, 20, 1.0, 1.0, 1.0, "M");

				viz.removeShape("tracking");
				viz.addText((boost::format("tracking:        %f fps") % (1.0 / tracking_time_)).str(),
					10, 60, 20, 1.0, 1.0, 1.0, "tracking");

				viz.removeShape("downsampling");
				viz.addText((boost::format("downsampling:    %f fps") % (1.0 / downsampling_time_)).str(),
					10, 80, 20, 1.0, 1.0, 1.0, "downsampling");

				viz.removeShape("computation");
				viz.addText((boost::format("computation:     %f fps") % (1.0 / computation_time_)).str(),
					10, 100, 20, 1.0, 1.0, 1.0, "computation");

				viz.removeShape("particles");
				viz.addText((boost::format("particles:     %d") % tracker_->getParticles()->points.size()).str(),
					10, 120, 20, 1.0, 1.0, 1.0, "particles");

			}
		}
		// Cloud is not new
		new_cloud_ = false;
	}

	// Filter point cloud by distance
	void filterPassThrough(const CloudConstPtr &cloud, Cloud &result)
	{
		CloudPtr cloud_tmp(new Cloud);
		FPS_CALC_BEGIN;
		pcl::PassThrough<PointType> pass;
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.0, 1.5);
		//pass.setFilterLimits (0.0, 1.5);
		//pass.setFilterLimits (0.0, 0.6);
		pass.setKeepOrganized(false);
		pass.setInputCloud(cloud);
		pass.filter(*cloud_tmp);
		pass.setFilterFieldName("x");
		pass.setFilterLimits(-0.5, 0.5);
		pass.setKeepOrganized(false);
		pass.setInputCloud(cloud_tmp);
		pass.filter(result);

		FPS_CALC_END("filterPassThrough");
	}

	// Remove outliers using a statisticaloutlierremoval filter
	void statisticalRemoval(const CloudConstPtr &cloud,
		Cloud &result, int numOfNeighbors,double sd)
	{
		pcl::StatisticalOutlierRemoval<PointType> sor;
		sor.setInputCloud(cloud);
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
		FPS_CALC_BEGIN;
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
		FPS_CALC_END("euclideanSegmentation");
	}


	// Filter point cloud by voxel grid
	void gridSample(const CloudConstPtr &cloud, Cloud &result, double leaf_size = 0.01)
	{
		// Begin calculation of FPS
		FPS_CALC_BEGIN;
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
		FPS_CALC_END("gridSample");
	}

	void gridSampleApprox(const CloudConstPtr &cloud, Cloud &result, double leaf_size = 0.01)
	{
		FPS_CALC_BEGIN;
		double start = pcl::getTime();
		//pcl::VoxelGrid<PointType> grid;
		pcl::ApproximateVoxelGrid<PointType> grid;
		grid.setLeafSize(static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
		grid.setInputCloud(cloud);
		grid.filter(result);
		//result = *cloud;
		double end = pcl::getTime();
		downsampling_time_ = end - start;
		FPS_CALC_END("gridSample");
	}

	// Function to segment plane from point cloud
	void planeSegmentation(const CloudConstPtr &cloud,
		pcl::ModelCoefficients &coefficients,
		pcl::PointIndices &inliers)
	{
		// Begin calculation for fps
		FPS_CALC_BEGIN;
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
		FPS_CALC_END("planeSegmentation");
	}

	// Project points onto plane
	void planeProjection(const CloudConstPtr &cloud,
		Cloud &result,
		const pcl::ModelCoefficients::ConstPtr &coefficients)
	{
		// Begin calculation of FPS
		FPS_CALC_BEGIN;
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
		FPS_CALC_END("planeProjection");
	}

	// Calculate convex hull of projected plane
	void convexHull(const CloudConstPtr &cloud,
		Cloud &,
		std::vector<pcl::Vertices> &hull_vertices)
	{
		// Begin calculation of FPS
		FPS_CALC_BEGIN;
		// Make convex hull object
		pcl::ConvexHull<PointType> chull;
		// Set input cloud
		chull.setInputCloud(cloud);
		// Get convex hull
		chull.reconstruct(*cloud_hull_, hull_vertices);
		// End calculation of FPS
		FPS_CALC_END("convexHull");
	}

	void normalEstimation(const CloudConstPtr &cloud,
		pcl::PointCloud<pcl::Normal> &result)
	{
		FPS_CALC_BEGIN;
		ne_.setInputCloud(cloud);
		ne_.compute(result);
		FPS_CALC_END("normalEstimation");
	}

	void tracking(const RefCloudConstPtr &cloud)
	{
		double start = pcl::getTime();
		FPS_CALC_BEGIN;
		tracker_->setInputCloud(cloud);
		tracker_->compute();
		double end = pcl::getTime();
		FPS_CALC_END("tracking");
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

	// Calculate object AABB
	void calcAABB(const CloudConstPtr &cloud)
	{

	}

	// Callback function when updating point cloud
	void
		cloud_cb(const CloudConstPtr &cloud)
	{
		cout << "accquire point cloud" << endl;
		// Lock mtx_ (why?) 
		boost::mutex::scoped_lock lock(mtx_);
		// Get the current time for FPS
		double start = pcl::getTime();
		// Begin calculation of FPS
		FPS_CALC_BEGIN;
		// Reset cloud_pass (why?)
		cloud_pass_.reset(new Cloud);
		// Reset cloud_pass_downsampled_ (why?)
		cloud_pass_downsampled_.reset(new Cloud);
		// Variable for plane model
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
		// Filter point cloud accuired from camera
		filterPassThrough(cloud, *cloud_pass_);


		// Mode to calculate the centroid of object
		if (calc_object_) {
			cout << "ŒvŽZ‚·‚é‚æ[" << endl;
			// Do statisticalremoval and Set filtered cloud to downsampled cloud
			statisticalRemoval(cloud_pass_, *cloud_pass_downsampled_, 50, 1.0);
			//	Pointer for objects point cloud 
			CloudPtr target_cloud;
			calc_object_ = false;
			// Segment plane from downsampled point cloud
			// Get plane indices(inliers)
			planeSegmentation(cloud_pass_downsampled_, *coefficients, *inliers);
			// Cheack whether plane is detected or not
			if (inliers->indices.size() > 3)
			{
				//	Variable for projected cloud 
				CloudPtr cloud_projected(new Cloud);
				// 
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
				PCL_WARN("cannot segment plane\n");
			}

			// Cheack if target_cloud is not empty
			if (target_cloud != NULL)
			{
				cout << "start clustering" << endl;
				// Segmentate each object by euclidan cluster
				PCL_INFO("segmentation, please wait...\n");
				// Get indices of each segmentated object
				std::vector<pcl::PointIndices> cluster_indices;
				euclideanSegment(target_cloud, cluster_indices);
				// Cheack if any objects exist
				if (cluster_indices.size() > 0)
				{
					// select the cluster to track
					CloudPtr temp_cloud(new Cloud);
					// Extract 0th object cluster from point cloud
					extractSegmentCluster(target_cloud, cluster_indices, 0, *temp_cloud);
					// Vector for centroid of object
					Eigen::Vector4f c;
					// Compute the centroid of extracted object cloud
					pcl::compute3DCentroid<RefPointType>(*temp_cloud, c);
					cout << "X: " << c[0] << endl;
					cout << "Y: " << c[1] << endl;
					cout << "Z: " << c[2] << endl;
	
					// Calculate object AABB
					drawOBB_ = true;
					pcl::MomentOfInertiaEstimation <PointType> feature_extractor;
					feature_extractor.setInputCloud(temp_cloud);
					feature_extractor.compute();
					feature_extractor.getMomentOfInertia(moment_of_inertia_);
					feature_extractor.getEccentricity(eccentricity_);
					feature_extractor.getOBB(min_point_OBB_, max_point_OBB_, position_OBB_, rotational_matrix_OBB_);
					feature_extractor.getEigenValues(major_value_, middle_value_, minor_value_);
					feature_extractor.getEigenVectors(major_vector_, middle_vector_, minor_vector_);
					feature_extractor.getMassCenter(mass_center_);
				}
				else
				{
					PCL_WARN("euclidean segmentation failed\n");
				}
			}
		}
		// 
		//if (counter_ < 10)
		//{
		//	// Filter point cloud by voxelgrid
		//	gridSample(cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);
		//}
		//// 
		//else if (counter_ == 10)
		//{
		//	// Why is this line commented out?
		//	//gridSample (cloud_pass_, *cloud_pass_downsampled_, 0.01);

		//	// Set filtered cloud to downsampled cloud
		//	cloud_pass_downsampled_ = cloud_pass_;
		//	//	Pointer for objects point cloud 
		//	CloudPtr target_cloud;
		//	// Use convex hull
		//	if (use_convex_hull_)
		//	{
		//		// Segment plane from downsampled point cloud
		//		// Get plane indices(inliers)
		//		planeSegmentation(cloud_pass_downsampled_, *coefficients, *inliers);
		//		// Cheack whether plane is detected or not
		//		if (inliers->indices.size() > 3)
		//		{
		//			//	Variable for projected cloud 
		//			CloudPtr cloud_projected(new Cloud);
		//			// 
		//			cloud_hull_.reset(new Cloud);
		//			nonplane_cloud_.reset(new Cloud);

		//			// Project downsampled cloud onto plane
		//			planeProjection(cloud_pass_downsampled_, *cloud_projected, coefficients);
		//			// Get convex hull of projected cloud
		//			convexHull(cloud_projected, *cloud_hull_, hull_vertices_);
		//			// Extract objects supported by a plane
		//			extractNonPlanePoints(cloud_pass_downsampled_, cloud_hull_, *nonplane_cloud_);
		//			// Objects point cloud
		//			target_cloud = nonplane_cloud_;
		//		}
		//		else
		//		{
		//			PCL_WARN("cannot segment plane\n");
		//		}
		//	}
		//	else
		//	{
		//		// Use downsampled cloud( not segmentated) 
		//		PCL_WARN("without plane segmentation\n");
		//		target_cloud = cloud_pass_downsampled_;
		//	}

		//	// Cheack if target_cloud is not empty
		//	if (target_cloud != NULL)
		//	{
		//		cout << "start clustering" << endl;
		//		// Segmentate each object by euclidan cluster
		//		PCL_INFO("segmentation, please wait...\n");
		//		// Get indices of each segmentated object
		//		std::vector<pcl::PointIndices> cluster_indices;
		//		euclideanSegment(target_cloud, cluster_indices);
		//		// Cheack if any objects exist
		//		if (cluster_indices.size() > 0)
		//		{
		//			// select the cluster to track
		//			CloudPtr temp_cloud(new Cloud);
		//			// Extract 0th object cluster from point cloud
		//			extractSegmentCluster(target_cloud, cluster_indices, 0, *temp_cloud);
		//			// Vector for centroid of object
		//			Eigen::Vector4f c;
		//			// Compute the centroid of extracted object cloud
		//			pcl::compute3DCentroid<RefPointType>(*temp_cloud, c);
		//			cout << "X: " << c[0] << endl;
		//			cout << "Y: " << c[1] << endl;
		//			cout << "Z: " << c[2] << endl;
		//			// What is this line doing?
		//			int segment_index = 0;
		//			// Compute segment distance (x^2 + y^2)
		//			double segment_distance = c[0] * c[0] + c[1] * c[1];

		//			for (size_t i = 1; i < cluster_indices.size(); i++)
		//			{
		//				temp_cloud.reset(new Cloud);
		//				extractSegmentCluster(target_cloud, cluster_indices, int(i), *temp_cloud);
		//				pcl::compute3DCentroid<RefPointType>(*temp_cloud, c);
		//				double distance = c[0] * c[0] + c[1] * c[1];
		//				if (distance < segment_distance)
		//				{
		//					segment_index = int(i);
		//					segment_distance = distance;
		//				}
		//			}

		//			segmented_cloud_.reset(new Cloud);
		//			extractSegmentCluster(target_cloud, cluster_indices, segment_index, *segmented_cloud_);
		//			//pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
		//			//normalEstimation (segmented_cloud_, *normals);
		//			RefCloudPtr ref_cloud(new RefCloud);
		//			//addNormalToCloud (segmented_cloud_, normals, *ref_cloud);
		//			ref_cloud = segmented_cloud_;
		//			RefCloudPtr nonzero_ref(new RefCloud);
		//			removeZeroPoints(ref_cloud, *nonzero_ref);

		//			PCL_INFO("calculating cog\n");

		//			RefCloudPtr transed_ref(new RefCloud);
		//			pcl::compute3DCentroid<RefPointType>(*nonzero_ref, c);
		//			Eigen::Affine3f trans = Eigen::Affine3f::Identity();
		//			trans.translation().matrix() = Eigen::Vector3f(c[0], c[1], c[2]);
		//			//pcl::transformPointCloudWithNormals<RefPointType> (*ref_cloud, *transed_ref, trans.inverse());
		//			pcl::transformPointCloud<RefPointType>(*nonzero_ref, *transed_ref, trans.inverse());
		//			CloudPtr transed_ref_downsampled(new Cloud);
		//			gridSample(transed_ref, *transed_ref_downsampled, downsampling_grid_size_);
		//			tracker_->setReferenceCloud(transed_ref_downsampled);
		//			tracker_->setTrans(trans);
		//			reference_ = transed_ref;
		//			tracker_->setMinIndices(int(ref_cloud->points.size()) / 2);
		//		}
		//		else
		//		{
		//			PCL_WARN("euclidean segmentation failed\n");
		//		}
		//	}
		//}
		//else
		//{
		//	//normals_.reset (new pcl::PointCloud<pcl::Normal>);
		//	//normalEstimation (cloud_pass_downsampled_, *normals_);
		//	//RefCloudPtr tracking_cloud (new RefCloud ());
		//	//addNormalToCloud (cloud_pass_downsampled_, normals_, *tracking_cloud);
		//	//tracking_cloud = cloud_pass_downsampled_;

		//	//*cloud_pass_downsampled_ = *cloud_pass_;
		//	//cloud_pass_downsampled_ = cloud_pass_;
		//	gridSampleApprox(cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);
		//	tracking(cloud_pass_downsampled_);
		//}

		new_cloud_ = true;
		double end = pcl::getTime();
		computation_time_ = end - start;
		FPS_CALC_END("computation");
		counter_++;
	}

	// Callback function when image updated
	void
		image_cb(const boost::shared_ptr<pcl::io::openni2::DepthImage>& depth)
	{
		// Calculate the FPS of image viewer
		FPS_CALC("image callback");
		// Lock image_mutex
		boost::mutex::scoped_lock lock(image_mutex_);
		// Set updated image to member image
		image_ = image;

		// Cheack if updated image is suited to pcl image viewer 
		if (image->getEncoding() != pcl::io::openni2::Image::RGB)
		{
			// Make suited image 
			if (depth_data_ < image->getWidth() * image->getHeight())
			{
				if (depth_data_)
					delete[] depth_data_;
				depth_data_ = image->getWidth() * image->getHeight();
				depth_data_ = new unsigned char[depth_data_ * 3];
			}
			image_->fillRGB(image_->getWidth(), image_->getHeight(), rgb_data_);
		}

	}

	// Keyboard callback function
	void
		keyboard_callback(const pcl::visualization::KeyboardEvent& event, void*)
	{
		if (event.getKeyCode())
			cout << "the key \'" << event.getKeyCode() << "\' (" << event.getKeyCode() << ") was";
		else
			cout << "the special key \'" << event.getKeySym() << "\' was";
		if (event.keyDown())
			cout << " pressed" << endl;
		else
			cout << " released" << endl;

		if (event.getKeySym() == "t" && event.keyDown())
			calc_object_ = true;
	}

	// Function to run openNI2Grabber
	void
		run()
	{
		//  Make OpenNI2Grabber object
		pcl::Grabber* interface = new pcl::io::OpenNI2Grabber(device_id_);
		// 
		boost::function<void(const CloudConstPtr&)> f =
			boost::bind(&OpenNISegmentTracking::cloud_cb, this, _1);
		// Register cloud callback function with grabber interface
		interface->registerCallback(f);
		// Run a callable object on th UI thread
		viewer_.runOnVisualizationThread(boost::bind(&OpenNISegmentTracking::viz_cb, this, _1), "viz_cb");
		viewer_.registerKeyboardCallback(&OpenNISegmentTracking::keyboard_callback, *this);
		// Start the streams
		interface->start();

		// 
		while (!viewer_.wasStopped())
			// 
			boost::this_thread::sleep(boost::posix_time::seconds(1));
		// Stop OpenNI2Grabber
		interface->stop();
	}

	pcl::visualization::CloudViewer viewer_;
	pcl::visualization::ImageViewer imgViewer_;

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
	boost::mutex image_mutex_;
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

	// Variables for image
	boost::shared_ptr<pcl::io::openni2::DepthImage> image_;
	unsigned char* depth_data_;

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

	// Make Camera object
	OpenNISegmentTracking<pcl::PointXYZRGBA> v(device_id, 8, downsampling_grid_size,
		use_convex_hull,
		visualize_non_downsample, visualize_particles,
		use_fixed);
	v.run();
}