/*

ROS node for point cloud cluster based segmentaion of cluttered objects on table

Author: Sean Cassero
7/15/15

*/


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <obj_recognition/SegmentedClustersArray.h>

// adding search library? And others...
#include <pcl/search/search.h>
#include <pcl/pcl_base.h>
#include <pcl/PointIndices.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/PCLPointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>


class segmentation {

public:

  explicit segmentation(ros::NodeHandle nh) : m_nh(nh)  {

    // define the subscriber and publisher
    m_sub = m_nh.subscribe ("/obj_recognition/point_cloud", 1, &segmentation::cloud_cb, this);
    m_clusterPub = m_nh.advertise<obj_recognition::SegmentedClustersArray> ("obj_recognition/pcl_clusters",1);
    dummy_pub = m_nh.advertise<sensor_msgs::PointCloud2> ("obj_recognition/dummy_cluster",1);
    //dummy_pub = m_nh.advertise<pcl::PCLPointCloud2> ("obj_recognition/dummy_cluster",1);

    // Implementing buffer and listener
    buffer_.reset(new sensor_msgs::PointCloud2);
    buffer_->header.frame_id = "world";

  }

private:

ros::NodeHandle m_nh;
ros::Publisher m_pub;
ros::Subscriber m_sub;
ros::Publisher m_clusterPub;
ros::Publisher dummy_pub;
// Implementing buffer and listener
tf::TransformListener listener_;
sensor_msgs::PointCloud2::Ptr buffer_;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

}; // end class definition


// define callback function
void segmentation::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

  // Container for original & filtered data
  //    we aren't declaring the filtered cloud as a const pointer -- makes sense since we're intending to change it
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  // These two lines are not in PCL-ROS-segmentation tutorial -- not in the readme, but they are in the
  //    segmentation.cpp file provided on the same github page...
  pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2Ptr cloudFilteredPtr (cloud_filtered);


  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);


  // Perform voxel grid downsampling filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.01, 0.01, 0.01);
  sor.filter (*cloudFilteredPtr);



  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtr (xyz_cloud); // need a boost shared pointer for pcl function inputs

  // convert the pcl::PointCloud2 tpye to pcl::PointCloud<pcl::PointXYZRGB>
  pcl::fromPCLPointCloud2(*cloudFilteredPtr, *xyzCloudPtr);


  //perform passthrough filtering to remove table leg

  // create a pcl object to hold the passthrough filtered results
  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrFiltered (xyz_cloud_filtered);

  // Create the filtering object (pass is the filtering object's name?)
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (xyzCloudPtr);
  pass.setFilterFieldName ("z");
  //pass.setFilterLimits (1.1, 2.0);
  pass.setFilterLimits (0.3, 2.0);
  pass.setFilterLimitsNegative (true);
  pass.filter (*xyzCloudPtrFiltered);

  // figure out if above is empty or not -- is the thing that xyzCloudPtrFiltered is pointing to empty?
  // it's not empty -- has size 1577
  //printf("%i\n",xyzCloudPtrFiltered->size());


  // create a pcl object to hold the ransac filtered results
  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_ransac_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrRansacFiltered (xyz_cloud_ransac_filtered);

  //printf("%i\n",xyzCloudPtrRansacFiltered->size());

  // perform ransac planar filtration to remove table top
  // This section should set the table as an inliers
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg1;
  // Optional
  seg1.setOptimizeCoefficients (true);
  // Mandatory
  seg1.setModelType (pcl::SACMODEL_PLANE);
  seg1.setMethodType (pcl::SAC_RANSAC);
  seg1.setDistanceThreshold (0.04);

  seg1.setInputCloud (xyzCloudPtrFiltered);
  seg1.segment (*inliers, *coefficients);

  // Error Check
  if (inliers->indices.size () == 0 ) {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
  }


  //printf("%i\n",xyzCloudPtrFiltered->size());


  //printf("%i\n",xyzCloudPtrRansacFiltered->size());

  // This section should be removing the table
  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  //extract.setInputCloud (xyzCloudPtrFiltered); 
  extract.setInputCloud (xyzCloudPtrFiltered);
  extract.setIndices (inliers);
  // Retrieve indices to all points in xyzCloudPtrFiltered except those
  // referenced by inliers
  extract.setNegative (true);
  extract.filter (*xyzCloudPtrRansacFiltered);

  
  //printf("%i\n",xyzCloudPtrRansacFiltered->size());

  // perform euclidean cluster segmentation to separate individual objects

  // Create the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (xyzCloudPtrRansacFiltered);




  // create the extraction object for the clusters
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  // specify euclidean cluster parameters
  ec.setClusterTolerance (0.0175); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (xyzCloudPtrRansacFiltered);
  // exctract the indices pertaining to each cluster (in xyzCloudPtrRansacFiltered)
  // and store in a vector of pcl::PointIndices named cluster_indices
  ec.extract (cluster_indices);

  //printf("%i\n", cluster_indices.begin () );
  //printf("%i\n", cluster_indices.end () );

  // decl//are an instance of the SegmentedClustersArray message
  obj_recognition::SegmentedClustersArray CloudClusters;

  // declare the output variable instances -- trying to add a reference frame to output
  sensor_msgs::PointCloud2 output;
  output.header = cloud_msg->header;
  pcl::PCLPointCloud2 outputPCL;

  // printing out output's header hopefully
  //std::cout << output.header << std::endl;

  int counter = 0;
  // here, cluster_indices is a vector of indices for each cluster. iterate through each indices object to work with them seporately
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    
    // create a new clusterData message object
    //obj_recognition::ClusterData clusterData;


    // create a pcl object to hold the extracted cluster
    pcl::PointCloud<pcl::PointXYZRGB> *cluster = new pcl::PointCloud<pcl::PointXYZRGB>;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterPtr (cluster);

    // now we are in a vector of indices pertaining to a single cluster.
    // Assign each point corresponding to this cluster in xyzCloudPtrPassthroughFiltered a specific color for identification purposes
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      {
      // iterates through the points of xyzRansacFiltered and appends them into cluster, using the pointer
      // that points to cluster, clusterPtr
      pcl::PointXYZRGB xyzpoint = xyzCloudPtrRansacFiltered->points[*pit];
      xyzpoint.r = (counter*50) % 255;
      xyzpoint.g = (counter*100) % 255;
      xyzpoint.b = (counter*150) % 255;
      clusterPtr->points.push_back(xyzpoint);


      // printing out pit
      //std::cout << *pit << std::endl;

      }


    // log the position of the cluster
    //clusterData.position[0] = (*cloudPtr).data[0];
    //clusterData.position[1] = (*cloudPtr).points.back().y;
    //clusterData.position[2] = (*cloudPtr).points.back().z;
    //std::string info_string = string(cloudPtr->points.back().x);
    //printf(clusterData.position[0]);

    // convert to pcl::PCLPointCloud2
    pcl::toPCLPointCloud2( *clusterPtr ,outputPCL);

    // Convert to ROS data type
    pcl_conversions::fromPCL(outputPCL, output);

    output.header = cloud_msg->header;

    // add the cluster to the array message
    //clusterData.cluster = output;
    CloudClusters.clusters.push_back(output);

    ros::Duration(0.5).sleep();
    // publishing the clusters one by one within the for loop as an attempt to observe them
    if (true) {
      dummy_pub.publish(output);
    }

    // printing out it
    //std::cout << *it << std::endl;
    counter++;
  }

  //std::cout << output.header << std::endl;

  //dummy_pub.publish(output);


  // publish the clusters
  m_clusterPub.publish(CloudClusters);



}



int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "segmentation");
  ros::NodeHandle nh;

  segmentation segs(nh);

  while(ros::ok())
  ros::spin ();

}
