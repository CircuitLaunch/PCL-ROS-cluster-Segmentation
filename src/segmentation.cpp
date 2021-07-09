/*

ROS node for point cloud cluster based segmentaion of cluttered objects on table

Author: Sean Cassero
7/15/15

*/
// /\ /\ Modified from this base



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
//#include <centroid.h>
#include <pcl/common/centroid.h>
#include <pcl/recognition/ransac_based/obj_rec_ransac.h>

#include <vector>
#include <iostream>
#include <algorithm>
#include <iterator>

class segmentation {

public:

  explicit segmentation(ros::NodeHandle nh) : m_nh(nh)  {

    // define the subscriber and publisher
    m_sub = m_nh.subscribe ("/camera/depth/color/points", 1, &segmentation::cloud_cb, this);
    m_clusterPub = m_nh.advertise<obj_recognition::SegmentedClustersArray> ("obj_recognition/pcl_clusters",1);

    // Publisher object for dummy_cluster    
    dummy_pub = m_nh.advertise<sensor_msgs::PointCloud2> ("obj_recognition/dummy_cluster",1);

    // Publisher object for dummy_cluster 1
    dummy_pub1 = m_nh.advertise<sensor_msgs::PointCloud2> ("obj_recognition/dummy_cluster_1",1);
    //dummy_segmenter_pub = m_nh.advertise<sensor_msgs::PointCloud2> ("obj_recognition/dummy_segmenter_pub", 1);  

    // Publisher object for dummy_cluster 2
    dummy_pub2 = m_nh.advertise<sensor_msgs::PointCloud2> ("obj_recognition/dummy_cluster_2",1);
    
    // Publisher object for dummy_cluster 3
    dummy_pub3 = m_nh.advertise<sensor_msgs::PointCloud2> ("obj_recognition/dummy_cluster_3",1);

    // Publisher to visualize the segmentation object -- Visualize the plane that's being removed before iterating through indices
    plane_cloud = m_nh.advertise<sensor_msgs::PointCloud2> ("obj_recognition/plane_cluster",1);


    // Implementing buffer and listener
    buffer_.reset(new sensor_msgs::PointCloud2);
    buffer_->header.frame_id = "camera_link";

  }

private:

// Publishers and Subscribers
ros::NodeHandle m_nh;
ros::Publisher m_pub;
ros::Subscriber m_sub;
ros::Publisher m_clusterPub;
ros::Publisher dummy_pub;
ros::Publisher dummy_pub1;
ros::Publisher dummy_pub2;
ros::Publisher dummy_pub3;
ros::Publisher plane_cloud;
// Implementing buffer and listener
tf::TransformListener listener_;
sensor_msgs::PointCloud2::Ptr buffer_;

// Function call
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





  //// Perform passthrough filtering to remove table leg

  // create a pcl object to hold the passthrough filtered results
  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrFiltered (xyz_cloud_filtered);


  // Create the filtering object (pass is the filtering object's name?)
  // Creating multiple passthrough objects in order to control 3 parameters
  pcl::PassThrough<pcl::PointXYZRGB> pass_z;
  pass_z.setInputCloud (xyzCloudPtr);
  pass_z.setFilterFieldName ("z");
  pass_z.setFilterLimits (0.0, 2.0);
  pass_z.setFilterLimitsNegative (true);
  pass_z.filter (*xyzCloudPtrFiltered);

  pcl::PassThrough<pcl::PointXYZRGB> pass_x;
  pass_x.setInputCloud (xyzCloudPtr);
  pass_x.setFilterFieldName ("x");
  pass_x.setFilterLimits (1.0, 2.0);
  pass_x.setFilterLimitsNegative (true);
  pass_x.filter (*xyzCloudPtrFiltered);

  pcl::PassThrough<pcl::PointXYZRGB> pass_y;
  pass_y.setInputCloud (xyzCloudPtr);
  pass_y.setFilterFieldName ("y");
  pass_y.setFilterLimits (0.0, 2.0);
  pass_y.setFilterLimitsNegative (true);
  pass_y.filter (*xyzCloudPtrFiltered);

  // create a pcl object to hold the ransac filtered results
  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_ransac_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrRansacFiltered (xyz_cloud_ransac_filtered);

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
  //seg1.setMaxIterations(100);
  // Distance to the model threshold (0.01 = 1 cm) -- is this distance from the plane? Larger values mean bigger planes
  // planes encompass larger spaces
  seg1.setDistanceThreshold (0.01);

  seg1.setInputCloud (xyzCloudPtrFiltered);
  seg1.segment (*inliers, *coefficients);

  // Error Check
  if (inliers->indices.size () == 0 ) {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
  }

  //// End of passthrough filtering ////






  //// Publish the "plane" that we're segmenting out ////

  // Trying to create cloud that's populated with inlier's indices, so that we can publish it
  // and therefore visualize the plane that's being removed when doing our clusters
  // for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)

  // create a pcl object to hold the extracted cluster for the plane
  pcl::PointCloud<pcl::PointXYZRGB> *plane_cluster = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_clusterPtr (plane_cluster);
  pcl::PointXYZRGB plane_xyzpoint;

  for (std::vector<int>::const_iterator pit = inliers->indices.begin (); pit != inliers->indices.end (); ++pit) {

    // Populate plane_cluster
    plane_xyzpoint = xyzCloudPtrFiltered->points[*pit];
    plane_xyzpoint.r = 50 % 255;
    plane_xyzpoint.g = 50 % 255;
    plane_xyzpoint.b = 50 % 255;
    plane_clusterPtr->points.push_back(plane_xyzpoint);


  }
  // Type conversions to publish plane_cluster
  
  // declare the plane_output variable instances
  sensor_msgs::PointCloud2 plane_output;
  plane_output.header = cloud_msg->header;
  pcl::PCLPointCloud2 plane_outputPCL;

  // convert to pcl::PCLPointCloud2
  pcl::toPCLPointCloud2( *plane_clusterPtr ,plane_outputPCL);

  // Convert to ROS data type
  pcl_conversions::fromPCL(plane_outputPCL, plane_output);

  // Reset Headers
  plane_output.header = cloud_msg->header;

  // Publish plane_output
  ros::Duration(0.5).sleep();
  plane_cloud.publish(plane_output);

  //// end of plane_cloud publishing ////




  //// Extra ////
  //std::cout << inliers->indices.size() << std::endl;

  //std::cerr << "Model coefficients: " << coefficients->values[0] << " "
  //          << coefficients->values[1] << " "
  //          << coefficients->values[2] << " "
  //          << coefficients->values[3] << std::endl;

  //printf("%i\n",xyzCloudPtrFiltered->size());
  //// End of Extra ////




  //// This section should be removing the planar section ////

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  //extract.setInputCloud (xyzCloudPtrFiltered); 
  extract.setInputCloud (xyzCloudPtrFiltered);
  extract.setIndices (inliers);
  // Retrieve indices to all points in xyzCloudPtrFiltered except those
  // referenced by inliers
  extract.setNegative (true);
  extract.filter (*xyzCloudPtrRansacFiltered);


  // perform euclidean cluster segmentation to separate individual objects

  // Create the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (xyzCloudPtrRansacFiltered);


  // create the extraction object for the clusters
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  // specify euclidean cluster parameters
  ec.setClusterTolerance (0.02); // 2cm -- Set the spatial cluster tolerance as a measure in the L2 Euclidean space
  // /\ set the search radius of of neighbor search to 0.02 = 2 cm
  ec.setMinClusterSize (100);
  //ec.setMinClusterSize (100);
  ec.setMaxClusterSize (1000);
  //ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  // Set xyzCloudPtrRansacFiltered as the main cloud for the ec extraction object
  ec.setInputCloud (xyzCloudPtrRansacFiltered);
  // exctract the indices pertaining to each cluster (in xyzCloudPtrRansacFiltered)
  // and store in a vector of pcl::PointIndices named cluster_indices
  ec.extract (cluster_indices);


  // declare an instance of the SegmentedClustersArray message
  obj_recognition::SegmentedClustersArray CloudClusters;

  //// End of removing the planar section ////




  //// Section on output visualizer instances ////

  // declare the output variable instances -- trying to add a reference frame to output
  sensor_msgs::PointCloud2 output;
  output.header = cloud_msg->header;
  pcl::PCLPointCloud2 outputPCL;
  // Declaring centroid variables
  pcl::CentroidPoint<pcl::PointXYZ> output_centroid;
  // don't need to populate this, just need to run output.get(output_centroid) and that should work?
  // no, I do need to populate output_centroid
  // actual: I need to create an instance of CentroidPoint<pcl::PointXYZ>, populate it with the indeces in
  // output, create an instance of pcl::PointXYZ, and do  [centroid instance].get([point instance])
  // Crafting centroid getter?
  pcl::PointXYZRGB c_getter;


  // declare the output_1 variable instances
  sensor_msgs::PointCloud2 output_1;
  output_1.header = cloud_msg->header;
  pcl::PCLPointCloud2 outputPCL_1;
  // Declaring centroid variables
  pcl::CentroidPoint<pcl::PointXYZ> output_1_centroid;
  // Declaring centroid 1 getter
  pcl::PointXYZRGB c_1_getter;


  // declare the output_2 variable instances
  sensor_msgs::PointCloud2 output_2;
  output_2.header = cloud_msg->header;
  pcl::PCLPointCloud2 outputPCL_2;
  // Declaring centroid variables
  pcl::CentroidPoint<pcl::PointXYZ> output_2_centroid;
  // Declaring centroid 2 getter
  pcl::PointXYZRGB c_2_getter;


  // declare the output_3 variable instances
  sensor_msgs::PointCloud2 output_3;
  output_3.header = cloud_msg->header;
  pcl::PCLPointCloud2 outputPCL_3;
  // Declaring centroid variables
  pcl::CentroidPoint<pcl::PointXYZ> output_3_centroid;
  // Declaring centroid 3 getter
  pcl::PointXYZRGB c_3_getter;

  //// End of output visualizer instances ////




  //// Needs work! Start of Object Recognition implementation ////
  // Defining recognition object
  //pcl::recognition::ObjRecRANSAC cube_checker(40, 5.0);
  // Adding models to the recognition object using .addModel()
  //cube_checker.addModel();
  //// End of Object Recognition dummy code ////





  //// Object declarations used during for loop iterations ////

  // create a pcl object to hold the extracted cluster
  pcl::PointCloud<pcl::PointXYZRGB> *cluster = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterPtr (cluster);

  // Creating the object to iterate over and populate with xyzCloudPtrRansacFiltered, xyzpoint
  pcl::PointXYZRGB xyzpoint;

  // Declaring objects for centroid adders(objects that add to centroids for each cluster we're tracking)
  pcl::PointXYZ centroid_adder;
  pcl::PointXYZ centroid_1_adder;
  pcl::PointXYZ centroid_2_adder;
  pcl::PointXYZ centroid_3_adder;

  //// End of for loop objct declarations ////




  //// Testing -- Declarations for storing previous values ////
  //pcl::CentroidPoint<pcl::PointXYZ> output_centroid_past;
  //pcl::PointXYZRGB c_getter_past;
  //pcl::PointXYZ centroid_adder_past;

  //std::vector< pcl::PointXYZRGB > centroid_holder = { };

  // Done with this for now -- get ready for demo and return to this

  //// End of second testing for object recognition ////




  //// For loops in order to iterate through clusters within the filtered point cloud, and then to iterate through each cluster idividually ////

  // Counter to log and check for which cluster we are in during the for loop. Resets to 0 every time callback function is called
  // (ie code runs through and counter is at some number. After the callback is run again, counter is reset to 0 thanks to this)
  int counter = 0;
  // here, cluster_indices is a vector of indices for each cluster. iterate through each indices object to work with them seporately
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {


    // now we are in a vector of indices pertaining to a single cluster.
    // Assign each point corresponding to this cluster in xyzCloudPtrPassthroughFiltered a specific color for identification purposes
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      {
      // iterates through the points of xyzRansacFiltered and appends them into cluster, using the pointer
      // that points to cluster, clusterPtr
      xyzpoint = xyzCloudPtrRansacFiltered->points[*pit];
      xyzpoint.r = (counter*50) % 255;
      xyzpoint.g = (counter*100) % 255;
      xyzpoint.b = (counter*150) % 255;
      clusterPtr->points.push_back(xyzpoint);

      // Populating centroid objects for 0th cloud
      pcl::copyPoint(xyzpoint, centroid_adder);
      output_centroid.add (centroid_adder);

      // Populating centroid objects for 1st cloud
      pcl::copyPoint(xyzpoint, centroid_1_adder);
      output_1_centroid.add (centroid_1_adder);

      // Populating centroid objects for 2nd cloud
      pcl::copyPoint(xyzpoint, centroid_2_adder);
      output_2_centroid.add (centroid_2_adder);

      // Populating centroid objects for 3rd cloud
      pcl::copyPoint(xyzpoint, centroid_3_adder);
      output_3_centroid.add (centroid_3_adder);

      }



    // log the position of the cluster
    //clusterData.position[0] = (*cloudPtr).data[0];
    //clusterData.position[1] = (*cloudPtr).points.back().y;
    //clusterData.position[2] = (*cloudPtr).points.back().z;
    //std::string info_string = string(cloudPtr->points.back().x);
    //printf(clusterData.position[0]);

    // convert to pcl::PCLPointCloud2
    pcl::toPCLPointCloud2( *clusterPtr ,outputPCL);

    pcl::toPCLPointCloud2( *clusterPtr ,outputPCL_1);

    pcl::toPCLPointCloud2( *clusterPtr ,outputPCL_2);

    pcl::toPCLPointCloud2( *clusterPtr ,outputPCL_3);

    // Convert to ROS data type
    pcl_conversions::fromPCL(outputPCL, output);

    pcl_conversions::fromPCL(outputPCL_1, output_1);

    pcl_conversions::fromPCL(outputPCL_2, output_2);

    pcl_conversions::fromPCL(outputPCL_3, output_3);

    // Reset Headers
    output.header = cloud_msg->header;

    output_1.header = cloud_msg->header;

    output_2.header = cloud_msg->header;

    output_3.header = cloud_msg->header;

    // add the cluster to the array message
    //clusterData.cluster = output;
    CloudClusters.clusters.push_back(output);


    
    //// Publishing the clusters one by one within the for loop as an attempt to observe them ////
    // Introucing delay in order to better visualize clusters
    ros::Duration(0.5).sleep();
    if (counter == 0 ) {
      dummy_pub.publish(output);
      // Displaying centroid
      output_centroid.get(c_getter);
      std::cout << "c_getter: " << c_getter << std::endl;
      // Portion where we attempt to store centroid values into centroid_holder
      //centroid_holder.push_back(c_getter);
    }

    
    if (counter == 1) {
      dummy_pub1.publish(output_1);
      // Displaying centroid
      output_centroid.get(c_1_getter);
      std::cout << "c_1_getter: " << c_1_getter << std::endl;
    }


    if (counter == 2) {
      dummy_pub2.publish(output_2);
      // Displaying centroid
      output_centroid.get(c_2_getter);
      std::cout << "c_2_getter: " << c_2_getter << std::endl;
    }


    if (counter == 3) {
      dummy_pub3.publish(output_3);
      // Displaying centroid
      output_centroid.get(c_3_getter);
      std::cout << "c_3_getter: " << c_3_getter << std::endl;
    }
    
    //// End of individual cluster visualizations

    counter++;
  } //// End of for loop population ////

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
