#include <ros/package.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tr1/memory>
#include "cv.h"
#include <sstream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/norms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/common/common.h>

#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <sys/types.h>
#include <boost/filesystem.hpp>
#include <sys/stat.h>
#include <errno.h>
#include <libgen.h>
#include <string.h>
#include "std_msgs/String.h"

// ROS
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <pose_estimation/PoseEstimation.h>
typedef pose_estimation::PoseEstimation MsgT;

using namespace std;
using namespace tr1;
using namespace message_filters;
using namespace sensor_msgs;
using namespace tf;
using namespace pcl_ros;
ros::ServiceClient global;
MsgT msgglobal;

std::string folder;
std::string counter;
typedef pcl::PointXYZRGBA PointT;
cv::Point3f getColor(int number);



void displayPose(MsgT out){
	pcl::PointCloud<PointT>::Ptr scene(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr object(new pcl::PointCloud<PointT>());
	pcl::fromROSMsg(out.response.scene, *scene);
	pcl::fromROSMsg(out.response.object, *object);
	int v1(0);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Pose estimation"));
	pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(scene);
	viewer->addPointCloud<PointT> (scene, rgb, "Scene", v1);
	viewer->setCameraPosition(0,0,-0.5, 0,0,1, 0,-1,0);

	//add detected objects
	for (int i = 0; i <  msgglobal.response.pose_value.size(); i++){
		std::stringstream name;
		name << i << ", pose goodness: " << msgglobal.response.pose_value[i];
		pcl::PointCloud<PointT>::Ptr final(new pcl::PointCloud<PointT>());

		tf::Transform transform;
		tf::transformMsgToTF(out.response.vizualizerPoses[i], transform);

		Eigen::Matrix4f m_init, m;
		transformAsMatrix(transform, m_init);

		m_init(12) = m_init(12)/1000;
		m_init(13) = m_init(13)/1000;
		m_init(14) = m_init(14)/1000;
		m = m_init;


		pcl::transformPointCloud(*object, *final, m);
		cv::Point3f color = getColor(i);
		viewer->addPointCloud<PointT> (final, name.str());
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, color.x, color.y, color.z, name.str());
		viewer->addText(name.str(), 0, 0 + i*30, color.x, color.y, color.z, name.str());
	}

	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}


}



bool printResults(MsgT msgglobal){
       // if (msgglobal.response.pose_value.size() == 0) return false;
	std::cout << "number of ints:" << msgglobal.response.labels_int.size() << std::endl;
	std::cout << "number of pose_value:" << msgglobal.response.pose_value.size() << std::endl;
	for(size_t n = 0; n < msgglobal.response.pose_value.size(); n++) {
		pcl::console::print_warn("Pose goodness value: %f\n", msgglobal.response.pose_value[n]);
		pcl::console::print_warn("Pose:\n");
		std::cout << msgglobal.response.poses[n] << std::endl;
	}


    pcl::PointCloud<PointT>::Ptr scene(new pcl::PointCloud<PointT>());

    pcl::fromROSMsg(msgglobal.response.scene, *scene);
    if (scene->size() > 0 ) pcl::io::savePCDFile("curr_scene.pcd", *scene);
    else pcl::console::print_error("curr scene is zero!");
	displayPose(msgglobal);
	return true;
}

void specifyObject(std::string& input) {
	if(!input.compare("1")) {
		ROS_INFO("Subscribing to detection service...");
		ros::service::waitForService("/inSceneDetector/detect");

			
		msgglobal.request.scenario = "save_point_clouds";
//for (int i = 0; i < 100; i++)
		if(global.call(msgglobal)) {
			printResults(msgglobal);
		}

	} else if(!input.compare("2")) {
		ROS_INFO("Subscribing to detection service...");
		ros::service::waitForService("/inSceneDetector/detect");
	//	msgglobal.request.visualize = true;
		msgglobal.request.scenario = "detect_rotorcaps_on_coveyour_belt";

		if(global.call(msgglobal)) {
while (msgglobal.response.pose_value.size() == 0) global.call(msgglobal);			
			printResults(msgglobal); 
			//global.call(msgglobal) ;
				
		}
	}
	else if(!input.compare("3")) {
		ROS_INFO("Subscribing to detection service...");
		ros::service::waitForService("/inSceneDetector/detect");
//		msgglobal.request.visualize = false;
		msgglobal.request.scenario = "detect_only_rotorcaps";

		if(global.call(msgglobal)) {
			printResults(msgglobal);
		}
	}
    else if(!input.compare("4")) {
        ROS_INFO("Subscribing to detection service...");
        ros::service::waitForService("/inSceneDetector/detect");
        msgglobal.request.scenario = "detect_rotorcaps_on_table";

        if(global.call(msgglobal)) {
            while (msgglobal.response.pose_value.size() == 0) global.call(msgglobal); 
printResults(msgglobal);
        }
    }
    else if(!input.compare("6")) {
        ROS_INFO("Subscribing to detection service...");
        ros::service::waitForService("/inSceneDetector/detect");
        msgglobal.request.scenario = "detect_rotorcaps_on_the_fixture";

        if(global.call(msgglobal)) {
            //while (msgglobal.response.poses.size() == 0) global.call(msgglobal);
           printResults(msgglobal);
        }
    }
   else if(!input.compare("7")) {
        ROS_INFO("Subscribing to detection service...");
        ros::service::waitForService("/inSceneDetector/detect");
        msgglobal.request.scenario = "detect_rotoraxles";

        if(global.call(msgglobal)) {
            //while (msgglobal.response.poses.size() == 0) global.call(msgglobal);
           printResults(msgglobal);
        }
    }
    else if(!input.compare("8")) {
        ROS_INFO("Subscribing to detection service...");
        ros::service::waitForService("/inSceneDetector/detect");
        msgglobal.request.scenario = "detect_ring";

        if(global.call(msgglobal)) {
            //while (msgglobal.response.poses.size() == 0) global.call(msgglobal);
           printResults(msgglobal);
        }
    }
    else if(!input.compare("9")) {
        ROS_INFO("Subscribing to detection service...");
        ros::service::waitForService("/inSceneDetector/detect");
        msgglobal.request.scenario = "detect_magnet";

        if(global.call(msgglobal)) {
            //while (msgglobal.response.poses.size() == 0) global.call(msgglobal);
           printResults(msgglobal);
        }
    }
 else if(!input.compare("5")) {
        ROS_INFO("Subscribing to detection service...");
        ros::service::waitForService("/inSceneDetector/detect");
        msgglobal.request.scenario = "detect_screen_shot";

        if(global.call(msgglobal)) {
            printResults(msgglobal);
        }
    }
//------------------------------------------------------------------
	unsigned found = folder.find_last_of("/\\");
	folder = folder.substr(0, found);
}


bool checkIfQuit(std::string& input) {

	if(!input.compare("quit"))
		exit(0);
	else if (input.compare("back"))
		return false;

	unsigned found = folder.find_last_of("/\\");
	folder = folder.substr(0, found);
	return true;
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "middle_node");
	ros::NodeHandle nodeHandle = ros::NodeHandle("~");


	global = nodeHandle.serviceClient<MsgT>("/inSceneDetector/detect");

	cout << "==========================================================================================" << endl;
	cout << "        Type 'quit' to close                                                              " << endl;
	cout << "        1 - save carmine & || stereo point clouds into files carmine_PC.pcd stereo_PC.pcd " << endl;
	cout << "        2 - detect conveyour belt and rotorcaps                                           " << endl;
	cout << "        3 - detect only rotorcaps (conveyour belt position from '2' is used)              " << endl;
        cout << "        4 - detect table and rotorcaps                                                    " << endl;
        cout << "        6 - detect rotorcaps on fixture                                                   " << endl;
  	cout << "        7 - detect rotoraxles                                                             " << endl;
  	cout << "        8 - detect ring                                                                   " << endl;
  	cout << "        9 - detect magnet                                                                 " << endl;
	cout << "==========================================================================================" << endl;

	while (ros::ok()) {

		while (ros::ok()) {
			while (ros::ok()) {

				while (ros::ok()) {
					cout << "${test}/> " << "Object to look for : ";
					cin >> counter;

					if (checkIfQuit(counter))
						break;
					specifyObject(counter);

					ros::spinOnce();
				}

			}
		}
	}

	return 0;
}

cv::Point3f getColor(int number){
	switch (number){
	case 0:
		return cv::Point3f(1,0,0);
		break;
	case 1:
		return cv::Point3f(0,1,0);
		break;
	case 2:
		return cv::Point3f(0,0,1);
		break;
	case 3:
		return cv::Point3f(1,1,0);
		break;
	case 4:
		return cv::Point3f(1,0,1);
		break;
	case 5:
		return cv::Point3f(0.3,1,0.5);
		break;
	case 6:
		return cv::Point3f(0.5,0,0);
		break;
	case 7:
		return cv::Point3f(0,1,1);
		break;
	case 8:
		return cv::Point3f(1,0.5,0.5);
		break;
	case 9:
		return cv::Point3f(0,0.5,0);
		break;
	case 10:
		return cv::Point3f(0,1,0.5);
		break;
	case 11:
		return cv::Point3f(0.3,0.7,0.1);
		break;
	case 12:
		return cv::Point3f(0.8,1.0,0.0);
		break;
	case 13:
		return cv::Point3f(0.01,0.54,0.5);
		break;
	case 14:
		return cv::Point3f(0.00,0.04,0.5);
		break;
	case 15:
		return cv::Point3f(0.01,0.01,0.0);
		break;
	case 16:
		return cv::Point3f(0.08,0.01,0.0);
		break;
	case 17:
		return cv::Point3f(0.08,0.1,0.3);
		break;
	case 18:
		return cv::Point3f(0.1,0.2,0.3);
		break;
	case 19:
		return cv::Point3f(0.4,0.5,0.6);
		break;
	case 20:
		return cv::Point3f(0.7,0,0.8);
		break;
	}
	return cv::Point3f(0.5,1,1);
}

