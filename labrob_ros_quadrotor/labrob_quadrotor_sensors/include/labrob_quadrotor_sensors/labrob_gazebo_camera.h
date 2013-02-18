/*
*  Gazebo - Outdoor Multi-Robot Simulator
*  Copyright (C) 2003  
*     Nate Koenig & Andrew Howard
*
*  This program is free software; you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation; either version 2 of the License, or
*  (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program; if not, write to the Free Software
*  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*
*/
/*
* Desc: A dynamic controller plugin that publishes ROS image topic for generic camera sensor.
* Author: John Hsu
* Date: 24 Sept 2008
* SVN: $Id$
*/
#ifndef ROS_CAMERA_HH
#define ROS_CAMERA_HH

#include <ros/ros.h>
#define USE_CBQ
#ifdef USE_CBQ
#include <ros/callback_queue.h>
#endif
#include "boost/thread/mutex.hpp"
#include <gazebo/Param.hh>
#include <gazebo/Controller.hh>

// image components
#include "cv_bridge/CvBridge.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
// used by polled_camera
#include "sensor_msgs/RegionOfInterest.h"

// prosilica components
// Stuff in image_common
#include <image_transport/image_transport.h>
#include <polled_camera/publication_server.h>
#include <polled_camera/GetPolledImage.h>

#ifdef SIMULATOR_GAZEBO_GAZEBO_ROS_CAMERA_DYNAMIC_RECONFIGURE
#include <gazebo_plugins/GazeboRosCameraConfig.h>
#include <dynamic_reconfigure/server.h>
#endif

#include <algorithm>
#include <assert.h>

#include <gazebo/Sensor.hh>
#include <gazebo/Model.hh>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
#include "gazebo/MonoCameraSensor.hh"


#include <sensor_msgs/fill_image.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <sensor_msgs/RegionOfInterest.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <boost/scoped_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/tokenizer.hpp>
#include <boost/thread.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <string>

namespace gazebo{
	class MonoCameraSensor;

/// @addtogroup gazebo_dynamic_plugins Gazebo ROS Dynamic Plugins
/// @{
/** \defgroup labrob_gazebo_camera Ros Camera Plugin XML Reference and Example

	\brief Gazebo ROS Prosilica Camera Plugin Controller.
	
	This is a controller that collects data from a simulated Prosilica camera sensor and makes image data available
	in the same manner prosilica_camera works.  The controller:gazebo_ros_prosilica block must be inside of a sensor:camera block.

	Example Usage:
	\verbatim
		<sensor:camera name="prosilica_sensor">
			<imageFormat>R8G8B8</imageFormat>
			<imageSize>2448 2050</imageSize>
			<hfov>45</hfov>
			<nearClip>0.1</nearClip>
			<farClip>100</farClip>
			<updateRate>20.0</updateRate>
			<controller:gazebo_ros_prosilica name="prosilica_controller" plugin="libgazebo_ros_prosilica.so">
				<alwaysOn>true</alwaysOn>
				<updateRate>20.0</updateRate>
				<imageTopicName>/prosilica/image_raw</imageTopicName>
				<cameraInfoTopicName>/prosilica/camera_info</cameraInfoTopicName>
				<pollServiceName>/prosilica/request_image</pollServiceName>
				<frameName>prosilica_frame</frameName>
				<CxPrime>1224.5</CxPrime>
				<Cx>1224.5</Cx>
				<Cy>1025.5</Cy>
				<focal_length>2955</focal_length> <!-- image_width / (2*tan(hfov_radian /2)) -->
				<distortion_k1>0.00000001</distortion_k1>
				<distortion_k2>0.00000001</distortion_k2>
				<distortion_k3>0.00000001</distortion_k3>
				<distortion_t1>0.00000001</distortion_t1>
				<distortion_t2>0.00000001</distortion_t2>
				<interface:camera name="prosilica_iface" />
			</controller:gazebo_ros_prosilica>
		</sensor:camera>
	\endverbatim

\{
*/

/**


		\brief labrob_gazebo_camera Controller.
					\li This is a controller that collects data from a simulated Prosilica camera sensor and makes image data available
	in the same manner prosilica_camera works.  The controller:gazebo_ros_prosilica block must be inside of a sensor:camera block.
					\li Example Usage:
	\verbatim
		<sensor:camera name="prosilica_sensor">
			<imageFormat>R8G8B8</imageFormat>
			<imageSize>2448 2050</imageSize>
			<hfov>45</hfov>
			<nearClip>0.1</nearClip>
			<farClip>100</farClip>
			<updateRate>20.0</updateRate>
			<controller:gazebo_ros_prosilica name="prosilica_controller" plugin="libgazebo_ros_prosilica.so">
				<alwaysOn>true</alwaysOn>
				<updateRate>20.0</updateRate>
				<imageTopicName>/prosilica/image_raw</imageTopicName>
				<cameraInfoTopicName>/prosilica/camera_info</cameraInfoTopicName>
				<pollServiceName>/prosilica/request_image</pollServiceName>
				<frameName>prosilica_frame</frameName>
				<CxPrime>1224.5</CxPrime>
				<Cx>1224.5</Cx>
				<Cy>1025.5</Cy>
				<focal_length>2955</focal_length> <!-- image_width / (2*tan(hfov_radian /2)) -->
				<distortion_k1>0.00000001</distortion_k1>
				<distortion_k2>0.00000001</distortion_k2>
				<distortion_k3>0.00000001</distortion_k3>
				<distortion_t1>0.00000001</distortion_t1>
				<distortion_t2>0.00000001</distortion_t2>
				<interface:camera name="prosilica_iface" />
			</controller:gazebo_ros_prosilica>
		</sensor:camera>
	\endverbatim
					.

*/

class labrob_gazebo_camera : public gazebo::Controller{
	public:
	/// \brief Constructor
	/// \param parent The parent entity, must be a Model or a Sensor
	labrob_gazebo_camera(Entity *parent);
	
	/// \brief Destructor
	virtual ~labrob_gazebo_camera();
// 	Entity* Controller;
	
	protected:
	/// \brief Load the controller
	/// \param node XML config node
	virtual void LoadChild(XMLConfigNode *node);
	
	/// \brief Init the controller
	virtual void InitChild();
	
	/// \brief Update the controller
	virtual void UpdateChild();
	
	/// \brief Finalize the controller, unadvertise topics
	virtual void FiniChild();
	
	private:
	/// \brief does nothing for now
	static void mouse_cb(int event, int x, int y, int flags, void* param) { };
	
	/// \brief Keep track of number of connctions
	int imageConnectCount;
	void ImageConnect();
	void ImageDisconnect();
	int infoConnectCount;
	void InfoConnect();
	void InfoDisconnect();
	
	void PutCameraData();
	void PublishCameraInfo();
	
	/// \brief A pointer to the parent camera sensor
	MonoCameraSensor *myParent;
	
	/// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
	ros::NodeHandle* rosnode_;
	
	/// \brief image_transport
	polled_camera::PublicationServer poll_srv_;      // Handles requests in polled mode
	
	std::string mode_;
	
	image_transport::ImageTransport* itnode_;
	image_transport::Publisher image_pub_;
	ros::Publisher camera_info_pub_;
	
	void pollCallback(polled_camera::GetPolledImage::Request& req,
														polled_camera::GetPolledImage::Response& rsp,
														sensor_msgs::Image& image, sensor_msgs::CameraInfo& info);
	
	/// \brief ros message
	/// \brief construct raw stereo message
	sensor_msgs::Image imageMsg;
	sensor_msgs::Image *roiImageMsg;
	sensor_msgs::CameraInfo cameraInfoMsg;
	sensor_msgs::CameraInfo *roiCameraInfoMsg;
	
	
	/// \brief Parameters
	ParamT<std::string> *imageTopicNameP;
	ParamT<std::string> *cameraInfoTopicNameP;
	ParamT<std::string> *pollServiceNameP;
	ParamT<std::string> *frameNameP;
	ParamT<std::string> *cameraNameP;
	
	ParamT<double> *CxPrimeP;           // rectified optical center x, for sim, CxPrime == Cx
	ParamT<double> *CxP;            // optical center x
	ParamT<double> *CyP;            // optical center y
	ParamT<double> *focal_lengthP;  // also known as focal length
	ParamT<double> *hack_baselineP;  // also known as focal length
	ParamT<double> *distortion_k1P; // linear distortion
	ParamT<double> *distortion_k2P; // quadratic distortion
	ParamT<double> *distortion_k3P; // cubic distortion
	ParamT<double> *distortion_t1P; // tangential distortion
	ParamT<double> *distortion_t2P; // tangential distortion
	ParamT<int>    *numParentP; // tangential distortion
	
	/// \brief for setting ROS name space
	ParamT<std::string> *robotNamespaceP;
	std::string robotNamespace;
	
	/// \brief ROS camera name
	std::string cameraName;
	
	/// \brief ROS image topic name
	std::string imageTopicName;
	std::string cameraInfoTopicName;
	std::string pollServiceName;
	double CxPrime;
	double Cx;
	double Cy;
	double focal_length;
	double hack_baseline;
	double distortion_k1;
	double distortion_k2;
	double distortion_k3;
	double distortion_t1;
	double distortion_t2;
	
	/// \brief ROS frame transform name to use in the image message header.
	///        This should typically match the link name the sensor is attached.
	std::string frameName;
	
	/// \brief A mutex to lock access to fields that are used in ROS message callbacks
	boost::mutex lock;
	
	/// \brief size of image buffer
	int height, width, depth;
	std::string type;
	int skip;

#ifdef SIMULATOR_GAZEBO_GAZEBO_ROS_CAMERA_DYNAMIC_RECONFIGURE
	// Allow dynamic reconfiguration of camera params
	dynamic_reconfigure::Server<gazebo_plugins::GazeboRosCameraConfig> *dyn_srv_;

	void configCallback(gazebo_plugins::GazeboRosCameraConfig &config, uint32_t level);

	// Name of camera
	std::string dynamicReconfigureName;
#endif

#ifdef USE_CBQ
	ros::CallbackQueue prosilica_queue_;
	void ProsilicaQueueThread();
	boost::thread prosilica_callback_queue_thread_;
#else
	void ProsilicaROSThread();
	boost::thread ros_spinner_thread_;
#endif

};

/** \} */
/// @}

}
#endif

