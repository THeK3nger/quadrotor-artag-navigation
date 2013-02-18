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
@mainpage
	Desc: labrob_gazebo_camera plugin for simulating cameras in Gazebo
	Author: Marco Cognetti
	Date: 01 July 2012
	SVN info: $Id$
@htmlinclude manifest.html
@b labrob_gazebo_camera plugin mimics after prosilica_camera package
*/

#include <labrob_quadrotor_sensors/labrob_gazebo_camera.h>

namespace gazebo{

GZ_REGISTER_DYNAMIC_CONTROLLER("labrob_gazebo_camera", labrob_gazebo_camera);

/////////////////////////////////////////////////////
/////////////////////// Constructor /////////////////
/////////////////////////////////////////////////////
labrob_gazebo_camera::labrob_gazebo_camera(Entity *parent): Controller(parent){
	myParent = dynamic_cast<MonoCameraSensor*>(parent);
	
	if (!myParent)
		gzthrow("labrob_gazebo_camera controller requires a Camera Sensor as its parent");
	
	Param::Begin(&parameters);
		robotNamespaceP = new ParamT<std::string>("robotNamespace", "/", 0);
		imageTopicNameP = new ParamT<std::string>("imageTopicName","image_raw", 0);
		cameraInfoTopicNameP = new ParamT<std::string>("cameraInfoTopicName","camera_info", 0);
		pollServiceNameP = new ParamT<std::string>("pollServiceName","request_image", 0);
		cameraNameP = new ParamT<std::string>("cameraName","", 0);
		frameNameP = new ParamT<std::string>("frameName","camera", 0);
		// camera parameters 
		CxPrimeP = new ParamT<double>("CxPrime",320, 0); // for 640x480 image
		CxP  = new ParamT<double>("Cx" ,320, 0); // for 640x480 image
		CyP  = new ParamT<double>("Cy" ,240, 0); // for 640x480 image
		focal_lengthP  = new ParamT<double>("focal_length" ,554.256, 0); // == image_width(px) / (2*tan( hfov(radian) /2))
		hack_baselineP  = new ParamT<double>("hackBaseline" ,0, 0); // hack for right stereo camera
		distortion_k1P  = new ParamT<double>("distortion_k1" ,0, 0);
		distortion_k2P  = new ParamT<double>("distortion_k2" ,0, 0);
		distortion_k3P  = new ParamT<double>("distortion_k3" ,0, 0);
		distortion_t1P  = new ParamT<double>("distortion_t1" ,0, 0);
		distortion_t2P  = new ParamT<double>("distortion_t2" ,0, 0);
		numParentP  = new ParamT<int>("numParent" ,0, 0);
	Param::End();
	
	imageConnectCount = 0;
	infoConnectCount = 0;
}

/////////////////////////////////////////////////////
//////////////////// Destructor /////////////////////
/////////////////////////////////////////////////////
labrob_gazebo_camera::~labrob_gazebo_camera(){
	delete robotNamespaceP;
	delete rosnode_;
	delete imageTopicNameP;
	delete cameraInfoTopicNameP;
	delete pollServiceNameP;
	delete frameNameP;
	delete CxPrimeP;
	delete CxP;
	delete CyP;
	delete focal_lengthP;
	delete hack_baselineP;
	delete distortion_k1P;
	delete distortion_k2P;
	delete distortion_k3P;
	delete distortion_t1P;
	delete distortion_t2P;
	delete numParentP;
}

/////////////////////////////////////////////////////
/////////////// Load the controller /////////////////
/////////////////////////////////////////////////////
void labrob_gazebo_camera::LoadChild(XMLConfigNode *node){
	robotNamespaceP->Load(node);
	robotNamespace = robotNamespaceP->GetValue();
	if (!ros::isInitialized()){
		int argc = 0;
		char** argv = NULL;
		ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
	}
	
	numParentP->Load(node);
	std::string topicToSub;
	std::cout << "robotNamespaceCamera: " << robotNamespace << std::endl;
	if(robotNamespace=="/"){
		gazebo::Entity* actParent;
		actParent = parent;
		for(int i=0;i<numParentP->GetValue();++i)
			actParent = actParent->GetParent();
		
		std::cout << "actParent->GetName(): " << actParent->GetName() << std::endl;
		robotNamespace = actParent->GetName();
		topicToSub = robotNamespace + "/";
	}
// 	robotNamespace += "/";
	
	std::cout << "robotNamespace: " << robotNamespace << std::endl;
	
	cameraNameP->Load(node);
	cameraName = cameraNameP->GetValue();
	rosnode_ = new ros::NodeHandle(/*robotNamespace+*/cameraName);
	rosnode_->setCallbackQueue(&prosilica_queue_);
	itnode_ = new image_transport::ImageTransport(*rosnode_);
	
	imageTopicNameP->Load(node);
	cameraInfoTopicNameP->Load(node);
	pollServiceNameP->Load(node);
	frameNameP->Load(node);
	CxPrimeP->Load(node);
	CxP->Load(node);
	CyP->Load(node);
	focal_lengthP->Load(node);
	hack_baselineP->Load(node);
	distortion_k1P->Load(node);
	distortion_k2P->Load(node);
	distortion_k3P->Load(node);
	distortion_t1P->Load(node);
	distortion_t2P->Load(node);
	
	imageTopicName = imageTopicNameP->GetValue();
	cameraInfoTopicName = cameraInfoTopicNameP->GetValue();
	pollServiceName = pollServiceNameP->GetValue();
	frameName = frameNameP->GetValue();
	CxPrime = CxPrimeP->GetValue();
	Cx = CxP->GetValue();
	Cy = CyP->GetValue();
	focal_length = focal_lengthP->GetValue();
	hack_baseline = hack_baselineP->GetValue();
	distortion_k1 = distortion_k1P->GetValue();
	distortion_k2 = distortion_k2P->GetValue();
	distortion_k3 = distortion_k3P->GetValue();
	distortion_t1 = distortion_t1P->GetValue();
	distortion_t2 = distortion_t2P->GetValue();
	if ((distortion_k1 != 0.0) || (distortion_k2 != 0.0) ||
			(distortion_k3 != 0.0) || (distortion_t1 != 0.0) ||
			(distortion_t2 != 0.0))
		ROS_WARN("gazebo_ros_prosilica simulation does not support non-zero distortion parameters right now, your simulation maybe wrong.");
	
	// camera mode for prosilica:
	// prosilica::AcquisitionMode mode_; /// @todo Make this property of Camera
	std::string mode_param_name;
	
	//ROS_ERROR("before trigger_mode %s %s",mode_param_name.c_str(),mode_.c_str());
	
	if (!rosnode_->searchParam("trigger_mode",mode_param_name)) ///\@todo: hardcoded per prosilica_camera wiki api, make this an urdf parameter
		mode_param_name = "trigger_mode";
	
	if (!rosnode_->getParam(mode_param_name,mode_))
			mode_ = "streaming";
	
	ROS_INFO("trigger_mode %s %s",mode_param_name.c_str(),mode_.c_str());
	
	
	if (mode_ == "polled"){
		poll_srv_ = polled_camera::advertise(*rosnode_,pollServiceName,&labrob_gazebo_camera::pollCallback,this);
	}
	else if (mode_ == "streaming"){
		ROS_DEBUG("do nothing here,mode: %s",mode_.c_str());
	}
	else{
		ROS_ERROR("trigger_mode is invalid: %s, using streaming mode",mode_.c_str());
	}
	
	std::cout << "robotNamespace: " << robotNamespace << std::endl;
	std::string topicNameToPub = robotNamespace+"/"+imageTopicName;
// 	std::string topicNameToPub;
// 	topicNameToPub = topicToSub + imageTopicName;
	
	std::cout << "topicNameToPub: " << topicNameToPub << std::endl;
	/// advertise topics for image and camera info
	image_pub_ = itnode_->advertise(
		topicNameToPub,1,
		boost::bind( &labrob_gazebo_camera::ImageConnect,this),
		boost::bind( &labrob_gazebo_camera::ImageDisconnect,this), ros::VoidPtr(), &prosilica_queue_);
	
	topicNameToPub = robotNamespace+"/"+cameraInfoTopicName;
// 	topicNameToPub = topicToSub + cameraInfoTopicName;
	std::cout << "topicNameToPub2: " << topicNameToPub << std::endl;
	ros::AdvertiseOptions camera_info_ao = ros::AdvertiseOptions::create<sensor_msgs::CameraInfo>(
		topicNameToPub,1,
		boost::bind( &labrob_gazebo_camera::InfoConnect,this),
		boost::bind( &labrob_gazebo_camera::InfoDisconnect,this), ros::VoidPtr(), &prosilica_queue_);
	camera_info_pub_ = rosnode_->advertise(camera_info_ao);

#ifdef SIMULATOR_GAZEBO_GAZEBO_ROS_CAMERA_DYNAMIC_RECONFIGURE
	if (!cameraName.empty()) {
		dyn_srv_ = new dynamic_reconfigure::Server<gazebo_plugins::GazeboRosCameraConfig>(*rosnode_);
		dynamic_reconfigure::Server<gazebo_plugins::GazeboRosCameraConfig>::CallbackType f = boost::bind(&labrob_gazebo_camera::configCallback, this, _1, _2);
		dyn_srv_->setCallback(f);
	}
#endif

}

#ifdef SIMULATOR_GAZEBO_GAZEBO_ROS_CAMERA_DYNAMIC_RECONFIGURE
/////////////////////////////////////////////////////
//////////////// Dynamic Reconfigure Callback ///////
/////////////////////////////////////////////////////
void labrob_gazebo_camera::configCallback(gazebo_plugins::GazeboRosCameraConfig &config, uint32_t level){
	ROS_INFO("Reconfigure request for the gazebo ros camera: %s. New rate: %.2f", cameraName.c_str(), config.imager_rate);
	
	(dynamic_cast<OgreCamera*>(myParent))->SetUpdateRate(config.imager_rate);
}
#endif

/////////////////////////////////////////////////////
/////////////// Initialize the controller ///////////
/////////////////////////////////////////////////////
void labrob_gazebo_camera::InitChild(){
	// sensor generation off by default
	myParent->SetActive(false);
	
	// set buffer size
	width            = myParent->GetImageWidth();
	height           = myParent->GetImageHeight();
	depth            = myParent->GetImageDepth();
	//ROS_INFO("image format in urdf is %s\n",myParent->GetImageFormat().c_str());
	if (myParent->GetImageFormat() == "L8"){
		type = sensor_msgs::image_encodings::MONO8;
		skip = 1;
	}else if (myParent->GetImageFormat() == "R8G8B8"){
		type = sensor_msgs::image_encodings::RGB8;
		skip = 3;
	}else if (myParent->GetImageFormat() == "B8G8R8"){
		type = sensor_msgs::image_encodings::BGR8;
		skip = 3;
	}else if (myParent->GetImageFormat() == "BAYER_RGGB8"){
		type = sensor_msgs::image_encodings::BAYER_RGGB8;
		skip = 1;
	}else if (myParent->GetImageFormat() == "BAYER_BGGR8"){
		type = sensor_msgs::image_encodings::BAYER_BGGR8;
		skip = 1;
	}else if (myParent->GetImageFormat() == "BAYER_GBRG8"){
		type = sensor_msgs::image_encodings::BAYER_GBRG8;
		skip = 1;
	}else if (myParent->GetImageFormat() == "BAYER_GRBG8"){
		type = sensor_msgs::image_encodings::BAYER_GRBG8;
		skip = 1;
	}else{
		ROS_ERROR("Unsupported Gazebo ImageFormat for Prosilica, using BGR8\n");
		type = sensor_msgs::image_encodings::BGR8;
		skip = 3;
	}
	
	/// Compute camera parameters if set to 0
	if (CxPrime == 0)
		CxPrime = ((double)width+1.0) /2.0;
	if (Cx == 0)
		Cx = ((double)width+1.0) /2.0;
	if (Cy == 0)
		Cy = ((double)height+1.0) /2.0;
	if (focal_length == 0)
		focal_length = ((double)width) / (2.0 *tan(myParent->GetHFOV().GetAsRadian()/2.0));
	
#ifdef USE_CBQ
	// start custom queue for prosilica
	prosilica_callback_queue_thread_ = boost::thread( boost::bind( &labrob_gazebo_camera::ProsilicaQueueThread,this ) );
#else
	// start ros spinner as it is done in prosilica node
	ros_spinner_thread_ = boost::thread( boost::bind( &labrob_gazebo_camera::ProsilicaROSThread,this ) );
#endif
}

/////////////////////////////////////////////////////
////////////////// Increment count //////////////////
/////////////////////////////////////////////////////
void labrob_gazebo_camera::ImageConnect(){
	imageConnectCount++;
}

/////////////////////////////////////////////////////
////////////////// Decrement count //////////////////
/////////////////////////////////////////////////////
void labrob_gazebo_camera::ImageDisconnect(){
	imageConnectCount--;
	
	if ((infoConnectCount == 0) && (imageConnectCount == 0))
		myParent->SetActive(false);
}

/////////////////////////////////////////////////////
/////////////////// Increment count /////////////////
/////////////////////////////////////////////////////
void labrob_gazebo_camera::InfoConnect(){
	infoConnectCount++;
}

/////////////////////////////////////////////////////
////////////////// Decrement count //////////////////
/////////////////////////////////////////////////////
void labrob_gazebo_camera::InfoDisconnect(){
	infoConnectCount--;
	
	if ((infoConnectCount == 0) && (imageConnectCount == 0))
		myParent->SetActive(false);
}

/////////////////////////////////////////////////////
////////////////// Update the controller ////////////
/////////////////////////////////////////////////////
void labrob_gazebo_camera::UpdateChild(){
	
	// should do nothing except turning camera on/off, as we are using service.
	/// @todo: consider adding thumbnailing feature here if subscribed.
	
	// as long as ros is connected, parent is active
	//ROS_ERROR("debug image count %d",imageConnectCount);
	if (!myParent->IsActive()){
		if (imageConnectCount > 0)
			// do this first so there's chance for sensor to run 1 frame after activate
			myParent->SetActive(true);
	}else{
		// publish if in continuous mode, otherwise triggered by poll
		if (mode_ == "streaming")
			PutCameraData();
	}
	
	/// publish CameraInfo if in continuous mode, otherwise triggered by poll
	if (infoConnectCount > 0)
		if (mode_ == "streaming")
			PublishCameraInfo();
}

/////////////////////////////////////////////////////
///////////// Put laser data to the interface ///////
/////////////////////////////////////////////////////
void labrob_gazebo_camera::PutCameraData(){
	const unsigned char *src;
	
	//boost::recursive_mutex::scoped_lock mr_lock(*Simulator::Instance()->GetMRMutex());
	
	// Get a pointer to image data
	src = myParent->GetImageData(0);
	
	if (src){
		//double tmpT0 = Simulator::Instance()->GetWallTime();
		
		unsigned char dst[width*height];
		
		lock.lock();
		// copy data into image
		imageMsg.header.frame_id = frameName;
#if GAZEBO_MAJOR_VERSION == 0 && GAZEBO_MINOR_VERSION >= 10
		imageMsg.header.stamp.fromSec(Simulator::Instance()->GetSimTime().Double());
#else
		imageMsg.header.stamp.fromSec(Simulator::Instance()->GetSimTime());
#endif
		
		//double tmpT1 = Simulator::Instance()->GetWallTime();
		//double tmpT2;
		
		/// @todo: don't bother if there are no subscribers
		if (image_pub_.getNumSubscribers() > 0){
			// do last minute conversion if Bayer pattern is requested but not provided, go from R8G8B8
			// deprecated in gazebo2 branch, keep for backwards compatibility
			if (myParent->GetImageFormat() == "BAYER_RGGB8" && depth == 3){
				for (int i=0;i<width;i++){
					for (int j=0;j<height;j++){
						//
						// RG
						// GB
						//
						// determine position
						if (j%2) // even column
							if (i%2) // even row, red
								dst[i+j*width] = src[i*3+j*width*3+0];
							else // odd row, green
								dst[i+j*width] = src[i*3+j*width*3+1];
						else // odd column
							if (i%2) // even row, green
								dst[i+j*width] = src[i*3+j*width*3+1];
							else // odd row, blue
								dst[i+j*width] = src[i*3+j*width*3+2];
					}
				}
				src=dst;
			}else if (myParent->GetImageFormat() == "BAYER_BGGR8" && depth == 3){
				for (int i=0;i<width;i++){
					for (int j=0;j<height;j++){
						//
						// BG
						// GR
						//
						// determine position
						if (j%2) // even column
							if (i%2) // even row, blue
								dst[i+j*width] = src[i*3+j*width*3+2];
							else // odd row, green
								dst[i+j*width] = src[i*3+j*width*3+1];
						else // odd column
							if (i%2) // even row, green
								dst[i+j*width] = src[i*3+j*width*3+1];
							else // odd row, red
								dst[i+j*width] = src[i*3+j*width*3+0];
					}
				}
				src=dst;
			}else if (myParent->GetImageFormat() == "BAYER_GBRG8" && depth == 3){
				for (int i=0;i<width;i++){
					for (int j=0;j<height;j++){
						//
						// GB
						// RG
						//
						// determine position
						if (j%2) // even column
							if (i%2) // even row, green
								dst[i+j*width] = src[i*3+j*width*3+1];
							else // odd row, blue
								dst[i+j*width] = src[i*3+j*width*3+2];
						else // odd column
							if (i%2) // even row, red
								dst[i+j*width] = src[i*3+j*width*3+0];
							else // odd row, green
								dst[i+j*width] = src[i*3+j*width*3+1];
					}
				}
				src=dst;
			}else if (myParent->GetImageFormat() == "BAYER_GRBG8" && depth == 3){
				for (int i=0;i<width;i++){
					for (int j=0;j<height;j++){
						//
						// GR
						// BG
						//
						// determine position
						if (j%2) // even column
							if (i%2) // even row, green
								dst[i+j*width] = src[i*3+j*width*3+1];
							else // odd row, red
								dst[i+j*width] = src[i*3+j*width*3+0];
						else // odd column
							if (i%2) // even row, blue
								dst[i+j*width] = src[i*3+j*width*3+2];
							else // odd row, green
								dst[i+j*width] = src[i*3+j*width*3+1];
					}
				}
				src=dst;
			}
			
			//ROS_ERROR("debug %d %d %d %d", type, height, width, skip);
			
			// copy from src to imageMsg
			fillImage(imageMsg,
								type,
								height,
								width,
								skip*width,
								(void*)src );
			
			//tmpT2 = Simulator::Instance()->GetWallTime();
			
			// publish to ros
			image_pub_.publish(imageMsg);
		}
		
		//double tmpT3 = Simulator::Instance()->GetWallTime();
		
		lock.unlock();
	}
	
}

/////////////////////////////////////////////////////
///////////// Put laser data to the interface ///////
/////////////////////////////////////////////////////
void labrob_gazebo_camera::PublishCameraInfo(){
	// fill CameraInfo
	cameraInfoMsg.header.frame_id = frameName;
#if GAZEBO_MAJOR_VERSION == 0 && GAZEBO_MINOR_VERSION >= 10
	cameraInfoMsg.header.stamp.fromSec(Simulator::Instance()->GetSimTime().Double());
#else
	cameraInfoMsg.header.stamp.fromSec(Simulator::Instance()->GetSimTime());
#endif
	cameraInfoMsg.height = height;
	cameraInfoMsg.width  = width;

	// distortion
#if ROS_VERSION_MINIMUM(1, 3, 0)
	cameraInfoMsg.distortion_model = "plumb_bob";
	cameraInfoMsg.D.resize(5);
#endif
	cameraInfoMsg.D[0] = distortion_k1;
	cameraInfoMsg.D[1] = distortion_k2;
	cameraInfoMsg.D[2] = distortion_k3;
	cameraInfoMsg.D[3] = distortion_t1;
	cameraInfoMsg.D[4] = distortion_t2;
	// original camera matrix
	cameraInfoMsg.K[0] = focal_length;
	cameraInfoMsg.K[1] = 0.0;
	cameraInfoMsg.K[2] = Cx;
	cameraInfoMsg.K[3] = 0.0;
	cameraInfoMsg.K[4] = focal_length;
	cameraInfoMsg.K[5] = Cy;
	cameraInfoMsg.K[6] = 0.0;
	cameraInfoMsg.K[7] = 0.0;
	cameraInfoMsg.K[8] = 1.0;
	// rectification
	cameraInfoMsg.R[0] = 1.0;
	cameraInfoMsg.R[1] = 0.0;
	cameraInfoMsg.R[2] = 0.0;
	cameraInfoMsg.R[3] = 0.0;
	cameraInfoMsg.R[4] = 1.0;
	cameraInfoMsg.R[5] = 0.0;
	cameraInfoMsg.R[6] = 0.0;
	cameraInfoMsg.R[7] = 0.0;
	cameraInfoMsg.R[8] = 1.0;
	// camera projection matrix (same as camera matrix due to lack of distortion/rectification) (is this generated?)
	cameraInfoMsg.P[0] = focal_length;
	cameraInfoMsg.P[1] = 0.0;
	cameraInfoMsg.P[2] = Cx;
	cameraInfoMsg.P[3] = -focal_length * hack_baseline;
	cameraInfoMsg.P[4] = 0.0;
	cameraInfoMsg.P[5] = focal_length;
	cameraInfoMsg.P[6] = Cy;
	cameraInfoMsg.P[7] = 0.0;
	cameraInfoMsg.P[8] = 0.0;
	cameraInfoMsg.P[9] = 0.0;
	cameraInfoMsg.P[10] = 1.0;
	cameraInfoMsg.P[11] = 0.0;
	camera_info_pub_.publish(cameraInfoMsg);
}

/////////////////////////////////////////////////////
//////////////// new prosilica interface ////////////
/////////////////////////////////////////////////////
void labrob_gazebo_camera::pollCallback(polled_camera::GetPolledImage::Request& req,
																			polled_camera::GetPolledImage::Response& rsp,
																			sensor_msgs::Image& image, sensor_msgs::CameraInfo& info){
	/// @todo Support binning (maybe just cv::resize)
	/// @todo Don't adjust K, P for ROI, set CameraInfo.roi fields instead
	/// @todo D parameter order is k1, k2, t1, t2, k3
	
	if (mode_ != "polled"){
		rsp.success = false;
		rsp.status_message = "Camera is not in triggered mode";
		return;
	}

	if (req.binning_x > 1 || req.binning_y > 1){
		rsp.success = false;
		rsp.status_message = "Gazebo Prosilica plugin does not support binning";
		return;
	}

/*
	// fill out the cam info part
	info.header.frame_id = frameName;
#if GAZEBO_MAJOR_VERSION == 0 && GAZEBO_MINOR_VERSION >= 10
	info.header.stamp.fromSec(Simulator::Instance()->GetSimTime().Double());
#else
	info.header.stamp.fromSec(Simulator::Instance()->GetSimTime());
#endif
	info.height = myParent->GetImageHeight();
	info.width  = myParent->GetImageWidth() ;
	// distortion
#if ROS_VERSION_MINIMUM(1, 3, 0)
	info.distortion_model = "plumb_bob";
	info.D.resize(5);
#endif
	info.D[0] = distortion_k1;
	info.D[1] = distortion_k2;
	info.D[2] = distortion_k3;
	info.D[3] = distortion_t1;
	info.D[4] = distortion_t2;
	// original camera matrix
	info.K[0] = focal_length;
	info.K[1] = 0.0;
	info.K[2] = Cx;
	info.K[3] = 0.0;
	info.K[4] = focal_length;
	info.K[5] = Cy;
	info.K[6] = 0.0;
	info.K[7] = 0.0;
	info.K[8] = 1.0;
	// rectification
	info.R[0] = 1.0;
	info.R[1] = 0.0;
	info.R[2] = 0.0;
	info.R[3] = 0.0;
	info.R[4] = 1.0;
	info.R[5] = 0.0;
	info.R[6] = 0.0;
	info.R[7] = 0.0;
	info.R[8] = 1.0;
	// camera projection matrix (same as camera matrix due to lack of distortion/rectification) (is this generated?)
	info.P[0] = focal_length;
	info.P[1] = 0.0;
	info.P[2] = Cx;
	info.P[3] = -focal_length * hack_baseline;
	info.P[4] = 0.0;
	info.P[5] = focal_length;
	info.P[6] = Cy;
	info.P[7] = 0.0;
	info.P[8] = 0.0;
	info.P[9] = 0.0;
	info.P[10] = 1.0;
	info.P[11] = 0.0;
*/

	// get region from request
	if (req.roi.x_offset <= 0 || req.roi.y_offset <= 0 || req.roi.width <= 0 || req.roi.height <= 0){
		req.roi.x_offset = 0;
		req.roi.y_offset = 0;
		req.roi.width = width;
		req.roi.height = height;
	}
	const unsigned char *src = NULL;
	ROS_ERROR("roidebug %d %d %d %d", req.roi.x_offset, req.roi.y_offset, req.roi.width, req.roi.height);

	// signal sensor to start update
	ImageConnect();
	// wait until an image has been returned
	while(!src){
		{
			boost::recursive_mutex::scoped_lock lock(*Simulator::Instance()->GetMRMutex());
			// Get a pointer to image data
			src = myParent->GetImageData(0);

			if (src)
			{

				// fill CameraInfo
				roiCameraInfoMsg = &info;
				roiCameraInfoMsg->header.frame_id = frameName;
#if GAZEBO_MAJOR_VERSION == 0 && GAZEBO_MINOR_VERSION >= 10
				roiCameraInfoMsg->header.stamp.fromSec( (dynamic_cast<OgreCamera*>(myParent))->GetLastRenderTime().Double());
#else
				roiCameraInfoMsg->header.stamp.fromSec( (dynamic_cast<OgreCamera*>(myParent))->GetLastRenderTime());
#endif
				roiCameraInfoMsg->width  = req.roi.width; //myParent->GetImageWidth() ;
				roiCameraInfoMsg->height = req.roi.height; //myParent->GetImageHeight();
				// distortion
#if ROS_VERSION_MINIMUM(1, 3, 0)
				roiCameraInfoMsg->distortion_model = "plumb_bob";
				roiCameraInfoMsg->D.resize(5);
#endif
				roiCameraInfoMsg->D[0] = distortion_k1;
				roiCameraInfoMsg->D[1] = distortion_k2;
				roiCameraInfoMsg->D[2] = distortion_k3;
				roiCameraInfoMsg->D[3] = distortion_t1;
				roiCameraInfoMsg->D[4] = distortion_t2;
				// original camera matrix
				roiCameraInfoMsg->K[0] = focal_length;
				roiCameraInfoMsg->K[1] = 0.0;
				roiCameraInfoMsg->K[2] = Cx - req.roi.x_offset;
				roiCameraInfoMsg->K[3] = 0.0;
				roiCameraInfoMsg->K[4] = focal_length;
				roiCameraInfoMsg->K[5] = Cy - req.roi.y_offset;
				roiCameraInfoMsg->K[6] = 0.0;
				roiCameraInfoMsg->K[7] = 0.0;
				roiCameraInfoMsg->K[8] = 1.0;
				// rectification
				roiCameraInfoMsg->R[0] = 1.0;
				roiCameraInfoMsg->R[1] = 0.0;
				roiCameraInfoMsg->R[2] = 0.0;
				roiCameraInfoMsg->R[3] = 0.0;
				roiCameraInfoMsg->R[4] = 1.0;
				roiCameraInfoMsg->R[5] = 0.0;
				roiCameraInfoMsg->R[6] = 0.0;
				roiCameraInfoMsg->R[7] = 0.0;
				roiCameraInfoMsg->R[8] = 1.0;
				// camera projection matrix (same as camera matrix due to lack of distortion/rectification) (is this generated?)
				roiCameraInfoMsg->P[0] = focal_length;
				roiCameraInfoMsg->P[1] = 0.0;
				roiCameraInfoMsg->P[2] = Cx - req.roi.x_offset;
				roiCameraInfoMsg->P[3] = -focal_length * hack_baseline;
				roiCameraInfoMsg->P[4] = 0.0;
				roiCameraInfoMsg->P[5] = focal_length;
				roiCameraInfoMsg->P[6] = Cy - req.roi.y_offset;
				roiCameraInfoMsg->P[7] = 0.0;
				roiCameraInfoMsg->P[8] = 0.0;
				roiCameraInfoMsg->P[9] = 0.0;
				roiCameraInfoMsg->P[10] = 1.0;
				roiCameraInfoMsg->P[11] = 0.0;
				camera_info_pub_.publish(*roiCameraInfoMsg);
				
				// copy data into imageMsg, then convert to roiImageMsg(image)
				imageMsg.header.frame_id    = frameName;
#if GAZEBO_MAJOR_VERSION == 0 && GAZEBO_MINOR_VERSION >= 10
				imageMsg.header.stamp.fromSec( (dynamic_cast<OgreCamera*>(myParent))->GetLastRenderTime().Double());
#else
				imageMsg.header.stamp.fromSec( (dynamic_cast<OgreCamera*>(myParent))->GetLastRenderTime());
#endif
				
				unsigned char dst[width*height];
				
				/// @todo: don't bother if there are no subscribers
				
				// do last minute conversion if Bayer pattern is requested but not provided, go from R8G8B8
				// deprecated in gazebo2 branch, keep for backwards compatibility
				if (myParent->GetImageFormat() == "BAYER_RGGB8" && depth == 3){
					for (int i=0;i<width;i++){
						for (int j=0;j<height;j++){
							//
							// RG
							// GB
							//
							// determine position
							if (j%2) // even column
								if (i%2) // even row, red
									dst[i+j*width] = src[i*3+j*width*3+0];
								else // odd row, green
									dst[i+j*width] = src[i*3+j*width*3+1];
							else // odd column
								if (i%2) // even row, green
									dst[i+j*width] = src[i*3+j*width*3+1];
								else // odd row, blue
									dst[i+j*width] = src[i*3+j*width*3+2];
						}
					}
					src=dst;
				}else if (myParent->GetImageFormat() == "BAYER_BGGR8" && depth == 3){
					for (int i=0;i<width;i++){
						for (int j=0;j<height;j++){
							//
							// BG
							// GR
							//
							// determine position
							if (j%2) // even column
								if (i%2) // even row, blue
									dst[i+j*width] = src[i*3+j*width*3+2];
								else // odd row, green
									dst[i+j*width] = src[i*3+j*width*3+1];
							else // odd column
								if (i%2) // even row, green
									dst[i+j*width] = src[i*3+j*width*3+1];
								else // odd row, red
									dst[i+j*width] = src[i*3+j*width*3+0];
						}
					}
					src=dst;
				}else if (myParent->GetImageFormat() == "BAYER_GBRG8" && depth == 3){
					for (int i=0;i<width;i++){
						for (int j=0;j<height;j++){
							//
							// GB
							// RG
							//
							// determine position
							if (j%2) // even column
								if (i%2) // even row, green
									dst[i+j*width] = src[i*3+j*width*3+1];
								else // odd row, blue
									dst[i+j*width] = src[i*3+j*width*3+2];
							else // odd column
								if (i%2) // even row, red
									dst[i+j*width] = src[i*3+j*width*3+0];
								else // odd row, green
									dst[i+j*width] = src[i*3+j*width*3+1];
						}
					}
					src=dst;
				}else if (myParent->GetImageFormat() == "BAYER_GRBG8" && depth == 3){
					for (int i=0;i<width;i++){
						for (int j=0;j<height;j++){
							//
							// GR
							// BG
							//
							// determine position
							if (j%2) // even column
								if (i%2) // even row, green
									dst[i+j*width] = src[i*3+j*width*3+1];
								else // odd row, red
									dst[i+j*width] = src[i*3+j*width*3+0];
							else // odd column
								if (i%2) // even row, blue
									dst[i+j*width] = src[i*3+j*width*3+2];
								else // odd row, green
									dst[i+j*width] = src[i*3+j*width*3+1];
						}
					}
					src=dst;
				}

				// copy from src to imageMsg
				fillImage(imageMsg,
									type,
									height,
									width,
									skip*width,
									(void*)src );
				
				/// @todo: publish to ros, thumbnails and rect image in the Update call?
				
				image_pub_.publish(imageMsg);
				
				// error if Bayer pattern is requested but not provided, roi not supported in this case
				// not supported in old image_pipeline as well, this might change, but ultimately
				// this is deprecated in gazebo2 branch, keep for backwards compatibility
				if (((myParent->GetImageFormat() == "BAYER_RGGB8") ||
						(myParent->GetImageFormat() == "BAYER_BGGR8") ||
						(myParent->GetImageFormat() == "BAYER_GBRG8") ||
						(myParent->GetImageFormat() == "BAYER_GRBG8") ) &&
						depth == 3){
					ROS_ERROR("prosilica does not support bayer roi, using full image");
					
					// copy from src to imageMsg
					fillImage(image,
										type,
										height,
										width,
										skip*width,
										(void*)src );
				}else{
					// copy data into ROI image
					roiImageMsg = &image;
					roiImageMsg->header.frame_id = frameName;
#if GAZEBO_MAJOR_VERSION == 0 && GAZEBO_MINOR_VERSION >= 10
					roiImageMsg->header.stamp.fromSec( (dynamic_cast<OgreCamera*>(myParent))->GetLastRenderTime().Double());
#else
					roiImageMsg->header.stamp.fromSec( (dynamic_cast<OgreCamera*>(myParent))->GetLastRenderTime());
#endif
					
					//sensor_msgs::CvBridge img_bridge_(&imageMsg);
					//IplImage* cv_image;
					//img_bridge_.to_cv( &cv_image );
					
					sensor_msgs::CvBridge img_bridge_;
					img_bridge_.fromImage(imageMsg);
					
					//cvNamedWindow("showme",CV_WINDOW_AUTOSIZE);
					//cvSetMouseCallback("showme", &labrob_gazebo_camera::mouse_cb, this);
					//cvStartWindowThread();
					
					//cvShowImage("showme",img_bridge_.toIpl());
					cvSetImageROI(img_bridge_.toIpl(),cvRect(req.roi.x_offset,req.roi.y_offset,req.roi.width,req.roi.height));
					IplImage *roi = cvCreateImage(cvSize(req.roi.width,req.roi.height),
																			img_bridge_.toIpl()->depth,
																			img_bridge_.toIpl()->nChannels);
					cvCopy(img_bridge_.toIpl(),roi);
					
					img_bridge_.fromIpltoRosImage(roi,*roiImageMsg);
					
					cvReleaseImage(&roi);
				}
			}
		}
		usleep(100000);
	}
	ImageDisconnect();
	rsp.success = true;
	return;
}

/////////////////////////////////////////////////////
/////////////// Finalize the controller /////////////
/////////////////////////////////////////////////////
void labrob_gazebo_camera::FiniChild(){
	myParent->SetActive(false);
	rosnode_->shutdown();
#ifdef USE_CBQ
	prosilica_queue_.clear();
	prosilica_queue_.disable();
	prosilica_callback_queue_thread_.join();
#else
	ros_spinner_thread_.join();
#endif

	poll_srv_.shutdown();
	image_pub_.shutdown();
	camera_info_pub_.shutdown();

}

#ifdef USE_CBQ
/////////////////////////////////////////////////////
///////////// Put laser data to the interface ///////
/////////////////////////////////////////////////////
void labrob_gazebo_camera::ProsilicaQueueThread(){
	static const double timeout = 0.01;
	
	while (rosnode_->ok()){
		prosilica_queue_.callAvailable(ros::WallDuration(timeout));
	}
}
#else
void labrob_gazebo_camera::ProsilicaROSThread(){
	ROS_INFO_STREAM("Callback thread id=" << boost::this_thread::get_id());
	
	ros::Rate rate(1000);
	
	while (rosnode_->ok()){
		//rate.sleep(); // using rosrate gets stuck on model delete
		usleep(1000);
		ros::spinOnce();
	}
}
#endif


}
