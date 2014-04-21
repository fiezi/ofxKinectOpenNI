#include "ofxKinect.h"
#include "ofMain.h"

// pointer to this class for static callback member functions
ofxKinect* ofxKinect::thisKinect = NULL;


void XN_CALLBACK_TYPE NewUser(xn::UserGenerator& generator, XnUserID user, void* pCookie){

    for (int i=0;i<ofxKinect::thisKinect->numDevicesFound;i++){

        cout << "*************************** NEW USER!!!!! *********************************" << endl;

        kinectUser* myUser= new kinectUser;
        myUser->userID=user;
        myUser->bCalibrated=false;
        printf("Look for pose\n");
        ofxKinect::thisKinect->sensors[i].userGenerator.GetPoseDetectionCap().StartPoseDetection("Psi", user);

        ofxKinect::thisKinect->sensors[i].users.push_back(myUser);

        cout << "size: " << ofxKinect::thisKinect->sensors[i].users.size() << " id: " << user << endl;

    //	ofxKinect::thisKinect->AssignPlayer(user);
    // 	if (g_nPlayer == 0)
    // 	{
    // 		printf("Assigned user\n");
    // 		g_userGenerator->GetSkeletonCap().LoadCalibrationData(user, 0);
    // 		g_UserGenerator.GetSkeletonCap().StartTracking(user);
    // 		g_nPlayer = user;
    // 	}

    }

}

void XN_CALLBACK_TYPE LostUser(xn::UserGenerator& generator, XnUserID user, void* pCookie){

    //remove user
    for (int i=0;i<ofxKinect::thisKinect->numDevicesFound;i++){


        printf("Lost user %d\n", user);
        for (int u=0; u<ofxKinect::thisKinect->sensors[i].users.size(); u++){

            if (ofxKinect::thisKinect->sensors[i].users[u]->userID == user){
                ofxKinect::thisKinect->sensors[i].users.erase(ofxKinect::thisKinect->sensors[i].users.begin() + u);
                return;
            }
        }
    }
}

void XN_CALLBACK_TYPE PoseDetected(xn::PoseDetectionCapability& pose, const XnChar* strPose, XnUserID user, void* cxt){
	printf("Found pose \"%s\" for user %d\n", strPose, user);

        kinectUser* myUser;
	    for (int i=0; i<ofxKinect::thisKinect->sensors[0].users.size();i++){
            if (ofxKinect::thisKinect->sensors[0].users[i]->userID==user)
                myUser=ofxKinect::thisKinect->sensors[0].users[i];
	    }

	ofxKinect::thisKinect->sensors[0].userGenerator.GetSkeletonCap().RequestCalibration(user, TRUE);
	ofxKinect::thisKinect->sensors[0].userGenerator.GetPoseDetectionCap().StopPoseDetection(user);
}

void XN_CALLBACK_TYPE CalibrationStarted(xn::SkeletonCapability& skeleton, XnUserID user, void* cxt){

	printf("Calibration started\n");
}

void XN_CALLBACK_TYPE CalibrationEnded(xn::SkeletonCapability& skeleton, XnUserID user, XnBool bSuccess, void* cxt){

	printf("Calibration done [%d] %ssuccessfully\n", user, bSuccess?"":"un");

    kinectUser* myUser;
    for (int i=0; i<ofxKinect::thisKinect->sensors[0].users.size();i++){
        if (ofxKinect::thisKinect->sensors[0].users[i]->userID==user)
            myUser=ofxKinect::thisKinect->sensors[0].users[i];
    }


	if (bSuccess)
	{

		if (!myUser->bCalibrated)
		{
			ofxKinect::thisKinect->sensors[0].userGenerator.GetSkeletonCap().SaveCalibrationData(user, 0);
			//myUser->userID = user;
			ofxKinect::thisKinect->sensors[0].userGenerator.GetSkeletonCap().StartTracking(user);
			myUser->bCalibrated = TRUE;
		}

        //stop pose detection for other users? aha aha
		XnUserID aUsers[10];
		XnUInt16 nUsers = 10;
		ofxKinect::thisKinect->sensors[0].userGenerator.GetUsers(aUsers, nUsers);
		for (int i = 0; i < nUsers; ++i)
			ofxKinect::thisKinect->sensors[0].userGenerator.GetPoseDetectionCap().StopPoseDetection(aUsers[i]);
	}


}

void XN_CALLBACK_TYPE CalibrationCompleted(xn::SkeletonCapability& skeleton, XnUserID user, XnCalibrationStatus eStatus, void* cxt){

	printf("Calibration done [%d] %ssuccessfully\n", user, (eStatus == XN_CALIBRATION_STATUS_OK)?"":"un");

        kinectUser* myUser;
	    for (int i=0; i<ofxKinect::thisKinect->sensors[0].users.size();i++){
            if (ofxKinect::thisKinect->sensors[0].users[i]->userID==user)
                myUser=ofxKinect::thisKinect->sensors[0].users[i];
	    }

	if (eStatus == XN_CALIBRATION_STATUS_OK)
	{
		if (!myUser->bCalibrated)
		{
			ofxKinect::thisKinect->sensors[0].userGenerator.GetSkeletonCap().SaveCalibrationData(user, 0);
			//ofxKinect::thisKinect->nPlayer = user;
			ofxKinect::thisKinect->sensors[0].userGenerator.GetSkeletonCap().StartTracking(user);
			myUser->bCalibrated = TRUE;
		}

		XnUserID aUsers[10];
		XnUInt16 nUsers = 10;
		ofxKinect::thisKinect->sensors[0].userGenerator.GetUsers(aUsers, nUsers);
		for (int i = 0; i < nUsers; ++i)
			ofxKinect::thisKinect->sensors[0].userGenerator.GetPoseDetectionCap().StopPoseDetection(aUsers[i]);
	}
}





//--------------------------------------------------------------------
ofxKinect::ofxKinect(){
	ofLog(OF_LOG_VERBOSE, "Creating ofxKinect.");

	//TODO: reset the right ones of these on close
	// common
	bVerbose 				= false;
	bGrabberInited 			= false;
	bUseTexture				= true;
	for (int i=0;i<MAX_NUM_DEVICES;i++){
        sensors[i].depthPixels				= NULL;
        sensors[i].depthPixelsRaw			= NULL;
        sensors[i].depthPixelsBack			= NULL;
        sensors[i].videoPixels		  		= NULL;
        sensors[i].videoPixelsBack			= NULL;
        sensors[i].calibratedRGBPixels		= NULL;
        sensors[i].distancePixels 			= NULL;
        sensors[i].bNeedsUpdate			    = false;
        sensors[i].bUpdateTex				= false;
	}

	bImage=true;
	bUser=false;

	ofxKinect::thisKinect = this;
    xml_path= "data/Sample-User.xml";
	cutOffFar=4096.0f;
}

//--------------------------------------------------------------------
ofxKinect::~ofxKinect(){
	close();
	clear();
}

//--------------------------------------------------------------------
void ofxKinect::setVerbose(bool bTalkToMe){
	bVerbose = bTalkToMe;
}

//---------------------------------------------------------------------------
unsigned char * ofxKinect::getPixels(int deviceID){
	return sensors[deviceID].videoPixels;
}

//---------------------------------------------------------------------------
unsigned char	* ofxKinect::getDepthPixels(int deviceID){
	return sensors[deviceID].depthPixels;
}

//---------------------------------------------------------------------------
unsigned short 	* ofxKinect::getRawDepthPixels(int deviceID){
	return sensors[deviceID].depthPixelsRaw;
}

//---------------------------------------------------------------------------
float* ofxKinect::getDistancePixels(int deviceID) {
	return sensors[deviceID].distancePixels;
}


//------------------------------------
ofTexture & ofxKinect::getTextureReference(){
	if(!sensors[0].videoTex.bAllocated()){
		ofLog(OF_LOG_WARNING, "ofxKinect: getTextureReference - texture is not allocated");
	}
	return sensors[0].videoTex.getTextureReference();
}

//---------------------------------------------------------------------------
ofTexture & ofxKinect::getDepthTextureReference(){

	if(!sensors[0].depthTex.bAllocated()){
		ofLog(OF_LOG_WARNING, "ofxKinect: getDepthTextureReference - texture is not allocated");
	}
	return sensors[0].depthTex.getTextureReference();

}


//--------------------------------------------------------------------
bool ofxKinect::open(){
	startThread(true, false);	// blocking, not verbose

	return true;
}

//---------------------------------------------------------------------------
void ofxKinect::close(){
	if(isThreadRunning()){
		waitForThread(true);
	}
}


//--------------------------------------------------------------------
bool ofxKinect::init(bool infrared, bool setUseTexture){

	clear();

    if (bImage)
        xml_path="data/Samples-Image.xml";
    else if(bUser)
        xml_path="data/Sample-User.xml";
    else
        xml_path="data/Samples-Depth.xml";

    numDevicesFound=0;
	bytespp = infrared?1:3;

	bUseTexture = true;

    for (int i=0;i<MAX_NUM_DEVICES;i++){
        int length = width*height;
        sensors[i].depthPixels = new unsigned char[length];
        sensors[i].depthPixelsRaw = new unsigned short[length];
        sensors[i].depthPixelsBack = new unsigned short[length];
        sensors[i].distancePixels = new float[length];

        sensors[i].videoPixels = new unsigned char[length*bytespp];
        sensors[i].videoPixelsBack = new unsigned char[length*bytespp];

        memset(sensors[i].depthPixels, 0, length*sizeof(unsigned char));
        memset(sensors[i].depthPixelsRaw, 0, length*sizeof(unsigned short));
        memset(sensors[i].depthPixelsBack, 0, length*sizeof(unsigned short));
        memset(sensors[i].distancePixels, 0, length*sizeof(float));

        memset(sensors[i].videoPixels, 0, length*bytespp*sizeof(unsigned char));
        memset(sensors[i].videoPixelsBack, 0, length*bytespp*sizeof(unsigned char));

        if(bUseTexture){
            sensors[i].depthTex.allocate(width, height, OF_IMAGE_GRAYSCALE);
            sensors[i].videoTex.allocate(width, height, OF_IMAGE_COLOR);
        }
    }
	bGrabberInited = true;

	ofLog(OF_LOG_VERBOSE, "ofxKinect: Inited");

	return bGrabberInited;
}

//---------------------------------------------------------------------------
void ofxKinect::clear(){

    for (int i=0;i<MAX_NUM_DEVICES;i++){

        if(sensors[i].depthPixels != NULL){
            delete[] sensors[i].depthPixels; sensors[i].depthPixels = NULL;
            delete[] sensors[i].depthPixelsRaw; sensors[i].depthPixelsRaw = NULL;
            delete[] sensors[i].depthPixelsBack; sensors[i].depthPixelsBack = NULL;
            delete[] sensors[i].distancePixels; sensors[i].distancePixels = NULL;

            delete[] sensors[i].videoPixels; sensors[i].videoPixels = NULL;
            delete[] sensors[i].videoPixelsBack; sensors[i].videoPixelsBack = NULL;
        }
    /*
        depthTex.clear();
        videoTex.clear();
    */
    }

    bGrabberInited = false;
}

//----------------------------------------------------------
void ofxKinect::update(){

    if ( this->lock() ) {

        for (int s=0;s<numDevicesFound;s++){

            //if (!sensors[s].bNeedsUpdate){
            //    return;
            //} else {
                sensors[s].bUpdateTex = true;
            //}


            int n = width * height;

            for(int i = 0; i < n; i++){
                    sensors[s].distancePixels[i] = (int)sensors[s].depthPixelsBack[i];
                    int dPixel = ((int)sensors[s].depthPixelsBack[i])*255.0 /cutOffFar;
                    if (dPixel>255)
                        sensors[s].depthPixels[i]=255;
                    else if (dPixel<=0)
                        sensors[s].depthPixels[i]=255;
                    else
                        sensors[s].depthPixels[i]=dPixel;
            }


            memcpy(sensors[s].videoPixels, sensors[s].videoPixelsBack, n * bytespp);

            //we have done the update
            sensors[s].bNeedsUpdate = false;

            if(bUseTexture){
                //depthTex.loadData(depthPixels, width, height, GL_LUMINANCE);
                sensors[s].depthTex.setFromPixels(sensors[s].depthPixels, width, height, OF_IMAGE_GRAYSCALE);
                sensors[s].depthTex.update();
                sensors[s].videoTex.setFromPixels(sensors[s].videoPixelsBack, width, height, OF_IMAGE_COLOR);
                sensors[s].videoTex.update();

                sensors[s].bUpdateTex = false;
            }
        }
    }
    this->unlock();

}


//------------------------------------
float ofxKinect::getDistanceAt(int x, int y, int deviceID) {
	return sensors[deviceID].distancePixels[y * width + x];
}

//------------------------------------
float ofxKinect::getDistanceAt(const ofPoint & p, int deviceID) {
	return getDistanceAt(p.x, p.y, deviceID);
}

//------------------------------------
ofPoint ofxKinect::getWorldCoordinateFor(int x, int y, int deviceID) {
	//Based on http://graphics.stanford.edu/~mdfisher/Kinect.html
	static const double fx_d = 1.0 / 5.9421434211923247e+02;
	static const double fy_d = 1.0 / 5.9104053696870778e+02;
	static const double cx_d = 3.3930780975300314e+02;
	static const double cy_d = 2.4273913761751615e+02;

	ofVec3f result;
	const double depth = getDistanceAt(x,y,deviceID)/100.0;
	result.x = float((x - cx_d) * depth * fx_d);
	result.y = float((y - cy_d) * depth * fy_d);
	result.z = depth;

	return result;
}


//------------------------------------
void ofxKinect::setUseTexture(bool bUse){
    bUseTexture = bUse;
}

//----------------------------------------------------------
void ofxKinect::draw(float _x, float _y, float _w, float _h, int deviceID){
	if(bUseTexture) {
		sensors[deviceID].videoTex.draw(_x, _y, _w, _h);
	}
}

//----------------------------------------------------------
void ofxKinect::draw(float _x, float _y, int deviceID){
	draw(_x, _y, (float)width, (float)height, deviceID);
}

//----------------------------------------------------------
void ofxKinect::drawDepth(float _x, float _y, float _w, float _h, int deviceID){
	if(bUseTexture) {
		sensors[deviceID].depthTex.draw(_x, _y, _w, _h);
	}
}

//---------------------------------------------------------------------------
void ofxKinect::drawDepth(float _x, float _y, int deviceID){
	drawDepth(_x, _y, (float)width, (float)height, deviceID);
}

//----------------------------------------------------------
float ofxKinect::getHeight(){
	return (float)height;
}

//---------------------------------------------------------------------------
float ofxKinect::getWidth(){
	return (float)width;
}


//---------------------------------------------------------------------------
void ofxKinect::threadedFunction(){

	openKinect();

	while (isThreadRunning())
	{
		if ( lock() )
		{

            for (int s=0;s<numDevicesFound;s++){

                rc = kinectContext->WaitOneUpdateAll(sensors[s].depth);
                if (rc != XN_STATUS_OK)
                {
                    printf("UpdateData failed: %s\n", xnGetStatusString(rc));
                    return;
                }

                sensors[s].depth.GetMetaData(sensors[s].depthMD);

                if (bImage){
                    sensors[s].image.GetMetaData(sensors[s].imageMD);
                    const XnUInt8* pImage = sensors[s].imageMD.Data();

                    sensors[s].videoPixelsBack=(unsigned char*)sensors[s].imageMD.Data();

                }

                if (bUser){
                    for (int i=0;i<sensors[0].users.size();i++){
                        sensors[0].userGenerator.GetUserPixels(sensors[0].users[i]->userID,sensors[0].sceneMD);
                        const XnLabel* pLabels=sensors[0].sceneMD.Data();

                        //color silhouette
                        for (int p=0;p<640*480*3;p++){
                            sensors[0].videoPixelsBack[p]=0;
                            if (pLabels[p/3]>0 ){
                                if (p%3==pLabels[p/3]-1)
                                    sensors[0].videoPixelsBack[p]=128;
                            }
                        }
                    }
                }
                const XnDepthPixel* pImageIR = sensors[s].depthMD.Data();

                memcpy(sensors[s].depthPixelsBack, sensors[s].depthMD.Data(), sensors[s].depthMD.XRes()*sensors[s].depthMD.YRes() * sizeof(unsigned short));




                //unsigned short maxValue=2048;
                thisKinect->sensors[s].bNeedsUpdate = true;

                unlock();
                ofSleepMillis(10);
            }
		}
	}

    kinectContext->Shutdown();
    free(kinectContext);

	ofLog(OF_LOG_VERBOSE, "ofxKinect: Connection closed");
}


int ofxKinect::openKinect(){

    kinectContext = new Context;


    rc = XN_STATUS_OK;

    rc = kinectContext->InitFromXmlFile(xml_path.c_str(), &errors);


    NodeInfoList devicesList;
    int devicesListCount = 0;
    rc = kinectContext->EnumerateProductionTrees(XN_NODE_TYPE_DEVICE, NULL, devicesList);
    for (NodeInfoList::Iterator it = devicesList.Begin(); it != devicesList.End(); ++it)
    {
       devicesListCount++;
    }
    CHECK_RC(rc, "Enumerate");

    numDevicesFound=devicesListCount;

       // Create the device node
       //devices start at 1, our array starts at 0
    int i=0;
   for (NodeInfoList::Iterator it = devicesList.Begin(); it != devicesList.End(); ++it, ++i)
   {
       NodeInfo deviceInfo = *it;
           rc = kinectContext->CreateProductionTree(deviceInfo, sensors[i].device);
           CHECK_RC(rc, "Create Device");

       // Create a query to depend on this node
       Query query;
       query.AddNeededNode(deviceInfo.GetInstanceName());

       // Copy the device name
       xnOSMemCopy(sensors[i].name,deviceInfo.GetInstanceName(),
                   xnOSStrLen(deviceInfo.GetInstanceName()));

        if (bImage){
               // now create a image generator over this device
               rc = kinectContext->CreateAnyProductionTree(XN_NODE_TYPE_IMAGE, &query, sensors[i].image);
               CHECK_RC(rc, "Create Image");

            rc = xnFPSInit(&xnFPS, 30);
            CHECK_RC(rc, "FPS Init");

            rc = kinectContext->StartGeneratingAll();
            CHECK_RC(rc, "StartGenerating");

            sensors[i].image.GetMetaData(sensors[i].imageMD);

        }else if (bUser){
            rc = kinectContext->FindExistingNode(XN_NODE_TYPE_USER, sensors[i].userGenerator);
            CHECK_RC(rc, "Find user generator");



            if (!sensors[i].userGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON) ||
                !sensors[i].userGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
            {
                printf("User generator doesn't support either skeleton or pose detection.\n");
                return XN_STATUS_ERROR;
            }


            rc = xnFPSInit(&xnFPS, 30);
            CHECK_RC(rc, "FPS Init");


            sensors[i].userGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

            rc = kinectContext->StartGeneratingAll();
            CHECK_RC(rc, "StartGenerating");

            XnCallbackHandle hUserCBs, hCalibrationStartCB, hCalibrationCompleteCB, hPoseCBs;

            rc = sensors[i].userGenerator.RegisterUserCallbacks(NewUser, LostUser, this, hUserCBs);

            rc = sensors[i].userGenerator.GetSkeletonCap().RegisterToCalibrationStart(CalibrationStarted, this, hCalibrationStartCB);
            CHECK_RC(rc, "Register to calbiration start");
            rc = sensors[i].userGenerator.GetSkeletonCap().RegisterToCalibrationComplete(CalibrationCompleted, this, hCalibrationCompleteCB);
            CHECK_RC(rc, "Register to calibration complete");
            rc = sensors[i].userGenerator.GetPoseDetectionCap().RegisterToPoseDetected(PoseDetected, this, hPoseCBs);
            CHECK_RC(rc, "Register to pose detected");

        }


            //ALWAYS CREATE DEPTH!!!

            cout << "********************************************************************************" << endl;

            // Now create a depth generator over this device
            rc = kinectContext->CreateAnyProductionTree(XN_NODE_TYPE_DEPTH, &query, sensors[i].depth);
            CHECK_RC(rc, "Create Depth");

            rc = xnFPSInit(&xnFPS, 25);
            CHECK_RC(rc, "FPS Init");

            rc = kinectContext->StartGeneratingAll();
            CHECK_RC(rc, "StartGenerating");

            sensors[i].depth.GetMetaData(sensors[i].depthMD);



   }
}


void ofxKinect::lostPlayer(){

    findPlayer();
}

void ofxKinect::findPlayer(){

	XnUserID aUsers[20];
	XnUInt16 nUsers = 20;
	sensors[0].userGenerator.GetUsers(aUsers, nUsers);

	for (int i = 0; i < nUsers; ++i)
	{
		if (AssignPlayer(aUsers[i]))
			return;
	}
}

XnBool ofxKinect::AssignPlayer(XnUserID user){

	XnPoint3D com;
	sensors[0].userGenerator.GetCoM(user, com);
	if (com.Z == 0)
		return false;

	printf("Matching for existing calibration\n");
	sensors[0].userGenerator.GetSkeletonCap().LoadCalibrationData(user, 0);
	sensors[0].userGenerator.GetSkeletonCap().StartTracking(user);
	//nPlayer = user;
	return true;

}
