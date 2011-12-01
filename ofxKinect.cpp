#include "ofxKinect.h"
#include "ofMain.h"

// pointer to this class for static callback member functions
ofxKinect* ofxKinect::thisKinect = NULL;

bool ofxKinect::lookupsCalculated = false;
float ofxKinect::distancePixelsLookup[2048];
unsigned char ofxKinect::depthPixelsLookupNearWhite[2048];
unsigned char ofxKinect::depthPixelsLookupFarWhite[2048];


void XN_CALLBACK_TYPE NewUser(xn::UserGenerator& generator, XnUserID user, void* pCookie){


    cout << "*************************** NEW USER!!!!! *********************************" << endl;

    kinectUser* myUser= new kinectUser;
    myUser->userID=user;
    myUser->bCalibrated=false;
	printf("Look for pose\n");
    ofxKinect::thisKinect->userGenerator.GetPoseDetectionCap().StartPoseDetection("Psi", user);

    ofxKinect::thisKinect->users.push_back(myUser);

    cout << "size: " << ofxKinect::thisKinect->users.size() << " id: " << user << endl;

//	ofxKinect::thisKinect->AssignPlayer(user);
// 	if (g_nPlayer == 0)
// 	{
// 		printf("Assigned user\n");
// 		g_userGenerator->GetSkeletonCap().LoadCalibrationData(user, 0);
// 		g_UserGenerator.GetSkeletonCap().StartTracking(user);
// 		g_nPlayer = user;
// 	}
}

void XN_CALLBACK_TYPE LostUser(xn::UserGenerator& generator, XnUserID user, void* pCookie){

    //remove user

	printf("Lost user %d\n", user);
	for (int i=0; i<ofxKinect::thisKinect->users.size(); i++){

        if (ofxKinect::thisKinect->users[i]->userID == user){
            ofxKinect::thisKinect->users.erase(ofxKinect::thisKinect->users.begin() + i);
            return;
        }
    }
}

void XN_CALLBACK_TYPE PoseDetected(xn::PoseDetectionCapability& pose, const XnChar* strPose, XnUserID user, void* cxt){
	printf("Found pose \"%s\" for user %d\n", strPose, user);

        kinectUser* myUser;
	    for (int i=0; i<ofxKinect::thisKinect->users.size();i++){
            if (ofxKinect::thisKinect->users[i]->userID==user)
                myUser=ofxKinect::thisKinect->users[i];
	    }

	ofxKinect::thisKinect->userGenerator.GetSkeletonCap().RequestCalibration(user, TRUE);
	ofxKinect::thisKinect->userGenerator.GetPoseDetectionCap().StopPoseDetection(user);
}

void XN_CALLBACK_TYPE CalibrationStarted(xn::SkeletonCapability& skeleton, XnUserID user, void* cxt){

	printf("Calibration started\n");
}

void XN_CALLBACK_TYPE CalibrationEnded(xn::SkeletonCapability& skeleton, XnUserID user, XnBool bSuccess, void* cxt){

	printf("Calibration done [%d] %ssuccessfully\n", user, bSuccess?"":"un");

    kinectUser* myUser;
    for (int i=0; i<ofxKinect::thisKinect->users.size();i++){
        if (ofxKinect::thisKinect->users[i]->userID==user)
            myUser=ofxKinect::thisKinect->users[i];
    }


	if (bSuccess)
	{

		if (!myUser->bCalibrated)
		{
			ofxKinect::thisKinect->userGenerator.GetSkeletonCap().SaveCalibrationData(user, 0);
			//myUser->userID = user;
			ofxKinect::thisKinect->userGenerator.GetSkeletonCap().StartTracking(user);
			myUser->bCalibrated = TRUE;
		}

        //stop pose detection for other users? aha aha
		XnUserID aUsers[10];
		XnUInt16 nUsers = 10;
		ofxKinect::thisKinect->userGenerator.GetUsers(aUsers, nUsers);
		for (int i = 0; i < nUsers; ++i)
			ofxKinect::thisKinect->userGenerator.GetPoseDetectionCap().StopPoseDetection(aUsers[i]);
	}


}

void XN_CALLBACK_TYPE CalibrationCompleted(xn::SkeletonCapability& skeleton, XnUserID user, XnCalibrationStatus eStatus, void* cxt){

	printf("Calibration done [%d] %ssuccessfully\n", user, (eStatus == XN_CALIBRATION_STATUS_OK)?"":"un");

        kinectUser* myUser;
	    for (int i=0; i<ofxKinect::thisKinect->users.size();i++){
            if (ofxKinect::thisKinect->users[i]->userID==user)
                myUser=ofxKinect::thisKinect->users[i];
	    }

	if (eStatus == XN_CALIBRATION_STATUS_OK)
	{
		if (!myUser->bCalibrated)
		{
			ofxKinect::thisKinect->userGenerator.GetSkeletonCap().SaveCalibrationData(user, 0);
			//ofxKinect::thisKinect->nPlayer = user;
			ofxKinect::thisKinect->userGenerator.GetSkeletonCap().StartTracking(user);
			myUser->bCalibrated = TRUE;
		}

		XnUserID aUsers[10];
		XnUInt16 nUsers = 10;
		ofxKinect::thisKinect->userGenerator.GetUsers(aUsers, nUsers);
		for (int i = 0; i < nUsers; ++i)
			ofxKinect::thisKinect->userGenerator.GetPoseDetectionCap().StopPoseDetection(aUsers[i]);
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
	depthPixels				= NULL;
	depthPixelsRaw			= NULL;
	depthPixelsBack			= NULL;
	videoPixels		  		= NULL;
	videoPixelsBack			= NULL;
	calibratedRGBPixels		= NULL;
	distancePixels 			= NULL;

	bNeedsUpdate			= false;
	bUpdateTex				= false;

	bDepthNearValueWhite	= false;

	bImage=false;

	ofxKinect::thisKinect = this;

	rgbDepthMatrix.getPtr()[0]=0.942040;
	rgbDepthMatrix.getPtr()[1]=-0.005672;
	rgbDepthMatrix.getPtr()[2]=0.000000;
	rgbDepthMatrix.getPtr()[3]=23.953022;
	rgbDepthMatrix.getPtr()[4]=0.004628;
	rgbDepthMatrix.getPtr()[5]=0.939875;
	rgbDepthMatrix.getPtr()[6]=0.000000;
	rgbDepthMatrix.getPtr()[7]=31.486654;
	rgbDepthMatrix.getPtr()[8]=0.000000;
	rgbDepthMatrix.getPtr()[9]=0.000000;
	rgbDepthMatrix.getPtr()[10]=0.000000;
	rgbDepthMatrix.getPtr()[11]=0.000000;
	rgbDepthMatrix.getPtr()[12]=0.000005;
	rgbDepthMatrix.getPtr()[13]=0.000003;
	rgbDepthMatrix.getPtr()[14]=0.000000;
	rgbDepthMatrix.getPtr()[15]=1.000000;

	calculateLookups();

    xml_path= "data/Sample-User.xml";
	cutOffFar=4096.0f;
}

void ofxKinect::calculateLookups() {
	if(!lookupsCalculated) {
		ofLog(OF_LOG_VERBOSE, "Setting up LUT for distance and depth values.");
		for(int i = 0; i < 2048; i++){
			if(i == 2047) {
				distancePixelsLookup[i] = 0;
				depthPixelsLookupNearWhite[i] = 0;
				depthPixelsLookupFarWhite[i] = 0;
			} else {
				// using equation from http://openkinect.org/wiki/Imaging_Information
				const float k1 = 0.1236;
				const float k2 = 2842.5;
				const float k3 = 1.1863;
				const float k4 = 0.0370;
				distancePixelsLookup[i] = k1 * tanf(i / k2 + k3) - k4; // calculate in meters
				distancePixelsLookup[i] *= 100; // convert to centimeters
				depthPixelsLookupNearWhite[i] = (float) (2048 * 256) / (i - 2048);
				depthPixelsLookupFarWhite[i] = 255 - depthPixelsLookupNearWhite[i];
			}
		}
	}
	lookupsCalculated = true;
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
unsigned char * ofxKinect::getPixels(){
	return videoPixels;
}

//---------------------------------------------------------------------------
unsigned char	* ofxKinect::getDepthPixels(){
	return depthPixels;
}

//---------------------------------------------------------------------------
unsigned short 	* ofxKinect::getRawDepthPixels(){
	return depthPixelsRaw;
}

//---------------------------------------------------------------------------
float* ofxKinect::getDistancePixels() {
	return distancePixels;
}

//---------------------------------------------------------------------------
unsigned char * ofxKinect::getCalibratedRGBPixels(){
	ofxVec3f texcoord3d;
	unsigned char * calibratedPixels = calibratedRGBPixels;
	for ( int y = 0; y < height; y++) {
		for ( int x = 0; x < width; x++) {
			texcoord3d.set(x,y,0);
			texcoord3d = rgbDepthMatrix * texcoord3d ;
			texcoord3d.x = ofClamp(texcoord3d.x,0,width);
			texcoord3d.y = ofClamp(texcoord3d.y,0,height);
			int pos = int(texcoord3d.y)*width*3+int(texcoord3d.x)*3;
			*calibratedPixels++ = videoPixels[pos];
			*calibratedPixels++ = videoPixels[pos+1];
			*calibratedPixels++ = videoPixels[pos+2];
		}
	}
	return calibratedRGBPixels;
}

//------------------------------------
ofTexture & ofxKinect::getTextureReference(){
	if(!videoTex.bAllocated()){
		ofLog(OF_LOG_WARNING, "ofxKinect: getTextureReference - texture is not allocated");
	}
	return videoTex;
}

//---------------------------------------------------------------------------
ofTexture & ofxKinect::getDepthTextureReference(){
	if(!depthTex.bAllocated()){
		ofLog(OF_LOG_WARNING, "ofxKinect: getDepthTextureReference - texture is not allocated");
	}
	return depthTex;
}

//--------------------------------------------------------------------
bool ofxKinect::isFrameNew(){
	if(isThreadRunning()){
		return !bNeedsUpdate;
	}
	return false;
}

//--------------------------------------------------------------------
bool ofxKinect::open(){
#ifndef OPENNI
	if(!bGrabberInited){
		ofLog(OF_LOG_WARNING, "ofxKinect: Cannot open, init not called");
		return false;
	}

	if (freenect_init(&kinectContext, NULL) < 0) {
		ofLog(OF_LOG_ERROR, "ofxKinect: freenet_init failed");
		return false;
	}

	int number_devices = freenect_num_devices(kinectContext);
	ofLog(OF_LOG_VERBOSE, "ofxKinect: Number of Devices found: " + ofToString(number_devices));

	if (number_devices < 1) {
		ofLog(OF_LOG_ERROR, "ofxKinect: Did not find a device");
		return false;
	}

	if (freenect_open_device(kinectContext, &kinectDevice, 0) < 0) {
		ofLog(OF_LOG_ERROR, "ofxKinect: Could not open device");
		return false;
	}

	freenect_set_user(kinectDevice, this);
#endif
	startThread(true, false);	// blocking, not verbose

	return true;
}

//---------------------------------------------------------------------------
void ofxKinect::close(){
	if(isThreadRunning()){
		waitForThread(true);
	}

	//usleep(500000); // some time while thread is stopping ...

	//libusb_exit(NULL);
}

//We update the value here - but apply it in kinect thread.
//--------------------------------------------------------------------
bool ofxKinect::setCameraTiltAngle(float angleInDegrees){

#ifndef OPENNI
	if(!kinectContext){
		return false;
	}

	targetTiltAngleDeg = ofClamp(angleInDegrees,-30,30);
	bTiltNeedsApplying = true;
#endif
	return true;

}

//--------------------------------------------------------------------
bool ofxKinect::init(bool infrared, bool setUseTexture){

	clear();

    if (bImage)
        xml_path="data/Samples-Image.xml";
    else
        xml_path="data/Sample-User.xml";


	bInfrared = infrared;
	bytespp = infrared?1:3;

	bUseTexture = setUseTexture;

	int length = width*height;
	depthPixels = new unsigned char[length];
	depthPixelsRaw = new unsigned short[length];
	depthPixelsBack = new unsigned short[length];
	distancePixels = new float[length];

	videoPixels = new unsigned char[length*bytespp];
	videoPixelsBack = new unsigned char[length*bytespp];
	calibratedRGBPixels = new unsigned char[length*bytespp];

	memset(depthPixels, 0, length*sizeof(unsigned char));
	memset(depthPixelsRaw, 0, length*sizeof(unsigned short));
	memset(depthPixelsBack, 0, length*sizeof(unsigned short));
	memset(distancePixels, 0, length*sizeof(float));

	memset(videoPixels, 0, length*bytespp*sizeof(unsigned char));
	memset(videoPixelsBack, 0, length*bytespp*sizeof(unsigned char));

	if(bUseTexture){
		depthTex.allocate(width, height, GL_LUMINANCE);
		videoTex.allocate(width, height, infrared?GL_LUMINANCE:GL_RGB);
	}

	bGrabberInited = true;

	ofLog(OF_LOG_VERBOSE, "ofxKinect: Inited");

	return bGrabberInited;
}

//---------------------------------------------------------------------------
void ofxKinect::clear(){
	if(depthPixels != NULL){
		delete[] depthPixels; depthPixels = NULL;
		delete[] depthPixelsRaw; depthPixelsRaw = NULL;
		delete[] depthPixelsBack; depthPixelsBack = NULL;
		delete[] distancePixels; distancePixels = NULL;

		delete[] videoPixels; videoPixels = NULL;
		//delete[] videoPixelsBack; videoPixelsBack = NULL;
	}
/*
	depthTex.clear();
    videoTex.clear();
*/
	bGrabberInited = false;
}

//----------------------------------------------------------
void ofxKinect::update(){

	if (!bNeedsUpdate){
		return;
	} else {
		bUpdateTex = true;
	}

	if ( this->lock() ) {

		int n = width * height;

        for(int i = 0; i < n; i++){
				distancePixels[i] = (int)depthPixelsBack[i];
				int dPixel = ((int)depthPixelsBack[i])*255.0 /cutOffFar;
				if (dPixel>255)
					depthPixels[i]=255;
				else if (dPixel<=0)
					depthPixels[i]=255;
				else
					depthPixels[i]=dPixel;
        }

		memcpy(videoPixels, videoPixelsBack, n * bytespp);

		//we have done the update
		bNeedsUpdate = false;

		this->unlock();
	}

	if(bUseTexture){
		depthTex.loadData(depthPixels, width, height, GL_LUMINANCE);
		videoTex.loadData(videoPixelsBack, width, height, bInfrared?GL_LUMINANCE:GL_RGB);
		bUpdateTex = false;
	}
}


//------------------------------------
float ofxKinect::getDistanceAt(int x, int y) {
	return distancePixels[y * width + x];
}

//------------------------------------
float ofxKinect::getDistanceAt(const ofPoint & p) {
	return getDistanceAt(p.x, p.y);
}

//------------------------------------
ofxPoint3f ofxKinect::getWorldCoordinateFor(int x, int y) {
	//Based on http://graphics.stanford.edu/~mdfisher/Kinect.html
	static const double fx_d = 1.0 / 5.9421434211923247e+02;
	static const double fy_d = 1.0 / 5.9104053696870778e+02;
	static const double cx_d = 3.3930780975300314e+02;
	static const double cy_d = 2.4273913761751615e+02;

	ofxVec3f result;
	const double depth = getDistanceAt(x,y)/100.0;
	result.x = float((x - cx_d) * depth * fx_d);
	result.y = float((y - cy_d) * depth * fy_d);
	result.z = depth;

	return result;
}

//------------------------------------
ofColor	ofxKinect::getColorAt(int x, int y) {
	int index = (y * width + x) * 3;
	ofColor c;
	c.r = videoPixels[index++];
	c.g = videoPixels[index++];
	c.b = videoPixels[index];
	c.a = 255;

	return c;
}

//------------------------------------
ofColor ofxKinect::getColorAt(const ofPoint & p) {
	return getColorAt(p.x, p.y);
}

//------------------------------------
ofColor ofxKinect::getCalibratedColorAt(int x, int y){
	ofxVec3f texcoord3d;
	texcoord3d.set(x,y,0);
	texcoord3d = rgbDepthMatrix * texcoord3d;
	return getColorAt(ofClamp(texcoord3d.x,0,width),ofClamp(texcoord3d.y,0,height));
}

//------------------------------------
ofColor ofxKinect::getCalibratedColorAt(const ofPoint & p){
	return getCalibratedColorAt(p.x,p.y);
}

//------------------------------------
ofxMatrix4x4 ofxKinect::getRGBDepthMatrix(){
	return rgbDepthMatrix;
}

//------------------------------------
void ofxKinect::setRGBDepthMatrix(const ofxMatrix4x4 & matrix){
	rgbDepthMatrix=matrix;
}

//------------------------------------
void ofxKinect::setUseTexture(bool bUse){
	bUseTexture = bUse;
}

//----------------------------------------------------------
void ofxKinect::draw(float _x, float _y, float _w, float _h){
	if(bUseTexture) {
		videoTex.draw(_x, _y, _w, _h);
	}
}

//----------------------------------------------------------
void ofxKinect::draw(float _x, float _y){
	draw(_x, _y, (float)width, (float)height);
}

//----------------------------------------------------------
void ofxKinect::drawDepth(float _x, float _y, float _w, float _h){
	if(bUseTexture) {
		depthTex.draw(_x, _y, _w, _h);
	}
}

//---------------------------------------------------------------------------
void ofxKinect::drawDepth(float _x, float _y){
	drawDepth(_x, _y, (float)width, (float)height);
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
ofPoint ofxKinect::getRawAccel(){
	return rawAccel;
}

//---------------------------------------------------------------------------
ofPoint ofxKinect::getMksAccel(){
	return mksAccel;
}

//---------------------------------------------------------------------------
void ofxKinect::enableDepthNearValueWhite(bool bEnabled){
	bDepthNearValueWhite = bEnabled;
}

//---------------------------------------------------------------------------
bool ofxKinect::isDepthNearValueWhite(){
	return bDepthNearValueWhite;
}

//---------------------------------------------------------------------------
void ofxKinect::threadedFunction(){

	openKinect();

	while (isThreadRunning())
	{
		if ( lock() )
		{


            rc = kinectContext->WaitOneUpdateAll(depth);
            if (rc != XN_STATUS_OK)
            {
                printf("UpdateData failed: %s\n", xnGetStatusString(rc));
                return;
            }

            depth.GetMetaData(depthMD);
            if (bImage){
                image.GetMetaData(imageMD);
                const XnUInt8* pImage = imageMD.Data();

                videoPixelsBack=(unsigned char*)imageMD.Data();

            }else{
                for (int i=0;i<users.size();i++){
                    userGenerator.GetUserPixels(users[i]->userID,sceneMD);
                    const XnLabel* pLabels=sceneMD.Data();

                    //color silhouette
                    for (int p=0;p<640*480*3;p++){
                        videoPixelsBack[p]=0;
                        if (pLabels[p/3]>0 ){
                            if (p%3==pLabels[p/3]-1)
                                videoPixelsBack[p]=128;
                        }
                    }
                }
            }
            const XnDepthPixel* pImageIR = depthMD.Data();

            memcpy(depthPixelsBack, depthMD.Data(), depthMD.XRes()*depthMD.YRes() * sizeof(unsigned short));




            //unsigned short maxValue=2048;
			thisKinect->bNeedsUpdate = true;

			unlock();
			ofSleepMillis(20);
		}
	}

    kinectContext->Shutdown();
    free(kinectContext);

	ofLog(OF_LOG_VERBOSE, "ofxKinect: Connection closed");
}


int ofxKinect::openKinect(){

#ifdef OPENNI
    kinectContext = new Context;

    rc = XN_STATUS_OK;

        rc = kinectContext->InitFromXmlFile(xml_path.c_str(), &errors);

	if (rc == XN_STATUS_NO_NODE_PRESENT)
	{
		XnChar strError[1024];
		errors.ToString(strError, 1024);
		printf("%s\n", strError);
		return (rc);
	}
	else if (rc != XN_STATUS_OK)
	{
		printf("Open failed: %s\n", xnGetStatusString(rc));
		return (rc);
	}


	rc = kinectContext->FindExistingNode(XN_NODE_TYPE_DEPTH, depth);
	CHECK_RC(rc, "Find depth generator");

    if (bImage){
        rc = kinectContext->FindExistingNode(XN_NODE_TYPE_IMAGE, image);
        CHECK_RC(rc, "Find image generator");


        XnBool isSupported = depth.IsCapabilitySupported("AlternativeViewPoint");
        if(isSupported){
          XnStatus res = depth.GetAlternativeViewPointCap().SetViewPoint(image);
          if(XN_STATUS_OK != res){
            printf("Getting and setting AlternativeViewPoint failed: %s\n", xnGetStatusString(res));
          }

        }
        rc = xnFPSInit(&xnFPS, 30);
        CHECK_RC(rc, "FPS Init");

        rc = kinectContext->StartGeneratingAll();
        CHECK_RC(rc, "StartGenerating");

    }else{
        rc = kinectContext->FindExistingNode(XN_NODE_TYPE_USER, userGenerator);
        CHECK_RC(rc, "Find user generator");



        if (!userGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON) ||
            !userGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
        {
            printf("User generator doesn't support either skeleton or pose detection.\n");
            return XN_STATUS_ERROR;
        }


        rc = xnFPSInit(&xnFPS, 30);
        CHECK_RC(rc, "FPS Init");


        userGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

        rc = kinectContext->StartGeneratingAll();
        CHECK_RC(rc, "StartGenerating");

        XnCallbackHandle hUserCBs, hCalibrationStartCB, hCalibrationCompleteCB, hPoseCBs;

        rc = userGenerator.RegisterUserCallbacks(NewUser, LostUser, this, hUserCBs);

        rc = userGenerator.GetSkeletonCap().RegisterToCalibrationStart(CalibrationStarted, this, hCalibrationStartCB);
        CHECK_RC(rc, "Register to calbiration start");
        rc = userGenerator.GetSkeletonCap().RegisterToCalibrationComplete(CalibrationCompleted, this, hCalibrationCompleteCB);
        CHECK_RC(rc, "Register to calibration complete");
        rc = userGenerator.GetPoseDetectionCap().RegisterToPoseDetected(PoseDetected, this, hPoseCBs);
        CHECK_RC(rc, "Register to pose detected");

    }


	depth.GetMetaData(depthMD);


	if (bImage)
        image.GetMetaData(imageMD);

#endif
}


void ofxKinect::lostPlayer(){

    findPlayer();
}

void ofxKinect::findPlayer(){

	XnUserID aUsers[20];
	XnUInt16 nUsers = 20;
	userGenerator.GetUsers(aUsers, nUsers);

	for (int i = 0; i < nUsers; ++i)
	{
		if (AssignPlayer(aUsers[i]))
			return;
	}
}

XnBool ofxKinect::AssignPlayer(XnUserID user){

	XnPoint3D com;
	userGenerator.GetCoM(user, com);
	if (com.Z == 0)
		return false;

	printf("Matching for existing calibration\n");
	userGenerator.GetSkeletonCap().LoadCalibrationData(user, 0);
	userGenerator.GetSkeletonCap().StartTracking(user);
	//nPlayer = user;
	return true;

}
