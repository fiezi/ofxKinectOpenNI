#ifndef _OFXKINECT
#define _OFXKINECT


#include "ofConstants.h"
#include "ofTexture.h"
#include "ofImage.h"
#include "ofGraphics.h"
#include "ofTypes.h"
#include "ofMain.h"


    #define OPENNI


    //---------------------------------------------------------------------------
    // Includes
    //---------------------------------------------------------------------------
    #include <XnOpenNI.h>
    #include <XnLog.h>
    #include <XnCodecIDs.h>
    #include <XnCppWrapper.h>
    #include <XnFPSCalculator.h>

    //---------------------------------------------------------------------------
    // Defines
    //---------------------------------------------------------------------------
    //#define SAMPLE_XML_PATH "data/SamplesConfig.xml"
    //#define SAMPLE_XML_PATH "data/Samples-Image.xml"
    //#define SAMPLE_XML_PATH "data/Sample-User.xml"

    //---------------------------------------------------------------------------
    // Macros
    //---------------------------------------------------------------------------
    #define CHECK_RC(rc, what)											\
        if (rc != XN_STATUS_OK)											\
        {																\
            printf("%s failed: %s\n", what, xnGetStatusString(rc));		\
            return rc;													\
        }

    using namespace xn;


#define MAX_NUM_DEVICES 3

struct kinectUser{
    int userID;
    bool bCalibrated;
};


struct DepthRgbSensors{
        char name[80];

        bool bNeedsUpdate;
		bool bUpdateTex;


        ProductionNode device;
        DepthGenerator depth;
        ImageGenerator image;

        vector<kinectUser*> users;
        UserGenerator userGenerator;

        ImageMetaData imageMD;
        DepthMetaData depthMD;
        SceneMetaData sceneMD;

        //image data

		unsigned short *	depthPixelsBack;	// depth back
		unsigned char *		videoPixelsBack;		// rgb back

		ofImage				    depthTex;			// the depth texture
		ofImage 				videoTex;				// the RGB texture
		unsigned char *			depthPixels;
		unsigned char *			videoPixels;
		unsigned char *			calibratedRGBPixels;

		unsigned short *		depthPixelsRaw;
		float * 				distancePixels;

};

class ofxKinect : public ofVideoGrabber, protected ofThread {

	public :

		ofxKinect();
		virtual ~ofxKinect();

		/// open the connection and start grabbing images
		bool open();

		/// close the connection and stop grabbing images
		void close();

		/// initialize resources, must be called before open()
		bool init(bool infrared=false, bool bTexture=true);

		/// updates the pixel buffers and textures - make sure to call this to update to the latetst incoming frames
		void update();

		/// clear resources
		void clear();

		float getDistanceAt(int x, int y, int deviceID=0);
		float getDistanceAt(const ofPoint & p, int deviceID=0);

		/// calculates the coordinate in the world for the pixel (perspective calculation). Center  of image is (0.0)
		ofPoint getWorldCoordinateFor(int x, int y, int deviceID=0);

		float 			getHeight();
		float 			getWidth();

		float           cutOffFar;

		/// get the pixels of the most recent rgb frame
		unsigned char	* getPixels(int deviceID=0);

		/// get the pixels of the most recent depth frame
		unsigned char 	* getDepthPixels(int deviceID=0);		// grey scale values
		unsigned short	* getRawDepthPixels(int deviceID=0);	// raw 11 bit values

		/// get the distance in centimeters to a given point
		float* getDistancePixels(int deviceID=0);

		/// get the rgb texture
		ofTexture &		getTextureReference();

		/// get the greyscale depth texture
		ofTexture &		getDepthTextureReference();

		void 			setVerbose(bool bTalkToMe);
		void 			setUseTexture(bool bUse);
		void 			draw(float x, float y, float w, float h,int deviceID=0);
		void 			draw(float x, float y,int deviceID=0);

		void 			drawDepth(float x, float y, float w, float h,int deviceID=0);
		void 			drawDepth(float x, float y,int deviceID=0);


		const static int	width = 640;
		const static int	height = 480;



		bool					bUseTexture;
		bool                    bImage;
		bool                    bUser;
		bool 					bVerbose;
		bool 					bGrabberInited;

        static ofxKinect*       thisKinect;


        Context * kinectContext;
        EnumerationErrors errors;

        XnStatus rc;
        XnFPSData xnFPS;


        DepthRgbSensors sensors[MAX_NUM_DEVICES];
        DepthGenerator depth;
        ImageGenerator image;

        string xml_path;

		int					bytespp;
        int                 numDevicesFound;
		// thread function
		void threadedFunction();

		int openKinect();

        XnBool AssignPlayer(XnUserID user);
        void findPlayer();
        void lostPlayer();

};

#endif
