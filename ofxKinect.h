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


struct kinectUser{
    int userID;
    bool bCalibrated;
};


class ofxKinect : public ofVideoGrabber, protected ofThread {

	public :

		ofxKinect();
		virtual ~ofxKinect();

		/// are the current frames new?
		bool isFrameNew();

		/// open the connection and start grabbing images
		bool open();

		/// close the connection and stop grabbing images
		void close();

		/// initialize resources, must be called before open()
		bool init(bool infrared=false, bool bTexture=true);

		bool setCameraTiltAngle(float angleInDegrees);

		/// updates the pixel buffers and textures - make sure to call this to update to the latetst incoming frames
		void update();

		/// clear resources
		void clear();

		float getDistanceAt(int x, int y);
		float getDistanceAt(const ofPoint & p);

		/// calculates the coordinate in the world for the pixel (perspective calculation). Center  of image is (0.0)
		ofPoint getWorldCoordinateFor(int x, int y);

		ofColor	getColorAt(int x, int y);
		ofColor getColorAt(const ofPoint & p);

		ofColor getCalibratedColorAt(int x, int y);
		ofColor getCalibratedColorAt(const ofPoint & p);

		ofMatrix4x4 getRGBDepthMatrix();
		void setRGBDepthMatrix(const ofMatrix4x4 & matrix);

		float 			getHeight();
		float 			getWidth();

		float           cutOffFar;

		ofPoint			getRawAccel();
		ofPoint			getMksAccel();

		/// get the pixels of the most recent rgb frame
		unsigned char	* getPixels();

		/// get the pixels of the most recent depth frame
		unsigned char 	* getDepthPixels();		// grey scale values
		unsigned short	* getRawDepthPixels();	// raw 11 bit values

		// get the rgb pixels corrected to match the depth frame
		unsigned char * getCalibratedRGBPixels();

		/// get the distance in centimeters to a given point
		float* getDistancePixels();

		/// get the rgb texture
		ofTexture &		getTextureReference();

		/// get the greyscale depth texture
		ofTexture &		getDepthTextureReference();

		/**
			set the near value of the pixels in the greyscale depth image to white?

			bEnabled = true : pixels close to the camera are brighter
			bEnabled = false: pixels closer to the camera are darker (default)
		**/
		void enableDepthNearValueWhite(bool bEnabled=true);
		bool isDepthNearValueWhite();

		void 			setVerbose(bool bTalkToMe);

		void 			setUseTexture(bool bUse);
		void 			draw(float x, float y, float w, float h);
		void 			draw(float x, float y);

		void 			drawDepth(float x, float y, float w, float h);
		void 			drawDepth(float x, float y);


		const static int	width = 640;
		const static int	height = 480;



		bool					bUseTexture;
		bool                    bImage;
		ofImage				    depthTex;			// the depth texture
		ofImage 				videoTex;				// the RGB texture
		bool 					bVerbose;
		bool 					bGrabberInited;

		unsigned char *			depthPixels;
		unsigned char *			videoPixels;
		unsigned char *			calibratedRGBPixels;

		unsigned short *		depthPixelsRaw;
		float * 				distancePixels;

		ofPoint rawAccel;
		ofPoint mksAccel;

		float targetTiltAngleDeg;
		bool bTiltNeedsApplying;

		static void calculateLookups();
		static bool lookupsCalculated;
		static float distancePixelsLookup[2048];
		static unsigned char depthPixelsLookupNearWhite[2048];
		static unsigned char depthPixelsLookupFarWhite[2048];

        static ofxKinect*   thisKinect;


        Context * kinectContext;
        EnumerationErrors errors;

        XnStatus rc;
        XnFPSData xnFPS;

        DepthGenerator depth;
        ImageGenerator image;

        vector<kinectUser*> users;
        UserGenerator userGenerator;

        string xml_path;

        ImageMetaData imageMD;
        DepthMetaData depthMD;
        SceneMetaData sceneMD;

		unsigned short *	depthPixelsBack;	// depth back
		unsigned char *		videoPixelsBack;		// rgb back

		bool bNeedsUpdate;
		bool bUpdateTex;

		bool bDepthNearValueWhite;

		ofMatrix4x4		rgbDepthMatrix;

		bool				bInfrared;
		int					bytespp;

		// thread function
		void threadedFunction();

		int openKinect();

        XnBool AssignPlayer(XnUserID user);
        void findPlayer();
        void lostPlayer();

		void readDepthAtPoint();
};

#endif
