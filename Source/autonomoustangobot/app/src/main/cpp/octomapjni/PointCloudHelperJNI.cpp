#include <jni.h>
#include <octomap/octomap_types.h>
#include <android/log.h>
#include "NativeObjectHelper.h"

extern "C" {

/// natively implemented helper functions to process pointclouds

/**
 * Transforms a depthbuffer to a PointCloud
 */
JNIEXPORT jint JNICALL
Java_com_thkoeln_jmoeller_autonomoustangobot_exploration_PointCloudHelperJNI_filterPointCloudNative(
        JNIEnv *env, jclass,
        jobject depthBuffer, jint depthBufferWidth, jint depthBufferHeight,
        jobject pointCloudPointsBuffer, jint sampleWidth, jint sampleHeight,
        jfloat focalLengthX, jfloat focalLengthY) {
    
    jfloat *depthFB = (jfloat *) env->GetDirectBufferAddress(depthBuffer);
    jfloat *pointCloudPointsFB = (jfloat *) env->GetDirectBufferAddress(pointCloudPointsBuffer);

    const float halfWidth = (float) depthBufferWidth / 2;
    const float halfHeight = (float) depthBufferHeight / 2;

    int numPoints = 0;
    for (int x = 0; x < sampleWidth; x++) {
        for (int y = 0; y < sampleHeight; y++) {
            int xPixel = (int) (((float) x / sampleWidth) * depthBufferWidth);
            int yPixel = (int) (((float) y / sampleHeight) * depthBufferHeight);
            int index = yPixel * depthBufferWidth + xPixel;

            float depth = depthFB[index];

            if (depth != 0) {
                *(pointCloudPointsFB++) = (xPixel - halfWidth) * (depth / focalLengthX);
                *(pointCloudPointsFB++) = (yPixel - halfHeight) * (depth / focalLengthY);
                *(pointCloudPointsFB++) = depth;
                *(pointCloudPointsFB++) = 1;
                
                numPoints++;
            }
        }
    }

    return (jint) numPoints;
}

octomap::point3d minCoordinatesPoint(octomap::point3d p1, octomap::point3d p2) {
    return octomap::point3d((p1.x() < p2.x()) ? p1.x() : p2.x(),
                            (p1.y() < p2.y()) ? p1.y() : p2.y(),
                            (p1.z() < p2.z()) ? p1.z() : p2.z());
}

octomap::point3d maxCoordinatesPoint(octomap::point3d p1, octomap::point3d p2) {
    return octomap::point3d((p1.x() > p2.x()) ? p1.x() : p2.x(),
                            (p1.y() > p2.y()) ? p1.y() : p2.y(),
                            (p1.z() > p2.z()) ? p1.z() : p2.z());
}

/// more than this % of max points must be in the boundingbox before a collision is returned
const float COLLISION_THRESHOLD_RELATIVE = 0.004;
const int MAX_POINTS_POINTCLOUD = 60000;
const int COLLISION_THRESHOLD_ABSOLUT = (int)(COLLISION_THRESHOLD_RELATIVE * MAX_POINTS_POINTCLOUD);

/*
 * Method for Collision Testing on PointClouds
 */
JNIEXPORT jboolean JNICALL
Java_com_thkoeln_jmoeller_autonomoustangobot_exploration_PointCloudHelperJNI_collisionWithPointCloud(
        JNIEnv *env, jclass,
        jobject pointsBuffer, jint numPoints, 
        jfloatArray jFABBXMin, jfloatArray jABBXMax) {
    
    jfloat *pointsFB = (jfloat *) env->GetDirectBufferAddress(pointsBuffer);
    
    octomap::point3d minPTango = jFloatArrayToVector3(env, jFABBXMin);
    octomap::point3d maxPTango = jFloatArrayToVector3(env, jABBXMax);
    
    octomap::point3d minPPC = octomap::point3d(minPTango.x(), - minPTango.z(), minPTango.y());
    octomap::point3d maxPPC = octomap::point3d(maxPTango.x(), - maxPTango.z(), maxPTango.y());
    
    octomap::point3d minP = minCoordinatesPoint(minPPC, maxPPC);
    octomap::point3d maxP = maxCoordinatesPoint(minPPC, maxPPC);
    
    int collidingPoints = 0;
    for (int i = 0; i < numPoints && collidingPoints < COLLISION_THRESHOLD_ABSOLUT; ++i) {
        octomap::point3d p((float) pointsFB[i * 4 + 0],
                           (float) pointsFB[i * 4 + 1],
                           (float) pointsFB[i * 4 + 2]);
        
        if(p.x() >= minP.x() &&
           p.y() >= minP.y() &&
           p.z() >= minP.z() &&
           p.x() <= maxP.x() &&
           p.y() <= maxP.y() &&
           p.z() <= maxP.z()){
            collidingPoints++;
        }
    }

//    __android_log_print(ANDROID_LOG_INFO, "POINTCLOUD HELPER", "collidingPoints: %d of %d\n"
//            "minPoint is %f %f %f\n"
//            "maxPoint is %f %f %f", 
//                        collidingPoints, numPoints, 
//                        minP.x(), minP.y(), minP.z(),
//                        maxP.x(), maxP.y(), maxP.z());
    
    return (jboolean) (collidingPoints >= COLLISION_THRESHOLD_ABSOLUT);
}
}