#include <jni.h>
#include "../octomapcustomization/NavigationOcTree.h"
#include "NativeObjectHelper.h"

using namespace octomapcustomization;
using namespace octomath;
using namespace std;

extern "C" {
/// C++ Part of the JNI

/**
* Instantiates a new empty octree with the given resolution.
*/
JNIEXPORT jlong JNICALL
Java_com_thkoeln_jmoeller_autonomoustangobot_octomapjni_OcTreeJNI_instantiate
        (JNIEnv *env, jobject instance, jdouble res) {
    NavigationOcTree *ocTree = new NavigationOcTree(res);
    
    return reinterpret_cast<jlong>(ocTree);
}

/**
* Deletes the instance of Octree bound to the Java Wrapper Object
*/
JNIEXPORT void JNICALL
Java_com_thkoeln_jmoeller_autonomoustangobot_octomapjni_OcTreeJNI_delete
        (JNIEnv *env, jobject instance) {
    NavigationOcTree *ocTree = getNativeObjectPointer<NavigationOcTree>(env, instance);
    __android_log_print(ANDROID_LOG_INFO, "OCTREE", "delete ocTree;");
    delete ocTree;
}

/**
 * This method writes an octree, given the pointer to the object and the
 * filename.
 */
JNIEXPORT jboolean JNICALL
Java_com_thkoeln_jmoeller_autonomoustangobot_octomapjni_OcTreeJNI_write
        (JNIEnv *env, jobject instance, jstring filename) {
    NavigationOcTree *ocTree = getNativeObjectPointer<NavigationOcTree>(env, instance);
    //convert jstring into native char*
    jboolean iscopy = (jboolean) false;
    const char *nativeFilename = env->GetStringUTFChars(filename, &iscopy);
    //write ocTree in filename
    bool value = ocTree->write(nativeFilename);
    //release memory of native char*
    env->ReleaseStringUTFChars(filename, nativeFilename);
    //return obtained value
    return (jboolean) value;
}

/**
 * This method reads an octree, given the filename and returns a pointer to the octree.
 */
JNIEXPORT jlong JNICALL
Java_com_thkoeln_jmoeller_autonomoustangobot_octomapjni_OcTreeJNI_readNative(JNIEnv *env,
                                                                             jclass,
                                                                             jstring filename_) {
    const char *filename = env->GetStringUTFChars(filename_, 0);

    AbstractOcTree *abstractOcTree = AbstractOcTree::read(filename);

    env->ReleaseStringUTFChars(filename_, filename);

    return reinterpret_cast<jlong>(abstractOcTree);
}

/**
 * This method inserts a pointCloud in the Tango Format into the octree.
 */
JNIEXPORT void JNICALL
Java_com_thkoeln_jmoeller_autonomoustangobot_octomapjni_OcTreeJNI_insertTangoPointCloud(
        JNIEnv *env,
        jobject instance,
        jobject jPointsFloatBuffer,
        jint numPoints,
        jfloatArray jATransformationMatrix,
        jfloatArray jASensorOrigin,
        jdouble maxrange,
        jdouble minrange
) {
    NavigationOcTree *ocTree = getNativeObjectPointer<NavigationOcTree>(env, instance);

    // Convert the float-array of coordinates to a point3d
    point3d sensorOrigin = jFloatArrayToVector3(env, jASensorOrigin);

    // get a pointer to the beginning of the points-float-buffer 
    jfloat *pointsFloatBuffer = (jfloat *) env->GetDirectBufferAddress(jPointsFloatBuffer);
    
    // get a pointer to the beginning of the transformMatrix array 
    jfloat *transformationMatrix = env->GetFloatArrayElements(jATransformationMatrix, 0);

    // insert the points from the buffer into the OcTree:
    ocTree->insertTangoPointCloud(pointsFloatBuffer, (int)numPoints, transformationMatrix, sensorOrigin, maxrange, minrange);
    
    env->ReleaseFloatArrayElements(jATransformationMatrix, transformationMatrix, 0);
}

JNIEXPORT jlong JNICALL
Java_com_thkoeln_jmoeller_autonomoustangobot_octomapjni_OcTreeJNI_getLeafNodeCount(JNIEnv *env,
                                                                                   jobject instance) {
    NavigationOcTree *ocTree = getNativeObjectPointer<NavigationOcTree>(env, instance);
    return static_cast<jlong>(ocTree->getNumLeafNodes());
}

JNIEXPORT jdouble JNICALL
Java_com_thkoeln_jmoeller_autonomoustangobot_octomapjni_OcTreeJNI_getResolution(JNIEnv *env,
                                                                                jobject instance) {
    NavigationOcTree *ocTree = getNativeObjectPointer<NavigationOcTree>(env, instance);
    return static_cast<jdouble>(ocTree->getResolution());
}

JNIEXPORT jboolean JNICALL
Java_com_thkoeln_jmoeller_autonomoustangobot_octomapjni_OcTreeJNI_testCollisionCylinder(JNIEnv *env,
                                                                                        jobject instance, 
                                                                                        jfloatArray jAPos,
                                                                                        jfloat radius,
                                                                                        jfloat height,
                                                                                        jint maxDepth) {
    NavigationOcTree *ocTree = getNativeObjectPointer<NavigationOcTree>(env, instance);
    
    // Convert the float-array of coordinates to a point3d
    point3d posPoint = jFloatArrayToVector3(env, jAPos);

    return (jboolean)ocTree->testCollisionWithCylinder(posPoint, (float)radius, (float)height, 
                                                       (unsigned char)maxDepth);
}

JNIEXPORT void JNICALL
Java_com_thkoeln_jmoeller_autonomoustangobot_octomapjni_OcTreeJNI_fillBox(JNIEnv *env,
                                                                          jobject instance,
                                                                          jfloatArray jAFrom,
                                                                          jfloatArray jATo,
                                                                          jboolean occupied) {
    NavigationOcTree *ocTree = getNativeObjectPointer<NavigationOcTree>(env, instance);

    // Convert the float-array of coordinates to a point3d
    point3d fromPoint = jFloatArrayToVector3(env, jAFrom);

    // Convert the float-array of coordinates to a point3d
    point3d toPoint = jFloatArrayToVector3(env, jATo);
    
    ocTree->fillBox(fromPoint, toPoint, (bool)occupied);
}

JNIEXPORT void JNICALL
Java_com_thkoeln_jmoeller_autonomoustangobot_octomapjni_OcTreeJNI_fillBoxAsMeasuremants(JNIEnv *env,
                                                                                        jobject instance,
                                                                                        jfloatArray jAFrom,
                                                                                        jfloatArray jATo,
                                                                                        jboolean occupied) {
    NavigationOcTree *ocTree = getNativeObjectPointer<NavigationOcTree>(env, instance);

    // Convert the float-array of coordinates to a point3d
    point3d fromPoint = jFloatArrayToVector3(env, jAFrom);

    // Convert the float-array of coordinates to a point3d
    point3d toPoint = jFloatArrayToVector3(env, jATo);

    ocTree->fillBoxAsMeasurements(fromPoint, toPoint, (bool)occupied);
}

/**
 * Iterating manually only over thoese nodes that are occupied
 */
JNIEXPORT jint JNICALL
Java_com_thkoeln_jmoeller_autonomoustangobot_octomapjni_OcTreeJNI_fillBufferWithNodePoints(
        JNIEnv *env, jobject instance, jobject buffer, jint bufferMaxVertsCount) {

    NavigationOcTree *ocTree = getNativeObjectPointer<NavigationOcTree>(env, instance);
    jfloat *pFB = (jfloat *) env->GetDirectBufferAddress(buffer);

    return (jint) ocTree->fillBufferWithNodePoints(pFB, (int)bufferMaxVertsCount);
}


JNIEXPORT jint JNICALL
Java_com_thkoeln_jmoeller_autonomoustangobot_octomapjni_OcTreeJNI_fillBufferWithNodePointsViaLeafIterator(
        JNIEnv *env, jobject instance, jobject buffer, jint bufferMaxVertsCount) {

    NavigationOcTree *ocTree = getNativeObjectPointer<NavigationOcTree>(env, instance);
    jfloat *pFB = (jfloat *) env->GetDirectBufferAddress(buffer);

    return ocTree->fillBufferWithNodePointsViaLeafIterator(pFB, bufferMaxVertsCount);
}


/**
 * @return 0 for all cells free, 1 for >=1 cell occupied, 2 for >=1 cell unknown space
 */
JNIEXPORT jint JNICALL
Java_com_thkoeln_jmoeller_autonomoustangobot_octomapjni_OcTreeJNI_raySteppingUp(JNIEnv *env,
                                                                                jobject instance,
                                                                                jfloatArray jAOrigin,
                                                                                jfloat height) {

    NavigationOcTree *ocTree = getNativeObjectPointer<NavigationOcTree>(env, instance);

    // Convert the float-array of coordinates to a point3d
    point3d origin = jFloatArrayToVector3(env, jAOrigin);

    if (origin.z() + height > ocTree->getNodeSize(1) ||
        origin.z() < -ocTree->getNodeSize(1)) {
        // Start or end of ray are positioned outside of the OcTree 
        return -1;
    }
    
    double maxZCoord = (double)(origin.z() + (float)height);

    OcTreeKey originKey = ocTree->coordToKey(origin);
    
    return ocTree->raySteppingUp(originKey, maxZCoord);
}

JNIEXPORT void JNICALL
Java_com_thkoeln_jmoeller_autonomoustangobot_octomapjni_OcTreeJNI_setChangedBBXToEntireTree(
        JNIEnv *env, jobject instance) {

    NavigationOcTree *ocTree = getNativeObjectPointer<NavigationOcTree>(env, instance);
    ocTree->setChangedBBXToEntireTree();
}

static long currentTimeInNanos() {

    struct timespec time;
    clock_gettime(CLOCK_MONOTONIC, &time);
    return (time.tv_sec * 1000000000) + time.tv_nsec;
}

JNIEXPORT void JNICALL
Java_com_thkoeln_jmoeller_autonomoustangobot_octomapjni_OcTreeJNI_findNavigableNodes(JNIEnv *env,
                                                                                       jobject instance,
                                                                                       jfloat radius,
                                                                                       jfloat height,
                                                                                       jfloat maxStepHeight) {

    NavigationOcTree *ocTree = getNativeObjectPointer<NavigationOcTree>(env, instance);
    
    long tempTime = currentTimeInNanos();
    bool success = ocTree->updateNavigabilityAndAdjacencyNet((float) radius, (float) height, (float) maxStepHeight);
    __android_log_print(ANDROID_LOG_INFO, "OCTREE", "updateNavigatabilityAndAdjacencyNet took: %.3lfms successfull: %d", (double)(currentTimeInNanos() - tempTime) / 1000000, success);
}

JNIEXPORT jint JNICALL
Java_com_thkoeln_jmoeller_autonomoustangobot_octomapjni_OcTreeJNI_findNextPath(JNIEnv *env,
                                                                               jobject instance,
                                                                               jobject wayPointBuffer,
                                                                               jint maxWayPoints,
                                                                               jfloatArray jAPos,
                                                                               jfloatArray jADir,
                                                                               jfloat botNodeSearchRange,
                                                                               jfloat clusterMaxRange) {

    NavigationOcTree *ocTree = getNativeObjectPointer<NavigationOcTree>(env, instance);

    jfloat *wpFB = (jfloat *) env->GetDirectBufferAddress(wayPointBuffer);
    
    // Convert the float-array of coordinates to a point3d
    point3d botPosition = jFloatArrayToVector3(env, jAPos);
    Vector3 botDirection = jFloatArrayToVector3(env, jADir);
    
    AStarPath* path = ocTree->findNextPath(botPosition, botDirection, (float)botNodeSearchRange, (float)clusterMaxRange);
    if(path == NULL){
        __android_log_print(ANDROID_LOG_INFO, "OCTREE", "No path was found");
        return (jint)0;
    }
    
    int wayPointCount = 0;
    for(point3d waypoint : *(path->wayPointList)){
        if(wayPointCount >= (int)maxWayPoints)
            break;
        
        wpFB[wayPointCount * 3 + 0] = (jfloat)waypoint.x();
        wpFB[wayPointCount * 3 + 1] = (jfloat)waypoint.y();
        wpFB[wayPointCount * 3 + 2] = (jfloat)waypoint.z();
        wayPointCount++;
    }

    return (jint)wayPointCount;
}

} // end extern "C"