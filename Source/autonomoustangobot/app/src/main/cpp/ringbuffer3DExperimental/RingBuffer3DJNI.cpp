#include <jni.h>
//TODO: get rid off relative directory location including
#include "../octomapjni/NativeObjectHelper.h"
#include "RingBuffer3D.h"

extern "C" {
    /**
    * Instantiates a new empty ringbuffer with the given resolution.
    */
    JNIEXPORT jlong JNICALL
    Java_com_thkoeln_jmoeller_autonomoustangobot_ringbuffer3dexperimental_RingBuffer3DJNI_instantiate
            (JNIEnv *env, jobject obj, jfloat res) {
        RingBuffer3D * rb3D = new RingBuffer3D(res);

        return reinterpret_cast<jlong>(rb3D);
    }

    /**
    * Deletes the instance of Octree bound to the Java Wrapper Object
    */
    JNIEXPORT void JNICALL
    Java_com_thkoeln_jmoeller_autonomoustangobot_ringbuffer3dexperimental_RingBuffer3DJNI_delete
            (JNIEnv *env, jobject obj) {
        RingBuffer3D * rb3D = reinterpret_cast<RingBuffer3D *>(
                env->GetLongField(obj,
                                  env->GetFieldID(
                                          env->GetObjectClass(obj),
                                          "nativeObjectPointer", "J")));

        delete rb3D;
    }

    JNIEXPORT void JNICALL
    Java_com_thkoeln_jmoeller_autonomoustangobot_ringbuffer3dexperimental_RingBuffer3DJNI_insertPointCloudFromBuffer(
            JNIEnv *env,
            jobject obj,
            jobject floatBuffer,
            jint numPoints,
            jfloatArray transformationMatrix,
            jfloatArray sensorOrigin){
        RingBuffer3D * rb3D = getNativeObjectPointer<RingBuffer3D>(env, obj);

        jfloat * pFB =(jfloat *) env->GetDirectBufferAddress(floatBuffer);
        jfloat * m4x4 = env->GetFloatArrayElements(transformationMatrix, 0);
        jfloat * sensorCoordArray = env->GetFloatArrayElements(sensorOrigin, 0);

        rb3D->insertPointCloud(pFB, numPoints, m4x4, sensorCoordArray);

//        env->ReleaseFloatArrayElements(sensorOrigin, sensorCoordArray, 0);
//        env->ReleaseFloatArrayElements(transformationMatrix, m4x4, 0);
    }
}