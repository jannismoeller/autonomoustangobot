#ifndef AUTONOMOUSTANGOBOT_POINTERHANDLER_H
#define AUTONOMOUSTANGOBOT_POINTERHANDLER_H

#include <jni.h>
#include <octomap/math/Vector3.h>

static jfieldID getNativeObjectPointerField(JNIEnv *env, jobject instance)
{
    return env->GetFieldID(env->GetObjectClass(instance), "nativeObjectPointer", "J");
}

/*
 * Method to handle inconvenience of handling the pointer to the native Object
 */
template <typename T>
T *getNativeObjectPointer(JNIEnv *env, jobject instance)
{
    return reinterpret_cast<T *>(env->GetLongField(instance, getNativeObjectPointerField(env, instance)));
}

/** 
 * Convert the float-array of coordinates to a point3d
 */
octomath::Vector3 jFloatArrayToVector3(JNIEnv *env, jfloatArray &floatArray);

#endif //AUTONOMOUSTANGOBOT_POINTERHANDLER_H