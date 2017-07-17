#include "NativeObjectHelper.h"

octomath::Vector3 jFloatArrayToVector3(JNIEnv *env, jfloatArray &floatArray) {
    jfloat *array = env->GetFloatArrayElements(floatArray, 0);
    octomath::Vector3 v((float) array[0],
                        (float) array[1],
                        (float) array[2]);
    env->ReleaseFloatArrayElements(floatArray, array, 0);
    return v;
}