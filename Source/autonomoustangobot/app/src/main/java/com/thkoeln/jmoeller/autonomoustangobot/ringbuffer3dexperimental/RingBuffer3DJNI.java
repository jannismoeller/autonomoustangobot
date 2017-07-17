package com.thkoeln.jmoeller.autonomoustangobot.ringbuffer3dexperimental;
import java.nio.FloatBuffer;

/**
 * Native Interface for the experimental implementation of a 3D Ringbuffer.
 * Currently not usable
 */
@Deprecated
public class RingBuffer3DJNI {
    static {
        System.loadLibrary("octomapjni");
    }

    protected final long nativeObjectPointer;

    public long getNativeObjectPointer(){return nativeObjectPointer;}

    public RingBuffer3DJNI(float resolution) {
        this.nativeObjectPointer = RingBuffer3DJNI.instantiate(resolution);
    }

    public native void delete();

    public static native long instantiate(float resolution);

    public native void insertPointCloudFromBuffer(FloatBuffer buffer, int numPoints, float[] transformationMatrix, float[] sensorOrigin);
}