package com.thkoeln.jmoeller.autonomoustangobot.exploration;

import java.nio.FloatBuffer;

/**
 * native implementation of helper-functions operating on pointclouds
 */
public class PointCloudHelperJNI {
    /**
     * Fills the pointCloudPointsBuffer by transforming the depthBuffer to a PointCloud
     * @return numPoints of the new pointCloud
     */
    public static native int filterPointCloudNative(FloatBuffer depthBuffer,
                                             int depthBufferWidth, int depthBufferHeight,
                                             FloatBuffer pointCloudPointsBuffer,
                                             int sampleWidth, int sampleHeight,
                                             float focalLengthX, float focalLengthY);
    
    /**
     * checks if there is a collision between the points of a PointCloud and a given Bounding-Box
     */
    public static native boolean collisionWithPointCloud(FloatBuffer pointsBuffer, int numPoints,
                                                         float[] jFABBXMin, float[] jABBXMax);
}
