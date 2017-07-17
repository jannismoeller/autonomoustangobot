package com.thkoeln.jmoeller.autonomoustangobot.exploration;

import android.util.Log;

import com.google.atap.tangohelperlib.BuildConfig;
import com.google.atap.tangoservice.TangoCameraIntrinsics;
import com.google.atap.tangoservice.TangoPointCloudData;
import com.google.atap.tangoservice.TangoPoseData;
import com.google.atap.tangoservice.experimental.TangoImageBuffer;
import com.projecttango.tangosupport.TangoSupport;

/**
 * Class which offers a Method to filter a given PointCloud
 * Currently because of slow performance and artifacts in the results not used
 */
public class PointCloudFilterer {
    private static final String TAG = PointCloudFilterer.class.getSimpleName();

    /**
     * Filter a given PointCloud with the TangoSupport Method upsampleImageBilateral and convert it
     * the depthBuffer back into a PointCloud
     */
    public static TangoPointCloudData filterPointCloud(TangoPointCloudData pointCloudData,
                                                       TangoImageBuffer imageBuffer,
                                                       TangoCameraIntrinsics colorCamIntrinsics){
        final boolean useApproximationForSampling = true;

        TangoPoseData colorCameraTPointCloud = TangoSupport.getPoseAtTime(
                pointCloudData.timestamp,
                TangoPoseData.COORDINATE_FRAME_CAMERA_COLOR,
                TangoPoseData.COORDINATE_FRAME_CAMERA_DEPTH,
                TangoSupport.TANGO_SUPPORT_ENGINE_TANGO,
                TangoSupport.TANGO_SUPPORT_ENGINE_TANGO,
                TangoSupport.ROTATION_IGNORED);
        
        long tempTime = System.currentTimeMillis();

        // depth sensor resolution: 224 x 172
        final int width = 1920 / 2 / 2; // 480
        final int height = 1080 / 2 / 2; // 270
        TangoSupport.DepthBuffer depthBuffer = TangoSupport.upsampleImageBilateral(useApproximationForSampling, pointCloudData, imageBuffer, colorCameraTPointCloud);
        Log.d(TAG, "Upsampling took: " + (System.currentTimeMillis() - tempTime) + " ms to complete");
        
        
        TangoPointCloudData resCloud = TangoPointCloudDataHelper.cloneTangoPointCloudData(
                pointCloudData, 
                Math.max(width * height, pointCloudData.numPoints));
        
        resCloud.points.rewind();

        if(resCloud == null)
            Log.wtf(TAG, "resCloud shouldn't be null because I made sure " +
                         "that the new buffer has at least the size of the original one");

        resCloud.numPoints = PointCloudHelperJNI.filterPointCloudNative(
                depthBuffer.depths, depthBuffer.width, depthBuffer.height,
                resCloud.points, width, height,
                (float) colorCamIntrinsics.fx, (float) colorCamIntrinsics.fy);
        
        return resCloud;
    }

}
