package com.thkoeln.jmoeller.autonomoustangobot.exploration;

import android.nfc.Tag;
import android.util.Log;

import com.google.atap.tangoservice.TangoPointCloudData;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * Helper-Class for custom operations on TangoPointCloudData
 */
public class TangoPointCloudDataHelper {

    private static final String TAG = TangoPointCloudDataHelper.class.getSimpleName();
    
    /**
     * @param pointCloudData TangoPointCloudData to clone
     * @return new TangoPointCloudData that holds a copy of the input TangoPointCloudData
     */
    public static TangoPointCloudData cloneTangoPointCloudData(TangoPointCloudData pointCloudData) {
        TangoPointCloudData pointCloudDataCopy = new TangoPointCloudData();
        pointCloudDataCopy.points = ByteBuffer.allocateDirect(pointCloudData.numPoints * 4 * 4).order(ByteOrder.nativeOrder()).asFloatBuffer();

        pointCloudDataCopy.numPoints = pointCloudData.numPoints;
        pointCloudDataCopy.timestamp = pointCloudData.timestamp;
        pointCloudDataCopy.pointCloudNativeFileDescriptor = pointCloudData.pointCloudNativeFileDescriptor;

        pointCloudData.points.rewind();
        pointCloudDataCopy.points.put(pointCloudData.points);
        pointCloudDataCopy.points.rewind();
        pointCloudData.points.rewind();
        return pointCloudDataCopy;
    }
    
    /**
     * @param pointCloudData TangoPointCloudData to clone
     * @param bufferSizeInPoints capacity of the new pointsBuffer
     * @return new TangoPointCloudData that holds a copy of the input TangoPointCloudData
     */
    public static TangoPointCloudData cloneTangoPointCloudData(TangoPointCloudData pointCloudData, int bufferSizeInPoints) {
        if(bufferSizeInPoints < pointCloudData.numPoints) {
            Log.e(TAG, "bufferSizeInPoints < pointCloudData.numPoints");
            return null;
        }
        TangoPointCloudData pointCloudDataCopy = new TangoPointCloudData();
        pointCloudDataCopy.points = ByteBuffer.allocateDirect(bufferSizeInPoints * 4 * 4).order(ByteOrder.nativeOrder()).asFloatBuffer();

        pointCloudDataCopy.numPoints = pointCloudData.numPoints;
        pointCloudDataCopy.timestamp = pointCloudData.timestamp;
        pointCloudDataCopy.pointCloudNativeFileDescriptor = pointCloudData.pointCloudNativeFileDescriptor;

        pointCloudData.points.rewind();
        pointCloudDataCopy.points.put(pointCloudData.points);
        pointCloudDataCopy.points.rewind();
        pointCloudData.points.rewind();
        return pointCloudDataCopy;
    }
}
