package com.thkoeln.jmoeller.autonomoustangobot.exploration;

import android.app.Activity;
import android.util.Log;
import android.widget.TextView;

import com.google.atap.tangoservice.TangoPointCloudData;
import com.google.atap.tangoservice.TangoPoseData;
import com.google.atap.tangoservice.experimental.TangoImageBuffer;
import com.projecttango.tangosupport.TangoSupport;
import com.thkoeln.jmoeller.autonomoustangobot.explorationdata.ExplorationData;
import com.thkoeln.jmoeller.autonomoustangobot.ringbuffer3dexperimental.RingBuffer3DJNI;

import java.nio.ByteBuffer;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.TimeUnit;

import static android.R.attr.width;

/**
 * Class that inserts the buffered TangoPointCloudData into the OcTree
 */
public class PointCloudProcessor implements Runnable {

    private static final String DEBUG_TAG = PointCloudProcessor.class.getSimpleName();
    
    private final ExplorationData explorationData;
    private BlockingQueue<TangoPointCloudData> pointCloudDataBlockingQueue;
    private Activity currentActivity;
    private TextView queueCountTextView;
    private TextView insertionTimeTextView;
    private boolean isRunning = true;

    private boolean useDriftCorrection;

    public PointCloudProcessor(ExplorationData explorationData, 
                               BlockingQueue<TangoPointCloudData> pointCloudDataBlockingQueue, 
                               Activity currentActivity, 
                               TextView queueCountTextView, 
                               TextView insertionTimeTextView,
                               boolean useDriftCorrection) {
        this.explorationData = explorationData;
        this.pointCloudDataBlockingQueue = pointCloudDataBlockingQueue;
        this.currentActivity = currentActivity;
        this.queueCountTextView = queueCountTextView;
        this.insertionTimeTextView = insertionTimeTextView;
        this.useDriftCorrection = useDriftCorrection;
    }

    @Override
    public void run() {
        TangoPointCloudData pointCloudData;

        while (isRunning){
            try {
                pointCloudData = pointCloudDataBlockingQueue.poll(1L, TimeUnit.SECONDS);
                // update the currently displayed debug information
                currentActivity.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        queueCountTextView.setText("BlockingQueue contains (elements): " + pointCloudDataBlockingQueue.size());
                    }
                });

                // state changed while waiting for a new pointCloudData Element to process
                // or the polling of it timed out
                if(!isRunning || pointCloudData == null){
                    continue;
                }

                // get the corresponding pose and transformation data 
                // that are needed for the insertion of the point cloud
                int baseFrame = useDriftCorrection?
                        TangoPoseData.COORDINATE_FRAME_AREA_DESCRIPTION :
                        TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE;
                
                TangoPoseData poseData =
                            TangoSupport.getPoseAtTime(
                                    pointCloudData.timestamp,
                                    baseFrame,
                                    TangoPoseData.COORDINATE_FRAME_CAMERA_DEPTH,
                                    TangoSupport.TANGO_SUPPORT_ENGINE_TANGO,
                                    TangoSupport.TANGO_SUPPORT_ENGINE_TANGO,
                                    TangoSupport.ROTATION_IGNORED);

                TangoSupport.TangoMatrixTransformData transformData =
                            TangoSupport.getMatrixTransformAtTime(
                                    pointCloudData.timestamp,
                                    baseFrame,
                                    TangoPoseData.COORDINATE_FRAME_CAMERA_DEPTH,
                                    TangoSupport.TANGO_SUPPORT_ENGINE_TANGO,
                                    TangoSupport.TANGO_SUPPORT_ENGINE_TANGO,
                                    TangoSupport.ROTATION_IGNORED);

                if (poseData.statusCode == TangoPoseData.POSE_VALID && transformData.statusCode == TangoPoseData.POSE_VALID) {
                    
                    long tempTime = System.currentTimeMillis();
                    // acquire lock on explorationData Object
                    synchronized (explorationData) {
                        Log.d(DEBUG_TAG, "Beginning Pointcloud-Insertion");
                        explorationData.getOcTree()
                                .insertTangoPointCloud(
                                        pointCloudData.points,
                                        pointCloudData.numPoints,
                                        transformData.matrix,
                                        poseData.getTranslationAsFloats(),
                                        explorationData.getMetaData().getMaxScanRange(),
                                        explorationData.getMetaData().getScanFilterMinRange());
                        Log.d(DEBUG_TAG, "Finished Pointcloud-Insertion.");
                    }

                    final long timeElapsed = System.currentTimeMillis() - tempTime;
                    // update the currently displayed debug information
                    currentActivity.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            insertionTimeTextView.setText("Last insertion took (ms): " + timeElapsed);
                        }
                    });
                }
                else {
                    Log.e(DEBUG_TAG, "Status invalid - PoseData: " + poseData.statusCode + " TransformData: " + transformData.statusCode);
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        Log.d(DEBUG_TAG, "Stopping PointCloudProcessor successfull");
    }

    public void stop() {
        isRunning = false;
        Log.d(DEBUG_TAG, "Stopping PointCloudProcessor requested");
    }
}
