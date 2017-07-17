package com.thkoeln.jmoeller.autonomoustangobot.activities.newexplorationactivities;

import android.app.ProgressDialog;
import android.content.Intent;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.os.Environment;
import android.preference.PreferenceManager;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import com.google.atap.tangoservice.Tango;
import com.google.atap.tangoservice.TangoCameraIntrinsics;
import com.google.atap.tangoservice.TangoConfig;
import com.google.atap.tangoservice.TangoCoordinateFramePair;
import com.google.atap.tangoservice.TangoErrorException;
import com.google.atap.tangoservice.TangoEvent;
import com.google.atap.tangoservice.TangoInvalidException;
import com.google.atap.tangoservice.TangoOutOfDateException;
import com.google.atap.tangoservice.TangoPointCloudData;
import com.google.atap.tangoservice.TangoPoseData;
import com.google.atap.tangoservice.TangoXyzIjData;
import com.google.atap.tangoservice.experimental.TangoImageBuffer;
import com.projecttango.tangosupport.TangoSupport;
import com.thkoeln.jmoeller.autonomoustangobot.R;
import com.thkoeln.jmoeller.autonomoustangobot.exploration.GeometryHelper;
import com.thkoeln.jmoeller.autonomoustangobot.botcontroll.Navigator;
import com.thkoeln.jmoeller.autonomoustangobot.exploration.PointCloudProcessor;
import com.thkoeln.jmoeller.autonomoustangobot.exploration.TangoPointCloudDataHelper;
import com.thkoeln.jmoeller.autonomoustangobot.explorationdata.BotData;
import com.thkoeln.jmoeller.autonomoustangobot.explorationdata.ExplorationData;
import com.thkoeln.jmoeller.autonomoustangobot.explorationdata.ExplorationDataLoader;
import com.thkoeln.jmoeller.autonomoustangobot.octomapjni.OcTreeJNI;
import com.thkoeln.jmoeller.autonomoustangobot.rendering.PointCloudGLSurfaceView;

import java.io.File;
import java.util.ArrayList;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;

import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

public class ExplorationActivity extends AppCompatActivity {
    
    private static final String TAG = ExplorationActivity.class.getSimpleName();

    private static boolean useDriftCorrection;
    private static final double POSITION_EPSILON = 0.03;    // 3cm
    private static final double EPSILON_DEGREES = 3;        // 3°
    private static final double ORIENTATION_EPSILON = Math.cos(EPSILON_DEGREES * (Math.PI / 180));

    private ExplorationData explorationData;
    private Tango tango;
    private TangoConfig tangoConfig;
    
    private ProgressDialog progressDialogConnecting;

    private BlockingQueue<TangoPointCloudData> pointCloudDataBlockingQueue;
    private final int BLOCKINGQUEUE_CAPACITY = 50;
    private PointCloudProcessor pointCloudProcessor;
    private TextView queueCountTextView;
    private TextView currentPositionTextView;

    private Navigator navigator;
    private PointCloudGLSurfaceView pointCloudView;

    // Fields needed for filtering (currently disabled)
    private TangoCameraIntrinsics ccIntrinsics;
    private TangoImageBuffer lastColorTangoImageBuffer;
    
    // Fields for managing PointCloud insertion into the blocking queue
    private double lastPointCloudTimeStamp = 0.01;
    private boolean isFirstValidPose = true;
    private boolean isProcessingToCatchUpQueue = false;
    private boolean toggleStopBot = true; // true if bot is allowed to move and thus currently stoppable

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_exploration);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        // load the preferences
        SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(this);
        useDriftCorrection = prefs.getBoolean(getResources().getString(R.string.pref_tango_usedriftcorrection_key), false);
        
        // try to extract ExplorationData from Intent
        Intent intent = getIntent();
        Object tmpExplorationData = intent.getSerializableExtra("ExplorationData");
        if(tmpExplorationData == null){
            Toast.makeText(this, "Didn't get ExplorationData from Intent", Toast.LENGTH_LONG).show();
            Log.e(TAG, "Couldn't extract ExplorationData from IntentExtra");

            finish();
            return;
        }
        
        explorationData = (ExplorationData) tmpExplorationData;
        
        setupOctree(explorationData);
        
        Log.d(TAG, "explorationData fetching successful!");

        queueCountTextView = (TextView) findViewById(R.id.blockingqueue_elementcount_textview);
        TextView insertionTimeTextView = (TextView) findViewById(R.id.pointcloud_insert_time_textview);
        currentPositionTextView = (TextView) findViewById(R.id.current_position_textview);

        pointCloudDataBlockingQueue = new ArrayBlockingQueue<>(BLOCKINGQUEUE_CAPACITY);
        pointCloudProcessor = new PointCloudProcessor(explorationData, pointCloudDataBlockingQueue, this, queueCountTextView, insertionTimeTextView, useDriftCorrection);


        pointCloudView = (PointCloudGLSurfaceView) findViewById(R.id.pointcloud_gl_surface_view);
        if(pointCloudView != null)
            pointCloudView.setExplorationData(explorationData);
        else 
            Log.e(TAG, "Couldn't find a PointCloudGLSurfaceView");

        navigator = new Navigator(explorationData, 
                                  prefs.getString(getResources().getString(
                                          R.string.pref_botproperties_ip_key), null), 
                                  pointCloudView.getRenderer());
        TextView nextWayPointTextView = (TextView) findViewById(R.id.next_waypoint_textview);
        if(nextWayPointTextView != null)
            navigator.setNextWayPointTextView(this, nextWayPointTextView);
        else
            Log.e(TAG, "Couldn't find next_waypoint_textview");
    }

    /**
     * because the bot cant see the ground at his start-position it is manually inserted 
     * into the Octree
     */
    private void setupOctree(ExplorationData explorationData){
        synchronized (explorationData) {
            OcTreeJNI ocTree = explorationData.getOcTree();
            BotData botData = explorationData.getMetaData().getBotProperties();
            float r = botData.getRadius() + (float) ocTree.getResolution() * 2;
            float h = botData.getHeight() + (float) ocTree.getResolution() * 3;
            Vector3f off = botData.getCameraOffset();

            ocTree.fillBox(new float[]{-r, -r, -off.z}, new float[]{r, r, -off.z + h}, false);
            ocTree.fillBox(new float[]{-r, -r, -off.z}, new float[]{r, r, -off.z}, true);
        }
    }

    @Override
    protected void onResume(){
        super.onResume();

        progressDialogConnecting = ProgressDialog.show(this, "", "Connecting to Tango...");
        tango = new Tango(ExplorationActivity.this, new Runnable() {
            @Override
            public void run() {
                synchronized (ExplorationActivity.this){
                    try {
                        TangoSupport.initialize();
                        tangoConfig = setUpTangoConfig(tango);
                        tango.connect(tangoConfig);
                        startUpTango();
                    } catch (TangoOutOfDateException e) {
                        Log.e(TAG, getString(R.string.exception_out_of_date), e);
                        showsToastAndFinishOnUiThread(R.string.exception_out_of_date);
                    } catch (TangoErrorException e) {
                        Log.e(TAG, getString(R.string.exception_tango_error), e);
                        showsToastAndFinishOnUiThread(R.string.exception_tango_error);
                    } catch (TangoInvalidException e) {
                        Log.e(TAG, getString(R.string.exception_tango_invalid), e);
                        showsToastAndFinishOnUiThread(R.string.exception_tango_invalid);
                    }
                }
            }
        });
        (new Thread(pointCloudProcessor)).start();
        isFirstValidPose = true;
        pointCloudView.onResume();
    }

    @Override
    protected void onPause() {
        super.onPause();

        pointCloudProcessor.stop();
        navigator.stop();

        pointCloudView.onPause();
        
        synchronized (this) {
            try {
                tango.disconnect();
                tango.disconnectCamera(TangoCameraIntrinsics.TANGO_CAMERA_COLOR);
            } catch (TangoErrorException e) {
                Log.e(TAG, getString(R.string.exception_tango_error), e);
            }
        }
    }

    @Override
    public void onBackPressed() {
        super.onBackPressed();
        finish();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        synchronized (explorationData) {
            explorationData.getOcTree().delete();
        }
    }

    /**
     * Setup Tango to additionally to default settings use depth perception, color image 
     * and drift correction
     */
    private TangoConfig setUpTangoConfig(Tango tango) {
        TangoConfig tangoConfig = tango.getConfig(TangoConfig.CONFIG_TYPE_DEFAULT);
        tangoConfig.putBoolean(TangoConfig.KEY_BOOLEAN_DEPTH, true);
        tangoConfig.putBoolean(TangoConfig.KEY_BOOLEAN_COLORCAMERA, true);
        tangoConfig.putBoolean(TangoConfig.KEY_BOOLEAN_DRIFT_CORRECTION, true);
        tangoConfig.putInt(TangoConfig.KEY_INT_DEPTH_MODE, TangoConfig.TANGO_DEPTH_MODE_POINT_CLOUD);
        return tangoConfig;
    }

    
    private void startUpTango() {
        final ArrayList<TangoCoordinateFramePair> framePairs =
                new ArrayList<>();
        if(useDriftCorrection)
            framePairs.add(new TangoCoordinateFramePair(
                TangoPoseData.COORDINATE_FRAME_AREA_DESCRIPTION,
                TangoPoseData.COORDINATE_FRAME_DEVICE));
        else 
            framePairs.add(new TangoCoordinateFramePair(
                TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                TangoPoseData.COORDINATE_FRAME_DEVICE));

        // needed for filtering (currently disabled)
//        ccIntrinsics = tango.getCameraIntrinsics(TangoCameraIntrinsics.TANGO_CAMERA_COLOR);
//        tango.experimentalConnectOnFrameListener(TangoCameraIntrinsics.TANGO_CAMERA_COLOR, new Tango.OnFrameAvailableListener() {
//            @Override
//            public void onFrameAvailable(TangoImageBuffer tangoImageBuffer, int cameraId) {
//                if(cameraId == TangoCameraIntrinsics.TANGO_CAMERA_COLOR) {
//                    lastColorTangoImageBuffer = tangoImageBuffer;
//                }
//                else 
//                    Log.wtf(TAG, "What a terrible failure, I subcribed to another camera, this message should never come up!");
//            }
//        });
        
        // Implement a Listener for new Tango data.
        tango.connectListener(framePairs, new Tango.TangoUpdateCallback() {
            /**
             * process a freshly available TangoPoseData Object by handing it over
             * to the correct objects of this application
             */
            @Override
            public void onPoseAvailable(final TangoPoseData pose) {
                int referenceFrame = useDriftCorrection ?
                                     TangoPoseData.COORDINATE_FRAME_AREA_DESCRIPTION :
                                     TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE;

                TangoPoseData correctCoordSysPoseData =
                        TangoSupport.getPoseAtTime(
                                pose.timestamp,
                                referenceFrame,
                                TangoPoseData.COORDINATE_FRAME_CAMERA_DEPTH,
                                TangoSupport.TANGO_SUPPORT_ENGINE_TANGO,
                                TangoSupport.TANGO_SUPPORT_ENGINE_TANGO,
                                TangoSupport.ROTATION_IGNORED);
                if(correctCoordSysPoseData.statusCode == TangoPoseData.POSE_VALID){
                    navigator.setCurrentPose(correctCoordSysPoseData);

                    Vector3d pos = new Vector3d(correctCoordSysPoseData.translation);
                    final String text = "Position:      " + GeometryHelper.vToStr(pos);
                    
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            currentPositionTextView.setText(text);
                        }
                    });
                    
                    if(isFirstValidPose){
                        isFirstValidPose = false;
                        if(progressDialogConnecting.isShowing())
                            progressDialogConnecting.dismiss();
                        navigator.start();
                    }
                }
                
                TangoSupport.TangoMatrixTransformData transformData =
                        TangoSupport.getMatrixTransformAtTime(
                                pose.timestamp,
                                TangoPoseData.COORDINATE_FRAME_CAMERA_DEPTH,
                                referenceFrame,
                                TangoSupport.TANGO_SUPPORT_ENGINE_OPENGL,
                                TangoSupport.TANGO_SUPPORT_ENGINE_TANGO,
                                TangoSupport.ROTATION_IGNORED);
                if(transformData.statusCode == TangoPoseData.POSE_VALID)
                    pointCloudView.setCurrentMatrixTransformation(transformData);
            }

            @Override
            public void onXyzIjAvailable(TangoXyzIjData xyzIj) {
                // We are not using onXyzIjAvailable for this app.
            }
            
            /**
             * process a freshly available TangoPointCloudData Object by handing it over
             * to the navigator and putting a copy of it into the blockingQueue if its 
             * corresponding Pose significantly differs from the last pose
             */
            @Override
            public void onPointCloudAvailable(final TangoPointCloudData pointCloudData) {
                if(pointCloudData.points == null) {
                    // e.g. when the sensor is being covered
                    Log.d(TAG, "pointCloudData didn't contain any Points");
                    return;
                }
                
                TangoPointCloudData pointCloudDataCopy = TangoPointCloudDataHelper.cloneTangoPointCloudData(pointCloudData);
                navigator.setCurrentPointCloud(pointCloudDataCopy);                
                
                TangoPoseData poseLastPointCloud = getPoseDataAt(lastPointCloudTimeStamp);
                TangoPoseData poseThisPointCloud = getPoseDataAt(pointCloudData.timestamp);
                
                if(poseThisPointCloud.statusCode != TangoPoseData.POSE_VALID) {
                    Log.d(TAG, "Dropped PointCloud because current Pose is invalid");
                    return;
                }
                
                if(!isProcessingToCatchUpQueue){
                
                    // If the pose of the last PointCloud the poses need to be compared 
                    // to decide if this PointCloud holds significant new data, 
                    // otherwise it is inserted anyways
                    if(poseLastPointCloud.statusCode == TangoPoseData.POSE_VALID){
                        
                        double positionDifference = GeometryHelper.poseDifferencePosition(poseLastPointCloud, poseThisPointCloud).length();
                        double orientationDifference = GeometryHelper.poseDifferenceRotation(poseLastPointCloud, poseThisPointCloud);
                        
                        // Only log the Position if it changed over x Meters
                        if(positionDifference > POSITION_EPSILON) {
                            explorationData.addPoseToHistory(poseThisPointCloud);
                            pointCloudView.getRenderer().addPosToHistoryBuffer(poseThisPointCloud);
                        }
                        if(positionDifference < POSITION_EPSILON 
                                && orientationDifference > ORIENTATION_EPSILON) {
    //                        Log.d(TAG, "Dropped PointCloud because no significant change in Pose has been detected");
                            return;
                        }
                    }

                    // Functionality for filtering (currently disabled)
//                    int referenceFrame = useDriftCorrection ?
//                                         TangoPoseData.COORDINATE_FRAME_AREA_DESCRIPTION :
//                                         TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE;

//                    long tempTime = System.currentTimeMillis();
//                    TangoPointCloudData pointCloudDataCopy = PointCloudFilterer.filterPointCloud(
//                            pointCloudData, lastColorTangoImageBuffer, ccIntrinsics);
                    

                    boolean addedDataToQueue = pointCloudDataBlockingQueue.offer(pointCloudDataCopy);
                    if(!addedDataToQueue) {
                        Log.d(TAG, "Dropped PointCloud because the BlockingQueue was full");
                        return;
                    }
                    
                    lastPointCloudTimeStamp = pointCloudDataCopy.timestamp;

                    updateBlockingQueueSizeText();
                    
                    if (pointCloudDataBlockingQueue.size() > 0.9 * BLOCKINGQUEUE_CAPACITY) {
                        // The Queue is about to get filled up. 
                        // Halt the bot to ensure the pose doesn't change and no new PointClouds are inserted
                        navigator.pauseBot(true);
                        isProcessingToCatchUpQueue = true;
                        Toast.makeText(ExplorationActivity.this,
                                       "Halting to catchup processing the PointCloudData",
                                       Toast.LENGTH_SHORT).show();
                    }
                }
                else {
                    Log.d(TAG, "Dropped PointCloud because " +
                               "the PointCloudProcessor needs to catch up");
                    
                    updateBlockingQueueSizeText();
                    
                    if (pointCloudDataBlockingQueue.size() < 0.5 * BLOCKINGQUEUE_CAPACITY) {
                        isProcessingToCatchUpQueue = false;
                        navigator.pauseBot(false);
                    }
                }
            }

            @Override
            public void onTangoEvent(final TangoEvent event) {
                // Ignoring TangoEvents.
            }

            @Override
            public void onFrameAvailable(int cameraId) {
                Log.wtf("WHY", "DOESNT THIS GET CALLED - EVER???");
                if(cameraId == TangoCameraIntrinsics.TANGO_CAMERA_COLOR){
                    Log.d(TAG, "onFrameAvailable: Color-Camera-FrameAvailable!");
                }
                else {
                    Log.d(TAG, "onFrameAvailable: FrameAvailable from camera with id: " + cameraId);
                }
            }
        });
    }

    private void updateBlockingQueueSizeText() {
        final int currentBlockingQueueSize = pointCloudDataBlockingQueue.size();
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                queueCountTextView.setText("BlockingQueue contains (elements): " + currentBlockingQueueSize);
            }
        });
    }

    /**
     * Convenience Wrapper to get the PoseData at a specific timestamp while considering the 
     * right referenceFrame
     */
    private TangoPoseData getPoseDataAt(double timestamp){
        int referenceFrame = useDriftCorrection ?
                             TangoPoseData.COORDINATE_FRAME_AREA_DESCRIPTION :
                             TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE;

        return TangoSupport.getPoseAtTime(
                timestamp,
                referenceFrame,
                TangoPoseData.COORDINATE_FRAME_CAMERA_DEPTH,
                TangoSupport.TANGO_SUPPORT_ENGINE_TANGO,
                TangoSupport.TANGO_SUPPORT_ENGINE_TANGO,
                TangoSupport.ROTATION_IGNORED);
    }

    public void onClickStopBot(View view) {
        // Toggle Button between stop and continue
        if(toggleStopBot){
            toggleStopBot = false;
            navigator.setBotMovementAllowed(false);
            Button stopButton = (Button)findViewById(R.id.stop_bot_button);
            stopButton.setText("Continue");
            stopButton.setBackgroundColor(0xb3cccccc);
            Toast.makeText(this, "Bot stopped", Toast.LENGTH_SHORT).show();
        }
        else{
            toggleStopBot = true;
            navigator.setBotMovementAllowed(true);
            Button stopButton = (Button)findViewById(R.id.stop_bot_button);
            stopButton.setText("Stop Bot");
            stopButton.setBackgroundColor(getResources().getColor(android.R.color.holo_red_light, null));
        }
    }

    public void onClickSaveExploration(View view) {
        pointCloudProcessor.stop();

        String fileName = "Exploration_" + explorationData.getDateAsFormattedString() + ".zip";
        PreferenceManager.setDefaultValues(this, R.xml.pref_general, false);
        SharedPreferences preferences = PreferenceManager.getDefaultSharedPreferences(this);
        String directoryPath = preferences.getString(
                getResources().getString(R.string.pref_explorations_filepath_key), "");
        File directoryFile = new File(Environment.getExternalStorageDirectory() + directoryPath);
        if(!directoryFile.exists())
            directoryFile.mkdirs();

        String filepath = directoryFile + File.separator + fileName;
        if(ExplorationDataLoader.writeToFile(explorationData, filepath, this))
            Toast.makeText(this, "Saved as " + fileName, Toast.LENGTH_SHORT).show();
        else
            Toast.makeText(this, "Saving failed", Toast.LENGTH_SHORT).show();
        
        finish();
    }

    /**
     * handles the Result of the permission request for google tango
     */
    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        // Check which request we're responding to.
        if (requestCode == Tango.TANGO_INTENT_ACTIVITYCODE) {
            // Make sure the request was successful.
            if (resultCode == RESULT_CANCELED) {
                Toast.makeText(this, R.string.no_permissions, Toast.LENGTH_LONG).show();
            }
            else {
                Log.d("onActivityResult", "resultCode is: " + resultCode + " , data is: " + data.getDataString());
                data.getDataString();
            }
        }
    }

    /**
     * Display a toast on the UI thread.
     * @param resId The resource id of the string resource to use. Can be formatted text.
     */
    private void showsToastAndFinishOnUiThread(final int resId) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(ExplorationActivity.this,
                        getString(resId), Toast.LENGTH_LONG).show();
                finish();
            }
        });
    }
}
