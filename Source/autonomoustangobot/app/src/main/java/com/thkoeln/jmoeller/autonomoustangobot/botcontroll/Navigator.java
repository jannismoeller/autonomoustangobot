package com.thkoeln.jmoeller.autonomoustangobot.botcontroll;

import android.app.Activity;
import android.util.Log;
import android.widget.TextView;

import com.google.atap.tangoservice.TangoPointCloudData;
import com.google.atap.tangoservice.TangoPoseData;
import com.thkoeln.jmoeller.autonomoustangobot.exploration.GeometryHelper;
import com.thkoeln.jmoeller.autonomoustangobot.exploration.PointCloudHelperJNI;
import com.thkoeln.jmoeller.autonomoustangobot.explorationdata.BotData;
import com.thkoeln.jmoeller.autonomoustangobot.explorationdata.ExplorationData;
import com.thkoeln.jmoeller.autonomoustangobot.rendering.PointCloudGLRenderer;

import java.nio.FloatBuffer;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

import static android.os.SystemClock.sleep;

/**
 * The Navigator calls the methods of the OcTree to find navigable nodes, 
 * to find frontier-node clusters as targets and to find a good path between them.
 * He then also is steering the Bot towards these targets 
 * and checks for collisions in the process
 */
public class Navigator {

    private static final String TAG = Navigator.class.getSimpleName();

    private static final int NAVIGATION_ANALYSIS_INTERVAL = 1000;
    
    private final ExplorationData explorationData;
    final BotData botData;
    private final String botIP;
    BotController botController;
    private PointCloudGLRenderer visualizationRenderer;

    private boolean isBotMovementAllowed = true;
    private boolean isBotPausing = false;

    /**
     * clear the current behaviourQueue, put the nextBehaviour at the first place 
     * and stop the currentBehaviour
     */
    public void interruptCurrentBehaviour(BotBehaviour nextBehaviour) {
        behaviourQueue = new LinkedBlockingQueue<>();
        behaviourQueue.offer(nextBehaviour);
        
        if(currentBehaviour != null)
            currentBehaviour.stop();
    }

    /**
     * append nextBehaviour to the end of the behaviourQueue
     */
    void setNextBehaviour(BotBehaviour nextBehaviour) {
//        Log.d(TAG, "setNextBehaviour: behaviour ready of class: " + nextBehaviour.getClass().getSimpleName());
        behaviourQueue.offer(nextBehaviour);
    }
    private BlockingQueue<BotBehaviour> behaviourQueue = new LinkedBlockingQueue<>();
    private BotBehaviour currentBehaviour;
    
    private final int FLOATS_PER_WAYPOINT;
    private final int MAX_WAY_POINT_COUNT;
    
    private FloatBuffer wayPointBufffer;
    int getCurrentPathWayPointCount() { return currentPathWayPointCount; }
    private int currentPathWayPointCount = 0;
    int getCurrentWayPointIndex() { return currentWayPointIndex; }
    void stepToNextWayPoint() {
        currentWayPointIndex++;
    }
    private int currentWayPointIndex = 0;
    Vector3d getCurrentWayPoint() {
        return new Vector3d(
                wayPointBufffer.get(currentWayPointIndex * FLOATS_PER_WAYPOINT),
                wayPointBufffer.get(currentWayPointIndex * FLOATS_PER_WAYPOINT + 1),
                wayPointBufffer.get(currentWayPointIndex * FLOATS_PER_WAYPOINT + 2));
    }

    private boolean readyForNewPath = false;
    void setReadyForNewPath(boolean readyForNewPath) { this.readyForNewPath = readyForNewPath; }
      
    
    private Activity currentActivity;
    private TextView nextWayPointTextView;
    public void setNextWayPointTextView(Activity currentActivity, TextView nextWayPointTextView) {
        this.currentActivity = currentActivity;
        this.nextWayPointTextView = nextWayPointTextView;
    }
    
    private boolean isRunning = true;

    private TangoPoseData currentPose = new TangoPoseData();
    private final Object currentPoseSync = new Object();
    TangoPoseData getCurrentPoseCopy() {
        TangoPoseData poseData = new TangoPoseData();
        synchronized (currentPoseSync) {
            poseData.translation = new double[currentPose.translation.length];
            System.arraycopy(currentPose.translation, 0, poseData.translation, 0, poseData.translation.length);

            poseData.rotation = new double[currentPose.rotation.length];
            System.arraycopy(currentPose.rotation, 0, poseData.rotation, 0, poseData.rotation.length);

            poseData.statusCode = currentPose.statusCode;
        }
        return poseData;
    }

    public void setCurrentPose(TangoPoseData pose) {
        synchronized (currentPoseSync) {
            currentPose.translation = new double[pose.translation.length];
            System.arraycopy(pose.translation, 0, currentPose.translation, 0, currentPose.translation.length);
            currentPose.rotation = new double[pose.rotation.length];
            System.arraycopy(pose.rotation, 0, currentPose.rotation, 0, currentPose.rotation.length);

            currentPose.statusCode = pose.statusCode;
        }
    }

    private static final float DISTANCE_COLLISION_CHECK_INFRONT = 0.05f;
    private TangoPointCloudData currentPointCloud;

    boolean hasCollisionInFront() { 
        if(currentPointCloudUpdated) {
            hasCollisionInFront = testCollisionInFront();
            currentPointCloudUpdated = false;
        }
        return hasCollisionInFront; 
    }
    private boolean hasCollisionInFront = false;
    private boolean currentPointCloudUpdated = false;
    public void setCurrentPointCloud(TangoPointCloudData currentPointCloud) { 
        this.currentPointCloud = currentPointCloud;
        currentPointCloudUpdated = true;
    }

    public Navigator(ExplorationData explorationData, String botIP, PointCloudGLRenderer visualizationRenderer) {
        this.explorationData = explorationData;
        this.botData = explorationData.getMetaData().getBotProperties();
        this.botIP = botIP;
        this.botController = new BotController();
        this.visualizationRenderer = visualizationRenderer;

        FLOATS_PER_WAYPOINT = PointCloudGLRenderer.FLOATS_PER_VERTEX_LINE;
        MAX_WAY_POINT_COUNT = PointCloudGLRenderer.MAX_VERTICES_PATH;
        wayPointBufffer = visualizationRenderer.getVertexBuffferPath();

        BotBehaviour.setBotMovementAllowed(true);
        BotBehaviour.isRunning = true;
    }

    public void start() {        
        botController.connect(botIP);
        setNextBehaviour(new HaltBehaviour(this, 2000L));
        setNextBehaviour(new LookAroundBehaviour(this));
        
        new Thread(new NavigationPlanner()).start();

        new Thread(new NavigationExecutor()).start();
        
    }

    public void stop(){
        isRunning = false;
        if(currentBehaviour != null)
            currentBehaviour.stop();
        
        Log.w(TAG, "Tried to stop bot");
        botController.stop();
        botController.disconnect();
    }
    
    private static final String PLANNING_TAG = " PLANNING";
    /**
     * Internal Class that implements the Navigation-Planing features
     */
    private class NavigationPlanner implements Runnable{
        @Override
        public void run() {
            readyForNewPath = true;
            
            int countConsecutiveNoPathFound = 0;
            
            while (isRunning) {
                sleep(NAVIGATION_ANALYSIS_INTERVAL);
                
                long tempTime = System.currentTimeMillis();
                synchronized (explorationData) {
                    Log.d(TAG + PLANNING_TAG, "Beginning Navigation planning\nWaited " + (System.currentTimeMillis() - tempTime) + "ms for lock on octree object");
                    // end thread if isRunning was set to false while sleeping or waiting for explorationData Lock
                    if (!isRunning) break;

                    tempTime = System.currentTimeMillis();

                    BotData botData = explorationData.getMetaData().getBotProperties();
                    explorationData.getOcTree().findNavigableNodes(botData.getRadius(), botData.getHeight(), botData.getStepHeight());
                    
                    visualizationRenderer.setBufferOctreeVertexCount(
                            explorationData.getOcTree().fillBufferWithNodePoints(
                                    visualizationRenderer.getVertexBuffferOcTree(),
                                    PointCloudGLRenderer.getMaxVerticesOctree()));
                    
                    if(readyForNewPath) {
                        readyForNewPath = false;
                        Vector2d dir = GeometryHelper.directionVector2d(currentPose);
                        currentPathWayPointCount = explorationData.getOcTree().
                                findNextPath(wayPointBufffer, MAX_WAY_POINT_COUNT,
                                             currentPose.getTranslationAsFloats(), 
                                             new float[]{(float) dir.x, (float) dir.y, 0}, 
                                             0.75f, 0.75f);
                        currentWayPointIndex = 0;
                        updateCurrentWayPointVisualizations();
                        
                        if(currentPathWayPointCount == 0){
                            // No path was found
                            countConsecutiveNoPathFound++;
                        }
                        else {
                            countConsecutiveNoPathFound = 0;
                        }
                        
                        // If 3 times no path was found change the behaviour to look around
                        if(countConsecutiveNoPathFound >= 3){
                            setNextBehaviour(new LookAroundBehaviour(Navigator.this));
                            countConsecutiveNoPathFound = 0;
                        }
                    }
                    Log.d(TAG + PLANNING_TAG, "Finished Navigation planning after: " + (System.currentTimeMillis() - tempTime) + "ms");
                }
            }
        }
    }

    private static final String EXECUTER_TAG = " EXECUTING";
    /**
     * Internal Class that implements the Navigation-Execution
     */
    private class NavigationExecutor implements Runnable{
        @Override
        public void run() {
            
            // Waiting for Connection to Bot
            while (isRunning && !botController.isConnected()) {
                Log.d(TAG + EXECUTER_TAG, "Botcontroller not connected");
                sleep(1000);
            }
            
            // Execute what ever behaviour is at the first place in the behaviourQueue
            while (isRunning){
                try {
                    currentBehaviour = behaviourQueue.poll(50L, TimeUnit.MILLISECONDS);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                    currentBehaviour = null;
                }
                
                if(currentBehaviour == null){
                    Log.d(TAG + EXECUTER_TAG, "Currently no behaviour is in the queue");
                }
                else {
                    currentBehaviour.executeBehaviour();

                    botController.stop();
                }
            }
        }
    }

    void updateCurrentWayPointVisualizations() {
        if(nextWayPointTextView != null){
            Vector3f nextWP = new Vector3f(wayPointBufffer.get(currentWayPointIndex * 3),
                                           wayPointBufffer.get(currentWayPointIndex * 3 + 1),
                                           wayPointBufffer.get(currentWayPointIndex * 3 + 2));
            final String text = "Next Waypoint: " + GeometryHelper.vToStr(nextWP);
            currentActivity.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    nextWayPointTextView.setText(text);
                }
            });
        }
        
        visualizationRenderer.setBufferPathCurrentIndex(Math.max(currentWayPointIndex - 1, 0));
        visualizationRenderer.setBufferPathVertexCount(currentPathWayPointCount);
    }

    /**
     * this Method is called when the user clicks the "STOP BOT" button
     * the bot may only move when both the user and the blocking queue agree with that
     */
    public void setBotMovementAllowed(boolean allowed){
        isBotMovementAllowed = allowed;
        Log.d(TAG, "setBotMovementAllowed: allowed: " + allowed);
        
        if(isBotMovementAllowed && !isBotPausing)
            BotBehaviour.setBotMovementAllowed(true);
        else 
            BotBehaviour.setBotMovementAllowed(false);
            
    }

    /**
     * this Method is called when blocking Queue is about to overflow
     * the bot may only move when both the user and the blocking queue agree with that
     */
    public void pauseBot(boolean pause){
        isBotPausing = pause;
        Log.d(TAG, "pauseBot: " + pause);

        if(isBotMovementAllowed && !isBotPausing)
            BotBehaviour.setBotMovementAllowed(true);
        else
            BotBehaviour.setBotMovementAllowed(false);
    }
    
    private boolean testCollisionInFront() {
        if(currentPointCloud == null)
            return false;

        Vector3f off = botData.getCameraOffset();
        float r = botData.getRadius();
        float h = botData.getHeight();
        Vector3f minPoint = new Vector3f(-r, DISTANCE_COLLISION_CHECK_INFRONT, 0);
        Vector3f maxPoint = new Vector3f(r, r + DISTANCE_COLLISION_CHECK_INFRONT, h);

        minPoint.sub(off);
        maxPoint.sub(off);

        float[] minPointArray = new float[]{minPoint.x,minPoint.y,minPoint.z};
        float[] maxPointArray = new float[]{maxPoint.x,maxPoint.y,maxPoint.z};

        return PointCloudHelperJNI.collisionWithPointCloud(
                currentPointCloud.points, currentPointCloud.numPoints,
                minPointArray, maxPointArray);
    }
}