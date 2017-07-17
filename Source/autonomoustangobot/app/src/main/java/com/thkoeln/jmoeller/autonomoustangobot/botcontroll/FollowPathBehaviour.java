package com.thkoeln.jmoeller.autonomoustangobot.botcontroll;

import android.util.Log;

import com.google.atap.tangoservice.TangoPoseData;
import com.thkoeln.jmoeller.autonomoustangobot.exploration.GeometryHelper;

import javax.vecmath.Vector3d;

import static android.os.SystemClock.sleep;

/**
 * TODO: Add a class header comment!
 */
class FollowPathBehaviour extends BotBehaviour {
    private static final String TAG = FollowPathBehaviour.class.getSimpleName();

    // How close does the bot have to come to the current WayPoint to reach it
    private static final double WAY_POINT_REACHED_EPSILON = 0.05;
    // Angle in degrees above which the bot will use the fast turn speed
    private static final double WAY_POINT_DIRECTION_EPSILON_FAST_TURN = 8;
    // Angle in degrees above which the bot will still use the slow turn speed
    // and below which the bot will switch to forward drive
    private static final double WAY_POINT_DIRECTION_EPSILON_SLOW_TURN = 3;
    // Angle below which the bot will use the fast forward drive speed
    private static final double WAY_POINT_DIRECTION_EPSILON_FAST_FORWARD = 1.75;

    FollowPathBehaviour(Navigator navigator) {
        super(navigator);
    }

    @Override
    public void executeBehaviour() {
        Log.d(TAG, "Entered FOLLOW_PATH");

        while (isRunning) {
            if (!isBotMovementAllowed) {
                botController.stop();
                sleep(MS_SLEEP_TIME);
                continue;
            }

            if (navigator.getCurrentWayPointIndex() >= navigator.getCurrentPathWayPointCount()) {
                Log.d(TAG, "Reached end of Path");
                navigator.setReadyForNewPath(true);
                navigator.setNextBehaviour(new HaltBehaviour(navigator, 2000L));
                navigator.setNextBehaviour(new FollowPathBehaviour(navigator));
                break;
            }

            TangoPoseData currentPose = navigator.getCurrentPoseCopy();
            if (currentPose.statusCode != TangoPoseData.POSE_VALID) {
                Log.d(TAG, "Current Pose INVALID!");
                sleep(MS_SLEEP_TIME);
                continue;
            }

            Vector3d currentWayPoint = navigator.getCurrentWayPoint();
            Log.d(TAG, "currentWayPoint: " + GeometryHelper.vToStr(currentWayPoint));

            Vector3d offSetFromWayPoint = new Vector3d();
            offSetFromWayPoint.sub(currentWayPoint, new Vector3d(currentPose.translation[0],
                                                                 currentPose.translation[1],
                                                                 currentWayPoint.z));

            // Waypoint reached, set next WayPoint as target
            if (offSetFromWayPoint.length() < WAY_POINT_REACHED_EPSILON) {
                Log.d(TAG, "Reached Waypoint[" + navigator.getCurrentWayPointIndex() + "]");
                navigator.stepToNextWayPoint();
                navigator.updateCurrentWayPointVisualizations();
                continue;
            }

            // find difference in angle to current WayPoint
            double angle = Math.toDegrees(GeometryHelper.signedAngleDifference(currentPose, currentWayPoint));
            Log.d(TAG, "angle: " + angle);
            
            // if it is above a certain threshold ...
            if (Math.abs(angle) > WAY_POINT_DIRECTION_EPSILON_FAST_TURN) {
                Log.d(TAG, "angle is above threshold");
                // rotate in the right direction
                if (angle > 0)
                    botController.rotateLeft(navigator.botData.getBotRotationSpeed());
                else
                    botController.rotateRight(navigator.botData.getBotRotationSpeed());
            } 
            else if(Math.abs(angle) > WAY_POINT_DIRECTION_EPSILON_SLOW_TURN){
                Log.d(TAG, "angle is above threshold");
                // rotate in the right direction
                if (angle > 0)
                    botController.rotateLeft((int) (navigator.botData.getBotRotationSpeed() * 0.3f));
                else
                    botController.rotateRight((int) (navigator.botData.getBotRotationSpeed() * 0.3f));
            }
            else {
                // follow direction while checking if a collision is ahead
                boolean collisionCheckResult = navigator.hasCollisionInFront();

                if (collisionCheckResult) {
                    Log.d(TAG, "Collision detected, changing behaviour to look-around");
                    navigator.setNextBehaviour(new LookAroundBehaviour(navigator));
                    break;
                } else {
                    int movementSpeed = navigator.botData.getBotMovementSpeed();
                    // use a faster forward drive speed if the angle is below the threshold
                    if(Math.abs(angle) < WAY_POINT_DIRECTION_EPSILON_FAST_FORWARD)
                        movementSpeed *= 1.5;
                    Log.d(TAG, "No Collision detected, moving forward");
                    botController.forward(movementSpeed);
                }
            }

            sleep(MS_SLEEP_TIME);
        }

        botController.stop();
        Log.d(TAG, "Exiting FOLLOW_PATH");
    }
}
