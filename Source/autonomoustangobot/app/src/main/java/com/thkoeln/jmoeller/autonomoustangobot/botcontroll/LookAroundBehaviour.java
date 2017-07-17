package com.thkoeln.jmoeller.autonomoustangobot.botcontroll;

import android.util.Log;

import com.google.atap.tangoservice.TangoPoseData;
import com.thkoeln.jmoeller.autonomoustangobot.exploration.GeometryHelper;

import javax.vecmath.Vector2d;

import static android.os.SystemClock.sleep;

/**
 * Behaviour for bot to make a 360° left rotation
 */
class LookAroundBehaviour extends BotBehaviour {
    private static final String TAG = LookAroundBehaviour.class.getSimpleName();

    LookAroundBehaviour(Navigator navigator) {
        super(navigator);
    }

    @Override
    public void executeBehaviour() {
        Log.d(TAG, "Entered LOOK_AROUND");
        navigator.setReadyForNewPath(true);

        TangoPoseData poseDataBegin = navigator.getCurrentPoseCopy();
        if(poseDataBegin.statusCode != TangoPoseData.POSE_VALID){
            Log.d(TAG, "Wanted to look around but current pose is invalid");
            botController.playSound(3);
            navigator.setNextBehaviour(new FollowPathBehaviour(navigator));
            return;
        }

        Vector2d lastDirection2D = GeometryHelper.directionVector2d(poseDataBegin);
        double signedAngleTurned = 0;
        while (isRunning){
            if(!isBotMovementAllowed){
                botController.stop();
                continue;
            }
            else {
                botController.rotateRight(navigator.botData.getBotRotationSpeed());
            }

            Vector2d currentDirection2D = GeometryHelper.directionVector2d(navigator.getCurrentPoseCopy());
            signedAngleTurned += GeometryHelper.signedAngleDifference(currentDirection2D, lastDirection2D);
            Log.d(TAG, "botLookAround: signedAngleTurned: " + signedAngleTurned);
            lastDirection2D = currentDirection2D;

            if(signedAngleTurned > 2 * Math.PI)
                break;

            sleep(MS_SLEEP_TIME);
        }

        botController.stop();
        navigator.setNextBehaviour(new FollowPathBehaviour(navigator));
        Log.d(TAG, "Exiting LOOK_AROUND");
    }
}
