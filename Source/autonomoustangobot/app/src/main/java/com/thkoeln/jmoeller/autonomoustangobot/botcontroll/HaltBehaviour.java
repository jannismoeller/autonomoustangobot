package com.thkoeln.jmoeller.autonomoustangobot.botcontroll;

import android.util.Log;

import static android.os.SystemClock.sleep;

/**
 * A simple behaviour that pauses the bots movement for a given tim
 */
class HaltBehaviour extends BotBehaviour {
    private static final String TAG = HaltBehaviour.class.getSimpleName();

    private final long msHaltTime;
    
    HaltBehaviour(Navigator navigator, long msHaltTime) {
        super(navigator);
        this.msHaltTime = msHaltTime;
    }

    @Override
    public void executeBehaviour() {
        Log.d(TAG, "Entered HaltBehaviour");
        
        botController.stop();
        long msAtStart = System.currentTimeMillis();
        while (isRunning &&
               msAtStart + msHaltTime > System.currentTimeMillis()){
            sleep(MS_SLEEP_TIME);
        }

        Log.d(TAG, "Exited HaltBehaviour");
    }
}
