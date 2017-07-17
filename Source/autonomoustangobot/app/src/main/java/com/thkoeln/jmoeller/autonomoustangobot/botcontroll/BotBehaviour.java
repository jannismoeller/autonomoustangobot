package com.thkoeln.jmoeller.autonomoustangobot.botcontroll;

import android.util.Log;

/**
 * Abstract class that the behaviour implementations should derive from
 */
abstract class BotBehaviour {
    private static final String TAG = BotBehaviour.class.getSimpleName();
    
    static final long MS_SLEEP_TIME = 50;
    
    // if set to false the behaviour should stop
    static boolean isRunning = true;

    // if set to false the behaviour should pause the bots movement
    static boolean isBotMovementAllowed = true;
    static void setBotMovementAllowed(boolean botMovementAllowed) {isBotMovementAllowed = botMovementAllowed; }
    
    Navigator navigator;
    BotController botController;

    BotBehaviour(Navigator navigator) {
        this.navigator = navigator;
        this.botController = navigator.botController;
    }

    abstract public void executeBehaviour();
    
    public void stop() {
        Log.d(TAG, "stop: BotBehaviour stop()");
        isRunning = false;
    }
}
