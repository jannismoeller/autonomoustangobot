package com.thkoeln.jmoeller.autonomoustangobot.botcontroll;

import android.os.AsyncTask;
import android.util.Log;

import java.io.IOException;

import lejos.hardware.Audio;
import lejos.remote.ev3.RemoteRequestEV3;
import lejos.robotics.RegulatedMotor;

/**
 * Provides methods to connect to and control the EV3 Bot
 */
public class BotController {

    private final String TAG = BotController.class.getSimpleName();

    private final String CONNECT = "connect";
    private final String DISCONNECT = "disconnect";
    private int currentMotorSpeed = 0;
    
    private enum SteeringAction { ROTATE_LEFT, ROTATE_RIGHT, STOP, FORWARD, BACKWARD }
    private SteeringAction lastAction = SteeringAction.STOP;
    
    private RemoteRequestEV3 ev3;
    private RegulatedMotor left, right;
    private Audio audio;

    public void connect(String hostIP){
        new Control().execute(CONNECT, hostIP);
    }

    public void disconnect(){
        new Control().execute(DISCONNECT);
    }

    public boolean isConnected(){return ev3 != null;}

    
    private void setMotorSpeed(int speed) {
        currentMotorSpeed = speed;
        left.setSpeed(speed);
        right.setSpeed(speed);
    }

    public void rotateLeft(int speed){
        if (ev3 == null) return;
        
        if(lastAction != SteeringAction.ROTATE_LEFT
                || speed != currentMotorSpeed) {
            lastAction = SteeringAction.ROTATE_LEFT;
            Log.d(TAG, "rotateLeft() called with: speed = [" + speed + "]");
            if(speed > currentMotorSpeed) {
                left.stop(true);
                right.stop(true);
            }
            setMotorSpeed(speed);
            left.backward();
            right.forward();
        }
    }

    public void rotateRight(int speed){
        if (ev3 == null) return;
        
        if(lastAction != SteeringAction.ROTATE_RIGHT
                || speed != currentMotorSpeed) {
            lastAction = SteeringAction.ROTATE_RIGHT;
            Log.d(TAG, "rotateRight() called with: speed = [" + speed + "]");
            if(speed > currentMotorSpeed) {
                left.stop(true);
                right.stop(true);
            }
            setMotorSpeed(speed);
            left.forward();
            right.backward();
        }
    }

    public void stop(){
        if (ev3 == null) return;
        
        if(lastAction != SteeringAction.STOP) {
            lastAction = SteeringAction.STOP;
            Log.d(TAG, "stop() called");
            setMotorSpeed(0);
            
            left.stop(true);
            right.stop(true);
        }
    }

    public void forward(int speed){
        if (ev3 == null) return;
        
        if(lastAction != SteeringAction.FORWARD
                || speed != currentMotorSpeed) {
            lastAction = SteeringAction.FORWARD;
            Log.d(TAG, "forward() called with: speed = [" + speed + "]");
            if(speed > currentMotorSpeed) {
                left.stop(true);
                right.stop(true);
            }
            setMotorSpeed(speed);
            left.forward();
            right.forward();
        }
    }

    public void backward(int speed){
        if (ev3 == null) return;

        if(lastAction != SteeringAction.BACKWARD
                || speed != currentMotorSpeed) {
            lastAction = SteeringAction.BACKWARD;
            Log.d(TAG, "backward() called with: speed = [" + speed + "]");
            if(speed > currentMotorSpeed) {
                left.stop(true);
                right.stop(true);
            }
            setMotorSpeed(speed);
            left.backward();
            right.backward();
        }
    }
    
    void playSound(int i){
        audio.systemSound(i);
    }

    private class Control extends AsyncTask<String, Integer, Long> {

        @Override
        protected Long doInBackground(String... cmd) {

            if (cmd[0].equals(CONNECT)) {
                try {
                    ev3 = new RemoteRequestEV3(cmd[1]);
                    left = ev3.createRegulatedMotor("B", 'L');
                    right = ev3.createRegulatedMotor("C", 'L');
                    audio = ev3.getAudio();
                    audio.systemSound(3);
                    return 0L;
                } catch (IOException e) {
                    Log.e("EV3 Connection Error", e.getMessage());
                    return 1L;
                }
            } else if (cmd[0].equals(DISCONNECT) && ev3 != null) {
                audio.systemSound(2);
                left.stop();
                left.close();
                right.stop();
                right.close();
                ev3.disConnect();
                ev3 = null;
                return 0L;
            }

            if (ev3 == null) return 2L;

            ev3.getAudio().systemSound(1);

            Log.wtf(TAG, "new Control().execute() was called with a non supported parameter");

            return 0L;
        }
    }
}
