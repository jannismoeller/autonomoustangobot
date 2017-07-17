package com.thkoeln.jmoeller.autonomoustangobot.explorationdata;

import java.io.Serializable;

import javax.vecmath.Vector3f;

/**
 * Data holder for various properties of the bot
 */
public class BotData implements Serializable{

    private final float radius;
    public float getRadius() { return radius; }

    private final float height;
    public float getHeight() { return height; }

    private final float stepHeight;
    public float getStepHeight() { return stepHeight; }
    
    private final Vector3f cameraOffset;
    public Vector3f getCameraOffset() { return cameraOffset; }

    private final int botRotationSpeed;
    public int getBotRotationSpeed() { return botRotationSpeed; }

    private final int botMovementSpeed;
    public int getBotMovementSpeed() { return botMovementSpeed; }
    
    public BotData(float radius, float height, float stepHeight, Vector3f cameraOffset, int botRotationSpeed, int botMovementSpeed) {
        this.radius = radius;
        this.height = height;
        this.stepHeight = stepHeight;
        this.cameraOffset = cameraOffset;
        this.botRotationSpeed = botRotationSpeed;
        this.botMovementSpeed = botMovementSpeed;
    }
}
