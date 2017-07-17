package com.thkoeln.jmoeller.autonomoustangobot.explorationdata;

import java.io.Serializable;
import java.util.Calendar;

/**
 * Class that holds metadata about the Exploration
 * objects can be converted to and from json
 */
public class ExplorationMetaData implements Serializable {

    private Calendar timestamp;
    public Calendar getTimestamp() {return timestamp;}

    private double maxScanRange;
    public double getMaxScanRange() {return maxScanRange;}

    private double scanFilterMinRange;
    public double getScanFilterMinRange() {return scanFilterMinRange;}

    private BotData botProperties;
    public BotData getBotProperties() { return botProperties; }
    
    public ExplorationMetaData(Calendar timestamp, double maxScanRange, double scanFilterMinRange, BotData botProperties) {
        this.timestamp = timestamp;
        this.maxScanRange = maxScanRange;
        this.scanFilterMinRange = scanFilterMinRange;
        this.botProperties = botProperties;
    }
}