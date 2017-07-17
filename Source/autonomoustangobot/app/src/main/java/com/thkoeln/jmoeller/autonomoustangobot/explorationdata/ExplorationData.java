package com.thkoeln.jmoeller.autonomoustangobot.explorationdata;

import com.google.atap.tangoservice.TangoPoseData;
import com.thkoeln.jmoeller.autonomoustangobot.octomapjni.OcTreeJNI;

import java.io.Serializable;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.LinkedList;

import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

/**
 * Class that holds all the data for an exploration
 * - OcTree
 * - Area Description File
 * - poseHistory
 * - Metadata
 */
public class ExplorationData implements Serializable{

    private OcTreeJNI ocTree;
    public OcTreeJNI getOcTree() {return ocTree;}

    public String adfUUID;
    
    private LinkedList<Vector3f> poseHistory;
    public LinkedList<Vector3f> getPoseHistory(){ return (LinkedList<Vector3f>)poseHistory.clone(); }
    public void addPoseToHistory(TangoPoseData pose){ poseHistory.add(new Vector3f(pose.getTranslationAsFloats())); }

    private ExplorationMetaData metaData;
    public ExplorationMetaData getMetaData() {return metaData;}
    
    public ExplorationData(OcTreeJNI ocTree, String adfUUID, ExplorationMetaData metaData){
        this(ocTree, adfUUID, new LinkedList<Vector3f>(), metaData);
    }
    
    public ExplorationData(OcTreeJNI ocTree, String adfUUID, LinkedList<Vector3f> poseHistory, ExplorationMetaData metaData){
        this.ocTree = ocTree;
        this.adfUUID = adfUUID;
        this.poseHistory = poseHistory;
        this.metaData = metaData;
    }

    private static final DateFormat df = new SimpleDateFormat("yyyyMMdd_HHmmss");
    public String getDateAsFormattedString(){
        if(metaData != null && metaData.getTimestamp() != null)
            return df.format(metaData.getTimestamp().getTime());
        else
            return "MissingTimestamp";
    }
    
}
