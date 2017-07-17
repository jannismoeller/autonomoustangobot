package com.thkoeln.jmoeller.autonomoustangobot.activities.newexplorationactivities;

import android.content.Intent;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.View;
import android.widget.EditText;

import com.thkoeln.jmoeller.autonomoustangobot.R;
import com.thkoeln.jmoeller.autonomoustangobot.activities.settingsactivities.SettingsActivity;
import com.thkoeln.jmoeller.autonomoustangobot.explorationdata.BotData;
import com.thkoeln.jmoeller.autonomoustangobot.explorationdata.ExplorationData;
import com.thkoeln.jmoeller.autonomoustangobot.explorationdata.ExplorationMetaData;
import com.thkoeln.jmoeller.autonomoustangobot.octomapjni.OcTreeJNI;

import java.util.Calendar;

import javax.vecmath.Vector3f;

/**
 * Activity that displays the explanation and starts a new exploration 
 * by initializing the ExplorationData with the apps settings
 */
public class NewExplorationActivity extends AppCompatActivity {

    private static final String DEBUG_TAG = NewExplorationActivity.class.getSimpleName();
    
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_new_exploration);
    }

    public void onClickStartNewExploration(View view) {

        PreferenceManager.setDefaultValues(this, R.xml.pref_octomap, false);
        PreferenceManager.setDefaultValues(this, R.xml.pref_botproperties, false);
        SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(this);
        
        double resolution = Double.parseDouble(prefs.getString(getResources().getString(R.string.pref_octomap_resolution_key), "0.1"));
        double maxrange = Double.parseDouble(prefs.getString(getResources().getString(R.string.pref_octomap_scanrange_key), "3"));
        // Tests ran by observing the PointCloud showed that surfaces closer than ~40cm 
        // didn't give reliable measurable infrared reflections
        double minrange = 0.4f; 
        
        OcTreeJNI ocTree = new OcTreeJNI(resolution);
        Log.d(DEBUG_TAG, "OcTree Pointer: " + ocTree.getNativeObjectPointer());
        
        
        BotData botData = new BotData(
                Float.parseFloat(prefs.getString(getResources().getString(R.string.pref_botproperties_radius_key), "0.2")),
                Float.parseFloat(prefs.getString(getResources().getString(R.string.pref_botproperties_height_key), "0.25")),
                Float.parseFloat(prefs.getString(getResources().getString(R.string.pref_botproperties_stepheight_key), "0.1")),
                new Vector3f(0, 0, Float.parseFloat(prefs.getString(getResources().getString(R.string.pref_botproperties_camOffset_key), "0.15"))),
                Integer.parseInt(prefs.getString(getResources().getString(R.string.pref_botproperties_rotationspeed_key), "50")),
                Integer.parseInt(prefs.getString(getResources().getString(R.string.pref_botproperties_movementspeed_key), "100"))
        );
        
        ExplorationMetaData metaData = new ExplorationMetaData(Calendar.getInstance(), maxrange, minrange, botData);
        ExplorationData newExplorationData = new ExplorationData(ocTree, null, metaData);

        Intent explorationActivityIntent = new Intent(this, ExplorationActivity.class);
        explorationActivityIntent.putExtra("ExplorationData", newExplorationData);
        startActivity(explorationActivityIntent);
    }

    public void onClickSettings(View view) {
        Intent settingsActivityIntent = new Intent(this, SettingsActivity.class);
        startActivity(settingsActivityIntent);
    }
}