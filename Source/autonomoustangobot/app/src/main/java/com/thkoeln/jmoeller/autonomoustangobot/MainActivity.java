package com.thkoeln.jmoeller.autonomoustangobot;

import android.content.Intent;
import android.content.pm.PackageInfo;
import android.content.pm.PackageManager;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.support.annotation.NonNull;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.support.v7.app.AppCompatActivity;
import android.support.v7.widget.Toolbar;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Toast;

import com.thkoeln.jmoeller.autonomoustangobot.activities.explorationvieweractivities.LoadExplorationActivity;
import com.thkoeln.jmoeller.autonomoustangobot.activities.newexplorationactivities.NewExplorationActivity;
import com.thkoeln.jmoeller.autonomoustangobot.activities.settingsactivities.SettingsActivity;

import java.util.ArrayList;
import java.util.List;

/**
 * Activity that shows the main menu, handles the preference initialisation 
 * and checks for permissions
 */
public class MainActivity extends AppCompatActivity {

    private static final int PERMISSIONS_REQUEST_CODE = 12345;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        Toolbar toolbar = (Toolbar) findViewById(R.id.toolbar);
        setSupportActionBar(toolbar);

        // Set all the preferences from this apps settings to the default values 
        // if they haven't been initialized yet
        setDefaultPreferencesNoOverride();
        
        // Check if all the permission needed to run this app are granted and request them if not
        checkPermissionsIfNeccessary();
    }

    private void setDefaultPreferencesNoOverride() {
        PreferenceManager.setDefaultValues(this, R.xml.pref_general, false);
        PreferenceManager.setDefaultValues(this, R.xml.pref_tango, false);
        PreferenceManager.setDefaultValues(this, R.xml.pref_octomap, false);
        PreferenceManager.setDefaultValues(this, R.xml.pref_botproperties, false);
    }

    /**
     * @return true if permissions where there
     */
    private boolean checkPermissionsIfNeccessary() {
        try {
            PackageInfo info = getPackageManager().getPackageInfo(this.getPackageName(), PackageManager.GET_PERMISSIONS);
            if (info.requestedPermissions != null) {
                List<String> permissionsNotGrantedYet = new ArrayList<>(info.requestedPermissions.length);
                for (String p : info.requestedPermissions) {
                    if (ContextCompat.checkSelfPermission(this, p) != PackageManager.PERMISSION_GRANTED) {
                        permissionsNotGrantedYet.add(p);
                    }
                }
                if(permissionsNotGrantedYet.size() > 0){
                    ActivityCompat.requestPermissions(this, permissionsNotGrantedYet.toArray(new String[permissionsNotGrantedYet.size()]),
                            PERMISSIONS_REQUEST_CODE);
                    return false;
                }
            }
        } catch (PackageManager.NameNotFoundException e) {
            e.printStackTrace();
        }

        return true;
    }

    @Override
    public void onRequestPermissionsResult(int requestCode,
                                           @NonNull String permissions[], 
                                           @NonNull int[] grantResults) {
        
        if(requestCode == PERMISSIONS_REQUEST_CODE) {
            boolean hasAllPermissions = true;
            // If request is cancelled, the result arrays are empty.
            if (grantResults.length == 0)
                hasAllPermissions = false;
            for (int result : grantResults) {
                if (result != PackageManager.PERMISSION_GRANTED)
                    hasAllPermissions = false;
            }

            if(!hasAllPermissions){
                finish();
            }
        }
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_main, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();

        switch (id) {
            case R.id.action_settings:
                Intent settingsActivityIntent = new Intent(this, SettingsActivity.class);
                startActivity(settingsActivityIntent);
                return true;
            case R.id.action_about:
                Toast.makeText(this, "Created by Jannis Möller", Toast.LENGTH_LONG).show();
                return true;
        }
        return super.onOptionsItemSelected(item);
    }

    public void onClickNewExploration(View view) {
        Intent newExplorationActivityIntent = new Intent(this, NewExplorationActivity.class);
        startActivity(newExplorationActivityIntent);
    }

    public void onClickLoadExploration(View view) {
        Intent loadExplorationActivityIntent = new Intent(this, LoadExplorationActivity.class);
        startActivity(loadExplorationActivityIntent);
    }
}
