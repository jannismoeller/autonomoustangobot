package com.thkoeln.jmoeller.autonomoustangobot.activities.explorationvieweractivities;

import android.content.Intent;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.os.Environment;
import android.preference.PreferenceManager;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.ListAdapter;
import android.widget.ListView;
import android.widget.TextView;
import android.widget.Toast;

import com.thkoeln.jmoeller.autonomoustangobot.R;
import com.thkoeln.jmoeller.autonomoustangobot.activities.newexplorationactivities.ExplorationActivity;
import com.thkoeln.jmoeller.autonomoustangobot.explorationdata.ExplorationData;
import com.thkoeln.jmoeller.autonomoustangobot.explorationdata.ExplorationDataLoader;
import com.thkoeln.jmoeller.autonomoustangobot.filemanagement.CustomFilenameFilter;

import java.io.File;
import java.util.Arrays;

/**
 * Activity that displays the File Chooser and manages the loading
 */
public class LoadExplorationActivity extends AppCompatActivity {

    private TextView directoryPathTextView;
    private ListAdapter adapter;
    private ListView fileListView;
    private File directory;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_load_exploration);

        directoryPathTextView = (TextView) findViewById(R.id.directory_path_textview);
        fileListView = (ListView) findViewById(R.id.explorationfile_listview);

        // get default filepath from preferences
        SharedPreferences preferences = PreferenceManager.getDefaultSharedPreferences(this);
        final String directoryPath = preferences.getString(
                getResources().getString(R.string.pref_explorations_filepath_key), "");
        // create the directory if necessary
        directory = new File(Environment.getExternalStorageDirectory() + directoryPath);
        if(!directory.isDirectory()){
            directory = Environment.getExternalStorageDirectory();
            if(!directory.isDirectory()) {
                directory.mkdirs();
            }
        }
        
        fileListView.setOnItemClickListener(new AdapterView.OnItemClickListener() {
            @Override
            public void onItemClick(AdapterView<?> parent, View view, int position, long id) {
                String filename = String.valueOf(parent.getItemAtPosition(position));
                
                File tempFile = new File(directory + "/" + filename);
                // differentiate between a click on a directory and on a file
                if(tempFile.isDirectory()) {
                    directory = tempFile;
                    setupListView(tempFile);
                }
                else {
                    // try to load the data from the file
                    ExplorationData loadedExplorationData = ExplorationDataLoader.readFromFile(tempFile.toString(), LoadExplorationActivity.this);

                    if (loadedExplorationData != null) {
                        Toast.makeText(LoadExplorationActivity.this, filename, Toast.LENGTH_SHORT).show();
                        
                        Intent explorationViewerActivityIntent = new Intent(LoadExplorationActivity.this, ExplorationViewerActivity.class);
                        explorationViewerActivityIntent.putExtra("ExplorationData", loadedExplorationData);
                        startActivity(explorationViewerActivityIntent);
                    } else {
                        Toast.makeText(LoadExplorationActivity.this, "Couldn't read file", Toast.LENGTH_SHORT).show();
                    }
                }
            }
        });

        setupListView(directory);
    }

    private void setupListView(File directory) {
        directoryPathTextView.setText(directory.toString());
        String[] fileOrDirNames = directory.list(new CustomFilenameFilter());
        Arrays.sort(fileOrDirNames);
        adapter = new ArrayAdapter<String>(this, android.R.layout.simple_list_item_1, fileOrDirNames);
        fileListView.setAdapter(adapter);
    }

    public void navigateUpDirectory(View view) {
        if(directory.getParentFile() != null && directory.getParentFile().canRead()) {
            directory = directory.getParentFile();
            setupListView(directory);
        }
        else {
            Toast.makeText(this, "TopLevel reached", Toast.LENGTH_SHORT).show();
        }
    }
}
