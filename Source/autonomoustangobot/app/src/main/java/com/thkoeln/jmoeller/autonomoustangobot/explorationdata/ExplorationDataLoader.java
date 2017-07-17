package com.thkoeln.jmoeller.autonomoustangobot.explorationdata;

import android.content.Context;
import android.content.SharedPreferences;
import android.media.MediaScannerConnection;
import android.os.Environment;
import android.preference.PreferenceManager;
import android.util.Log;

import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;
import com.thkoeln.jmoeller.autonomoustangobot.R;
import com.thkoeln.jmoeller.autonomoustangobot.octomapjni.OcTreeJNI;

import org.zeroturnaround.zip.ZipUtil;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Reader;
import java.io.Writer;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import javax.vecmath.Vector3f;

/**
 * Manages loading and writing of ExplorationData from and to the filesystem
 */
public class ExplorationDataLoader {

    private static final String TAG = ExplorationDataLoader.class.getSimpleName();

    private static final String FILENAME_OCTREE = "Octree.ot";
    private static final String FILENAME_ADF = "AreaDescriptionFile.adf";
    private static final String FILENAME_POSE_HISTORY = "PoseHistory.json";
    private static final String FILENAME_METADATA = "MetaData.json";

    /**
     * Warning: Do not call if localisation is lost!
     * @param filepath filename plus path
     */
    public static boolean writeToFile(ExplorationData explorationData, String filepath, Context context) {

        // get the default file directory and try to create it if it doesn't already exist
        SharedPreferences preferences = PreferenceManager.getDefaultSharedPreferences(context);
        String workingDirectoryPath = preferences.getString(
                context.getResources().getString(R.string.pref_working_directory_key), "");
        File workingDirectory = new File(Environment.getExternalStorageDirectory() + workingDirectoryPath);
        Log.d(TAG, "workingDirectory: " + workingDirectory);

        if(!workingDirectory.canWrite() && workingDirectory.isDirectory()) {
            Log.d(TAG, "Failed to write, no rights for directory: " + workingDirectory);
            return false;
        }
        else if(!workingDirectory.isDirectory()){
            Log.d(TAG, "Created directory: " + workingDirectory);
            workingDirectory.mkdirs();
        }

        File octreeFile = new File(workingDirectory + File.separator + FILENAME_OCTREE);
        File adfFile = new File(workingDirectory + File.separator + FILENAME_ADF);
        File poseHistoryFile = new File(workingDirectory + File.separator + FILENAME_POSE_HISTORY);
        File metaDataFile = new File(workingDirectory + File.separator + FILENAME_METADATA);

        // get the lock on the explorationData Object to make sure no one manipulates it 
        // while this class is trying to write it to file
        synchronized (explorationData) {
            if (!explorationData.getOcTree().write(octreeFile.toString())) {
                Log.d(TAG, "Failed to write " + FILENAME_OCTREE + " to file");
                return false;
            }
            
            try (Writer writer = new FileWriter(poseHistoryFile.toString())) {
                Gson gson = new Gson();
                gson.toJson(explorationData.getPoseHistory(), writer);
            } catch (IOException e) {
                Log.e(TAG, "Failed to write " + FILENAME_POSE_HISTORY + " to file");
                e.printStackTrace();
            }
            
            try (Writer writer = new FileWriter(metaDataFile.toString())) {
                Gson gson = new Gson();
                gson.toJson(explorationData.getMetaData(), writer);
            } catch (IOException e) {
                Log.e(TAG, "Failed to write " + FILENAME_METADATA + " to file");
                e.printStackTrace();
            }
        }

        // Write the content to file
        try {
            poseHistoryFile.createNewFile();
            adfFile.createNewFile();
        } catch (IOException e) {
            e.printStackTrace();
        }

        File[] filesToPack = {octreeFile, adfFile, poseHistoryFile, metaDataFile};

        // Pack all files to a zip-archive
        ZipUtil.packEntries(filesToPack, new File(filepath));

        MediaScannerConnection.scanFile(context, new String[]{filepath}, null, null);

        for (File file : filesToPack) {
            file.delete();
        }

        return true;
    }

    public static ExplorationData readFromFile(String filename, Context context) {

        PreferenceManager.setDefaultValues(context, R.xml.pref_general, false);
        SharedPreferences preferences = PreferenceManager.getDefaultSharedPreferences(context);
        String workingDirectoryPath = preferences.getString(
                context.getResources().getString(R.string.pref_working_directory_key), "");
        File workingDirectory = new File(Environment.getExternalStorageDirectory() + workingDirectoryPath);

        // Pack the zip-archive
        ZipUtil.unpack(new File(filename), workingDirectory);

        File[] extractedFiles = workingDirectory.listFiles();

        OcTreeJNI ocTree = null;
        ExplorationMetaData metaData = null;
        LinkedList<Vector3f> poseHistory = null;

        for (File extractedFile : extractedFiles) {
            switch (extractedFile.getName()){
                case FILENAME_OCTREE:
                    ocTree = OcTreeJNI.read(extractedFile.toString());
                    break;
                case FILENAME_ADF:
                    // Import of ADF currently disabled
                    break;
                case FILENAME_POSE_HISTORY:
                    try (Reader reader = new FileReader(extractedFile.toString())) {
                        Gson gson = new Gson();
                        poseHistory = new LinkedList<Vector3f>(
                                (ArrayList<Vector3f>) gson.fromJson(
                                        reader, new TypeToken<List<Vector3f>>(){}.getType()
                                ));
                    } catch (IOException e) {
                        Log.e(TAG, "Failed to write json to file");
                        e.printStackTrace();
                    }
                    break;
                case FILENAME_METADATA:
                    try (Reader reader = new FileReader(extractedFile.toString())) {
                        Gson gson = new Gson();
                        metaData = gson.fromJson(reader, ExplorationMetaData.class);
                    } catch (IOException e) {
                        Log.e(TAG, "Failed to write json to file");
                        e.printStackTrace();
                    }
                    break;
            }
        }

        if (ocTree == null ||
//            adfUUID == null ||
            poseHistory == null ||
            metaData == null) {
            Log.e(TAG, "At least one file couldn't be found or processed");
            return null;
        }

        return new ExplorationData(ocTree, null, poseHistory, metaData);
    }

    private static void createIfNotDirectory(String directory) {
        File file = new File(directory);
        if(!file.isDirectory())
            file.mkdirs();
    }
}
