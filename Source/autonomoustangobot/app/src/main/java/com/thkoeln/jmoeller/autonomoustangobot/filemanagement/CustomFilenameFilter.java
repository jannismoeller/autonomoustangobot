package com.thkoeln.jmoeller.autonomoustangobot.filemanagement;

import java.io.File;
import java.io.FilenameFilter;

/**
 * FileFilter that filters directories and files that end with ".zip" 
 * Used for the simplified File-Explorer
 */
public class CustomFilenameFilter implements FilenameFilter {
    @Override
    public boolean accept(File dir, String filename) {
        File file = new File(dir + "/" + filename);
        String lowerCaseFilename = filename.toLowerCase();
        return file.canRead() && (file.isDirectory() || lowerCaseFilename.endsWith(".zip"));
    }
}