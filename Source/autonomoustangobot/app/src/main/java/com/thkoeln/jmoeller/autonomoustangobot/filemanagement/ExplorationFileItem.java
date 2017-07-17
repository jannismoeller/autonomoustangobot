package com.thkoeln.jmoeller.autonomoustangobot.filemanagement;

/**
 * Class that holds the data of an ExplorationFile that is displayed in the File-Listview
 * @deprecated Currently a simple String ArrayAdapter is used.
 * This class will eventually be used once more info should be displayed
 */
@Deprecated
public class ExplorationFileItem {

    private String fileName;
    public String getFileName(){return fileName;}
    public void setFileName(String fileName){this.fileName = fileName;}

    public ExplorationFileItem(String fileName){
        this.fileName = fileName;
    }


}
