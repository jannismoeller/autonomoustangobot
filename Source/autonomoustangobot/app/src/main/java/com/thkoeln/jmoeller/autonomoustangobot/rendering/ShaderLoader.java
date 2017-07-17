package com.thkoeln.jmoeller.autonomoustangobot.rendering;

import android.content.Context;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;

/**
 * Helper-Class to load shaders from the assets folder
 */
public class ShaderLoader {

    public static String getFileContentAsString(Context context, String assetPath){
        StringBuilder stringBuilder = new StringBuilder();
        try {
            InputStream inputStream = context.getAssets().open(assetPath);
            BufferedReader bufferedReader = new BufferedReader(new InputStreamReader(inputStream, "UTF-8"));
            String string;
            while ((string = bufferedReader.readLine()) != null){
                stringBuilder.append(string);
                // append escape character for new line to support line comments
                stringBuilder.append("\n");
            }

            bufferedReader.close();
        } catch (IOException e) {
            e.printStackTrace();
            return null;
        }

        return stringBuilder.toString();
    }
}
