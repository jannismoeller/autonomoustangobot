package com.thkoeln.jmoeller.autonomoustangobot.activities.explorationvieweractivities;

import android.content.Intent;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.widget.Toast;

import com.thkoeln.jmoeller.autonomoustangobot.R;
import com.thkoeln.jmoeller.autonomoustangobot.explorationdata.BotData;
import com.thkoeln.jmoeller.autonomoustangobot.explorationdata.ExplorationData;
import com.thkoeln.jmoeller.autonomoustangobot.octomapjni.OcTreeJNI;
import com.thkoeln.jmoeller.autonomoustangobot.rendering.PointCloudGLRenderer;
import com.thkoeln.jmoeller.autonomoustangobot.rendering.PointCloudGLSurfaceView;

import java.nio.FloatBuffer;

import javax.vecmath.Vector3f;

/**
 * Activity that displays the visualisation of a loaded exploration
 */
public class ExplorationViewerActivity extends AppCompatActivity {

    private static final String TAG = ExplorationViewerActivity.class.getSimpleName();

    private ExplorationData explorationData;
    private PointCloudGLSurfaceView pointCloudView;
    
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_exploration_viewer);

        Intent intent = getIntent();
        Object tmpExplorationData = intent.getSerializableExtra("ExplorationData");
        if(tmpExplorationData == null){
            Toast.makeText(this, "Didn't get ExplorationData from Intent", Toast.LENGTH_LONG).show();
            Log.e(TAG, "Couldn't extract ExplorationData from IntentExtra");

            finish();
            return;
        }

        explorationData = (ExplorationData) tmpExplorationData;
        Log.d(TAG, "explorationData fetching successful!");
        Log.d(TAG, "OcTree Leafcount: " + explorationData.getOcTree().getLeafNodeCount());

        pointCloudView = (PointCloudGLSurfaceView) findViewById(R.id.pointcloud_gl_surface_view);
        if(pointCloudView != null)
            pointCloudView.setExplorationData(explorationData);
        else
            Log.e(TAG, "Couldn't find a PointCloudGLSurfaceView");

        pointCloudView.setUseTouchControlls(true);
    }

    @Override
    protected void onResume(){
        super.onResume();

        PointCloudGLRenderer renderer = pointCloudView.getRenderer();
        synchronized (explorationData) {
            BotData botData = explorationData.getMetaData().getBotProperties();
            OcTreeJNI ocTree = explorationData.getOcTree();
            
            // after loading run the navigation-update one time 
            // to find the navigable and the frontier-nodes
            ocTree.setChangedBBXToEntireTree();
            ocTree.findNavigableNodes(botData.getRadius(), botData.getHeight(), botData.getStepHeight());
            
            renderer.setBufferOctreeVertexCount(
                    ocTree.fillBufferWithNodePoints(
                            renderer.getVertexBuffferOcTree(),
                            PointCloudGLRenderer.getMaxVerticesOctree()));
        }
        renderer.setBufferHistoryVertexCount(
                fillBufferWithPosFromHistory(
                        renderer.getVertexBuffferHistory(), 
                        PointCloudGLRenderer.getMaxVerticesHistory()));
        pointCloudView.onResume();
    }

    private int fillBufferWithPosFromHistory(FloatBuffer vertexBuffferHistory, int maxVerticesHistory) {
        int floatsPerVertexLine = PointCloudGLRenderer.FLOATS_PER_VERTEX_LINE;
        
        int vertCount = 0;
        for (Vector3f pos: explorationData.getPoseHistory()) {
            if(vertCount >= maxVerticesHistory)
                break;
            
            vertexBuffferHistory.put(vertCount * floatsPerVertexLine, pos.x);
            vertexBuffferHistory.put(vertCount * floatsPerVertexLine + 1, pos.y);
            vertexBuffferHistory.put(vertCount * floatsPerVertexLine + 2, pos.z);
            vertCount++;
        }

        Log.d(TAG, "fillBufferWithPosFromHistory: vertCount: " + vertCount);
        
        return vertCount;
    }

    @Override
    protected void onPause() {
        super.onPause();
        
        pointCloudView.onPause();
    }

    @Override
    public void onBackPressed() {
        super.onBackPressed();
        finish();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        synchronized (explorationData) {
            explorationData.getOcTree().delete();
        }
    }
}