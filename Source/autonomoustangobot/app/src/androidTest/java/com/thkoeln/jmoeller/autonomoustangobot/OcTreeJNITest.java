package com.thkoeln.jmoeller.autonomoustangobot;

import android.media.MediaScannerConnection;
import android.os.Environment;
import android.support.annotation.NonNull;
import android.support.test.InstrumentationRegistry;
import android.support.test.runner.AndroidJUnit4;
import android.util.Log;

import com.thkoeln.jmoeller.autonomoustangobot.explorationdata.BotData;
import com.thkoeln.jmoeller.autonomoustangobot.octomapjni.OcTreeJNI;

import org.junit.Test;
import org.junit.runner.RunWith;

import java.io.File;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;

import javax.vecmath.Vector3f;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;

/**
 * Testing of the OcTreeJNI Methods writing and reading to and from file
 */
@RunWith(AndroidJUnit4.class)
public class OcTreeJNITest {
    private static final String DEBUG_TAG = OcTreeJNITest.class.getSimpleName();

    /**
     * Test for the Methods for writing and reading to and from file
     */
    @Test
    public void testOcTreeWriteAndRead() {
        String filepath = Environment.getExternalStorageDirectory() + "/AutonomousTangoBot/instrumentedTest";
        File dir = new File(filepath);
        dir.mkdirs();

        assertTrue(dir.isDirectory());
        assertTrue(dir.canRead());
        assertTrue(dir.canWrite());

        File file = new File(dir + "/OctreeTestWriteAndRead.ot");


        OcTreeJNI ocTreeJNI1 = new OcTreeJNI(0.6);

        assertTrue(ocTreeJNI1.write(file.toString()));

        OcTreeJNI ocTreeJNI2 = OcTreeJNI.read(file.toString());

        assertEquals(ocTreeJNI1.getLeafNodeCount(), ocTreeJNI2.getLeafNodeCount());
        assertEquals(ocTreeJNI1.getResolution(), ocTreeJNI2.getResolution(), 0);
        assertNotEquals(ocTreeJNI1.getNativeObjectPointer(), ocTreeJNI2.getNativeObjectPointer());

        ocTreeJNI1.delete();
        ocTreeJNI2.delete();
    }

    /**
     * Test for the Methods for filling Boxes and writing to file
     */
    @Test
    public void testOcTreeFilling() {
        OcTreeJNI ocTreeJNI = setUpOcTree(0.05);
        OcTreeJNI ocTreeJNISimple = setUpSimpleOcTree(0.05);
        OcTreeJNI ocTreeJNIComplex = setUpComplexOcTree(0.05);


        String filepath = Environment.getExternalStorageDirectory() + "/AutonomousTangoBot/instrumentedTest";
        File dir = new File(filepath);
        dir.mkdirs();

        assertTrue(dir.isDirectory());
        assertTrue(dir.canRead());
        assertTrue(dir.canWrite());

        File file = new File(dir + "/OctreeTestFilling.ot");
        File fileSimple = new File(dir + "/SimpleOctreeTestFilling.ot");
        File fileComplex = new File(dir + "/ComplexOctreeTestFilling.ot");

        assertTrue(ocTreeJNI.write(file.toString()));
        assertTrue(ocTreeJNISimple.write(fileSimple.toString()));
        assertTrue(ocTreeJNIComplex.write(fileComplex.toString()));

        // Register the change in the android file system
        MediaScannerConnection.scanFile(InstrumentationRegistry.getContext(), 
                new String[]{file.toString(), fileSimple.toString(), fileComplex.toString()}, null, null);

        ocTreeJNI.delete();
    }

    /**
     * Test for the Methods for collision-detection
     */
    @Test
    public void testOcTreeCollision() {
        double res = 0.05;
        long tempTime;

        OcTreeJNI ocTreeJNI = new OcTreeJNI(res);

        // Base of the "test-box"
        tempTime = System.currentTimeMillis();
        ocTreeJNI.fillBox(new float[]{-1, -1, 0}, new float[]{1, 1, 2}, false);
        ocTreeJNI.fillBox(new float[]{-1, -1, 0}, new float[]{1, 1, 0}, true);
        Log.d(DEBUG_TAG, "Time for filling test-box: " + (System.currentTimeMillis() - tempTime));

        // Stuck in ground:
        tempTime = System.currentTimeMillis();
        assertTrue(ocTreeJNI.testCollisionCylinder(new float[]{0, 0, 0}, 1, 1, 0));
        Log.d(DEBUG_TAG, "Time for collision against test-box 1: " + (System.currentTimeMillis() - tempTime));

        // Just above ground:
        tempTime = System.currentTimeMillis();
        assertFalse(ocTreeJNI.testCollisionCylinder(new float[]{0, 0, (float) res}, 1, 1, 0));
        Log.d(DEBUG_TAG, "Time for collision against test-box 1: " + (System.currentTimeMillis() - tempTime));

        // Adding pillars in the corners of the "test-box"
        ocTreeJNI.fillBox(new float[]{-1, -1, 0}, new float[]{-1, -1, 2}, true);
        ocTreeJNI.fillBox(new float[]{-1, 1, 0}, new float[]{-1, 1, 2}, true);
        ocTreeJNI.fillBox(new float[]{1, -1, 0}, new float[]{1, -1, 2}, true);
        ocTreeJNI.fillBox(new float[]{1, 1, 0}, new float[]{1, 1, 2}, true);

        // Bot should still not collide because no occupied cell is in his cuboid bounding box
        assertFalse(ocTreeJNI.testCollisionCylinder(new float[]{0, 0, (float) res}, 0.5f, 0.5f, 0));
        // Bot should still not collide because of the cylindrical shape
        assertFalse(ocTreeJNI.testCollisionCylinder(new float[]{0, 0, (float) res}, 1, 1, 0));

        // Adding pillars in the middle of the edges of the "test-box"
        ocTreeJNI.fillBox(new float[]{0, -1, 0}, new float[]{0, -1, 2}, true);
        ocTreeJNI.fillBox(new float[]{-1, 0, 0}, new float[]{-1, 0, 2}, true);
        ocTreeJNI.fillBox(new float[]{0, 1, 0}, new float[]{0, 1, 2}, true);
        ocTreeJNI.fillBox(new float[]{1, 0, 0}, new float[]{1, 0, 2}, true);

        // Bot should collide because of the added pillars that are just at the edge of his collision-range
        assertTrue(ocTreeJNI.testCollisionCylinder(new float[]{0, 0, (float) res}, 1, 1, 0));

        ocTreeJNI.delete();
    }

    /**
     * Test of the Methods for iterating the occupied nodes of a tree 
     * and writing their positions to a buffer
     */
    @Test
    public void testOcTreeOccupiedNodeIteration() {
        OcTreeJNI ocTreeJNI = setUpOcTree(0.05);

        int leafNodeCount = (int)ocTreeJNI.getLeafNodeCount();

        Log.d(DEBUG_TAG, "Octree Leaf-Nodecount: " + leafNodeCount);

        int floatsPerVertex = 4;
        int maxFloatCount = leafNodeCount * floatsPerVertex;
        int bytesPerFloat = 4;

        FloatBuffer floatBuffer1 = ByteBuffer.allocateDirect(maxFloatCount * bytesPerFloat).order(ByteOrder.nativeOrder()).asFloatBuffer();
        FloatBuffer floatBuffer2 = ByteBuffer.allocateDirect(maxFloatCount * bytesPerFloat).order(ByteOrder.nativeOrder()).asFloatBuffer();

        long tempTime;
        tempTime = System.currentTimeMillis();
        int count1 = ocTreeJNI.fillBufferWithNodePointsViaLeafIterator(floatBuffer1, maxFloatCount);
        Log.d(DEBUG_TAG, "Leaf-Iterator: " + count1 + " took: " + (System.currentTimeMillis() - tempTime) + "ms to iterate");

        tempTime = System.currentTimeMillis();
        int count2 = ocTreeJNI.fillBufferWithNodePoints(floatBuffer2, maxFloatCount);
        Log.d(DEBUG_TAG, "Self implemented iteration: " + count2 + " took: " + (System.currentTimeMillis() - tempTime) + "ms to iterate");

        assertEquals(count1, count2);
        
        ocTreeJNI.delete();
    }

    @NonNull
    private OcTreeJNI setUpOcTree(double res) {
        OcTreeJNI ocTreeJNI = new OcTreeJNI(res);

        // Base of the "test-box"
        ocTreeJNI.fillBox(new float[]{-1, -1, 0}, new float[]{1, 1, 2}, false);
        ocTreeJNI.fillBox(new float[]{-1, -1, 0}, new float[]{1, 1, 0}, true);

        // Adding pillars in the corners of the "test-box"
        ocTreeJNI.fillBox(new float[]{-1, -1, 0}, new float[]{-1, -1, 2}, true);
        ocTreeJNI.fillBox(new float[]{-1, 1, 0}, new float[]{-1, 1, 2}, true);
        ocTreeJNI.fillBox(new float[]{1, -1, 0}, new float[]{1, -1, 2}, true);
        ocTreeJNI.fillBox(new float[]{1, 1, 0}, new float[]{1, 1, 2}, true);

        // Adding pillars in the middle of the edges of the "test-box"
        ocTreeJNI.fillBox(new float[]{0, -1, 0}, new float[]{0, -1, 2}, true);
        ocTreeJNI.fillBox(new float[]{-1, 0, 0}, new float[]{-1, 0, 2}, true);
        ocTreeJNI.fillBox(new float[]{0, 1, 0}, new float[]{0, 1, 2}, true);
        ocTreeJNI.fillBox(new float[]{1, 0, 0}, new float[]{1, 0, 2}, true);

        return ocTreeJNI;
    }

    @NonNull
    private OcTreeJNI setUpSimpleOcTree(double res) {
        OcTreeJNI ocTreeJNI = new OcTreeJNI(res);

        // Base of the "test-box"
        ocTreeJNI.fillBox(new float[]{-1, -1, 0}, new float[]{1, 1, 2}, false);
        ocTreeJNI.fillBox(new float[]{-1, -1, 0}, new float[]{1, 1, 0}, true);

        return ocTreeJNI;
    }

    @NonNull
    private OcTreeJNI setUpComplexOcTree(double res) {
        OcTreeJNI ocTreeJNI = new OcTreeJNI(res);

        // Base cross, connection of the cluster-boxes
        ocTreeJNI.fillBox(new float[]{-2, -0.3f, 0}, new float[]{2, 0.3f, 2}, false);
        ocTreeJNI.fillBox(new float[]{-2, -0.3f, 0}, new float[]{2, 0.3f, 0}, true);
        
        ocTreeJNI.fillBox(new float[]{-0.3f, -2, 0}, new float[]{0.3f, 2, 2}, false);
        ocTreeJNI.fillBox(new float[]{-0.3f, -2, 0}, new float[]{0.3f, 2, 0}, true);

        
        // Adding cluster-boxes to the ends of the cross
        ocTreeJNI.fillBox(new float[]{-3.0f, -0.5f, 0.0f}, new float[]{-2.0f, 0.5f, 2.0f}, false);
        ocTreeJNI.fillBox(new float[]{-3.0f, -0.5f, 0.0f}, new float[]{-2.0f, 0.5f, 0.0f}, true);
        
        ocTreeJNI.fillBox(new float[]{2.5f, -0.5f, 0.0f}, new float[]{1.5f, 0.5f, 2.0f}, false);
        ocTreeJNI.fillBox(new float[]{2.5f, -0.5f, 0.0f}, new float[]{1.5f, 0.5f, 0.0f}, true);
        
        // 3rd is to shallow for the botHeight
        ocTreeJNI.fillBox(new float[]{-0.5f, 2.0f, 0.0f}, new float[]{0.5f, 3.0f, 0.1f}, false);
        ocTreeJNI.fillBox(new float[]{-0.5f, 2.0f, 0.0f}, new float[]{0.5f, 3.0f, 0.0f}, true);

        // 4th is missing intentionally
        
        return ocTreeJNI;
    }

    /**
     * Test for the Method for searching for free space above a node
     */
    @Test
    public void testOcTreeRayUp() {
        double res = 0.05;
        OcTreeJNI ocTreeJNI = setUpOcTree(res);
        
        assertEquals(1, ocTreeJNI.raySteppingUp(new float[]{0, 0, 0}, 1));
        assertEquals(0, ocTreeJNI.raySteppingUp(new float[]{0, 0, (float)res}, 1));
        assertEquals(2, ocTreeJNI.raySteppingUp(new float[]{0, 0, 1.5f}, 1));
        
        ocTreeJNI.delete();
    }

    /**
     * Test of the Method for finding all navigable Nodes
     */
    @Test
    public void testFindNavigatables() {
        double res = 0.05;
        OcTreeJNI ocTreeJNI = setUpSimpleOcTree(res);
//        OcTreeJNI ocTreeJNI = setUpOcTree(res);
//        OcTreeJNI ocTreeJNI = setUpComplexOcTree(res);

        long tempTime = System.currentTimeMillis();
        ocTreeJNI.findNavigableNodes(1, 1, 1);
        Log.d(DEBUG_TAG, "findNavigableNodes took: " + (System.currentTimeMillis() - tempTime) + "ms to complete");
        
        Log.d("OCTREE", "leafnodecount: " + ocTreeJNI.getLeafNodeCount());

        ocTreeJNI.delete();
    }

    /**
     * Test of the Method for clustering the nodes that lay on the frontier of unknown space
     */
    @Test
    public void testClusterFrontierNodes(){
        double res = 0.05;
        OcTreeJNI ocTreeJNI = setUpComplexOcTree(res);

        // creating test bot-data
        BotData bot = new BotData(0.18f, 0.22f, 0.05f, new Vector3f(0,0,0), 100, 100);
        
        // perform the search and creation of the adjacency graph
        ocTreeJNI.findNavigableNodes(bot.getRadius(), bot.getHeight(), bot.getStepHeight());

        // allocate a buffer for the returned path
        int FLOATS_PER_WAYPOINT = 3;
        int BYTES_PER_FLOAT = 4;
        float maxPathLength = 10.0f;
        int maxWayPointCount = Math.round(maxPathLength / (float)res);
        FloatBuffer wayPointBuffer = ByteBuffer.allocateDirect(maxWayPointCount * FLOATS_PER_WAYPOINT * BYTES_PER_FLOAT).order(ByteOrder.nativeOrder()).asFloatBuffer();      

        // execute the method that is beeing tested
        int wayPointsReturned = ocTreeJNI.findNextPath(wayPointBuffer, maxWayPointCount, new float[]{0,0,0}, new float[]{0,1,0}, 0.5f, 0.5f);
        
        // checking the results
        assertTrue(wayPointBuffer.capacity() == maxWayPointCount * FLOATS_PER_WAYPOINT);
        
        String debugString = "";
        for (int i = 0; i < wayPointsReturned; i++) {
            debugString += wayPointBuffer.get(i * 3) + ", ";
            debugString += wayPointBuffer.get(i * 3 + 1) + ", ";
            debugString += wayPointBuffer.get(i * 3 + 2);
            debugString += "\n";
        }
        Log.d(DEBUG_TAG, "Result of Pathfinding: " + wayPointsReturned + " waypoints extracted:\n" + debugString);
        ocTreeJNI.delete();
    }
}
