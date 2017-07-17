package com.thkoeln.jmoeller.autonomoustangobot.octomapjni;

import java.io.Serializable;
import java.nio.FloatBuffer;

/**
 * Java Wrapper-Class for the C++-Class "NavigationOcTree"
 */
public class OcTreeJNI implements Serializable {
    
    static final String DEBUG_TAG = OcTreeJNI.class.getSimpleName();
    
    // Load the library in which the native methods declared in this class are defined
    static { System.loadLibrary("octomapjni"); }

    /**
     * Field that holds the Pointer to the C++ OcTree-Object belonging to this Java-Object
     */
    protected final long nativeObjectPointer;

    public long getNativeObjectPointer(){return nativeObjectPointer;}

    /**
     * @return Traverses the OcTree to count the Leaf-Nodes
     */
    public native long getLeafNodeCount();

    /**
     * @return The size of the smallest possible Node
     */
    public native double getResolution();

    /**
     * Standard Constructor
     * @param resolution Smallest possible node-size
     */
    public OcTreeJNI(double resolution) {
        this.nativeObjectPointer = OcTreeJNI.instantiate(resolution);
    }

    /**
     * Private Constructor called e.g. when an OcTree is read from a file
     * @param nativeObjectPointer Pointer to the C++ OcTree-Object
     */
    private OcTreeJNI(long nativeObjectPointer) {
        this.nativeObjectPointer = nativeObjectPointer;
    }

    /**
     * Method to delete the C++ OcTree-Object from the heap
     */
    public native void delete();

    /**
     * Method used internally by the standard-constructor
     * @param resolution Smallest possible node-size
     * @return Pointer to the C++ OcTree-Object
     */
    private static native long instantiate(double resolution);

    /**
     * Writes OcTree to a binary file
     * @param filename path and file the OcTree should be written to
     * @return true if successful, false if not
     */
    public native boolean write(String filename);

    /**
     * Native Method to instantiate an OcTree from file
     * @param filename path and file the OcTree is saved in
     * @return Pointer to the C++ OcTree-Object
     */
    private static native long readNative(String filename);

    /**
     * Static wrapper for the native method to read an OcTree from file
     * @param filename path and file the OcTree is saved in
     * @return Java-Object of this class
     */
    public static OcTreeJNI read(String filename) {
        return new OcTreeJNI(readNative(filename));
    }

    /**
     * Insert the PointCloudData received from a Google Tango Callback into the OcTree
     */
    public native void insertTangoPointCloud(FloatBuffer pointsBuffer, int numPoints, float[] transformationMatrix, float[] sensorOrigin, double maxrange, double minrange);

    /**
     * Tests if the OcTree collides with a given Cylinder
     * @param maxDepth maximum query depth, -1 will use the full depth
     * @return true if colliding, false if in free or unknown space
     */
    public native boolean testCollisionCylinder(float[] position, float radius, float height, int maxDepth);

    /**
     * Method to fill a box by setting the values directly
     * @param from one corner of the box
     * @param to opposite corner of the box
     * @param occupied set nodes to
     */
    public native void fillBox(float[] from, float[] to, boolean occupied);

    /**
     * Method to fill a box by updating the values as if it was a measurement
     * @param from one corner of the box
     * @param to opposite corner of the box
     * @param occupied set nodes to
     */
    public native void fillBoxAsMeasuremants(float[] from, float[] to, boolean occupied);

    /**
     * Fill the buffer with the 3D-Positions of the occupied nodes by 
     * iterating the tree manually for better performance
     */
    public native int fillBufferWithNodePoints(FloatBuffer buffer, int bufferMaxVertsCount);

    /**
     * Fill the buffer with the 3D-Positions of the occupied nodes by 
     * iterating the tree via the leaf-iterator that OctoMap provides
     */
    public native int fillBufferWithNodePointsViaLeafIterator(FloatBuffer buffer, int bufferMaxVertsCount);
    
    /**
     * @return 0 for all cells free, 1 for >=1 cell occupied, 2 for >=1 cell unknown space
     */
    public native int raySteppingUp(float[] position, float height);

    /**
     * Find all navigable Nodes of the Octree and build a Adjacency Graph from them
     * operates only on the changedBBX
     */
    public native void findNavigableNodes(float radius, float height, float maxStepHeight);

    /**
     * Cluster all Frontier-Nodes,
     * find paths to them and determine the best for the one next to take
     */
    public native int findNextPath(FloatBuffer wayPointBuffer, int maxWayPoints, float[] position, float[] direction, float botNodeSearchRange, float clusterMaxRange);

    /**
     * set the changedBBX so that it contains the entire Tree
     * use this after loading a tree from file if you wish for the navigation-update to have any effect
     */
    public native void setChangedBBXToEntireTree();
}