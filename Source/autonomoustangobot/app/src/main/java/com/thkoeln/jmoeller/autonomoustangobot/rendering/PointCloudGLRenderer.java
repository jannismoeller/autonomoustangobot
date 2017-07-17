package com.thkoeln.jmoeller.autonomoustangobot.rendering;

import android.content.Context;
import android.opengl.GLES20;
import android.opengl.GLSurfaceView;
import android.opengl.Matrix;
import android.util.Log;

import com.google.atap.tangoservice.TangoPoseData;
import com.thkoeln.jmoeller.autonomoustangobot.explorationdata.ExplorationData;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;
import javax.vecmath.Matrix4d;
import javax.vecmath.Matrix4f;
import javax.vecmath.Vector2f;
import javax.vecmath.Vector3f;

/**
 * This is the class that manages all the rendering done to visualize the OcTree and the 
 * additional information
 */
public class PointCloudGLRenderer implements GLSurfaceView.Renderer {

    private final String TAG = PointCloudGLRenderer.class.getSimpleName();

    private int programHandlePoints;
    private final String vertexShaderPointsCode;
    private final String fragmentShaderPointsCode;
    
    private int programHandleLines;
    private final String vertexShaderLinesCode;
    private final String fragmentShaderLinesCode;

    public boolean useCameraPose;

    public FloatBuffer getVertexBuffferOcTree() {return vertexBuffferOcTree;}
    private FloatBuffer vertexBuffferOcTree;
    public static int getMaxVerticesOctree() {return MAX_VERTICES_OCTREE;}
    private static final int MAX_VERTICES_OCTREE = 500000;
    private int bufferOctreeVertexCount = 0;
    public void setBufferOctreeVertexCount(int bufferOctreeVertexCount) {this.bufferOctreeVertexCount = bufferOctreeVertexCount;}


    public FloatBuffer getVertexBuffferPath() {return vertexBuffferPath;}
    private FloatBuffer vertexBuffferPath;
    public static int getMaxVerticesPath() {return MAX_VERTICES_PATH;}
    public static final int MAX_VERTICES_PATH = 5000;
    private int bufferPathVertexCount = 0;
    public void setBufferPathVertexCount(int bufferPathVertexCount) {this.bufferPathVertexCount = bufferPathVertexCount;}
    private int bufferPathCurrentIndex = 0;
    public void setBufferPathCurrentIndex(int index) {this.bufferPathCurrentIndex = index;}
    

    public FloatBuffer getVertexBuffferHistory() {return vertexBuffferHistory;}
    private FloatBuffer vertexBuffferHistory;
    public static int getMaxVerticesHistory() {return MAX_VERTICES_HISTORY;}
    private static final int MAX_VERTICES_HISTORY = 100000;
    private int bufferHistoryVertexCount = 0;
    public void setBufferHistoryVertexCount(int bufferHistoryVertexCount) {this.bufferHistoryVertexCount = bufferHistoryVertexCount;}
    public boolean addPosToHistoryBuffer(TangoPoseData pose){
        if(bufferHistoryVertexCount < MAX_VERTICES_HISTORY){
            float[] translation = pose.getTranslationAsFloats();
            
            vertexBuffferHistory.put(bufferHistoryVertexCount * FLOATS_PER_VERTEX_LINE,     translation[0]);
            vertexBuffferHistory.put(bufferHistoryVertexCount * FLOATS_PER_VERTEX_LINE + 1, translation[1]);
            vertexBuffferHistory.put(bufferHistoryVertexCount * FLOATS_PER_VERTEX_LINE + 2, translation[2]);
            
            bufferHistoryVertexCount++;
            return true;
        }
        else 
            return false;
    }

    public static final int FLOATS_PER_VERTEX_LINE = 3;
    private static final int FLOATS_PER_VERTEX_POINTS = 4;
    private static final int BYTES_PER_FLOAT = 4;
    private ExplorationData explorationData;
    private int[] texture_map;

    void setExplorationData(ExplorationData explorationData) {this.explorationData = explorationData;}

    /**
     * Field of view in degrees for Y-Direction
     */
    private static final float FOVY = 80;
    private static final float NEAR = 0.1f;
    private static final float FAR = 30.0f;

    // mvpMatrix is an abbreviation for "Model View Projection Matrix"
    // the model (octree) has no transformation applied so the modelMatrix isn't needed
    private final float[] projectionMatrix = new float[16];
    private float[] viewMatrix = new float[16];
    private final float[] tempMatrix = new float[16];
    private final float[] mTangoToOGLMatrix = {
            1,0,0,0,
            0,0,-1,0,
            0,1,0,0,
            0,0,0,1
    };
    private final float[] mvpMatrix = new float[16];

    private int textureUniformHandle;
    private int mvpMatrixHandlePoints;
    private int ocTreeResolutionHandle;
    private int positionHandlePoints;
    
    private int mvpMatrixHandleLines;
    private int lineColorHandle;
    private int positionHandleLines;
    
    private static final float[] PATH_LINE_COLOR = new float[]{0, 0, 0.8f, 1};
    private static final float[] HISTORY_LINE_COLOR = new float[]{0.9f, 0, 0, 1};
    
    PointCloudGLRenderer(Context context) {
        vertexShaderPointsCode = ShaderLoader.getFileContentAsString(context, "vertex_shader_points.glsl");
        fragmentShaderPointsCode = ShaderLoader.getFileContentAsString(context, "fragment_shader_points.glsl");
        
        vertexShaderLinesCode = ShaderLoader.getFileContentAsString(context, "vs_linestrip.glsl");
        fragmentShaderLinesCode = ShaderLoader.getFileContentAsString(context, "fragment_shader_points.glsl");
        

        if (vertexShaderPointsCode == null || fragmentShaderPointsCode == null ||
                vertexShaderLinesCode  == null || fragmentShaderLinesCode == null) {
            Log.e(TAG, "Error: Could not load shaders from aasets folder!");
        } else {
            Log.d(TAG, "Loading Shader succesfull, Vertex-Shader-Points:\n" + vertexShaderPointsCode);
            Log.d(TAG, "Loading Shader succesfull, Fragment-Shader-Points:\n" + fragmentShaderPointsCode);
            Log.d(TAG, "Loading Shader succesfull, Fragment-Shader-Lines:\n" + vertexShaderLinesCode);
            Log.d(TAG, "Loading Shader succesfull, Fragment-Shader-Lines:\n" + fragmentShaderLinesCode);
        }

        vertexBuffferOcTree = ByteBuffer.allocateDirect(MAX_VERTICES_OCTREE * FLOATS_PER_VERTEX_POINTS * BYTES_PER_FLOAT).order(ByteOrder.nativeOrder()).asFloatBuffer();
        vertexBuffferPath = ByteBuffer.allocateDirect(MAX_VERTICES_PATH * FLOATS_PER_VERTEX_LINE * BYTES_PER_FLOAT).order(ByteOrder.nativeOrder()).asFloatBuffer();
        vertexBuffferHistory = ByteBuffer.allocateDirect(MAX_VERTICES_HISTORY * FLOATS_PER_VERTEX_LINE * BYTES_PER_FLOAT).order(ByteOrder.nativeOrder()).asFloatBuffer();        
    }

    private void throwGLErrorIfNecessary(String glAction) {
        int errorCode = GLES20.glGetError();
        if (errorCode != GLES20.GL_NO_ERROR) {
            throw new RuntimeException(glAction + " -> " + errorCode);
        }
    }

    /**
     * compile the shader and return the handle
     */
    private int createShader(int shaderType, String shaderSourceCode) {
        int shaderHandle = GLES20.glCreateShader(shaderType);
        GLES20.glShaderSource(shaderHandle, shaderSourceCode);
        throwGLErrorIfNecessary("glShaderSource");
        GLES20.glCompileShader(shaderHandle);
        throwGLErrorIfNecessary("glCompileShader");
        return shaderHandle;
    }

    @Override
    public void onSurfaceCreated(GL10 gl, EGLConfig config) {
        // Set ClearColor
        GLES20.glClearColor(0.9f, 0.9f, 0.9f, 1.0f);
        GLES20.glEnable(GLES20.GL_DEPTH_TEST);

        setupProgramPoints();
        
        setupProgramLines();

        Matrix.setRotateM(mRotation, 0, 90, 0, 0, 1);
    }

    /**
     * Setup the shader Program for the line rendering of the posHistory and the wayPoints
     */
    private void setupProgramLines() {
        int vertexShaderHandle = createShader(GLES20.GL_VERTEX_SHADER, vertexShaderLinesCode);
        int fragmentShaderHandle = createShader(GLES20.GL_FRAGMENT_SHADER, fragmentShaderLinesCode);

        programHandleLines = GLES20.glCreateProgram();
        throwGLErrorIfNecessary("glCreateProgram");
        GLES20.glAttachShader(programHandleLines, vertexShaderHandle);
        throwGLErrorIfNecessary("glAttachShader(vertexShaderHandle)");
        GLES20.glAttachShader(programHandleLines, fragmentShaderHandle);
        throwGLErrorIfNecessary("glAttachShader(fragmentShaderHandle)");
        GLES20.glLinkProgram(programHandleLines);
        throwGLErrorIfNecessary("glLinkProgram");

        mvpMatrixHandleLines = GLES20.glGetUniformLocation(programHandleLines, "u_MVPMatrix");
        lineColorHandle = GLES20.glGetUniformLocation(programHandleLines, "u_LineColor");
        positionHandleLines = GLES20.glGetAttribLocation(programHandleLines, "a_Position");
    }

    /**
     * Setup the shader Program for the point rendering of the OcTree-Nodes
     */
    private void setupProgramPoints() {
        int vertexShaderHandle = createShader(GLES20.GL_VERTEX_SHADER, vertexShaderPointsCode);
        int fragmentShaderHandle = createShader(GLES20.GL_FRAGMENT_SHADER, fragmentShaderPointsCode);

        programHandlePoints = GLES20.glCreateProgram();
        throwGLErrorIfNecessary("glCreateProgram");
        GLES20.glAttachShader(programHandlePoints, vertexShaderHandle);
        throwGLErrorIfNecessary("glAttachShader(vertexShaderHandle)");
        GLES20.glAttachShader(programHandlePoints, fragmentShaderHandle);
        throwGLErrorIfNecessary("glAttachShader(fragmentShaderHandle)");
        GLES20.glLinkProgram(programHandlePoints);
        throwGLErrorIfNecessary("glLinkProgram");

        textureUniformHandle = GLES20.glGetUniformLocation(programHandlePoints, "u_ColorTex1D");
        mvpMatrixHandlePoints = GLES20.glGetUniformLocation(programHandlePoints, "u_MVPMatrix");
        ocTreeResolutionHandle = GLES20.glGetUniformLocation(programHandlePoints, "u_OcTreeResolution");
        positionHandlePoints = GLES20.glGetAttribLocation(programHandlePoints, "a_Position");

        createAndBindLookUpTexture();
    }

    /**
     * create the look-up-texture for node colorization
     */
    private void createAndBindLookUpTexture() {
        int width = 8, height = 8;
        byte[] lutColors = setupLuTColorByteArray(width, height);

        ByteBuffer buffer = ByteBuffer.allocateDirect(lutColors.length);
        buffer.put(lutColors);
        buffer.rewind();

        texture_map = new int[1];
        GLES20.glGenTextures(1, texture_map, 0);
        GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, texture_map[0]);

        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_LINEAR);
        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_LINEAR);
        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_WRAP_S, GLES20.GL_CLAMP_TO_EDGE);
        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_WRAP_T, GLES20.GL_CLAMP_TO_EDGE);

        GLES20.glTexImage2D(GLES20.GL_TEXTURE_2D, 0, GLES20.GL_RGBA, width, height, 0, GLES20.GL_RGBA, GLES20.GL_UNSIGNED_BYTE, buffer);
        throwGLErrorIfNecessary("glTexImage2D");

        GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, 0);
    }

    /**
     * programmatically fill a byte array with content for a look up texture for node colorization
     */
    private byte[] setupLuTColorByteArray(int width, int height) {
        byte lutColors[] = new byte[width * height * 4];

        byte colorIntensity =       (byte) 200;
        byte nonColorIntensity =    (byte) 0;
        byte alpha =                (byte) 255;

        int y = 0;
        for (int x = 0; x < width; x++) {
            int pos = (x * width + y) * 4;
            lutColors[pos] = nonColorIntensity;     // Red
            lutColors[pos + 1] = nonColorIntensity; // Green
            lutColors[pos + 2] = colorIntensity;    // Blue
            lutColors[pos + 3] = alpha;             // Alpha
        }
        y++;
        for (int x = 0; x < width; x++) {
            int pos = (x * width + y) * 4;
            lutColors[pos] = nonColorIntensity;
            lutColors[pos + 1] = colorIntensity;
            lutColors[pos + 2] = colorIntensity;
            lutColors[pos + 3] = alpha;
        }
        y++;
        for (int x = 0; x < width; x++) {
            int pos = (x * width + y) * 4;
            lutColors[pos] = nonColorIntensity;
            lutColors[pos + 1] = colorIntensity;
            lutColors[pos + 2] = nonColorIntensity;
            lutColors[pos + 3] = alpha;
        }
        y++;
        for (int x = 0; x < width; x++) {
            int pos = (x * width + y) * 4;
            lutColors[pos] = colorIntensity;
            lutColors[pos + 1] = colorIntensity;
            lutColors[pos + 2] = nonColorIntensity;
            lutColors[pos + 3] = alpha;
        }
        y++;
        for (int x = 0; x < width; x++) {
            int pos = (x * width + y) * 4;
            lutColors[pos] = colorIntensity;
            lutColors[pos + 1] = nonColorIntensity;
            lutColors[pos + 2] = nonColorIntensity;
            lutColors[pos + 3] = alpha;
        }
        y++;
        for (int x = 0; x < width; x++) {
            int pos = (x * width + y) * 4;
            lutColors[pos] = colorIntensity;
            lutColors[pos + 1] = nonColorIntensity;
            lutColors[pos + 2] = colorIntensity;
            lutColors[pos + 3] = alpha;
        }
        y++;
        for (int x = 0; x < width; x++) {
            int pos = (x * width + y) * 4;
            lutColors[pos] = nonColorIntensity;
            lutColors[pos + 1] = nonColorIntensity;
            lutColors[pos + 2] = colorIntensity;
            lutColors[pos + 3] = alpha;
        }
        y++;
        for (int x = 0; x < width; x++) {
            int pos = (x * width + y) * 4;
            lutColors[pos] = nonColorIntensity;
            lutColors[pos + 1] = colorIntensity;
            lutColors[pos + 2] = colorIntensity;
            lutColors[pos + 3] = alpha;
        }

        return lutColors;
    }

    /**
     * handle the Surface Changed event by updating the perspective projection matrix
     */
    @Override
    public void onSurfaceChanged(GL10 gl, int width, int height) {
        GLES20.glViewport(0, 0, width, height);

        Log.d(TAG, "Width: " + width + " Height: " + height);

        // Create a new perspective projection matrix. The height will stay the same
        // while the width will adjust depending on the aspect ratio.
        final float ratio = (float) width / height;
        Log.d(TAG, "ratio: " + ratio);

        Matrix.perspectiveM(projectionMatrix, 0, FOVY, ratio, NEAR, FAR);
    }

    private final float[] mRotation = new float[16];

    /**
     * gets called once before every frame is rendered
     */
    @Override
    public void onDrawFrame(GL10 gl) {
        // Clear background with ClearColor
        GLES20.glClear(GLES20.GL_DEPTH_BUFFER_BIT | GLES20.GL_COLOR_BUFFER_BIT);

        setupMVPMatrix();
        
        renderOcTree();
        
        renderPath();
        
        renderPosHistory();
    }

    /**
     * Calculate the mvp matrix for this frame
     */
    private void setupMVPMatrix() {
        if(useCameraPose) {
            updateViewMatrixToPose();
            Matrix.multiplyMM(tempMatrix, 0, mTangoToOGLMatrix, 0, viewMatrix, 0);
        }
        else {
            Matrix.multiplyMM(tempMatrix, 0, mRotation, 0, viewMatrix, 0);
        }
        Matrix.multiplyMM(mvpMatrix, 0, projectionMatrix, 0, tempMatrix, 0);
    }

    /**
     * set the uniforms and attributes up for the rendering of the pose history
     */
    private void renderPosHistory() {
        GLES20.glUseProgram(programHandleLines);
        GLES20.glLineWidth(10);

        GLES20.glUniformMatrix4fv(mvpMatrixHandleLines, 1, false, mvpMatrix, 0);
        GLES20.glUniform4fv(lineColorHandle, 1, HISTORY_LINE_COLOR, 0);

        //Send the vertices
        GLES20.glVertexAttribPointer(positionHandleLines, FLOATS_PER_VERTEX_LINE, GLES20.GL_FLOAT, false, 0, vertexBuffferHistory);
        GLES20.glEnableVertexAttribArray(positionHandleLines);

        //Draw the lines
        GLES20.glDrawArrays(GLES20.GL_LINE_STRIP, 0, bufferHistoryVertexCount);
    }

    /**
     * set the uniforms and attributes up for the rendering of the way points
     */
    private void renderPath() {
        GLES20.glUseProgram(programHandleLines);
        GLES20.glLineWidth(20);

        GLES20.glUniformMatrix4fv(mvpMatrixHandleLines, 1, false, mvpMatrix, 0);
        GLES20.glUniform4fv(lineColorHandle, 1, PATH_LINE_COLOR, 0);

        //Send the vertices
        GLES20.glVertexAttribPointer(positionHandleLines, FLOATS_PER_VERTEX_LINE, GLES20.GL_FLOAT, false, 0, vertexBuffferPath);
        GLES20.glEnableVertexAttribArray(positionHandleLines);

        //Draw the lines
        int numRemainingWayPoints = bufferPathVertexCount - bufferPathCurrentIndex;
        GLES20.glDrawArrays(GLES20.GL_LINE_STRIP, bufferPathCurrentIndex, numRemainingWayPoints);
    }

    /**
     * set the uniforms and attributes up for the rendering of the Tree-Nodes
     */
    private void renderOcTree() {
        GLES20.glUseProgram(programHandlePoints);

        // Set the active texture unit to texture unit 0.
        GLES20.glActiveTexture(GLES20.GL_TEXTURE0);

        // Bind the texture to this unit.
        GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, texture_map[0]);

        // Tell the texture uniform sampler to use this texture in the shader by binding to texture unit 0.
        GLES20.glUniform1i(textureUniformHandle, 0);

        GLES20.glUniformMatrix4fv(mvpMatrixHandlePoints, 1, false, mvpMatrix, 0);

        GLES20.glUniform1f(ocTreeResolutionHandle, (float)explorationData.getOcTree().getResolution());

        //Send the vertices
        GLES20.glVertexAttribPointer(positionHandlePoints, FLOATS_PER_VERTEX_POINTS, GLES20.GL_FLOAT, false, 0, vertexBuffferOcTree);
        GLES20.glEnableVertexAttribArray(positionHandlePoints);

        //Draw the points
        GLES20.glDrawArrays(GLES20.GL_POINTS, 0, bufferOctreeVertexCount);
    }

    private Vector2f currentCameraAngle = new Vector2f();
    private Vector3f currentCameraPos = new Vector3f();

    /// The following 3 Methods are only used in TouchControll-Mode
    private void updateViewMatrixToPose(){
        Matrix.setRotateM(viewMatrix, 0, currentCameraAngle.y, 1, 0, 0);
        Matrix.rotateM(viewMatrix, 0, currentCameraAngle.x, 0, 0 ,1);
        Matrix.translateM(viewMatrix, 0, currentCameraPos.x, currentCameraPos.y, currentCameraPos.z);
    }
    
    void translateCamera(float dx, float dy){
        Vector3f delta = new Vector3f(dx, dy, 0);
        Matrix4f rotationMatrix = new Matrix4f();
        rotationMatrix.rotX((float)Math.toRadians(currentCameraAngle.y));
        rotationMatrix.transform(delta);
        rotationMatrix.rotZ((float)Math.toRadians(currentCameraAngle.x));
        rotationMatrix.transform(delta);
        
        currentCameraPos.x += delta.x;
        currentCameraPos.y -= delta.y;
        currentCameraPos.z += delta.z;
    }
    
    void rotateCamera(float angleX, float angleY) {
        currentCameraAngle.x -= angleX;
        currentCameraAngle.y -= angleY;
        
        // Clamp the rotation along the x axis to prevent the camera to turn further than vertical
        currentCameraAngle.y = Math.min(Math.max(currentCameraAngle.y, -90), 90);
    }


    /**
     * This Method is used to set the virtual camera position and orientation to the real TangoPose
     */
    void setCurrentMatrixTransformation(float[] matrix){
        System.arraycopy(matrix, 0, viewMatrix, 0, viewMatrix.length);
    }
}