package com.thkoeln.jmoeller.autonomoustangobot.rendering;

import android.content.Context;
import android.opengl.GLSurfaceView;
import android.util.AttributeSet;
import android.view.MotionEvent;

import com.projecttango.tangosupport.TangoSupport;
import com.thkoeln.jmoeller.autonomoustangobot.explorationdata.ExplorationData;

import javax.vecmath.Vector2f;

/**
 * This is the View that displays the visualisation of the OcTree in the ExplorationActivity and 
 * the ExplorationViewerActivity
 * TODO: Rename class when additional things aside from the PointCloud are being rendered!
 */
public class PointCloudGLSurfaceView extends GLSurfaceView {

    private static final String TAG = PointCloudGLSurfaceView.class.getSimpleName();
    
    public PointCloudGLRenderer getRenderer() {return renderer;}
    private final PointCloudGLRenderer renderer;

    public PointCloudGLSurfaceView(Context context, AttributeSet attrs) {
        super(context, attrs);

        setEGLContextClientVersion(2);
        // set Buffer sizes
        setEGLConfigChooser(8, 8, 8, 8, 16, 0);

        renderer = new PointCloudGLRenderer(context);

        setRenderer(renderer);
        
        requestRender();
    }

    @Override
    public void onResume() {
        super.onResume();
        requestRender();
    }

    public void setExplorationData(ExplorationData explorationData) {renderer.setExplorationData(explorationData);}
    public void setCurrentMatrixTransformation(TangoSupport.TangoMatrixTransformData matrtixTransformData) {
        renderer.setCurrentMatrixTransformation(matrtixTransformData.matrix);
    }

    private boolean useTouchControlls = false;
    public void setUseTouchControlls(boolean useTouchControlls) { 
        this.useTouchControlls = useTouchControlls;
        renderer.useCameraPose = useTouchControlls;
    }
    private static final float TOUCH_ROTATE_FACTOR = 0.1f;
    private static final float TOUCH_TRANSLATE_FACTOR = 0.002f;
    private Vector2f lastTouch0 = new Vector2f();
    private Vector2f lastTouch1 = new Vector2f();
    
    private long timeStampLastDoubleTouch = 0;
    // This is needed to prevent the view from suddenly jumping around after you release one press
    private static long TIME_DELTA_2_TOUCH_TO_1_TOUCH = 50;
    
    @Override
    public boolean onTouchEvent(MotionEvent e) {
        if(!useTouchControlls)
            return false;        
        
        float x0 = e.getX(0);
        float y0 = e.getY(0);
        float dx0 = x0 - lastTouch0.x;
        float dy0 = y0 - lastTouch0.y;
        
        // Handling 1 Touch: rotate
        if(e.getPointerCount() == 1 && e.getAction() == MotionEvent.ACTION_MOVE
                && (timeStampLastDoubleTouch + TIME_DELTA_2_TOUCH_TO_1_TOUCH) < System.currentTimeMillis()) {
            renderer.rotateCamera(dx0 * TOUCH_ROTATE_FACTOR, dy0 * TOUCH_ROTATE_FACTOR);
            
            requestRender();
        }
        // Handling 2 Touches: translate
        else if(e.getPointerCount() >= 2){
            float x1 = e.getX(1);
            float y1 = e.getY(1);
            float dx1 = x1 - lastTouch1.x;
            float dy1 = y1 - lastTouch1.y;
            float dxSum = dx0 + dx1;
            float dySum = dy0 + dy1;
            
            if(e.getAction() == MotionEvent.ACTION_MOVE){
                renderer.translateCamera(dxSum * TOUCH_TRANSLATE_FACTOR, dySum * TOUCH_TRANSLATE_FACTOR);
        
                requestRender();
            }

            lastTouch1.x = x1;
            lastTouch1.y = y1;

            timeStampLastDoubleTouch = System.currentTimeMillis();
        }

        lastTouch0.x = x0;
        lastTouch0.y = y0;
        return true;
    }
}