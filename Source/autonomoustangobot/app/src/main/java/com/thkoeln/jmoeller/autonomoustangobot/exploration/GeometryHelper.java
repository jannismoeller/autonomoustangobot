package com.thkoeln.jmoeller.autonomoustangobot.exploration;

import android.support.annotation.NonNull;
import android.util.Log;

import com.google.atap.tangoservice.TangoPoseData;

import javax.vecmath.Matrix4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

/**
 * Helper-Class for various different geometric calculations
 */
public class GeometryHelper {

    /**
     * Compares two directions 
     * @returns the smallest angle between them
     */
    public static double dotOfDirections(Quat4d orientation1, Quat4d orientation2) {
        Matrix4d rotMat1 = new Matrix4d();
        Matrix4d rotMat2 = new Matrix4d();
        rotMat1.set(orientation1);
        rotMat2.set(orientation2);
//        System.out.println("rotMat1:\n" + rotMat1 + "\nrotMat2:\n" + rotMat2);
        Vector3d dir1 = new Vector3d(0, 0, 1);
        Vector3d dir2 = new Vector3d(0, 0, 1);
        rotMat1.transform(dir1);
        rotMat2.transform(dir2);

//        System.out.println("dir1: " + dir1 + ", dir2: " + dir2);
        return dir1.dot(dir2);
    }

    /**
     * @returns position2 - position1
     */
    public static Vector3d poseDifferencePosition(TangoPoseData pose1, TangoPoseData pose2){
        Vector3d position1 = new Vector3d(pose1.translation);
        Vector3d position2 = new Vector3d(pose2.translation);
        Vector3d difference = new Vector3d();
        difference.sub(position2, position1);
        return difference;
    }

    /**
     * Compares the two poses directions 
     * @returns the smallest angle between them
     */
    public static double poseDifferenceRotation(TangoPoseData pose1, TangoPoseData pose2){
        Quat4d orientation1 = new Quat4d(pose1.rotation);
        Quat4d orientation2 = new Quat4d(pose2.rotation);
        
        return dotOfDirections(orientation1, orientation2);
    }

    /**
     * 
     * @param direction1 normalized vector pointing in direction 1
     * @param direction2 normalized vector pointing in direction 2
     * @return angle in rads from 1 to 2. Sign is - if the rotation is counterclockwise if viewed from above
     */
    public static double signedAngleDifference(Vector2d direction1, Vector2d direction2){
        Vector3d dir1 = new Vector3d(direction1.x, direction1.y, 0);
        Vector3d dir2 = new Vector3d(direction2.x, direction2.y, 0);
        double dot = dir1.dot(dir2);
        
        // If the two vectors are parallel it doesn't make sense to calculate the normal
        if(dot == 1)
            return 0;
        if(dot == -1)
            return Math.PI;
        
        Vector3d norm = new Vector3d();
        norm.cross(dir1, dir2);
        return Math.signum(norm.z) * Math.acos(dir1.dot(dir2));
    }

    /**
     * @param pose pose of the device
     * @param point point in space
     * @return angle between poses 2d forward vector and the 2d direction from the device to the point
     */
    public static double signedAngleDifference(TangoPoseData pose, Vector3d point){
        Vector2d forward2d = directionVector2d(pose);
        
        Vector3d posToPoint = new Vector3d();
        posToPoint.sub(point, new Vector3d(pose.translation));
        Vector2d posToPoint2d = new Vector2d(posToPoint.x, posToPoint.y);
        posToPoint2d.normalize();
        
        double angle = signedAngleDifference(forward2d, posToPoint2d);
        Log.d("signedAngleDifference", "forward: " + vToStr(forward2d) + " posToPoint: " + vToStr(posToPoint2d) + "\nangle: " + String.format("%.4f", Math.toDegrees(angle)));
        return angle;
    }

    /**
     * @param pose pose of the device
     * @param direction2D direction on horizontal plane
     * @return angle between poses 2d forward vector and the 2d direction
     */
    public static double signedAngleDifference(TangoPoseData pose, Vector2d direction2D){
        Vector2d forward2d = directionVector2d(pose);

        Vector2d direction2DNormalized = new Vector2d();
        direction2DNormalized.normalize(direction2D);

        return signedAngleDifference(forward2d, direction2DNormalized);
    }
    
    public static double signedAngleToFullAngle(double angle){
        double simpleAngle = angle % 360.0;
        if(simpleAngle < 0)
            simpleAngle += 360;
        return simpleAngle;
    }
    
    public static double fullAngleToSignedAngle(double angle){
        double simpleAngle = angle % 360.0;
        if(simpleAngle > 180)
            simpleAngle -= 360;
        return simpleAngle;
    }

    /**
     * @param pose
     * @return 2D normalized direction vector when projected to the ground plane (XY)
     */
    @NonNull
    public static Vector2d directionVector2d(TangoPoseData pose) {
        Vector3d forward = new Vector3d(0, 0, 1);
        Matrix4d rotMat = new Matrix4d();
        rotMat.set(new Quat4d(pose.rotation));
        rotMat.transform(forward);
        Vector2d forward2d = new Vector2d(forward.x, forward.y);
        forward2d.normalize();
        return forward2d;
    }

    public static String vToStr(Vector3d v){
        return String.format("(%.4f %.4f %.4f)", v.x, v.y, v.z);
    }

    public static String vToStr(Vector3f v){
        return String.format("(%.4f %.4f %.4f)", v.x, v.y, v.z);
    }
    
    public static String vToStr(Vector2d v){
        return String.format("(%.4f %.4f)", v.x, v.y);
    }
}
