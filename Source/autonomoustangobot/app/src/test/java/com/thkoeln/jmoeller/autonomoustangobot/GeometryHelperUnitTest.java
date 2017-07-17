package com.thkoeln.jmoeller.autonomoustangobot;

import com.thkoeln.jmoeller.autonomoustangobot.exploration.GeometryHelper;

import org.junit.Test;

import javax.vecmath.Matrix4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector2d;

import static org.junit.Assert.*;

/**
 * Local unit test, which will execute on the development machine (host).
 */
public class GeometryHelperUnitTest {
    private static final double DOUBLE_DELTA = 1e-15;
    
    @Test
    public void getDotOfDirections_isCorrect() throws Exception {
        double degree = Math.PI / 2.0;

        Quat4d quat4d1 = new Quat4d();
        Quat4d quat4d2 = new Quat4d();

        System.out.println("quat4d1: " + quat4d1 + ", quat4d2: " + quat4d2);
        assertEquals(1, GeometryHelper.dotOfDirections(quat4d1, quat4d2), DOUBLE_DELTA);
        
        Matrix4d matrix4d = new Matrix4d();

        matrix4d.setZero();
        matrix4d.rotZ(degree);
        matrix4d.get(quat4d1);

        System.out.println("quat4d1: " + quat4d1 + ", quat4d2: " + quat4d2);
        assertEquals(1, GeometryHelper.dotOfDirections(quat4d1, quat4d2), DOUBLE_DELTA);

        matrix4d.setZero();
        matrix4d.rotX(degree);
        matrix4d.get(quat4d2);
        Matrix4d helper = new Matrix4d();
        helper.rotZ(degree);
        helper.mul(matrix4d); //Quat4d1 is rotatet around x by 90° then rotated around y by 90°
        helper.get(quat4d1);

        System.out.println("quat4d1: " + quat4d1 + ", quat4d2: " + quat4d2);
        assertEquals(0, GeometryHelper.dotOfDirections(quat4d1, quat4d2), DOUBLE_DELTA);

        matrix4d.setZero();
        matrix4d.rotX(degree);
        matrix4d.get(quat4d2);
        helper = new Matrix4d();
        helper.rotZ(degree);
        matrix4d.mul(helper); //Quat4d1 is rotatet around y by 90° then rotated around x by 90°
        matrix4d.get(quat4d1);

        System.out.println("quat4d1: " + quat4d1 + ", quat4d2: " + quat4d2);
        assertEquals(1, GeometryHelper.dotOfDirections(quat4d1, quat4d2), DOUBLE_DELTA);
        quat4d2 = new Quat4d();
        
        matrix4d.setZero();
        matrix4d.rotY(degree);
        matrix4d.get(quat4d1);

        System.out.println("quat4d1: " + quat4d1 + ", quat4d2: " + quat4d2);
        assertEquals(0, GeometryHelper.dotOfDirections(quat4d1, quat4d2), DOUBLE_DELTA);

        matrix4d.setZero();
        matrix4d.rotY(degree * 2);
        matrix4d.get(quat4d1);
        
        System.out.println("quat4d1: " + quat4d1 + ", quat4d2: " + quat4d2);
        assertEquals(-1, GeometryHelper.dotOfDirections(quat4d1, quat4d2), DOUBLE_DELTA);
    }

    @Test
    public void signedAngleDifference_isCorrect() throws Exception {
        double andgle = GeometryHelper.signedAngleDifference(new Vector2d(1, 0), new Vector2d(0, 1));
        assertEquals(Math.PI / 2, andgle, DOUBLE_DELTA);
        
        andgle = GeometryHelper.signedAngleDifference(new Vector2d(1, 0), new Vector2d(0, -1));
        assertEquals(-Math.PI / 2, andgle, DOUBLE_DELTA);
                
        Vector2d quarterPi = new Vector2d(1, 1);
        quarterPi.normalize();
        andgle = GeometryHelper.signedAngleDifference(new Vector2d(1, 0), quarterPi);
        assertEquals(Math.PI / 4, andgle, DOUBLE_DELTA);
        
        andgle = GeometryHelper.signedAngleDifference(new Vector2d(1, 0), new Vector2d(-1, 0));
        assertEquals(Math.PI, andgle, DOUBLE_DELTA);
    }
    
    @Test
    public void signedFullangleConversionTest() throws Exception {
        assertEquals(0, GeometryHelper.fullAngleToSignedAngle(0), DOUBLE_DELTA);
        assertEquals(0, GeometryHelper.signedAngleToFullAngle(0), DOUBLE_DELTA);
        
        assertEquals(90, GeometryHelper.fullAngleToSignedAngle(90), DOUBLE_DELTA);
        assertEquals(90, GeometryHelper.signedAngleToFullAngle(90), DOUBLE_DELTA);

        assertEquals(180, GeometryHelper.fullAngleToSignedAngle(180), DOUBLE_DELTA);
        assertEquals(180, GeometryHelper.signedAngleToFullAngle(180), DOUBLE_DELTA);

        assertEquals(-175, GeometryHelper.fullAngleToSignedAngle(185), DOUBLE_DELTA);
        assertEquals(185, GeometryHelper.signedAngleToFullAngle(-175), DOUBLE_DELTA);
    }
}