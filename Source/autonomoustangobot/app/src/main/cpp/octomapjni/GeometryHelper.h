#ifndef AUTONOMOUSTANGOBOT_GEOMETRYHELPER_H
#define AUTONOMOUSTANGOBOT_GEOMETRYHELPER_H

#include <octomap/math/Vector3.h>

using namespace octomath;

class GeometryHelper {
private:
    /**
     * @param val 
     * @return 1 if val > 0, 0 if val == 0 amd -1 if val < 0
     */
    static int sgn(double val){
        return (val > 0.0) - (val < 0.0);
    }
public:
    /**
     * @param direction1 normalized vector pointing in direction 1
     * @param direction2 normalized vector pointing in direction 2
     * @return angle in rads from 1 to 2. Sign is - if the rotation is counterclockwise if viewed from above
     */
    static double signedAngleDifference(const Vector3 direction1, const Vector3 direction2){
        Vector3 dir1(direction1.x(), direction1.y(), 0);
        Vector3 dir2(direction2.x(), direction2.y(), 0);
        dir1.normalize();
        dir2.normalize();
        double dot = dir1.dot(dir2);

        // If the two vectors are parallel it doesn't make sense to calculate the normal
        if(dot == 1)
            return 0;
        if(dot == -1)
            return M_PI;

        Vector3 norm = dir1.cross(dir2);
        return sgn(norm.z()) * acos(dot);
    }

    /**
     * @param direction1 normalized vector pointing in direction 1
     * @param direction2 normalized vector pointing in direction 2
     * @return angle in rads between 1 to 2 if viewed from above
     */
    static double angleDifference(const Vector3 direction1, const Vector3 direction2){
        Vector3 dir1(direction1.x(), direction1.y(), 0);
        Vector3 dir2(direction2.x(), direction2.y(), 0);
        return dir1.angleTo(dir2);
    }
};


#endif //AUTONOMOUSTANGOBOT_GEOMETRYHELPER_H
