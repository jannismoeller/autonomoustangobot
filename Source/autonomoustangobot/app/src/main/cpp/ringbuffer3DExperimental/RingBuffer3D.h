#include <jni.h>

#ifndef AUTONOMOUSTANGOBOT_RINGBUFFER3D_H
#define AUTONOMOUSTANGOBOT_RINGBUFFER3D_H

/**
 * Experimental implementation of a 3d Ring Buffer.
 * Implementation not complete
 */
class RingBuffer3D {
private:
    const double resolution;
    const double resolution_inverse;
    const int xLength, yLength, zLength;
    const int yzLength;
    unsigned char * buffer;
    const int measurementVal[2] = { -1, 2 };
    const int ODD_MAX = 127;
    const int ODD_MIN = 0;
//    const int POSITIVE_OCCUPATION = 2;
//    const int NEGATIVE_OCCUPATION = -1;
    inline unsigned char clamp(int);
    inline bool wasCellUpdated(int index);
    inline void setCellUpdated(int index);
    inline void setCellNotUpdated(int index);

public:
    RingBuffer3D(float resolution, int xLength, int yLength, int zLength);
    RingBuffer3D(float resolution);
    ~RingBuffer3D();
    int getSizeOfBuffer();
    inline const int getIndex(int, int, int);
    inline void toGrid(float, float, float, int*, int*, int*);
    inline void wrapToBuffer(int*, int*, int*);
    void updateCell(int index, bool occupied);
    void insertPointCloud(jfloat * pFB,
                          jint numPoints,
                          jfloat * m4x4,
                          jfloat * sensorCoordArray);

};

#endif //AUTONOMOUSTANGOBOT_RINGBUFFER3D_H
