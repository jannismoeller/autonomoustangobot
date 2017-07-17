#include <vector>
#include <android/log.h>

#include "RingBuffer3D.h"

/**
 * Experimental implementation of a 3d Ring Buffer.
 * Implementation not complete
 */

using namespace std;

RingBuffer3D::RingBuffer3D(float resolution, int xLength, int yLength, int zLength) :
        resolution(resolution),
        resolution_inverse(1./resolution),
        xLength(xLength),
        yLength(yLength),
        zLength(zLength),
        yzLength(yLength * zLength)
{
    int sizeOfBuffer = getSizeOfBuffer();
    buffer = new unsigned char[sizeOfBuffer];
    unsigned char neutralVal = static_cast<unsigned char>((ODD_MAX - ODD_MIN) / 2);

    for (int i = 0; i < sizeOfBuffer; ++i) {
        buffer[i] = neutralVal;
    }

    __android_log_print(ANDROID_LOG_INFO, "RINGBUFFER3D", "neutralVal: %d", neutralVal);
}

RingBuffer3D::RingBuffer3D(float resolution) : RingBuffer3D(resolution, 256, 256, 256) { }

RingBuffer3D::~RingBuffer3D() {
    delete [] buffer;
}

int RingBuffer3D::getSizeOfBuffer() {
    return xLength * yLength * zLength;
}

inline
const int RingBuffer3D::getIndex(int x, int y, int z) {
    return x * yzLength + y * zLength + z;
}

/**
 * https://codereview.stackexchange.com/a/73957
 */
inline
unsigned char RingBuffer3D::clamp(int n) {
    n = n > ODD_MAX ? ODD_MAX : n;
    return static_cast<unsigned char>(n < ODD_MIN ? ODD_MIN : n);
}

inline
void RingBuffer3D::toGrid(float x, float y, float z, int * xG, int * yG, int * zG) {
    *xG = static_cast<int>(x * resolution_inverse);
    *yG = static_cast<int>(y * resolution_inverse);
    *zG = static_cast<int>(z * resolution_inverse);
}

inline
void RingBuffer3D::wrapToBuffer(int * x, int * y, int * z) {
    *x = (*x %= xLength) < 0 ? *x + xLength : *x;
    *y = (*y %= yLength) < 0 ? *y + yLength : *y;
    *z = (*z %= zLength) < 0 ? *z + zLength : *z;
}

void RingBuffer3D::updateCell(int index, bool occupied) {
    buffer[index] = clamp(buffer[index] + measurementVal[occupied]);
    setCellUpdated(index);
}

unsigned char changedFlagBitmask = 0x80;
unsigned char oddValueBitmask = 0x7F;

inline
bool RingBuffer3D::wasCellUpdated(int index) {
    return (buffer[index] & changedFlagBitmask) >> 7;
}

inline
void RingBuffer3D::setCellUpdated(int index) {
    buffer[index] = buffer[index] | changedFlagBitmask;
}

inline
void RingBuffer3D::setCellNotUpdated(int index) {
    buffer[index] = buffer[index] & oddValueBitmask;
}

void RingBuffer3D::insertPointCloud(
    jfloat * pFB,
    jint numPoints,
    jfloat * m4x4,
    jfloat * sensorCoordArray){

    vector<int> occupiedCells;
    occupiedCells.reserve(static_cast<unsigned int>(numPoints));
    vector<int> freeCells;
    occupiedCells.reserve(static_cast<unsigned int>(numPoints * yLength / 2));

    float p[3];
    int x = 0, y = 0, z = 0;

    for (int i = 0; i < numPoints * 4; i+=4) {
        p[0] = m4x4[0] * pFB[i + 0] + m4x4[4] * pFB[i + 1] + m4x4[8] * pFB[i + 2] + m4x4[12];
        p[1] = m4x4[1] * pFB[i + 0] + m4x4[5] * pFB[i + 1] + m4x4[9] * pFB[i + 2] + m4x4[13];
        p[2] = m4x4[2] * pFB[i + 0] + m4x4[6] * pFB[i + 1] + m4x4[10] * pFB[i + 2] + m4x4[14];

        toGrid(p[0], p[1], p[2], &x, &y, &z);
        wrapToBuffer(&x, &y, &z);

        int index = getIndex(x, y, z);

        if(!wasCellUpdated(index)){
            updateCell(index, true);
            occupiedCells.push_back(index);

            for (int j = 1; j < xLength / 2; j++) {
//                index = getIndex(x, y, z);
                index = getIndex(x, y, j);

                if(!wasCellUpdated(index)){
                    updateCell(index, false);
                    freeCells.push_back(index);
                }
            }
        }
    }

    __android_log_print(ANDROID_LOG_INFO, "RINGBUFFER3D", "unique occupied cell count: %u\nunique free cell count: %u", (unsigned int)occupiedCells.size(), (unsigned int)freeCells.size());

    vector<int>::const_iterator it;
    for(it = occupiedCells.begin(); it != occupiedCells.end(); it++){
        setCellNotUpdated(*it);
    }
    for(it = freeCells.begin(); it != freeCells.end(); it++){
        setCellNotUpdated(*it);
    }
}
