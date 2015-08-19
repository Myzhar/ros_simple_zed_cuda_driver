//StandardHeaders.h

#ifndef __UTILS_HPP__
#define __UTILS_HPP__

#include "GL/glut.h"    /* OpenGL Utility Toolkit header */

#ifdef WIN32
#define M_PI 3.141592653
#endif


struct POINT3D {
    float x;
	float y;
	float z;
    //!color [0 1];
	float r;
	float g;
	float b;

	POINT3D() : x(0), y(0), z(0), r(0), g(0), b(0) {
    }

	POINT3D(float x_, float y_, float z_) : x(x_), y(y_), z(z_), r(0), g(255), b(255) {
    }

	POINT3D(float x_, float y_, float z_, float r_, float g_, float b_) : x(x_), y(y_), z(z_), r(r_), g(g_), b(b_) {
    }

	POINT3D operator*(float val) {
        this->x *= val;
        this->y *= val;
        this->z *= val;
        return *this;
    }

	POINT3D operator/(float val) {
        this->x /= val;
        this->y /= val;
        this->z /= val;
        return *this;
    }

    float getColorFloat() const {
        uint8_t R = r * 255.f;
        uint8_t G = g * 255.f;
        uint8_t B = b * 255.f;
        uint32_t rgb_32 = ((uint32_t) R << 16 | (uint32_t) G << 8 | (uint32_t) B);
        return *reinterpret_cast<float*> (&rgb_32);
    }

	void setColor(float r_, float g_, float b_) {
        r = r_;
        g = g_;
        b = b_;
    }

    void setColor(const unsigned char* Color) {
        r = Color[2] / 255.f;
        g = Color[1] / 255.f;
        b = Color[0] / 255.f;
    }

};


inline float d2r(float degree) {
    return degree * M_PI / 180.0f;
}

inline float r2d(float radians) {
    return radians * 180.0f / M_PI;
}

struct vect3 {
public:
    GLfloat x;
    GLfloat y;
    GLfloat z;

    vect3() {
        x = y = z = 0;
    };

    vect3(GLfloat x, GLfloat y, GLfloat z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    void normalise() {
        GLfloat length = (GLfloat) sqrt(x * x + y * y + z * z);
        x = x / length;
        y = y / length;
        z = z / length;
    }

    void rotate(float angle, vect3 axis) {
        float rangle = d2r(angle);
        // Rotation Matrix
        //  ( a b c )
        //  | d e f |
        //  ( g h i )
        float cc = cos(rangle);
        float ss = sin(rangle);
        float a = axis.x * axis.x + (1 - axis.x * axis.x) * cc;
        float b = axis.x * axis.y * (1 - cc) - axis.z * ss;
        float c = axis.x * axis.z * (1 - cc) + axis.y * ss;
        float d = axis.x * axis.y * (1 - cc) + axis.z * ss;
        float e = axis.y * axis.y + (1 - axis.y * axis.y) * cc;
        float f = axis.y * axis.z * (1 - cc) - axis.x * ss;
        float g = axis.x * axis.z * (1 - cc) - axis.y * ss;
        float h = axis.y * axis.z * (1 - cc) + axis.x * ss;
        float i = axis.z * axis.z + (1 - axis.z * axis.z) * cc;

        float nx = x * a + y * b + z * c;
        float ny = x * d + y * e + z * f;
        float nz = x * g + y * h + z * i;

        x = nx;
        y = ny;
        z = nz;
    }

    static float length(vect3 u) {
        return sqrtf(u.x * u.x + u.y * u.y + u.z * u.z);
    }

    static float dot(vect3 u, vect3 v) {
        return u.x * v.x + u.y * v.y + u.z * v.z;
    }

    static float getAngle(vect3 a, vect3 o, vect3 b) {
        vect3 oa(a.x - o.x, a.y - o.y, a.z - o.z);
        vect3 ob(b.x - o.x, b.y - o.y, b.z - o.z);
        float s = acosf(dot(oa, ob) / (length(oa) * length(ob)));
        return r2d(s);
    }
};

#endif /*__UTILS_HPP__*/
