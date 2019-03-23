#include <iostream>
#include <array>

#include <cmath>
#include "util.h"

typedef float num_type;
typedef std::array<num_type,3> vec3;
typedef std::array<vec3,3> mat3;
typedef std::array<num_type,16> mat4;

num_type sign(num_type x)
{
    if(x >= 0.0)
        return 1.0;
    else
        return -1.0;
}

num_type norm(const vec3 &v)
{
    return std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
}

vec3 operator-(const vec3 &v)
{
    return {-v[0], -v[1], -v[2]};
}

vec3 operator+(const vec3 &u, const vec3 &v)
{
    return {u[0]+v[0], u[1]+v[1], u[2]+v[2]};
}

vec3 operator-(const vec3 &u, const vec3 &v)
{
    return {u[0]-v[0], u[1]-v[1], u[2]-v[2]};
}

vec3 operator*(num_type s, const vec3 &v)
{
    return {s*v[0], s*v[1], s*v[2]};
}

vec3 operator/(const vec3 &v, num_type s)
{
    return {s/v[0], s/v[1], s/v[2]};
}

vec3 unit3(const vec3 &v)
{
    return v/norm(v);
}

num_type dot3(const vec3 &u, const vec3 &v)
{
    return u[0]*v[0]+u[1]*v[1]+u[2]*v[2];
}

vec3 cross(const vec3 &u, const vec3 &v)
{
    return {u[1]*v[2] - u[2]*v[1],
            u[2]*v[0] - u[0]*v[2],
            u[0]*v[1] - u[1]*v[0]};
}

vec3 project3(const vec3 &u, const vec3 &v)
{
    return dot3(unit3(v), u)*v;
}

num_type angle3(const vec3 &u, const vec3 &v)
{
    num_type x = dot3(u, v);
    if (x < -1.0) return M_PI;
    if (x > 1.0) return 0.0;
    return std::acos(x);
}

vec3 operator*(const mat3 &M, const vec3 &v)
{
    return {dot3(M[0], v), dot3(M[1], v), dot3(M[2], v)};
}

mat3 transpose(const mat3 &M)
{
    vec3 l1 = {M[0][0],M[1][0],M[2][0]};
    vec3 l2 = {M[0][1],M[1][1],M[2][1]};
    vec3 l3 = {M[0][2],M[1][2],M[2][2]};
    return {l1, l2, l3};
}

vec3 zaxis(const mat3 &M)
{
    return {M[0][2], M[1][2], M[2][2]};
}

mat3 rotMatrix(const vec3 &axis, num_type angle)
{
    num_type cosTheta = std::cos(angle);
    num_type sinTheta = std::sin(angle);
    num_type t = 1.0 - cosTheta;
    vec3 l1 = {t * axis[0]*axis[0] + cosTheta,
            t * axis[0] * axis[1] - sinTheta * axis[2],
            t * axis[0] * axis[2] + sinTheta * axis[1]
            };
    vec3 l2 = {t * axis[0] * axis[1] + sinTheta * axis[2],
            t * axis[1]*axis[1] + cosTheta,
            t * axis[1] * axis[2] - sinTheta * axis[0]
            };
    vec3 l3 = {t * axis[0] * axis[2] - sinTheta * axis[1],
            t * axis[1] * axis[2] + sinTheta * axis[0],
            t * axis[2]*axis[2] + cosTheta
            };
    return {l1,l2,l3};
}

mat4 makeOpenGLMatrix(const mat3 R, const vec3 p)
{
    return {R[0][0], R[1][0], R[2][0], 0.0,
            R[0][1], R[1][1], R[2][1], 0.0,
            R[0][2], R[1][2], R[2][2], 0.0,
            p[0], p[1], p[2], 1.0};
}


