#ifndef UTIL_H_INCLUDED
#define UTIL_H_INCLUDED

typedef float num_type;
typedef std::array<num_type,3> vec3;
typedef std::array<vec3,3> mat3;
typedef std::array<num_type,16> mat4;

num_type sign(num_type x);

num_type norm(const vec3 &v);

vec3 operator-(const vec3 &v);

vec3 operator+(const vec3 &u, const vec3 &v);

vec3 operator-(const vec3 &u, const vec3 &v);

vec3 operator*(num_type s, const vec3 &v);

vec3 operator/(const vec3 &v, num_type s);

vec3 unit3(const vec3 &v);

num_type dot3(const vec3 &u, const vec3 &v);

vec3 cross(const vec3 &u, const vec3 &v);

vec3 project3(const vec3 &u, const vec3 &v);

num_type angle3(const vec3 &u, const vec3 &v);

vec3 operator*(const mat3 &M, const vec3 &v);

mat3 transpose(const mat3 &M);

vec3 zaxis(const mat3 &M);

mat3 rotMatrix(const vec3 &axis, num_type angle);

mat4 makeOpenGLMatrix(const mat3 R, const vec3 p);

#endif // UTIL_HPP_INCLUDED
