#include "vec3.h"

#include <iostream>
#include <cassert>
#include <ode/ode.h>


// Empty constructor
vec3::vec3() {
    for(int i=0; i<4; ++i)
        vec[i] = 0.0;
} 

// Constructors based on ODE dVector3 objects
vec3::vec3(dVector3 _vec) {
    for(int i=0; i<4; ++i)
        vec[i] = _vec[i];
}

vec3::vec3(const dVector3& _vec) { 
    for(int i=0; i<4; ++i)
        vec[i] = _vec[i]; 
}

// Constructor based on each component
vec3::vec3(dReal vec0, dReal vec1, dReal vec2) {
    vec[0] = vec0;
    vec[1] = vec1;
    vec[2] = vec2;
    vec[3] = 1.0;
}

// Get at index operator
const dReal& vec3::operator[](std::size_t index) const {
    if(index < 0 && index > 3) {
        std::cerr<<"Error: Try to access vec3["<<index<<"]"<<std::endl;
        assert(false);
        abort();
    }
    return vec[index];
}

// Set at index operator
dReal& vec3::operator[](std::size_t index) {
    if(index < 0 && index > 3) {
        std::cerr<<"Error: Try to access vec3["<<index<<"]"<<std::endl;
        assert(false);
        abort();
    }
    return vec[index];
}

vec3 vec3::operator+=(const vec3& v) {
    return { vec[0]+v[0], vec[1]+v[1], vec[2]+v[2] };
}

dReal norm(const vec3 &v) {
    return std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

vec3 operator-(const vec3 &v) {
    return {-v[0], -v[1], -v[2]};
}

vec3 operator+(const vec3 &u, const vec3 &v) {
    return {u[0]+v[0], u[1]+v[1], u[2]+v[2]};
}

vec3 operator-(const vec3 &u, const vec3 &v) {
    return {u[0]-v[0], u[1]-v[1], u[2]-v[2]};
}

vec3 operator*(dReal s, const vec3 &v) {
    return {s*v[0], s*v[1], s*v[2]};
}

vec3 operator/(const vec3 &v, dReal s) {
    return {v[0]/s, v[1]/s, v[2]/s};
}

vec3 unit3(const vec3 &v) {
    return v/norm(v);
}

dReal dist3(const vec3 &u, const vec3 &v) {
    return norm(u-v);
}

dReal dot3(const vec3 &u, const vec3 &v) {
    return u[0]*v[0] + u[1]*v[1] + u[2]*v[2];
}

vec3 cross(const vec3 &u, const vec3 &v) {
    return {u[1]*v[2] - u[2]*v[1],
            u[2]*v[0] - u[0]*v[2],
            u[0]*v[1] - u[1]*v[0]};
}

vec3 project3(const vec3 &u, const vec3 &v) {
    return dot3(unit3(v), u)*v;
}

dReal angle3(const vec3 &u, const vec3 &v) {
    dReal x = dot3(u, v);
    if (x < -1.0) return M_PI;
    if (x > 1.0) return 0.0;
    return std::acos(x);
}


