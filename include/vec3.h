#ifndef VEC3_H
#define VEC3_H

#include <ode/ode.h>

/*
 *  vec3 class: a simple wrapper for the type dVector3 class of the ODE library that supports
 *  simple vector operations. 
 */
class vec3 {
    public:
        dVector3 vec;
        
        // Constructors
        vec3();
        vec3(const dReal *vec);
        vec3(dReal vec0, dReal vec1, dReal vec2);

        // Operators
        const dReal& operator[](std::size_t index) const; // get at index operator
        dReal& operator[](std::size_t index); // set at index operator
        vec3 operator-(); // invert vector (multiply each component by -1)
        vec3 operator+=(const vec3& v); // vector addition-assignment
        vec3 operator-=(const vec3& v); // vector subtraction-assignement 
};

vec3 operator+(const vec3& u, const vec3& v); // standard (component by component) addition
vec3 operator-(const vec3& u, const vec3& v); // standard (component by component) subtracion
vec3 operator*(dReal s, const vec3 &v); // multiplication by scalar
vec3 operator/(const vec3 &v, dReal s); // division by scalar

dReal norm(const vec3 &v); // euclidian norm of the vector
vec3 unit3(const vec3 &v); // normalize vector
dReal dist3(const vec3 &u, const vec3 &v); // euclidian distance between two points
dReal dot3(const vec3 &u, const vec3 &v); // dot product of two vectors
vec3 cross(const vec3 &u, const vec3 &v); // cross product of two vectors
vec3 project3(const vec3 &u, const vec3 &v); // projection of the vector u into v
dReal angle3(const vec3 &u, const vec3 &v); // the angle from u to v

#endif
