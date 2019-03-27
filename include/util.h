#ifndef UTIL_H_INCLUDED
#define UTIL_H_INCLUDED
#include <iostream>
#include <array>
#include <cmath>
#include "GL/gl.h"
#include "GL/glext.h"
#include "GL/glu.h"
#include <ode/ode.h>


typedef dReal num_type; // pour pouvoir facilement passé à un double si besoin
typedef std::array<num_type,3> vec3; // vecteur de position de dim 3
typedef std::array<vec3,3> mat3; // matrice de 3 vecteurs de dim 3, les vecteurs sont des lignes
typedef std::array<num_type,16> mat4; //matrice 4*4

num_type sign(num_type x); //retourne -1 ou +1 selon le signe de x

num_type norm(const vec3 &v); //retourne la norme euclidienne du vecteur passé en argument

vec3 operator-(const vec3 &v); //retourne l'opposé du vecteur passé en argument

vec3 operator+(const vec3 &u, const vec3 &v); //addition terme à terme de vecteurs

vec3 operator-(const vec3 &u, const vec3 &v); // u-v, soustraction terme à terme de vecteurs

vec3 operator*(num_type s, const vec3 &v); //multiplication d'un vecteur par un scalaire

vec3 operator/(const vec3 &v, num_type s); //division d'un vecteur par un scalaire

vec3 unit3(const vec3 &v); //normalise le vecteur retourne, v/norm(v)

num_type dist3(const vec3 &u, const vec3 &v); //calcule la distance entre 2 vecteurs

num_type dot3(const vec3 &u, const vec3 &v); //multiplie le vecteur ligne u par le vecteur colonne v, renvoie un numtype

vec3 cross(const vec3 &u, const vec3 &v); //produit vectoriel entre 2 vecteurs

vec3 project3(const vec3 &u, const vec3 &v); //Returns projection of 3-vector v onto unit 3-vector d

num_type angle3(const vec3 &u, const vec3 &v); //Returns the angle between unit 3-vectors a and b

vec3 operator*(const mat3 &M, const vec3 &v); //Returns the rotation of 3-vector v by 3x3 (row major) matrix m

mat3 transpose(const mat3 &M); //retourne la transposé de M

vec3 zaxis(const mat3 &M); //Returns the z-axis vector from 3x3 (row major) rotation matrix m

mat3 rotMatrix(const vec3 &axis, num_type angle); //Returns the row-major 3x3 rotation matrix defining a rotation around axis by angle.

mat4 makeOpenGLMatrix(const mat3 R, const vec3 p); //Returns an OpenGL compatible (column-major, 4x4 homogeneous) transformation matrix from ODE compatible (row-major, 3x3) rotation matrix r and position vector p

std::ostream& operator<<(std::ostream& s, const vec3& v); //surcharge de l'opérateur <<



#endif // UTIL_HPP_INCLUDED
