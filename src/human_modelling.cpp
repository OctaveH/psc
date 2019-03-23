#include <iostream>

#include "../include/util.h"

void test1() // test partie 1 opération sur les vecteurs
{
    num_type n= 1.5;
    vec3 u={1, 1, 1};
    vec3 v={2, 3, 4};
    mat3 A={u,u,u};

    std :: cout << "u=" <<u << ", v=" << v <<", n=" << n << std::endl;

    vec3 a=u+v;
    vec3 b=u-v;
    vec3 c=-u;
    vec3 d=n*v;
    vec3 e=v/n;
    vec3 f=unit3(u);
    vec3 g=cross(u, v);
    vec3 h=project3(u, v);
    vec3 i=A*v;
    vec3 j=zaxis(A);

    num_type s=sign(n);
    num_type m=norm(u);
    num_type t=dot3(u, v);
    num_type z=angle3(u, v);

    mat3 B=transpose(A);
    mat3 C=rotMatrix(u, n);

    mat4 W=makeOpenGLMatrix(A, v);

    std::cout << "test addition u+v: " << a << std::endl;
    std::cout << "test soustraction u-v: " << b << std::endl;
    std::cout << "test opposé -u: " << c << std::endl;
    std::cout << "test multiplication par un scalaire n*v: " << d << std::endl;
    std::cout << "test division par un scalaire v/n: " << e << std::endl;
    std::cout << "test unit3(u): " << f << std::endl;
    std::cout << "test cross(u,v): " << g << std::endl;
    std::cout << "test project3(u,v): " << h << std::endl;
    std::cout << "test A*v, multiplication par une matrice: " << i << std::endl;
    std::cout << "test zaxis(A): " << j << std::endl;

    std::cout << "sign(n): " << s << ",  norm(u): " << m << ", dot3(u,v): " << t << ", angle3(u,v): " << z << std::endl;
}

int main()
{
    test1();
    return 0;
}
