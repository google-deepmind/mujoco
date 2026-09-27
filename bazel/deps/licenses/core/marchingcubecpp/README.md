## Marching Cube C++
A public domain header-only marching cube implementation in C/C++ without anything fancy. Only dependencies are cmath 
and vector headers. I could get rid of vector and just use plain arrays, but it is more convenient for my personal 
projects to keep that way, though one could easily modify the implementation.

This work was motivated by this public domain implementation of the marching cube: https://github.com/nsf/mc which gives 
a fully runnable example in OpenGL. I thought it could be simplified and could make a good header-only library.

## Behaviour
Calling MC::marching_cube will create an indexed mesh with vertices and normals defined in their own std::vector's.
By default, data is allocated for vertices, normals and triangle indices. You can change the default allocated 
space using the function MC::setDefaultArraySizes.

The output mesh represents the zero-isosurface of the input scalar field. Coordinates of the vertices will follow
the grid discretization: a 128x128x128 grid will create a mesh embedded in the axis-aligned box from Vec3(0) to Vec3(128).

You can optionally enable double precision by defining MC_CPP_USE_DOUBLE_PRECISION before including the header.

## Use
First define MC_CPP_ENABLE and then include MC.h in your project. Example:
```cpp
#define MC_IMPLEM_ENABLE
#include "MC.h"

int main()
{
	// First compute a scalar field
	const int n = 100;
	MC::MC_FLOAT* field = new MC::MC_FLOAT[n * n * n];
	// [...]
	
	// Compute isosurface using marching cube
	MC::mcMesh mesh;
	MC::marching_cube(field, n, n, n, mesh);

	// Dot whatever you want with the mesh
	// [...]

	return 0;
}
```

You may find a complete example in main.cpp file. Running the program in the repository will output an obj file showing 
the isosurface of a single perlin noise octave.

## Licence
Double Licence, whatever fits you best: Public domain or MIT License.
