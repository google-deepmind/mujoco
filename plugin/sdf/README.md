<h1>
  <a href="#"><img alt="MuJoCo" src="../../banner.png" width="100%"/></a>
</h1>

## Signed distance function (SDF) plugins

These are first-party plugins that implement implicit geometries using SDFs. They can be applied to **geoms** and
**meshes** (in the **asset** section). Sample models can be found in [this folder](../../model/plugin/sdf/).

### Bolt

Implemented in [bolt.cc](bolt.cc). Example usage in [nutbolt.xml](../../model/plugin/sdf/nutbolt.xml).

This plugin implements a bolt with a hexagonal head, similar to https://www.shadertoy.com/view/XtffzX.

Parameters:

   - `radius` [m]: bolt radius (default `0.26`).

### Bowl

Implemented in [bowl.cc](bowl.cc). Example usage in [bowl.xml](../../model/plugin/sdf/bowl.xml).

The plugin implements a cut hollow sphere from https://www.shadertoy.com/view/7tVXRt.

Parameters:

   - `height` [m]: location of the cut plane (default `0.4`).
   - `radius` [m]: radius of the sphere (default `1`).
   - `thickness` [m]: thickness of the bowl (default `0.02`).

### Gear

Implemented in [gear.cc](gear.cc). Example usage in [gear.xml](../../model/plugin/sdf/gear.xml).

The plugin implements a 3D extrusion of the 2D gear geometry from https://www.shadertoy.com/view/3lG3WR.

Parameters:

   - `alpha` [m]: initial angle of rotation of the gear (default `0`).
   - `diameter` [m]: gear diameter (default `2.8`).
   - `teeth` []: number of teeth (default `25`).

### Nut

Implemented in [nut.cc](nut.cc). Example usage in [nutbolt.xml](../../model/plugin/sdf/nutbolt.xml).

This plugin implements a hexagonal nut identical to the bolt head from https://www.shadertoy.com/view/XtffzX.

Parameters:

   - `radius` [m]: nut radius (default `0.26`).

### Torus

Implemented in [torus.cc](torus.cc). Example usage in [torus.xml](../../model/plugin/sdf/torus.xml).

This plugin implements a torus.

Parameters:

  - `radius1` [m]: major radius (default `0.35`).
  - `radius1` [m]: minor radius (default `0.15`).

### SdfLib

Implemented in [sdflib.cc](sdflib.cc). Example usage in [cow.xml](../../model/plugin/sdf/cow.xml).

This plugin uses the library [SdfLib](https://github.com/UPC-ViRVIG/SdfLib) to compute a voxel-based approximation of a
user-specified mesh. The mesh can be arbitrary and not necessarily convex. This offers an alternative to
convex-decomposed meshes. The performance is likely to be slower than that of analytic SDFs, since a cubic
approximation has to be evaluated on the convex grid. However, the SDF generation is done automatically, simplifying the
task of creating an SDF, which can be difficult for complex shapes.

### How to make your own SDF

Create your `MySDF.h` and `MySDF.cc` files in the SDF folder, where this README is located. Implement your SDF using the
following interface:

```
struct MySDFAttribute {
  static constexpr int nattribute =
  /* insert the number of attributes */;
  static constexpr char const* names[nattribute] =
  /* an array of attributes with the same order as the attribute array in your SDF class */;
  static constexpr mjtNum defaults[nattribute] =
  /* an array of default values for your attributes */;
};

class MySDF {
 public:
  // creates a new MySDF instance or returns null on failure.
  static std::optional<MySDF> Create(const mjModel* m, mjData* d, int instance);
  MySDF(MySDF&&) = default;
  ~MySDF() = default;

  // functions that return the SDF and its gradient at a query point
  mjtNum Distance(const mjtNum point[3]) const;
  void Gradient(mjtNum grad[3], const mjtNum point[3]) const;

  // a call to this needs to be added to register.cc
  static void RegisterPlugin();

  // an array of attributes with the same order as in the struct above
  mjtNum attribute[MySDFAttribute::nattribute];

 private:
  MySDF(const mjModel* m, mjData* d, int instance);
};
```







