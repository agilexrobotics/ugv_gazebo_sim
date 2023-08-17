^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package urdf_geometry_parser
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2020-04-05)
------------------
* Remove tf2 dependency
* Add license
* Bump CMake version to avoid CMP0048 warning
* Cleanup CMakeLists.txt and package.xml
* Contributors: Bence Magyar, Matt Reynolds, Vincent Rousseau

0.0.3 (2017-08-07)
------------------
* add travis config, based on industrial_ci
* Make sure to include urdfdom_compatibility.h.
  This ensures that urdf_geometry_parser will build on all distros
  (including older Debian Jessie).
  Signed-off-by: Chris Lalancette <clalancette@osrfoundation.org>
* Contributors: Bence Magyar, Chris Lalancette, Mathias Lüdtke

0.0.2 (2017-07-04)
------------------
* use urdf typedefs (fix builds on Ubuntu Y, Z and Debian Stretch)
* Contributors: Mikael Arguedas

0.0.1 (2017-07-03)
------------------
* First release of urdf_geometry_parser
* Contributors: Vincent Rousseau
