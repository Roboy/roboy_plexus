#                                               -*- cmake -*-
#
#  BulletConfig.cmake(.in)
#

# Use the following variables to compile and link against Bullet:
#  BULLET_FOUND              - True if Bullet was found on your system
#  BULLET_USE_FILE           - The file making Bullet usable
#  BULLET_DEFINITIONS        - Definitions needed to build with Bullet
#  BULLET_INCLUDE_DIR        - Directory where Bullet-C-Api.h can be found
#  BULLET_INCLUDE_DIRS       - List of directories of Bullet and it's dependencies
#  BULLET_LIBRARIES          - List of libraries to link against Bullet library
#  BULLET_LIBRARY_DIRS       - List of directories containing Bullet' libraries
#  BULLET_ROOT_DIR           - The base directory of Bullet
#  BULLET_VERSION_STRING     - A human-readable string containing the version

set ( BULLET_FOUND 1 )
set ( BULLET_USE_FILE     "lib/arm-linux-gnueabihf/cmake/bullet/UseBullet.cmake" )
set ( BULLET_DEFINITIONS  "-DBT_USE_DOUBLE_PRECISION" )
set ( BULLET_INCLUDE_DIR  "include/bullet" )
set ( BULLET_INCLUDE_DIRS "include/bullet" )
set ( BULLET_LIBRARIES    "LinearMath-float64;Bullet3Common-float64;BulletInverseDynamics-float64;BulletCollision-float64;BulletDynamics-float64;BulletSoftBody-float64" )
set ( BULLET_LIBRARY_DIRS "lib/arm-linux-gnueabihf" )
set ( BULLET_ROOT_DIR     "/usr" )
set ( BULLET_VERSION_STRING "2.88" )
