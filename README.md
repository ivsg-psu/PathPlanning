# Path Planning

<img src="https://github.com/ivsg-psu/PathPlanning/images/PathPlanning_cropped.jpg" alt="Path Planning" width="1280" height="377">

Welcome to the path-planning work for the team. Code in this area spans topics from geometric methods for maps, to kinematics, to map creation and path planning, path manipulation and averaging, and path navigation with rapid replanning.

<img src="https://github.com/ivsg-psu/PathPlanning/images/PathPlanning_org.png" alt="Path Planning Org Chart" width="1280" height="150">

<!-- TABLE OF CONTENTS -->
## Table of Contents

<details open>
  <summary> Click to see/unsee </summary>

  <ol>
    <li>
      <a href="#geometric-tools">Geometric Tools</a>
      This includes common geometric calculations associated with path determination such as inner and outer tangents to circles, visibility arcs of
      circles, etc.
    </li>
    <li>
        <a href="#path-kinematics">Path Kinematics</a>
        These are functions that determine the fastest velocity that can be achieved in a segment based on kinematics, for example given a speed limit,
        accel, and decel limits.
    </li>
    <li>
      <a href="#map-tools">Map Tools</a>
           This includes map generation and object shrinkage and representations, map visibility calculations to plan paths, path cost analysis, path planners, and map scope constraining as
           function of path planning.
      <!--ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul-->
    </li>
    <li>
      <a href="#gridfree-path-planners">Gridfree Path Planners</a>
           This includes path planning algorithms that do not use an occupancy grid, e.g. polytope bounding areas. Path planners include Djkstra's algorithm, A-star, constrained A-star, RRTs, and potential fields. Also included here are the high-speed steady-state planners which can convert point-to-point paths into high-speed paths.
    </li>
    <li>
      <a href="#gridbased-path-planners">Gridbased Path Planners</a>
           This includes path planning algorithms that use an occupancy grid, e.g. grid-based methods. Path planners include Potential Field, A-star, D-
           star, Wildfire, and RRTs.
    </li>
    <li>
        <a href="#path-tools">Path Tools</a>
        These are functions that operate on paths after they are generated, for example calculating path offsets, intersections between paths, averages
        of multiple paths, random paths about a given path, smoothing a path, snapping points to a path, converting s-coordinates to xyz coordinates, and
        click-to-draw functions for paths.
    </li>
    <li>
        <a href="#path-navigation">Path Navigation</a>
        These are functions that - given a path - determine distortions around the path to consider as alternatives in the case of obstacles.
    </li>
    <li>
        <a href="#tips-and-tricks">Tips and Tricks</a>
        This includes common plotting utilities for paths, geometric transforms for pose calculations, etc.
    </li>
  </ol>
</details>

<a href="#path-planning">Back to top</a>

***

## Geometric Tools

This includes common geometric calculations associated with path determination such as inner and outer tangents to circles, visibility arcs of
circles, etc.

<!-- GEOMETRIC TOOLS -->
<details closed>
  <summary> Click to see/unsee </summary>
  <ul>
    <li>
      <a href="https://github.com/ivsg-psu/PathPlanning_GeomTools_GeomClassLibrary/wiki">
      The Geometry Class Library
      <br>
      <img src="https://github.com/ivsg-psu/PathPlanning/images/PathPlanning_GeomTools_GeomClassLibrary.jpg" height="200" width="200" >
      </a>
      <br>
      This includes many of the core functions for calculating path geometry, including most of the functions below.
    </li>
    <li>
      <a href="https://github.com/ivsg-psu/PathPlanning_GeomTools_3DGeomClassLibrary/wiki">
      PathPlanning_GeomTools_3DGeomClassLibrary - The 3D Geometry Class Library
      <br>
      <img src="https://github.com/ivsg-psu/PathPlanning/images/PathPlanning_GeomTools_3DGeomClassLibrary.jpg" height="200" width="200" >
      </a>
      <br>
      This includes core functions for calculating 3D geometric features such as intersections of segments with plane patches.
    </li>
    <li>
      <a href= "https://github.com/ivsg-psu/PathPlanning_GeomTools_AlignCoordinates">
      PathPlanning_GeomTools_AlignCoordinates - Regression methods to align coordinate systems to each other
      <br>
      <img src="https://github.com/ivsg-psu/PathPlanning/images/PathPlanning_GeomTools_AlignCoordinates.jpg" height="200" width="200" >
      </a>
      <br>
      This repo methods that use regression fitting to determine correspondence between two different coordinate systems.
    </li>
    <li>
      <a href= "https://github.com/ivsg-psu/PathPlanning_GeomTools_SlopeInterceptFromNPoints/wiki">
      Determining the slope and intercept from N points
      <br>
      <img src="https://github.com/ivsg-psu/PathPlanning_GeomTools_SlopeInterceptFromNPoints/blob/master/Figures/FIG_Example2_ManyPoints.jpg"
      height="100" width="100" >
      </a>
      <br>
      This repo hosts a simple function that determines the best-fit line and slope from N points using least-squares regression.
    </li>
    <li>
      <a href= "https://github.com/ivsg-psu/PathPlanning_GeomTools_PlotEllipseUsingAffineTransform/wiki">
      Plotting an Ellipse using an Affine Transform on a Circle
      <br>
      <img src="https://github.com/ivsg-psu/PathPlanning_GeomTools_PlotEllipseUsingAffineTransform/blob/master/plotEllipse_AffineTransform%20Thumbnail.jpg" height="100" width="100" >
      </a>
      <br>
      This repo shows a neat plotting tool to represent an ellipse. It does the plotting quickly by converting a circle representation into an ellipse
      using an affine transform.
    </li>
    <li>
      <a href= "https://github.com/ivsg-psu/PathPlanning_GeomTools_FindPointsInsideEllipse/wiki">
      Calculating which points are inside or outside of an ellipse bound
      <br>
      <img src="https://github.com/ivsg-psu/PathPlanning_GeomTools_FindPointsInsideEllipse/blob/master/icon_FindPointsInsideEllipse.png" height="100"
      width="150" >
      </a>
      <br>
      This repo hosts two functions that check to see if points are within an ellipse boundary, useful for constraining which part of a map has to be
      included for search.
    </li>
    <li>
      <a href= "https://github.com/ivsg-psu/PathPlanning_GeomTools_PolytopeUnitNormalCalculation/wiki">
      Calculating the Unit Normal Vectors for Each Surface of a Polytope
      <br>
      <img src="https://github.com/ivsg-psu/PathPlanning_GeomTools_PolytopeUnitNormalCalculation/blob/master/Single_polygon.png" height="100" width="100"
      >
      </a>
      <br>
      This repo hosts a function for determining and plotting the unit normal vectors for the edges of given polytopes.    </li>
  </ul>
</details>

<a href="#path-planning">Back to top</a>

***

## Path Kinematics

These are functions that determine the fastest velocity that can be achieved in a segment based on kinematics, for example given a speed limit, accel, and decel limits.

<!-- PATH KINEMATICS -->
<details closed>
  <summary> Click to see/unsee </summary>
  <ul>
    <li>
      <a href="https://github.com/ivsg-psu/PathPlanning_PathKinematics_PathKinematicsClass/wiki">
      PathPlanning_PathKinematics_PathKinematicsClass
      <br>
      <img src="https://github.com/ivsg-psu/PathPlanning/images/PathPlanning_PathKinematics_PathKinematicsClass.jpg" height="200" width="200" >
      </a>
      <br>
      This repo hosts the class library for path kinematics, which includes most of the functions listed below in class format.
    </li>
    <li>
      <a href="https://github.com/ivsg-psu/PathPlanning_GeomTools_FastestVelocityOverSegments/wiki">
      PathPlanning_GeomTools_FastestVelocityOverSegments
      <br>
      <img src="https://github.com/ivsg-psu/PathPlanning_GeomTools_FindCircleRadius/blob/master/pathplanning.jpg" height="200" width="200" >
      </a>
      <br>
      This repo hosts functions to calculate the fastest speeds that a vehicle can hit while traversing fixed-length segments within a path.
    </li>
    <li>
      <a href= "https://github.com/ivsg-psu/PathPlanning_GeomTools_FastestVelocityConstantAccelerationDeceleration/wiki">
      PathPlanning_GeomTools_FastestVelocityConstantAccelerationDeceleration
      <br>
      <img src="https://github.com/ivsg-psu/PathPlanning_GeomTools_FastestVelocityConstantAccelerationDeceleration/blob/master/fcn_costs_FastestVelFromConstAccelDecel.m%20Thumbnail.jpg" height="100" width="150" >
      </a>
      <br>
      This repo hosts a function to determines the maximum velocity that can be achieved in a set of connected segments given an initial velocity, final
      velocity, as well as acceleration and deceleration limits. It also returns the minimum time.
    </li>
    <li>
      <a href="https://github.com/ivsg-psu/PathPlanning_GeomTools_TargetVelocityBetweenSegmentsWithSpeedLimits/wiki">
      PathPlanning_GeomTools_TargetVelocityBetweenSegmentsWithSpeedLimits
      <br>
      <img src="https://github.com/ivsg-psu/PathPlanning_GeomTools_TargetVelocityBetweenSegmentsWithSpeedLimits/blob/master/TargetVelBetweenSegmentsWithSpeedLimits%20Thumbnail.jpg" height="100" width="150" >
      </a>
      <br>
      This is a function that determines the fastest velocity that can be achieved in a set of connected segments given a speed limit at the endpoints of
      the segments, as well as acceleration and deceleration limits.  
    </li>
    <li>
      <a href= "https://github.com/ivsg-psu/PathPlanning_Offroad_ObstacleField_FastestTraversal/wiki">
      PathPlanning_Offroad_ObstacleField_FastestTraversal
      <br>
      <img src="https://github.com/ivsg-psu/PathPlanning_GeomTools_SlopeInterceptFromNPoints/blob/master/Figures/FIG_Example2_ManyPoints.jpg"
      height="100" width="100" >
      </a>
      <br>
      This repo hosts a function to determine the maximum velocity that can be achieved in an off-road environment given the obstacles, the friction
      limits, and vehicle limitations (steering, turn radius, acceleration, size, etc.). This is Vern Grunning thesis work.
    </li>
    <li>
      <a href= "https://skat.ihmc.us/rid=1K7WQT337-XQJP8C-1YHM/Randomized%20Kinodynamic%20Planning.pdf">
      Websites or papers on Kinematic planning
      <br>
      <img src="https://github.com/ivsg-psu/PathPlanning_GeomTools_PlotEllipseUsingAffineTransform/blob/master/plotEllipse_AffineTransform%20Thumbnail.jpg" height="100" width="100" >
      </a>
      <br>
      Randomized Kinodynamic Planning by Steven LaValle and James Kuffer, International Journal of Robotics Research, 2001.
    </li>
  </ul>
</details>

<a href="#path-planning">Back to top</a>

***

## Map Tools

This includes map generation, map visibility calculations to plan paths, path cost analysis, path planners, and map scope constraining as function of path planning.

<!-- MAP TOOLS -->
<details closed>
  <summary> Click to see/unsee </summary>
  <ul>
    <li>
      <a href="https://github.com/ivsg-psu/PathPlanning_MapTools_MapGenClassLibrary/wiki">
      PathPlanning_MapTools_MapGenClassLibrary
      <br>
      <img src="https://github.com/ivsg-psu/PathPlanning/images/PathPlanning_MapTools_MapGenClassLibrary.jpg" height="200" width="200" >
      </a>
      <br>
      This repo functions used to generate obstacle field maps via polytopes in a mesh-free representation.
    </li>
    <li>
      <a href="https://github.com/ivsg-psu/PathPlanning_MapTools_VertexSkeleton">
      PathPlanning_MapTools_VertexSkeleton
      </a>
      <br>
      This repo holds functions used to calculate the vertex skeleton of polytopes, namely where vertices move if the edges are shrunk inwards. The result is quite similar to the medial axis calculation but is simpler to calculate as there is one exact answer rather than an approximation, and all connections are line segments whereas medial axis causes parabolas and planes.
    </li>
    <li>
      <a href="https://github.com/ivsg-psu/PathPlanning_MapTools_VisibilityCalculations_VisibilityTesting/wiki">
      PathPlanning_MapTools_VisibilityCalculations_VisibilityTesting
      <br>
      <img src="https://github.com/ivsg-psu/PathPlanning/images/PathPlanning_MapTools_MapGenClassLibrary.jpg" height="200" width="200" >
      </a>
      <br>
      This repo tests visibility calculations and functions.
    </li>
  </ul>
</details>

<a href="#path-planning">Back to top</a>

***

## Gridfree Path Planners

This includes path planning algorithms that do not use an occupancy grid, e.g. polytope bounding areas. Path planners include Djkstra's
algorithm, A-star, constrained A-star, RRTs, and potential fields.

<!-- GRIDFREE PATH PLANNERS -->
<details closed>
  <summary> Click to see/unsee </summary>
  <ul>
    <li>
      <a href="https://github.com/ivsg-psu/PathPlanning_GridFreePathPlanners_VisibilityGraph">
      PathPlanning_GridFreePathPlanners_VisibilityGraph
      <br>
      <img src="https://github.com/ivsg-psu/PathPlanning/images/PathPlanning_GridFreePathPlanners_VisibilityGraph.png" height="200" width="200" >
      </a>
      <br>
      Tools to calculate the visibility graph for a grid-free map of polytopes.
    </li>
    <li>
      <a href="https://github.com/ivsg-psu/PathPlanning_GridFreePathPlanners_GridFreePathPlanLibrary/wiki">
      PathPlanning_GridFreePathPlanners_GridFreePathPlanLibrary
      <br>
      <img src="https://github.com/ivsg-psu/PathPlanning/images/PathPlanning_GridFreePathPlanners_GridFreePathPlanLibrary.jpg" height="200" width="200" >
      </a>
      <br>
      This is the GridFree path plan class library listing all functions within this class. It includes many of the functions listed below including
      comprehensive test cases. This is the primary library for grid-free path planning operations in MATLAB.
    </li>
    <li>
      <a href="https://github.com/ivsg-psu/PathPlanning_GridFreePathPlanners_HighSpeedSteadyState">
      PathPlanning_GridFreePathPlanners_HighSpeedSteadyState
      <br>
      <img src="https://github.com/ivsg-psu/PathPlanning/images/PathPlanning_GridFreePathPlanners_HighSpeedSteadyState.jpg" height="200" width="200" >
      </a>
      <br>
      This is the grid-free path planning codes that convert point-to-point paths into high-speed paths assuming steady-state high-speed dynamics (e.g. constant acceleration). This is the main repo for Michael Pagan's work.
    </li>
  </ul>
</details>

<a href="#path-planning">Back to top</a>

***

## Gridbased Path Planners

This includes path planning algorithms that use an occupancy grid, e.g. grid-based methods. Path planners include Potential Field, A-star, D-star, Wildfire, and RRTs.

<!-- GRIDBASED PATH PLANNERS -->
<details closed>
  <summary> Click to see/unsee </summary>
  <ul>
    <li>
      <a href="https://github.com/ivsg-psu/PathPlanning_GridBasedPathPlanners_GridBasedPathPlanLibrary/wiki">
      PathPlanning_GridBasedPathPlanners_GridBasedPathPlanLibrary
      <br>
      <img src="https://github.com/ivsg-psu/PathPlanning/images/PathPlanning_GridBasedPathPlanners_GridBasedPathPlanLibrary.jpg" height="200" width="200" >
      </a>
      <br>
      This is the GridFree path plan class library listing all functions within this class. It includes many of the functions listed below including
      comprehensive test cases. This is the primary library for grid-free path planning operations in MATLAB.
    </li>
  </ul>
</details>

<a href="#path-planning">Back to top</a>

***

## Path Tools

These are functions that operate on paths after they are generated, for example calculating path offsets, intersections between paths, averages of multiple paths, random paths about a given path, smoothing a path, snapping points to a path, converting s-coordinates to xyz coordinates, and click-to-draw functions for paths.

<!-- PATH TOOLS -->
<details closed>
  <summary> Click to see/unsee </summary>
  <ul>
    <li>
      <a href="https://github.com/ivsg-psu/PathPlanning_PathTools_PathClassLibrary/wiki">
      PathPlanning_PathTools_PathClassLibrary
      <br>
      <img src="https://github.com/ivsg-psu/PathPlanning/images/PathPlanning_PathClassLibrary.jpg" height="200" width="200" >
      </a>
      <br>
      This is the Path class library and listing of all functions within this class. It includes many of the functions listed below including
      comprehensive test cases. This is the primary library for path operations in MATLAB.
    </li>
    <li>
      <a href="https://github.com/ivsg-psu/PathPlanning_GeomTools_CalcParallelCurves/wiki">
      PathPlanning_GeomTools_CalcParallelCurves
      <br>
      <img src="https://github.com/ivsg-psu/PathPlanning/images/PathPlanning_PathTools_CalcParallelCurves.jpg" height="200" width="200" >
      </a>
      <br>
      This is a function that calculates parallel curves to a given curve in 2D or 3D, for example the inner and outer parallel curves to a 2D curve.
      Also, the normal vector of each point is outputted. If we want to interpolate the curve before calculating the parallel, please refer to the file:
      interparc.m. The function can interpolate new points at any fractional point along the curve defined by a list of points in 2 or more dimensions.
      The curve may be defined by any sequence of non-replicated points. The interpolation method may be any of 'linear', 'spline', or 'pchip', 'csape'.
    </li>
    <li>
      <a href="https://github.com/ivsg-psu/PathPlanning_PathTools_SnapQueryPointToPath/wiki">
      PathPlanning_PathTools_SnapQueryPointToPath
      <br>
      <img src="https://github.com/ivsg-psu/PathPlanning/images/PathPlanning_PathTools_SnapQueryPointToPath.jpg" height="200" width="200" >
      </a>
      <br>
      This is a repo for the function: fcn_pathtools_snap_point_onto_path. This finds the location on a path that is closest to a given point, e.g.
      snapping the point onto the path
    </li>
    <li>
      <a href="https://github.com/ivsg-psu/PathPlanning_PathTools_FindPathSXYSegmentContainingGivenSBounds/wiki">
      PathPlanning_PathTools_FindPathSXYSegmentContainingGivenSBounds
      <br>
      <img src="https://github.com/ivsg-psu/PathPlanning/images/PathPlanning_PathTools_FindPathSXYSegmentContainingGivenSBounds.jpg" height="200" width="200" >
      </a>
      <br>
      This is a repo for the function: fcn_pathtools_FindPathSegmentWithinSBounds. This function finds a portion of a SXY-type path that contains the
      user-given s_coordinates, starting from s_coord_start to s_coord_end.
    </li>
    <li>
      <a href="https://github.com/ivsg-psu/PathPlanning_PathTools_GetUserInputPath/wiki">
      PathPlanning_PathTools_GetUserInputPath
      <br>
      <img src="https://github.com/ivsg-psu/PathPlanning/images/PathPlanning_PathTools_GetUserInputPath.jpg" height="200" width="200" >
      </a>
      <br>
      This is a repo for the function: fcn_pathtools_getUserInputPath.m A function for the user to click on the figure to generate XY path until the user
      hits the "return" key.
    </li>
    <li>
      <a href="https://github.com/ivsg-psu/PathPlanning_PathTools_PlotPath/wiki">
      PathPlanning_PathTools_PlotPath
      <br>
      <img src="https://github.com/ivsg-psu/PathPlanning/images/PathPlanning_PathTools_PlotPath.jpg" height="200" width="200" >
      </a>
      <br>
      This is a repo for the function: fcn_pathtools_getUserInputPath.m .  This is a function that plots either XY or SXY type paths.
    </li>
  </ul>
</details>

TO ADD:
extraction of individual lanes for multi-lane roads with lane-changes, and determination of decision points given trajectories
Move PathPlanning_GeomTools_CalcParallelCurves to PathPlanning_PathTools_CalcParallelCurves

<a href="#path-planning">Back to top</a>

***

## Path Navigation

These are functions that - given a path - determine distortions around the path to consider as alternatives in the case of obstacles.

<!-- PATH NAVIGATION -->
<details closed>
  <summary> Click to see/unsee </summary>
  <ul>
    <li>
      <a href="https://github.com/ivsg-psu/PathPlanning_PathNavigation_PathNavClassLibrary/wiki">
      PathPlanning_PathNavigation_PathNavClassLibrary
      <br>
      <img src="https://github.com/ivsg-psu/PathPlanning/images/PathPlanning_PathNavigation_PathNavClass.jpg" height="200" width="200" >
      </a>
      <br>
      This is the Path Navigation class library and listing of all functions within this class. (In development)
    </li>
  </ul>
</details>

<a href="#path-planning">Back to top</a>

TO DO:
Rename PathPlanning_PathNavigation_PathNavClass

***

## Tips and Tricks

This includes common tips and tricks such as plotting utilities for paths, geometric transforms for pose calculations, etc.

<!-- TIPS AND TRICKS -->
<details closed>
  <summary> Click to see/unsee </summary>

Here's a quick example of MATLAB method to plot the color of an XY plot using data to represent the change in color. This is useful to plot velocity or fuel usage versus XY position:

<img src="https://github.com/ivsg-psu/PathPlanning/images/PathPlanning_TipsAndTricks_ExampleColorBar.png" height="400" width="600" >  

``` Matlab
max_colorbar = 1;

% Create a multi-colored plot
x = 0:0.01:10;
y = sin(x);
z = zeros(size(x));
col = cos(x);  % This is the color, vary with velocity
h_surf = surface([x;x],[y;y],[z;z],[col;col],...
    'facecol','no',...
    'edgecol','interp',...
    'linew',2);
% [max_vel,ind_max_vel] = max(all_velocities);
% text(all_distances(ind_max_vel),max_vel,path_type)
caxis([0 max_colorbar]);
colorbar
```

### Online resources

See [Peter Corke's Spatial Math toolbox](https://petercorke.com/toolboxes/spatial-math-toolbox/) for a great resource. As per his website:

> This toolbox contains functions and classes to represent orientation and pose in 2D and 3D (SO(2), SE(2), SO(3), SE(3)) as matrices, quaternions, twists, triple angles, and matrix exponentials. The Toolbox also provides functions for manipulating and converting between datatypes such as vectors, homogeneous transformations and unit-quaternions which are necessary to represent 3-dimensional position and orientation.

</details>

<a href="#path-planning">Back to top</a>

This page is maintained primarily by S. Brennan. Any comments, please email <sbrennan@psu.edu>.
