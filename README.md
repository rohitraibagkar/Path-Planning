# Path-Planning
The robotics project for path planning with D* and PRM Algorithms to avoid obstacles.

# Files in the repo::

- `PathPlanning.m`
- README.md
- Report Project Assignment 3.pdf
- `project3pathplan.fig`
- `project3pathplan.m`



### `PathPlanning.m`

`PathPlanning.m` is a class defined for data and processes.

Process to get 2D co-ordinates of path from start point to end point using PRM (Probabilistic Road Maps) and D* algorithms with interpolation methods are defined in `PathPlanning.m`


Process to plan, generate and interpolate path from start point to end point using D* algorithm is defined in method `dStarAlgo`
function, from line `446` to `521`



Process to plan, generate and interpolate path from start point to end point using PRM (Probabilistic Road Map) algorithm is
defined in method `prmAlgo` function, from line `523` to `600`

Process to generate and draw 2D trajectory without interpolation is defined in method `NoInterPol` function,
from line `268` to line `287`

Process to generate and draw 2D trajectory using multi-axes trajectory with quintic polynomial (5th order polynomial) interpolation is
defined in method `quinticTpoly` function, from line `289` to line `316`

Process to generate and draw 2D trajectory using multi-axes trajectory with parabolic blend interpolation is
defined in method `parabolicBlend` function, from line `318` to line `345`

Process to generate and draw 2D trajectory using linear segments with parabolic interpolation is
defined in method `onlyLspb` function, from line `347` to line `374`

Process to generate and draw 2D trajectory using quintic polynomial (5th order polynomial) interpolation is
defined in method `onlyTpoly` function, from line `376` to line `403`

Process to generate and draw 2D trajectory using spline interpolation is defined in method `onlySpline` function, from line `405` to line `432`
