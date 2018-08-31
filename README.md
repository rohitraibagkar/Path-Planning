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
