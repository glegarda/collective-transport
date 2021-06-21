# collective-transport
Collective transport of heavy items with a swarm of industrial robots.   
Developed and tested in Ubuntu 18.04.

## Requirements
The following should be downloaded and installed before attempting to run the code:
- [OpenGL](https://www.opengl.org/)
- [Box2D](https://github.com/erincatto/box2d)
- [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP)
- [Groot](https://github.com/BehaviorTree/Groot/tree/master/bt_editor)

## Build
Run `./build.sh`

## Run
Two executables are generated in the `build` directory:
- `src/gp` runs the GP algorithm and produces a report in the current directory.
- `tests/sim` renders a complete run. The BT controller must have been defined in the source file

## Visualisation
Copy and paste the contents of a BT into the `groot_template.xml` file and open the resulting file in Groot.
