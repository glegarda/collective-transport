# collective-transport
Automatic design of control software for multi-object collective transport with robot swarms using genetic programming.
For an overview of the implementation and results, see

G. Legarda Herranz, S. Hauert, and S. Jones (2022). “Decentralised negotiation for multi-object collective transport with robot swarms,” in _2022 IEEE International Conference on Autonomous Robot Systems and Competitions (ICARSC)_, P. Fonseca, L. Louro, P. Neto, and R. Ventura, Eds., doi: [10.1109/ICARSC55462.2022.9784801](https://doi.org/10.1109/ICARSC55462.2022.9784801)

## Build
The following instructions should be followed to build the contents of the repository in an Ubuntu 20.04 machine. To build for macOS, see `macOS.md`.
1. Install dependencies
```
sudo apt install cmake build-essential libglfw3-dev libxinerama-dev libxcursor-dev libxi-dev libzmq3-dev libboost-all-dev libncurses-dev qtbase5-dev libqt5svg5-dev libdw-dev libglew-dev libglm-dev gnuplot
```
2. Install [Box2D](https://github.com/erincatto/box2d)
```
git clone https://github.com/erincatto/box2d.git
cd box2d
mkdir build
cd build
cmake --build .
sudo cmake --build . --target install
```
3. Install [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP)
```
git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
cd BehaviorTree.CPP
mkdir build
cd build
cmake ..
make
sudo make install
```
4. Build [Groot](https://github.com/BehaviorTree/Groot/tree/master/bt_editor)
```
git clone https://github.com/BehaviorTree/Groot.git
cd Groot
git submodule update --init --recursive
mkdir build
cd build
cmake ..
make
```
5. Build
```
git clone https://github.com/glegarda/collective-transport.git
cd collective-transport
./build.sh
```
## Run
Two executables are generated in the `build` directory:
- `src/gp` runs the genetic programming algorithm and produces a report in the current directory.
- `tests/sim` renders a complete run. Run `./tests/sim -h` to view different run options. If no behaviour tree controller is specified using the `-tf <file>` option, the swarm will execute a random walk.
