### Building on macOS

git clone https://github.com/glegarda/collective-transport.git
git clone https://github.com/erincatto/box2d.git
git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git

cd box2d
mkdir build; cd build
cmake ..
make
sudo make install

cd ../../BehaviorTree.CPP
mkdir build; cd build
cmake ..
make
sudo make install

brew install glfw
brew install glm

cd ../../collective-transport
mkdir build; cd build
cmake ..
make
