# scratch2ros
from https://github.com/Affonso-Gui/scratch3-ros-vm

## Setup
```sh
sudo apt-get install aptitude
sudo aptitude install npm

curl -fsSL https://deb.nodesource.com/setup_lts.x | sudo -E bash -
sudo apt-get install -y nodejs
```

## How to use

```sh
# set up scratch-gui
git clone --depth 1 https://github.com/LLK/scratch-gui.git
cd scratch-gui
npm install

# set up ros
git clone https://github.com/noriokun4649/scratch2ros.git
sh scratch2ros/install.sh
```

## How to run
```sh
npm start # cd is scratch-gui
```
