# cs225a
Repository for class resources: CS225a experimental robotics

## Installation from Scratch

Please refer to the README at [tmigimatsu/cs225a-dist](https://github.com/tmigimatsu/cs225a-dist/blob/master/README.md) for instructions.

## Linking to an external installation

If you already set up cs225a-dist elsewhere, you can add the SAI2 simulation library to this repo as symlinks. The code exampls below are for bash; if you're on Windows, you should use `mklink /J` as a replacement for `ln -s` and switch the order of the directories.

```
ln -s "/absolute/path/to/sai2-simulation/" ./
```

You will still need to clone the sai2 common library and build:
```
git clone https://github.com/manips-sai/sai2-common.git
cd sai2-common
mkdir build
cd build
cmake ..
make -j4
```

Then you can make the current project:
```
sh make.sh
```

