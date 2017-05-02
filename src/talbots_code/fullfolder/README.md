# Talbot's Code
We're leaving things in here and removing / migrating things out piece by piece.

For now, in this folder:

```
mkdir build
cd build
cmake ..
make
cd ..
```

Then copy the Puma assets into the bin directory:
```
mkdir resources
mkdir resources/puma
cp ../../../bin/resources/hw1/Puma.urdf resources/puma/puma.urdf
cp ../../../bin/resources/hw1/Puma.urdf bin/resources/puma.urdf
cp -r ../../../bin/resources/hw1/puma_graphics resources/puma
```

Then, from the `bin` directory, we can run things:

### Run with the haptic device

Run `redis_test` and either `force_sim` (for force feedback) or `haptic_sim` (for no force feedback).
The `*_sim` files require a numerical argument, either `1` or `2` or `3`. The difference is that `1` has only position control, `2` has position and orientation control; `3` doesn't work on `haptic_sim`, while `3` on `force_sim` gives a smaller workspace.

### Run without the haptic device

This is similar, but rather than running `redis_test` we should run `redis_sim.py` (currently in the `talbots_code` directory)
Note that `redis_sim.py` requires the `redis` Python package.

