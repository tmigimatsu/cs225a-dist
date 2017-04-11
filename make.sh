set -e

# Build
mkdir -p build
cd build
cmake ..
make -j4
cd ..

# Download resource files
cd bin
if [ -f "hw1" ]; then
	cd resources/hw1
	if [ ! -d "puma_graphics" ]; then
		curl -L http://cs.stanford.edu/groups/manips/teaching/cs225a/resources/puma_graphics.zip -o puma_graphics.zip
		unzip puma_graphics.zip
		rm puma_graphics.zip
	fi
	cd ../..
fi
cd ..

# Insert helper scripts into bin directory
cd bin

# Make script
cat <<EOF > make.sh
cd ..
mkdir -p build
cd build
cmake ..
make -j4
cd ../bin
EOF
chmod +x make.sh

# Run hw0
if [ -f "hw0" ]; then
	cat <<EOF > run_hw0.sh
# This script calls ./visualizer and ./hw0 simultaneously.
# All arguments to this script will be passed onto ./hw0

trap 'kill %1' SIGINT
./visualizer resources/hw0/world.urdf resources/hw0/RRPbot.urdf RRPbot & ./hw0 "\$@"
EOF
	chmod +x run_hw0.sh
fi

# Run generic controller
if [ -f "hw1" ]; then
	cat <<EOF > run_controller.sh
if [ "\$#" -lt 4 ]; then
	cat <<EOM
This script calls ./visualizer, ./simulator, and the specified controller simultaneously.
All the arguments after the controller will be passed directly to it.

Usage: sh run.sh <controller-executable> <path-to-world.urdf> <path-to-robot.urdf> <robot-name> <extra_controller_args>
EOM
else
	trap 'kill %1; kill %2' SIGINT
	./simulator \$2 \$3 \$4 > simulator.log & ./visualizer \$2 \$3 \$4 > visualizer.log & ./"\$@"
	# ./visualizer \$2 \$3 \$4 & ./simulator \$2 \$3 \$4 & ./"\$@"
fi
EOF
	chmod +x run_controller.sh
fi

cd ..
