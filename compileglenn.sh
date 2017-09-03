
datestring=$(date +%Y_%m_%d_%H_%M_%S)
cd /home/ubuntu
mkdir -p "logs/$datestring"

cd /home/ubuntu/workspace/NASA-RMC-2017/

# copy config
cp -r config glennifer/HCI/
cp -r config glennifer/robot_state
cp -r config glennifer/motor_dispatch/src/python

# compile the things
cd glennifer/HCI
mvn package
cd ..

cd robot_state
mvn package
cd ..

cd autodrill
mvn package
cd ..