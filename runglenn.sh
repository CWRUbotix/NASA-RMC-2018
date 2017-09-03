
datestring=$(date +%Y_%m_%d_%H_%M_%S)
cd /home/ubuntu/
mkdir -p "logs/$datestring"

cd /home/ubuntu/workspace/NASA-RMC-2017/

# start the things
#cd glennifer/HCI
#nohup /usr/lib/jvm/java-8-openjdk-armhf/jre/bin/java -cp target/hci-1.0-SNAPSHOT.jar com.cwrubotix.glennifer.hci.ModuleMain &> "/home/ubuntu/logs/$datestring/ModuleMainOutput.log" &
#cd ..

cd glennifer
cd robot_state
nohup /usr/lib/jvm/java-8-openjdk-armhf/jre/bin/java -cp target/robot_state-1.0-SNAPSHOT.jar com.cwrubotix.glennifer.robot_state.StateModule &> "/home/ubuntu/logs/$datestring/StateModuleOutput.log" &
cd ..

sleep 2

cd autodrill
nohup /usr/lib/jvm/java-8-openjdk-armhf/jre/bin/java -cp target/autodrill-1.0-SNAPSHOT.jar com.cwrubotix.glennifer.autodrill.AutoDrillModule &> "/home/ubuntu/logs/$datestring/AutoDrillModuleOutput.log" &
cd ..

cd motor_dispatch/src/python
nohup python3 locomotion.py &> "/home/ubuntu/logs/$datestring/LocomotionPyOutput.log" &
cd ../../../..

cd client-cameras
nohup python client-cam-send.py &> "/home/ubuntu/logs/$datestring/CameraSendPyOutput.log" &
cd ..

cd ..

