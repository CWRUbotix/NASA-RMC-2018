
datestring=$(date +%Y_%m_%d_%H_%M_%S)
#cd /home/ubuntu/
#mkdir -p "logs/$datestring"

#cd /home/ubuntu/workspace/NASA-RMC-2017/

# start the things
#cd glennifer/HCI
#nohup /usr/lib/jvm/java-8-openjdk-amd64/jre/bin/java -cp target/hci-1.0-SNAPSHOT.jar com.cwrubotix.glennifer.hci.ModuleMain #&> "/home/ubuntu/logs/$datestring/ModuleMainOutput.log" &
#cd ../..

cd glennifer/robot_state
nohup java -cp target/robot_state-1.0-SNAPSHOT.jar com.cwrubotix.glennifer.robot_state.StateModule & #> "/home/ubuntu/logs/$datestring/StateModuleOutput.log" &
cd ../..

cd glennifer/automodule
#nohup /usr/lib/jvm/java-8-openjdk-amd64/jre/bin/java -cp target/automodule-0.0.1-SNAPSHOT.jar com.cwrubotix.glennifer.automodule.AutoModule
#nohup /usr/lib/jvm/java-8-openjdk-amd64/jre/bin/java -cp target/automodule-0.0.1-SNAPSHOT.jar com.cwrubotix.glennifer.automodule.AutoTransit
nohup java -cp target/automodule-0.0.1-SNAPSHOT.jar com.cwrubotix.glennifer.automodule.AutoDrillModule &
#nohup /usr/lib/jvm/java-8-openjdk-amd64/jre/bin/java -cp target/automodule-0.0.1-SNAPSHOT.jar com.cwrubotix.glennifer.automodule.AutoDump
nohup java -cp target/automodule-0.0.1-SNAPSHOT.jar com.cwrubotix.glennifer.automodule.LogModule &
cd ../

cd localization/Default
./localization &
cd ../..

cd obstacle_detection/examples
python obstacle_detection.py test & 
cd ../..

sleep 2

#cd motor_dispatch/src/python
#nohup python3 locomotion.py #&> "/home/ubuntu/logs/$datestring/LocomotionPyOutput.log" &
#cd ../../../..
