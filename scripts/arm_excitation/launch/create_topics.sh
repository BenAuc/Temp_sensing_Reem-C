#!/bin/bash

topicName="skinArmExcitation"

ind=0
driverList[$((ind++))]="/h1/skin/right/upper_arm"
driverList[$((ind++))]="/h1/skin/right/lower_arm"
# driverList[$((ind++))]="/h1/skin/right/torso"
# driverList[$((ind++))]="/h1/skin/right/flank"
# driverList[$((ind++))]="/h1/skin/right/upper_leg"
# driverList[$((ind++))]="/h1/skin/right/lower_leg"

# driverList[$((ind++))]="/h1/skin/left/upper_arm"
# driverList[$((ind++))]="/h1/skin/left/lower_arm"
# driverList[$((ind++))]="/h1/skin/left/torso"
driverList[$((ind++))]="/h1/skin/left/flank"
# driverList[$((ind++))]="/h1/skin/left/upper_leg"
# driverList[$((ind++))]="/h1/skin/left/lower_leg"


lenDriverList=${#driverList[@]}

for (( j=0; j<lenDriverList; j++ ))
do
    driver=${driverList[${j}]}
    
    #cmd="rosservice call ${driver}/setCmd '{cmd: udr 0}'"
    
    cmd="rosservice call ${driver}/createSkinCellDataPub '{topicName: ${topicName}, cellId: [] }'"
    echo "$cmd"
    eval "$cmd"
    
done

exit 0





