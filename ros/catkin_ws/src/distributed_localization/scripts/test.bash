#!/bin/bash

#TODO: Fix script to stop on ^C

# Check that their are enough args
if [ "$#" -le 0 ]; then
    echo Usage: test.bash num_tests [duration]
    exit 1
fi

# Set duration
DURATION=600
if [ "$#" -eq 2 ]; then
    DURATION=$2
fi

# Print out message about what will occur
echo "Running $1 tests, each for $DURATION seconds. Press ^C at any time to stop"

sleep 3

# Delete all the test files
ls testing_data/test* > /dev/null 2> /dev/null
if [ $? -eq 0 ]; then
    rm testing_data/test*
fi

# Run alternating amcl, amcl+distrib tests
for (( c=1; c<=$1; c++ ));do
	# Start up ros stuff
    roslaunch distributed_localization localization.launch &
    sleep 5

    rosparam set "/use_sim_time" true

    # Start up logging
    python log_pose.py -t $c &
    LOG_PID=$!
    sleep 1

    # Start up reweight if even test
    if [ $(( $c % 2 )) -eq 0 ]; then
        python reweight_data.py &
        REWEIGHT_PID=$!
    fi

    # Sleep for duration
    sleep $DURATION

    # Stop all processes
    kill -9 "$LOG_PID"
    if [ $(( $c % 2 )) -eq 0 ]; then
         kill -9 "$REWEIGHT_PID"
    fi
    rosnode kill --all

    if [ $c -lt $1 ]; then
        sleep 60
    fi
done