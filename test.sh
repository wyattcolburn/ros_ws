#!/bin/bash
echo "hello World"

LAST_LINE=$(tail -n 1 ~/ros_ws/trial_results_world_num.csv)
TRIAL_RESULT=$(echo "$LAST_LINE" | cut -d',' -f8)

echo "TRIAL_RESULT is: $TRIAL_RESULT"

