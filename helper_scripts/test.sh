#!/bin/bash

    LAST_LINE=$(tac ~/ros_ws/baseline.csv | grep -m1 .)

    # Trim newline and extract the 8th field (result)
    TRIAL_RESULT=$(echo "$LAST_LINE" | tr -d '\r\n' | cut -d',' -f8)
    # LAST_LINE=$(tail -n 1 ~/ros_ws/trial_results_world_num.csv)
    # TRIAL_RESULT=$(echo "$LAST_LINE" | cut -d',' -f8)

    echo "Last line: '$LAST_LINE'"
    echo "Parsed result: '$TRIAL_RESULT'"

