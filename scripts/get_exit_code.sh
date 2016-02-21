#!/usr/bin/env bash

# The first argument is the process folder where the .proc file is written
PROC_DIR=$1
printf "Will write .proc file in %s\n" $PROC_DIR
# Shift all arguments once; the remainder is the actual command with its
# arguments that is passed onto this scrip and will be executed
shift
# Execute the given command with its arguments as a background  process
#exec "$*" & # Do not use nohup
# TODO: find replacement for eval
eval "$*" & # Do not use nohup
# Retrieve the PID of the newly create process
PID=$!
printf "Starting background process with PID %d\n" $PID
PROC_FILE="$PROC_DIR/.proc"
printf "Writing PID %d to .proc file "%s"..." $PID $PROC_FILE
# Write PID to .proc file
echo "$PID" > "$PROC_FILE"
printf "Done\n"

printf "Running..."
# Wait for executed command to finish
wait "$PID"
# Call right after that $? to get the exit code
# Write the exit code to the .proc file
EXIT_CODE=$?
printf "Done\n"

# Write exit code to .proc file
printf "Writing exit code %d to .proc file %s..." $EXIT_CODE $PROC_FILE
echo $EXIT_CODE >> $PROC_FILE
printf "Done\n"
