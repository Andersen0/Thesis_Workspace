#!/bin/bash

# Define the SSH login credentials and target command
SSH_USER="thorvald"
SSH_HOST="172.20.10.5"
SSH_PASS="visitNorway"
COMMAND="sudo shutdown -h 0"

# Run the command over SSH using sshpass
sshpass -p "$SSH_PASS" ssh -o StrictHostKeyChecking=no "$SSH_USER@$SSH_HOST" "echo '$SSH_PASS' | sudo -S $COMMAND"
