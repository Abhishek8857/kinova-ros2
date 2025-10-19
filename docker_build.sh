#!/bin/bash
set -e

CONTAINER_NAME=$(cat container_name.cfg)

LOW_MEMORY=false

# Default values
# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -low)
            echo "Low memory build enabled."
            LOW_MEMORY=true
            shift
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [-low]"
            exit 1
            ;;
    esac
done

# Build the Docker image and pass the boolean
docker build \
    --build-arg LOW_MEMORY="$LOW_MEMORY" \
    -t "$CONTAINER_NAME" .