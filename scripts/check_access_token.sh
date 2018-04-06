#!/bin/bash

# Usage: ./check_access_token.sh ACCESS_TOKEN
# Returns non-zero exit code if ACCESS_TOKEN is invalid.

if [ "$#" -ne 1 ]; then
  echo "Please provide an access token to $0" 1>&2
  exit 1
fi
token=$1

set -e
function on_error {
  echo "Failed to validate GitHub access token!" 1>&2
  exit 1
}
trap on_error ERR

test_response=$(curl -s https://api.github.com/?access_token=${token})

echo $test_response | grep -ivq "bad credentials"
echo $"GitHub access token is valid."
