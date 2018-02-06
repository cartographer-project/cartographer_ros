# Copyright 2018 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Definition of status code constants equivalent to the gRPC API.
# https://developers.google.com/maps-booking/reference/grpc-api/status_codes
uint8 OK=0
uint8 CANCELLED=1
uint8 UNKNOWN=2
uint8 INVALID_ARGUMENT=3
uint8 DEADLINE_EXCEEDED=4
uint8 NOT_FOUND=5
uint8 ALREADY_EXISTS=6
uint8 PERMISSION_DENIED=7
uint8 RESOURCE_EXHAUSTED=8
uint8 FAILED_PRECONDITION=9
uint8 ABORTED=10
uint8 OUT_OF_RANGE=11
uint8 UNIMPLEMENTED=12
uint8 INTERNAL=13
uint8 UNAVAILABLE=14
uint8 DATA_LOSS=15
