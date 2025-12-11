#!/bin/bash
cd src/proto
nanopb_generator -D ../protoc --strip-path notificationbeacon.proto
nanopb_generator -D ../protoc --strip-path disoverybeaconreply.proto
nanopb_generator -D ../protoc --strip-path nodepresentation.proto
nanopb_generator -D ../protoc --strip-path pathrouting.proto
nanopb_generator -D ../protoc --strip-path nodepresentationrx.proto
cd -