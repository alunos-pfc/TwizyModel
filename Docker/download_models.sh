#!/bin/bash

# Get the absolute path of the directory this script is in
CURRENT_DIR="$(pwd)"

# Check if the models folder exists, and create it if not
if [ ! -d "${CURRENT_DIR}/models" ]; then
    mkdir "${CURRENT_DIR}/models"
fi

# Download gazebo models
wget -r -np -nH -R "index.html*" \
    -P "${CURRENT_DIR}/models" \
    http://models.gazebosim.org/sun/ \
    http://models.gazebosim.org/ground_plane/ \
    http://models.gazebosim.org/baylands/ \
    http://models.gazebosim.org/gas_station/ \
    http://models.gazebosim.org/house_1/ \
    http://models.gazebosim.org/hatchback/ \
    http://models.gazebosim.org/polaris_ranger_ev/ \
    http://models.gazebosim.org/person_standing/ \
    http://models.gazebosim.org/unit_box/ \
    http://models.gazebosim.org/unit_box_0/ \
    http://models.gazebosim.org/unit_sphere/ \
    http://models.gazebosim.org/unit_cylinder/ \
    http://models.gazebosim.org/house_2/