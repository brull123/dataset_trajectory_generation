ORIENTATION="northeast"
python3 generate_trajectories.py worlds/world_temesvar_field_123.yaml \
	--platform platforms/x500.yaml\
      	--pattern dataset-random-walk-moving  \
       	--static-camera-circle-clearance 5 \
      	--static-camera-circle-point $ORIENTATION\
      	--minimum-distance 7.5\
       	--maximum-distance 60.0 \
       	--random-seed 481 \
	--show-plot\
	--plot \
	--horizontal-margin 0 \
       	--duration 500 \
	--constraint-profile fast \
       	--moving-camera-radius 6 \
	--moving-camera-heading-walk 40\
       	--random-waypoints 100 \
	--output-dir temesvar_2_$ORIENTATION \
	--placement-direction center
