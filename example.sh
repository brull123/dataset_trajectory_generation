ORIENTATION="north"
python3 generate_trajectories.py worlds/world_temesvar_field_2.yaml \
	--platform platforms/x500.yaml\
      	--pattern dataset-random-walk-moving  \
       	--static-camera-circle-clearance 5 \
      	--static-camera-circle-point $ORIENTATION\
      	--minimum-distance 5.0\
       	--maximum-distance 50.0 \
       	--random-seed 456 \
	--show-plot\
	--plot \
	--horizontal-margin 5.0 \
       	--duration 500 \
	--constraint-profile fast \
       	--moving-camera-radius 7.5 \
	--moving-camera-heading-walk 65\
       	--random-waypoints 125 \
	--output-dir temesvar_2_$ORIENTATION \
	--placement-direction southwest
