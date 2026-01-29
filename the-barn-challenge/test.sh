#!/bin/bash
for i in {0..299} ; do          
    # run the test
    python3 run.py --world_idx $i 
    sleep 2
done