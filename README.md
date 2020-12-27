# Visualization of local reference frames (LRF)

### Example output
![Example output](https://github.com/dilaragokay/LRF-visualization/blob/master/img/example.png)

### Requirements
* Install [PCL](https://pointclouds.org/downloads/) (for step 1), [Open3D](http://www.open3d.org/) (for step 2), and [NumPy](https://numpy.org/install/) (for step 2)
* Change the relative path of your input file in [visualize_lrf.py](https://github.com/dilaragokay/LRF-visualization/blob/master/visualize_lrf.py#L5) and [flare/flare_estimation.cpp](https://github.com/dilaragokay/LRF-visualization/blob/master/flare/flare_estimation.cpp#L26)

### 1. Compute local reference frames using [FLARE](http://www.vision.deis.unibo.it/research/78-cvlab/82-lrf)
```
cd flare
mkdir build && cd build
cmake .. && make
./flare_estimation
cd ../..
```

### 2. Visualize LRF
```
python visualize_lrf.py
```

### Render LRF
This part is heavily inspired by [Mitsuba2PointCloudRenderer](https://github.com/tolgabirdal/Mitsuba2PointCloudRenderer). Make sure that [these dependencies](https://github.com/tolgabirdal/Mitsuba2PointCloudRenderer#dependencies) are satisfied and Step 1 is completed.
```
python3.6 render_lrf.py
```

### References
* A. Petrelli, L. Di Stefano, "A repeatable and efficient canonical reference for surface matching", 3DimPVT, 2012. [[PDF](http://www.vision.deis.unibo.it/LRF/LRF_repeatability_3DimPvt2012.pdf)]
