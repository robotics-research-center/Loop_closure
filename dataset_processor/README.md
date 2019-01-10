## Description: ##

1. **src/calculate_rt.cpp** : Main source file. It performs:
	1. Parse image directory (**class** : `ParseImages`).
	2. Parse trajectory file (**class** : `FileParser`).
	3. Generate 180 degree image pairs based on trajectory info (**class** : `MatchImagePair`).
	4. Parse g2o file (**class** : `g2oParser`).
	5. Generate relative transforms between image pairs based on g2o file (**class** : `GenerateRT`).

2. **Build** :
	1. `mkdir build && cd build`
	2. `cmake .. && make -j4`

3. **Usage** :
	1. `./calculate_rt /path/to/rgb_images /path/to/depth_images trajectories.txt file.g2o` 
	2. Example : `./calculate_rt images/rgb/ images/depth/ 180_pairs.txt poses.g2o`

4. [**Dataset**](https://drive.google.com/drive/folders/1nrFxShFa1-8Cr8XVQevKQ1QCmhjb5Xi5) 