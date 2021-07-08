# Observation

##### Origin type 1
![equation](https://latex.codecogs.com/gif.latex?%5Chat%7BX%7D%280%29%3D%20id) and  ![equation](https://latex.codecogs.com/gif.latex?%5Cxi%5E%5Ccirc%3D%5Chat%7B%5Cxi%7D%280%29)

##### Origin type 2
![equation](https://latex.codecogs.com/gif.latex?%5Cxi%5E%5Ccirc%3D%28I_4%2C%200%29) and the corresponding ![](https://latex.codecogs.com/gif.latex?%5Chat%7BX%7D%280%29) such that ![](https://latex.codecogs.com/gif.latex?%5Cphi%28%5Chat%7BX%7D%280%29%2C%5Cxi%5E%5Ccirc%29%3D%5Chat%7B%5Cxi%7D%280%29)

##### Origin type 3
Change ![](https://latex.codecogs.com/gif.latex?C%5E%5Ccirc) in such a way that ![](https://latex.codecogs.com/gif.latex?x_%7BP%5E%5Ccirc%7D) is the average of ![](https://latex.codecogs.com/gif.latex?p%5E%5Ccirc_i)  so that ![](https://latex.codecogs.com/gif.latex?y_i%5E%5Ccirc) will be distributed around the origin

### Comparison

##### Case 1
![test 1](https://github.com/HiyaGada/SLAM_internship/blob/main/Tests%20%2B%20B/case1.png)

The above picture, but zoomed:
![test 2](https://github.com/HiyaGada/SLAM_internship/blob/main/Tests%20%2B%20B/case1_zoomed.png)


We consider the initial estimate ![](https://latex.codecogs.com/gif.latex?%5Chat%7B%5Cxi%7D%280%29) which is closest to the actual value, i.e. [0.5; 0.5; 0] for the landmarks. 

Error comparison:
Type 1 < Type 3 << Type 2

##### Case 2
![test 3](https://github.com/HiyaGada/SLAM_internship/blob/main/Tests%20%2B%20B/case2.png)

The above picture, but zoomed:
![test 4](https://github.com/HiyaGada/SLAM_internship/blob/main/Tests%20%2B%20B/case2_zoomed.png)


We consider the initial estimate ![](https://latex.codecogs.com/gif.latex?%5Chat%7B%5Cxi%7D%280%29) which is farther than the actual value (unlike Case 1), i.e. [1; 1; 0] for the landmarks.

Error comparison:
Type 3 < Type 1 << Type 2

##### Case 3

![test 5](https://github.com/HiyaGada/SLAM_internship/blob/main/Tests%20%2B%20B/case3.png)

The above picture zoomed:
![test 6](https://github.com/HiyaGada/SLAM_internship/blob/main/Tests%20%2B%20B/case3_zoomed.png)

We consider high noise in this case and the same initial estimate as Case 2, i.e. [1; 1; 0] for the landmarks.

Error comparison:
Type 3 < Type 1 < Type 2

##### Case 4

![test 7](https://github.com/HiyaGada/SLAM_internship/blob/main/Tests%20%2B%20B/case4.png)

The above picture zoomed:
![test 8](https://github.com/HiyaGada/SLAM_internship/blob/main/Tests%20%2B%20B/case4_zoomed.png)

We consider high noise in this case and the same initial estimate as Case 1, i.e. [0.5; 0.5; 0] for the landmarks.

Error comparison:
Type 1 < Type 3 << Type 2

##### Case 5
![test 9](https://github.com/HiyaGada/SLAM_internship/blob/main/Tests%20%2B%20B/case5.png)

The above picture, but zoomed:
![test 10](https://github.com/HiyaGada/SLAM_internship/blob/main/Tests%20%2B%20B/case5_zoomed.png)


We consider the initial estimate ![](https://latex.codecogs.com/gif.latex?%5Chat%7B%5Cxi%7D%280%29)  as [2; 2; 0] for the landmarks (farther than Case 2). 

Error comparison:
Type 1 < Type 3 <<< Type 2

#### Special case

In this case, we take the initial estimate farthest from the actual values, i.e. [5; 5; 0] for the landmarks.

Type 1 origin shows error when the code is run.
Type 2 and Type 3 origins show no error when the code is run. 

Type 2:
![test 11](https://github.com/HiyaGada/SLAM_internship/blob/main/Tests%20%2B%20B/special%20case%20(type%202).png)

Type 3:
![test 12](https://github.com/HiyaGada/SLAM_internship/blob/main/Tests%20%2B%20B/special%20case%20(type%203).png)

Error comparison:
Type 3 <<< Type 2

Type 3 origin ensures that the floating point number is distributed about 0, hence giving more precision and allowing to take origins which are far away from the landmarks. This was not possible in other cases.

### Conclusion

Type 3 should be preferred over Type 2 if the origin is far away. 

Type 1 gave good results only when the origin was closer. 

Type 3 is the best if our input has a lot of noise and when the origin is far away, which is usually the worst possible case.

In conclusion, we must focus more on the matrix ![](https://latex.codecogs.com/gif.latex?C%5E%5Ccirc), and try to reduce its condition number (theoretical perspective). Also, using a method like Type 3 would give us more precision (practical perspective).  

It is theoretically unclear why Type 2 shows no error when the code is run for the special case.


















