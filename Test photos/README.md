# Observation

##### Origin type 1
![equation](https://latex.codecogs.com/gif.latex?%5Chat%7BX%7D%280%29%3D%20id) and  ![equation](https://latex.codecogs.com/gif.latex?%5Cxi%5E%5Ccirc%3D%5Chat%7B%5Cxi%7D%280%29)

##### Origin type 2
![equation](https://latex.codecogs.com/gif.latex?%5Cxi%5E%5Ccirc%3D%28I_4%2C%200%29) and the corresponding ![](https://latex.codecogs.com/gif.latex?%5Chat%7BX%7D%280%29) such that ![](https://latex.codecogs.com/gif.latex?%5Cphi%28%5Chat%7BX%7D%280%29%2C%5Cxi%5E%5Ccirc%29%3D%5Chat%7B%5Cxi%7D%280%29)

##### Origin type 3
Change ![](https://latex.codecogs.com/gif.latex?C%5E%5Ccirc) in such a way that ![](https://latex.codecogs.com/gif.latex?x_%7BP%5E%5Ccirc%7D) is the average of ![](https://latex.codecogs.com/gif.latex?p%5E%5Ccirc_i)  so that ![](https://latex.codecogs.com/gif.latex?y_i%5E%5Ccirc) will be distributed around the origin

### Comparison

##### Case 1
![test 1](https://github.com/HiyaGada/SLAM_internship/blob/main/Test%20photos/test1%20(first%20type).png)

![test 4](https://github.com/HiyaGada/SLAM_internship/blob/main/Test%20photos/test4%20(second%20type).png)

![test 7](https://github.com/HiyaGada/SLAM_internship/blob/main/Test%20photos/test7%20(third%20type).png)

We consider the same initial estimate ![](https://latex.codecogs.com/gif.latex?%5Chat%7B%5Cxi%7D%280%29) which is closest to the actual value. 

The first type gave the least error followed by the third type followed by the second type of origin. 
Type 1 < Type 3 < Type 2

##### Case 2
![test 2](https://github.com/HiyaGada/SLAM_internship/blob/main/Test%20photos/test2%20(first%20type).png)

![test 5](https://github.com/HiyaGada/SLAM_internship/blob/main/Test%20photos/test5%20(second%20type).png)

![test 8](https://github.com/HiyaGada/SLAM_internship/blob/main/Test%20photos/test8%20(third%20type).png)

We consider the same initial estimate ![](https://latex.codecogs.com/gif.latex?%5Chat%7B%5Cxi%7D%280%29) which is farther than the actual value compared to Case 1.

The second type gave the least error followed by the first type followed by the third type of origin. The third type showed much more error in general.
Type 2 < Type 1 << Type 3




##### Case 3

![test 3](https://github.com/HiyaGada/SLAM_internship/blob/main/Test%20photos/test3%20(first%20type).png)

![test 6](https://github.com/HiyaGada/SLAM_internship/blob/main/Test%20photos/test6%20(second%20type).png)

![test 9](https://github.com/HiyaGada/SLAM_internship/blob/main/Test%20photos/test9%20(third%20type).png)

We consider high noise in this case and the same initial estimate as Case 1.

The third type gave the least error followed by the second type followed by the first type of origin. The first type showed much more error in general.

Type 3 < Type 2 << Type 1

##### Special case

![test 10](https://github.com/HiyaGada/SLAM_internship/blob/main/Test%20photos/test%2010%20special%20case.png)

Type 3 origin ensures that the floating point number is distributed about 0, hence giving more precision and allowing to take origins which are far away from the landmarks. This was not possible in other cases.

### Conclusion

Type 3 should be preferred if the origin is far away. 

Type 2 gave good results in all 3 cases. 

Type 3 is the best if our input has a lot of noise, which is usually the case.

In conclusion, we must focus more on the matrix ![](https://latex.codecogs.com/gif.latex?C%5E%5Ccirc), and try to reduce its condition number (theoretical perspective). Also, using a method like Type 3 would give us more precision (practical perspective).  

















