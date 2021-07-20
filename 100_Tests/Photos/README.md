# Observation

##### Origin type 1
![equation](https://latex.codecogs.com/gif.latex?%5Chat%7BX%7D%280%29%3D%20id) and  ![equation](https://latex.codecogs.com/gif.latex?%5Cxi%5E%5Ccirc%3D%5Chat%7B%5Cxi%7D%280%29)

##### Origin type 2
![equation](https://latex.codecogs.com/gif.latex?%5Cxi%5E%5Ccirc%3D%28I_4%2C%200%29) and the corresponding ![](https://latex.codecogs.com/gif.latex?%5Chat%7BX%7D%280%29) such that ![](https://latex.codecogs.com/gif.latex?%5Cphi%28%5Chat%7BX%7D%280%29%2C%5Cxi%5E%5Ccirc%29%3D%5Chat%7B%5Cxi%7D%280%29)

##### Origin type 3
Change ![](https://latex.codecogs.com/gif.latex?C%5E%5Ccirc) in such a way that ![](https://latex.codecogs.com/gif.latex?x_%7BP%5E%5Ccirc%7D) is the average of ![](https://latex.codecogs.com/gif.latex?p%5E%5Ccirc_i)  so that ![](https://latex.codecogs.com/gif.latex?y_i%5E%5Ccirc) will be distributed around the origin

### Comparison

##### Case 1
![1_ln_1](https://github.com/HiyaGada/SLAM_internship/blob/main/100_Tests/Photos/%5B0.5%3B0.5%3B0%5D_lownoise_type1.png)

![1_ln_2](https://github.com/HiyaGada/SLAM_internship/blob/main/100_Tests/Photos/%5B0.5%3B0.5%3B0%5D_lownoise_type2.png)

![1_ln_3](https://github.com/HiyaGada/SLAM_internship/blob/main/100_Tests/Photos/%5B0.5%3B0.5%3B0%5D_lownoise_type3.png)

We consider the initial estimate ![](https://latex.codecogs.com/gif.latex?%5Chat%7B%5Cxi%7D%280%29) closest to the actual values, i.e. [0.5; 0.5; 0] for the landmarks. 

Error comparison:
Type 3 < Type 1 < Type 2

##### Case 2
![1_hn_1](https://github.com/HiyaGada/SLAM_internship/blob/main/100_Tests/Photos/%5B0.5%3B0.5%3B0%5D_highnoise_type1.png)

![1_hn_2](https://github.com/HiyaGada/SLAM_internship/blob/main/100_Tests/Photos/%5B0.5%3B0.5%3B0%5D_highnoise_type2.png)

![1_hn_3](https://github.com/HiyaGada/SLAM_internship/blob/main/100_Tests/Photos/%5B0.5%3B0.5%3B0%5D_highnoise_type3.png)


We consider the initial estimate ![](https://latex.codecogs.com/gif.latex?%5Chat%7B%5Cxi%7D%280%29) same as Case 1, but with higher noise.

Error comparison:
Type 3 << Type 2 < Type 1

##### Case 3

![1_hhn_1](https://github.com/HiyaGada/SLAM_internship/blob/main/100_Tests/Photos/%5B0.5%3B0.5%3B0%5D_highhighnoise_type1.png)

![1_hhn_2](https://github.com/HiyaGada/SLAM_internship/blob/main/100_Tests/Photos/%5B0.5%3B0.5%3B0%5D_highhighnoise_type2.png)

![1_hhn_3](https://github.com/HiyaGada/SLAM_internship/blob/main/100_Tests/Photos/%5B0.5%3B0.5%3B0%5D_highhighnoise_type3.png)

We consider the initial estimate ![](https://latex.codecogs.com/gif.latex?%5Chat%7B%5Cxi%7D%280%29) same as Case 1, but with the highest noise.

Error comparison:
Type 1 < Type 3 << Type 2

##### Case 4
![2_ln_1](https://github.com/HiyaGada/SLAM_internship/blob/main/100_Tests/Photos/%5B1%3B1%3B0%5D_lownoise_type1.png)

![2_ln_2](https://github.com/HiyaGada/SLAM_internship/blob/main/100_Tests/Photos/%5B1%3B1%3B0%5D_lownoise_type2.png)

![2_ln_3](https://github.com/HiyaGada/SLAM_internship/blob/main/100_Tests/Photos/%5B1%3B1%3B0%5D_lownoise_type3.png)

We consider the initial estimate ![](https://latex.codecogs.com/gif.latex?%5Chat%7B%5Cxi%7D%280%29) a little farther from to the actual values, i.e. [1; 1; 0] for the landmarks. 

Error comparison:
Type 1 < Type 3 <<< Type 2

##### Case 5
![2_hn_1](https://github.com/HiyaGada/SLAM_internship/blob/main/100_Tests/Photos/%5B1%3B1%3B0%5D_highnoise_type1.png)

![2_hn_2](https://github.com/HiyaGada/SLAM_internship/blob/main/100_Tests/Photos/%5B1%3B1%3B0%5D_highnoise_type2.png)

![2_hn_3](https://github.com/HiyaGada/SLAM_internship/blob/main/100_Tests/Photos/%5B1%3B1%3B0%5D_highnoise_type3.png)

We consider the initial estimate ![](https://latex.codecogs.com/gif.latex?%5Chat%7B%5Cxi%7D%280%29) same as Case 4, but with a higher noise.

Error comparison:
Type 1 < Type 2 < Type 3

##### Case 6

![3_ln_1](https://github.com/HiyaGada/SLAM_internship/blob/main/100_Tests/Photos/%5B2%3B2%3B0%5D_lownoise_type1.png)

![3_ln_2](https://github.com/HiyaGada/SLAM_internship/blob/main/100_Tests/Photos/%5B2%3B2%3B0%5D_lownoise_type2.png)

![3_ln_3](https://github.com/HiyaGada/SLAM_internship/blob/main/100_Tests/Photos/%5B2%3B2%3B0%5D_lownoise_type3.png)

We consider the initial estimate ![](https://latex.codecogs.com/gif.latex?%5Chat%7B%5Cxi%7D%280%29) farther from the actual values, i.e. [2; 2; 0] for the landmarks. 

Error comparison:
Type 1 < Type 3 <<<< Type 2

##### Case 7

![4_ln_2](https://github.com/HiyaGada/SLAM_internship/blob/main/100_Tests/Photos/%5B5%3B5%3B0%5D_lownoise_type2.png)

![4_ln_3](https://github.com/HiyaGada/SLAM_internship/blob/main/100_Tests/Photos/%5B5%3B5%3B0%5D_lownoise_type3.png)

We consider the initial estimate ![](https://latex.codecogs.com/gif.latex?%5Chat%7B%5Cxi%7D%280%29) farthest from the actual values, i.e. [5; 5; 0] for the landmarks. 

Type 1 does not run during simulation possibly due to floating point error.

Error comparison:
Type 3 <<<<<< Type 2


### Conclusion

Type 3 should be preferred over Type 2 if the origin is far away. 

Type 1 gave fairly good results in most cases. However it gave a singular matrix error when the origin was taken too far away from the landmarks. Decreasing the time interval would help but would increase the cost.

Type 3 is the best if our input has a lot of noise and when the origin is far away, which is usually the worst possible case.

Type 2 gives worse value as we take the origin (landmark part) far away from the locality of the actual landmarks.

Type 1 and Type 3, counter intuitively, show a somewhat decreasing value as we take the origin farther away.

As we increase noise, the value increases for all types, as expected.



















