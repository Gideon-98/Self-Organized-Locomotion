## adaptive_controller.h

6. comment the following line:
```cpp
#include "rbfn.h"
```
7. decomment the following line:
```cpp
// #include "rbfn_semicircle.h"
```
69. comment the following line:
```cpp
    // rbfn* rbfNet[6];
```
70. decomment the following line:
```cpp
    rbfn_semicircle* rbfNetSemicircle[6];
```


## duallearner.h
145. cast use_semicircle_RBF to TRUE
```cpp
    bool use_semicircle_RBF = true;
```
If I change only the line in duallearner.h, it does not seem to change anything.
I'll try to change also the adaptive_controller.h file.

## rbfn.h
51. change the following line:
```cpp
string file_directory = "/home/gian/catkin_ws/src/stick_insect_sim_new_pkg/src/";
```

## rbfn_semicircle.h
48. change the following line:
```cpp
string file_directory = "/home/gian/catkin_ws/src/stick_insect_sim_new_pkg/src/";
```
1. change the following line:
```cpp
    #ifndef RBFN_SEMICIRCLE_H
    #define RBFN_SEMICIRCLE_H
```
38. change the following line:
```cpp
    double sigma = 1;                       // The variance of the data; the higher the bigger spread
    int n_kernels = 40;                 // Number of kernels 
    int n_outputs = 4;                  // Number of network outputs 
    int n_inputs = 2;                   // Number of network inputs 
    double learning_rate = 0.01;       // The rate of learning                
    int learning_iter;                  // The learning itteration index
    int id;                             // The ID of the current leg (Used for saving and reading the data to files)
```

53. change the following line:
```cpp
    activated_kernel = RBF_activation(act_kernel, sigma); //sigma
```
