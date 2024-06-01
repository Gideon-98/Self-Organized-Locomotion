// #ifndef RBFN_SEMICIRCLE_H
// #define RBFN_SEMICIRCLE_H

#include <iostream>
#include <vector>
#include <math.h>
#include <cmath>
#include <stdio.h> 
#include <stdlib.h>
#include <fstream>
#include <string>
#include <chrono>
#include <thread>

using namespace std;


// **************************************************************//
//       This class can only use for **trained** RBF weight      //
//                      Developed by Joe                         //
//          for training -- google colab code                    //
// https://colab.research.google.com/drive/1hEARiAw8Aj5xMY22WAzoHui73YtKTChU?authuser=3#scrollTo=s-Y7JrDBOC1v
//                                                               //
// **************************************************************//


class rbfn_semicircle
{
    public:

        rbfn_semicircle(int leg_id, double sig, int nKernel);
        vector<double> getNetworkOutput(vector<double> input);

        ~rbfn_semicircle();

    private:

        double sigma;                       // The variance of the data; the higher the bigger spread
        int n_kernels = 41;                 // Number of kernels 
        int n_outputs = 4;                  // Number of network outputs 
        int n_inputs = 2;                   // Number of network inputs 
        double learning_rate = 0.075;       // The rate of learning                
        int learning_iter;                  // The learning itteration index
        int id;                             // The ID of the current leg (Used for saving and reading the data to files)

        // PATH OF THE FILES - BOTH TRAINING DATA AND WEIGHTS, CENTERS AND MEANS) 
        // string file_directory = "/home/alexdupond/The-code/controllers/medaextra/locomotion_control/";
        string file_directory = "/home/gian/catkin_ws/src/stick_insect_sim_pkg/src/";


        // vector<vector<double>> mu;                       // Neuron prototype vector or center (mean = mu)
        vector<vector<double>> kernel_means;                 // Neuron prototype vector or center (mean = mu)
        vector<int> probe;                                  // Used to "probe" the training data for only using every M sample
        vector<double> rbf_neurons;                         // The RBF network neurons 
        vector<vector<double>> weights;                     // Network weights 
        vector<vector<double>> target_joint_angles;         // The target data for training the network
        vector<vector<double>> training_cpg_cycle;          // The input training data - CPG cycle

        
        bool initializeCenters();                                               // Initializing the center (Loads when not training)
        bool initializeWeights();                                               // Initializing the weights (Loads when not training)
        // double activationFunc(double in1, double mu1, double in2, double mu2);  // Activation function given the two inputs 
        double RBF_activation(double act, double sigma);

        // HELPER FUNCTIONS //

        // double gaussian(double r, double sig);                                          // Gaussian function 
        // double normDiff(vector<double> x, vector<double> y);                            // Calculate the norm between two vectors
        // double linearInterpolation(double x1, double x2, double y1, double y2, int x);  // Calculate new y-value from two points and a x-value

        // void saveData(std::vector<std::vector<double>> data, int id);                   // Saves data into csv files 
        // vector<vector<double>> readCSVData(string filename, int nCols, bool header);    // Read csv files and returns vectors 
        // vector<vector<double>> rescaleCSVData(vector<vector<double>> data, int id);     // Used to rescale the training data

};