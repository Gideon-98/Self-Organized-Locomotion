#include "rbfn_semicircle.h"
#include <vector>

#define FT0 2
#define CT0 1
#define TC0 0

rbfn_semicircle::rbfn_semicircle(int leg_id, double sig, int nKernels){

    printf("\nClass call1");
    id = leg_id;
    n_kernels = nKernels;
    sigma = sig;

    // initialize centers of RBF kernels
    initializeCenters();
    
    // initialize weights
    initializeWeights();
    // printf("\nClass call2");
}


bool rbfn_semicircle::initializeCenters(){
    // printf("\nCenter in");

    //read centers from file
    ifstream rbfn_centers_file; 
    // rbfn_centers_file.open(file_directory+"rbfn_semicircle_files/rbfn_centers_"+to_string(id)+".txt", ios::in);
    
    // Use same center
    rbfn_centers_file.open(file_directory+"rbfn_semicircle_files/rbfn_centers.txt", ios::in);
    string rbfn_centers_line;
    if (rbfn_centers_file.is_open()) {
        string new_number;
        vector<double> new_mu;
        vector<double> centerPoint1;
        vector<double> centerPoint2;
        while (getline(rbfn_centers_file,rbfn_centers_line)) {
            new_number = "";
            new_mu.clear();
            while (rbfn_centers_line[0] != ',') {
                new_number += rbfn_centers_line[0];
                rbfn_centers_line.erase(rbfn_centers_line.begin());
            }

            // new_mu.push_back(stod(new_number));
            // printf("\nstod %f", stod(new_number));
            centerPoint1.push_back(stod(new_number));

            rbfn_centers_line.erase(rbfn_centers_line.begin());
            // new_mu.push_back(stod(rbfn_centers_line));
            // printf(", stod %f", stod(rbfn_centers_line));
            centerPoint2.push_back(stod(rbfn_centers_line));
            // mu.push_back(new_mu);
        }
        kernel_means.push_back(centerPoint1);
        kernel_means.push_back(centerPoint2);
        rbfn_centers_file.close();

        // printf("\n joe ---- ");
        // for(int j=0; j<40; j++){
        //     printf("\n %f, %f", kernel_means[0][j], kernel_means[1][j]);
        // }
        
        return true;
    }// if can open file


    else{
        cout<<"Unable to open RBFN Centers File!"<<endl;
        return false;
    }// cannot open file
} // initializeCenters



bool rbfn_semicircle::initializeWeights(){
    printf("\n weight in");
    ifstream rbfn_weights_file;
    printf("\n id leg == %d", id);
    // version 0
    // string filename = file_directory+"rbfn_semicircle_files/rbfn_weights_"+to_string(id)+".txt"; 
    
    // version 1
    string filename = file_directory+"rbfn_semicircle_files/1_rbfn_weights_"+to_string(id)+".txt"; 
    
    // test version
    // string filename = file_directory+"rbfn_semicircle_files/rbfn_weights_0.txt"; 
    
    rbfn_weights_file.open(filename, ios::in);
    string rbfn_weights_line;

    vector<double> new_weights;
    // cout << rbfn_weights_file.is_open();
    if(rbfn_weights_file.is_open()) {
        // printf("\n Can open file");
        string new_number;
        vector<double> w_output;
        // vector<double> w_o1;
        weights.resize(0);
        for (int i=0; i<n_outputs; i++) {
            new_weights.clear();
            // getline(rbfn_weights_file,rbfn_weights_line);
            for (int j=0; j<n_kernels; j++) {
                
                getline(rbfn_weights_file,rbfn_weights_line);
                new_weights.push_back(stod(rbfn_weights_line));
            }
            // getline(rbfn_weights_file,rbfn_weights_line);
            weights.push_back(new_weights);
        }// for n_output

        rbfn_weights_file.close();

        // int i = 0;
        // for(int j=0; j<n_kernels; j++){
        //     // printf("\nline %d : %f, %f, %f, %f",j, weights[i][j], weights[i+1][j], weights[i+2][j], weights[i+3][j]);
        //     printf("\nline %d: %f, ",j, weights[i][j]);
        //     printf("%f, ", weights[i+1][j]);
        //     printf("%f, ", weights[i+2][j]);
        //     printf("%f ", weights[i+3][j]);
        // }
        // printf("\nend");
        return true;
    } // if open file

    else{
        cout<<"Unable to open weight file: " << filename <<endl;
        return false;
    }

} // initializeWeights


vector<double> rbfn_semicircle::getNetworkOutput(vector<double> input){
// double rbfn_semicircle::getNetworkOutput(double input1, double input2){

    vector<double> output;
    double act_kernel = 0;
    double pre_output = 0;
    double activated_kernel = 0;
    for( int o=0; o<n_outputs; o++){

        pre_output = 0;
        for(int k=0; k<n_kernels; k++){
            
            // This case can use only 2 inputs,
            // unless you need to imprement an advance version
            act_kernel = 0;
            act_kernel = pow(input[0] - kernel_means[0][k], 2);
            act_kernel += pow(input[1] - kernel_means[1][k], 2);
            activated_kernel = RBF_activation(act_kernel, 0.2); //sigma
            pre_output += activated_kernel*weights[o][k];
        } // for kernel
        // pre_output = weights[0][o];
        output.push_back(pre_output);
    }// for output

    // output0 : TC-joint signal 
    // output1 : CT-joint signal
    // output2 : FT-joint signal
    // output3 : predicted force
    
    return output;

} //getNetworkOutput


// vector<double> rbfn_semicircle::getNetworkOutput(vector<double> input)
// {
//     vector<double> output;
//     rbf_neurons.clear();
//     vector<double> mu;
//     //Calculating Network output
//     double rbf_sum;
//     for (int i=0; i<n_outputs; i++) {
//         rbf_sum = 0;
//         for (int k=0; k<n_kernels; k++) {
//             double activation = gaussian(normDiff(input, mu[k]), sigma);        // Calculation the activation function with the norm difference as input (Different between input and center)
//             rbf_neurons.push_back(activation);                                  // Updating the activtion output 
//             rbf_sum += weights[i][k] * activation;                          // Summing up the outputs for the differnet kenerls 
//         }                                   
//         output.push_back(rbf_sum);                                              // Added the result to the vector 
//     }

// 	return output;
// }

// **************************************************************** //
// ********************* Helper function ************************** //
// **************************************************************** //

double rbfn_semicircle::RBF_activation(double act, double sigma){
    double k = exp( -(act/ (2*pow(sigma,2))) );
    return k;
} // RBF_activation

