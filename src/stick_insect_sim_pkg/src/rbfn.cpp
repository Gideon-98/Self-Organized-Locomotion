#include "rbfn.h"

#define FT0 2
#define CT0 1
#define TC0 0

rbfn::rbfn(int leg_id, double sig, int nKernels) 
{
    training = false;
    id = leg_id;
    kernel_count = nKernels;
    sigma = sig;

    //intialize centers
    initializeCenters();

    //initialize weights
    initializeWeights();
}

void rbfn::saveData(std::vector<std::vector<double>> data, int id){
    ofstream rbfn_target_scaled;
    rbfn_target_scaled.open(file_directory+"rbfn_files/rbfn_target_scaled_"+to_string(id)+".txt", ios::trunc);

    for (int i=0; i<data.size(); i++) {
        rbfn_target_scaled << data[i][0] << ", " << data[i][1] << ", " << data[i][2] << ", " << data[i][3] << "\n"; 
    }
    rbfn_target_scaled.close();
}

rbfn::rbfn(int leg_id, vector<vector<double>> cpg_cycle, double sig, int nKernels)
{
    training = true;
    kernel_count = nKernels;
    sigma = sig;
    id = leg_id;
    learning_iter = 0;


    // Reading input traning data 
    training_cpg_cycle = readCSVData(file_directory + "rbfn_files/cpg-data.csv", nInputs, true);

    
    // We shift the training data by 300 samples to align the input and output data - Should be adjusted for new training data
    int shiftSize =  300;

    for (size_t i = 0; i < shiftSize; i++)
    {
        training_cpg_cycle.push_back(training_cpg_cycle[0]);
        training_cpg_cycle.erase(training_cpg_cycle.begin()); 
    }
    

    //intialize centers
    initializeCenters();

    //initialize weights
    initializeWeights();

    // The training data is loaded differently if we have 3 or 4 outputs.
    string path = file_directory + "rbfn_files/rbfn_joint_targets_";

    if(nOutputs > 3){
        path = path +  "withz_me_";
    }

    // Preparing the training data 
    if(id < 3){
        //target_joint_angles = readCSVData(path+to_string(id)+".csv", nOutputs, true);                     // Without rescaling the target data
        target_joint_angles = rescaleCSVData(readCSVData(path+to_string(id)+".csv", nOutputs, true), id);   // With scaling the target data
        saveData(target_joint_angles, id);  // We only save the training data for the right most legs, as the legs legs are identical
    }else{ 
        //target_joint_angles = readCSVData(path+to_string(id-3)+".csv", nOutputs, true);                       // Without rescaling the target data
        target_joint_angles = rescaleCSVData(readCSVData(path+to_string(id-3)+".csv", nOutputs, true), id-3);   // With scaling the target data
    }
}



vector<vector<double>> rbfn::rescaleCSVData(vector<vector<double>> data, int id){

    vector<vector<double>> newdata; 
    vector<double> scaledTime; 

    int splitPoint[3] = {221, 308, 280};                // The data index point where the data splits from swing to stance phase (From observations)
    
    double stanceScale[3] = {2.24, 1.773, 1.872};       // The scaling factor for scaling up the stance phase
    double swingScale[3] = {0.4595, 0.433, 0.455};      // The scaling factor for scaling down the swing phase 

    double factor = (splitPoint[id] * stanceScale[id]) / (swingScale[id] * (data.size() - splitPoint[id])); 


    // Create scaled time vector 
    for (size_t i = 0; i < data.size(); i++)
    {
        double scaledIndex;

        if(i < splitPoint[id]){
            scaledIndex = i*stanceScale[id]; //i*swingScale[id]; 
        }else{
            scaledIndex += swingScale[id]; //stanceScale[id]; 
        }    

        scaledTime.push_back(scaledIndex);
    }

    if(scaledTime[scaledTime.size()-1] < (scaledTime.size()-1) ){
        std::cout << "ID[" << id << "] ERROR IN RESCALING CSV - THE LAST TIMESTAMP IN THE LIST IS TO LOW = " <<  scaledTime[scaledTime.size()-1] << std::endl;
        throw; 
    }else{
        std::cout << "ID[" << id << "] THE LAST ELEMENT IN THE LIST IS = " <<  scaledTime[scaledTime.size()-1] << std::endl;
    }
    

    // Go through each new point. This should result in the same size vector
    for (size_t i = 0; i < data.size(); i++)
    {
        vector<double> dataRow; 
        double lowerIndex = -1; 
        double upperIndex = -1; 

        double lowerVal = 99; 
        double upperVal = 99; 


        for (size_t j = 0; j < data.size(); j++)
        {   
            // Find upper and lower number 
        
            double diff = (scaledTime[j] - (double)i); 
            if(diff <= 0 && abs(diff) < abs(lowerVal) ){         // If this scaled time is less or equal then save as lower as the new lowest value
                lowerVal = diff; 
                lowerIndex = j; 
            }

            if(diff >= 0 && abs(diff) < abs(upperVal)){
                upperVal = diff; 
                upperIndex = j; 
            }

        }


        for (size_t k = 0; k < nOutputs; k++)
        {
            dataRow.push_back(linearInterpolation(scaledTime[lowerIndex], scaledTime[upperIndex], data[lowerIndex][k], data[upperIndex][k], i));
        }

        newdata.push_back(dataRow);
        
    }

    
    return newdata; 
    
}


double rbfn::linearInterpolation(double t0, double t1, double x0, double x1, int input){
    if(t1 == t0)
        return x0; 
    else 
        return x0 + (input - t0) * ((x1-x0)/(t1-t0)); 

}

vector<vector<double>> rbfn::readCSVData(string filename, int nCols, bool header){
    //std::cout << "Reading file" << std::endl;
    ifstream joint_file; 
    joint_file.open(filename, ios::in);

    string joint_line;
    vector<double> joint_position;
    vector<vector<double>> joint_positions;
    string new_number;
    int testing = 0;

    if (joint_file.is_open()) {
        if(header) getline(joint_file, joint_line);

        while (getline(joint_file, joint_line)) {

            //read joint values line by line
            joint_position.clear();
            for (int i=0; i < (nCols-1); i++) {
                new_number = "";
                while (joint_line[0] != ',') {
                    new_number += joint_line[0];
                    joint_line.erase(joint_line.begin());
                    testing++;
                }
                joint_position.push_back(stod(new_number));
                joint_line.erase(joint_line.begin());
            }
            joint_position.push_back(stod(joint_line));
            joint_positions.push_back(joint_position);
        }
        joint_file.close();
    }else{
        std::cout << "Unable to open joint target file: " << filename << std::endl;
    }
    std::cout << "CSV data table of size " << joint_positions.size() << " x " << joint_positions[0].size() << " sucessfully read." << std::endl;
    return joint_positions;
}

bool rbfn::initializeWeights() {
    if (training) {
        vector<double> new_weights;
        srand(time(NULL));
        for (int i=0; i<nOutputs; i++) {
            new_weights.clear();
            for (int j=0; j<kernel_count; j++) {
                new_weights.push_back(rand()%1000/10000.0);
            }
            weights.push_back(new_weights);
        }
    }
    else {
        ifstream rbfn_weights_file; 
        string filename = file_directory+"rbfn_files/rbfn_weights_"+to_string(id)+".txt"; 
        rbfn_weights_file.open(filename, ios::in);
        string rbfn_weights_line;




        vector<double> new_weights;
        if (rbfn_weights_file.is_open()) {
            for (int i=0; i<nOutputs; i++) {
                new_weights.clear();
                for (int j=0; j<kernel_count; j++) {
                    getline(rbfn_weights_file,rbfn_weights_line);
                    //cout<<"Line read: "<<rbfn_weights_line<<endl;
                    new_weights.push_back(stod(rbfn_weights_line));
                }
                getline(rbfn_weights_file,rbfn_weights_line);
                weights.push_back(new_weights);
            }
            rbfn_weights_file.close();
        }
        else cout<<"Unable to open weight file: " << filename <<endl;
    }

}

bool rbfn::initializeCenters() 
{
    if (training) {
        int jmp = (training_cpg_cycle.size()-1)/(kernel_count);
        vector<double> new_mu;
        ofstream rbfn_centers_file;
        rbfn_centers_file.open(file_directory+"rbfn_files/rbfn_centers_"+to_string(id)+".txt", ios::trunc);
    
        for (int i=0; i<training_cpg_cycle.size(); i+=jmp) {
            new_mu.clear();
            probe.push_back(i);
            new_mu.push_back(training_cpg_cycle[i][0]);
            new_mu.push_back(training_cpg_cycle[i][1]);
            rbfn_centers_file<<training_cpg_cycle[i][0]<<", "<<training_cpg_cycle[i][1]<<"\n";
            mu.push_back(new_mu);
        }
        std::cout << "Jmp = " << jmp << ", probe size = " << probe.size() << std::endl;
        rbfn_centers_file.close();
    }
    else {
        //read centers from file
        ifstream rbfn_centers_file; 
        rbfn_centers_file.open(file_directory+"rbfn_files/rbfn_centers_"+to_string(id)+".txt", ios::in);
        string rbfn_centers_line;
        if (rbfn_centers_file.is_open()) {
            string new_number;
            vector<double> new_mu;
            while (getline(rbfn_centers_file,rbfn_centers_line)) {
                new_number = "";
                new_mu.clear();
                while (rbfn_centers_line[0] != ',') {
                    new_number += rbfn_centers_line[0];
                    rbfn_centers_line.erase(rbfn_centers_line.begin());
                }
                new_mu.push_back(stod(new_number));
                rbfn_centers_line.erase(rbfn_centers_line.begin());
                new_mu.push_back(stod(rbfn_centers_line));
                mu.push_back(new_mu);
            }
            rbfn_centers_file.close();

        }
        else {cout<<"Unable to open RBFN Centers File!"<<endl;}
    }
}

double rbfn::gaussian(double r, double sig){
    return exp(-pow(r, 2)/sig);
}

double rbfn::normDiff(vector<double> x, vector<double> y){
    double sum = 0;
    if(x.size() != y.size())
    {
        std::cout << "Error in norm difference - dimensions don't match [" << x.size() << ", " << y.size() << "]" << std::endl;
    }

    for (size_t i = 0; i < x.size(); i++)
    {
        sum = sum + pow(x[i]-y[i], 2);
    }
    return sqrt(sum);
}

double rbfn::activationFunc(double in1, double mu1, double in2, double mu2){
    double n = pow(in1-mu1,2.0)+pow(in2-mu2,2);
    return exp(-(n)/sigma);
}

vector<double> rbfn::getNetworkOutput(vector<double> input)
{
    vector<double> output;
    rbf_neurons.clear();

    //Calculating Network output
    double rbf_sum;
    for (int i=0; i<nOutputs; i++) {
        rbf_sum = 0;
        for (int k=0; k<kernel_count; k++) {
            double activation = gaussian(normDiff(input, mu[k]), sigma);        // Calculation the activation function with the norm difference as input (Different between input and center)
            rbf_neurons.push_back(activation);                                  // Updating the activtion output 
            rbf_sum += weights[i][k] * activation;                          // Summing up the outputs for the differnet kenerls 
        }                                   
        output.push_back(rbf_sum);                                              // Added the result to the vector 
    }

	return output;
}



vector<double> rbfn::getJointAngles(double cpg1, double cpg2)
{
    vector<double> joint_angles;
    //Calculation of rbf neurons output
    rbf_neurons.clear();
    for (int i=0; i<kernel_count ; i++) {
        double new_neuron_output = activationFunc(cpg1, mu[i][0], cpg2, mu[i][1]);
        rbf_neurons.push_back(new_neuron_output);
    }

    //Calculating Network output
    double rbf_sum;
    for (int i=0; i<nOutputs; i++) {
        rbf_sum = 0;
        for (int j=0; j<kernel_count; j++) {
            rbf_sum += weights[i][j] * rbf_neurons[j];
        }
        joint_angles.push_back(rbf_sum);
    }

	return joint_angles;
}



bool rbfn::train_rbfn_gd() 
{
    std::cout << "Trainig leg with ID[" << id << "]" << std::endl;
    vector<double> joint_angles;
    int nEpochs = 0;                            // Epoch counter
    int epochs = 500;                           // Number of epochs to run
    int step = 20;                              // Downsampling training data
    int nSamples = training_cpg_cycle.size();

    bool learning_done[4] = {false};


    if(training_cpg_cycle.size() != target_joint_angles.size()){
        std::cout << "Input and output training data is not of same dimensions [" << training_cpg_cycle.size() << " != " << target_joint_angles.size() << "]" << std::endl;
        return false;
    }




    while (!learning_finished && nEpochs < epochs) {
    

        ofstream rbfn_weights_file;
        rbfn_weights_file.open(file_directory + "rbfn_files/rbfn_weights_" + to_string(id) + ".txt");

        // CALCULATING THE DELTA ERROR MATRIX 

        for (size_t i = 0; i < nOutputs; i++)
        {
            weight_adjusted = false; 
            if(!learning_done[i]){
                for (size_t m = 0; m < kernel_count; m++)
                {
                    double deltaError = 0;
                    double deltaW = 0;
                    double sse = 0; 
                    for (size_t n = 0; n < nSamples; n+=step)
                    {
                        double error = target_joint_angles[n][i] - getNetworkOutput(training_cpg_cycle[n])[i];              // APPROXIMATING THE ERROR AT THE I'TH INPUT - Subtracting the current output from the target output (eq. 3.12)
                        sse += sqrt(error*error);                                                                           // SUM OF SQUARES ERROR
                        deltaError = deltaError +  error * gaussian(normDiff(training_cpg_cycle[n], mu[m]), sigma);         // SUMMING UP THE ERRORS FOR THE M-KERNEL AND I'TH INPUT

                    }
                    deltaError = -1.0* (2.0/(nSamples/step)) * deltaError;                                                  // CALCULATING THE DELTA ERROR (eq. 3.13)
                    deltaW = -1.0 * learning_rate * deltaError;                                                             // CALCULATING THE DELTA WEIGHT (eq. 3.15)

                    if(sse > errTol){
                        weights[i][m] = weights[i][m] +  deltaW;
                        weight_adjusted = true;
                    }
                } 
                
                if(weight_adjusted == false){
                    learning_done[i] = true; 
                    std::cout << "Done training output[" << i << "]" << std::endl;
                }
            }
        }

        if(!weight_adjusted){
            learning_finished = true;
        }

        nEpochs++;
    
    }
    std::cout << "Training done after " << nEpochs << " epochs!" << std::endl;
    return true;
}



bool rbfn::train_rbfn() 
{
    std::cout << "Trainig leg with ID[" << id << "]" << std::endl;
    vector<double> joint_angles;
    int nCal = 0;
    std::vector<double> errSum = {0, 0, 0, 0};
    std::vector<double> errSumOld = {0, 0, 0, 0};
    int nEpochs = 0;                            // Epoch counter

    // ***** Open error file
    ofstream rbfn_sse_file;
    rbfn_sse_file.open(file_directory + "rbfn_files/rbfn_sse_" + to_string(id) + ".txt");

    while (!learning_finished && nCal < 10000) {
        nCal++;

        // Get Network output
        joint_angles.clear();
        joint_angles = getNetworkOutput(training_cpg_cycle[probe[learning_iter]]);

        // Read the network weights 
        ofstream rbfn_weights_file;
        rbfn_weights_file.open(file_directory+"rbfn_files/rbfn_weights_"+to_string(id)+".txt");


        //Calculate the error for each output neuron of the network
        vector<double> deltaOutput;
        for (int i = 0; i < nOutputs; i++)
        {
            deltaOutput.push_back(target_joint_angles[probe[learning_iter]][i] - joint_angles[i]);
        }
        

        vector<vector<double>> deltaWeights;
        vector<double> new_deltaWeights;

        //calculate the dela weight for every neuron
        for (int i=0; i<nOutputs; i++) { //3
            new_deltaWeights.clear();
            for (int j=0; j<kernel_count; j++) {
                if (j==learning_iter)
                {
                    new_deltaWeights.push_back(learning_rate*deltaOutput[i]*rbf_neurons[j]);
                }
                else
                {
                    new_deltaWeights.push_back(0.0);
                }
                rbfn_weights_file<<weights[i][j]<<"\n";
            }


            deltaWeights.push_back(new_deltaWeights);
            rbfn_weights_file<<"\n";
        }
        

        //if the error is bigger than some value update the weigths
        double err = 0;
        for (int i = 0; i < nOutputs; i++)
        {
            err = sqrt(deltaOutput[i]*deltaOutput[i]);
            errSum[i] +=err;
            

            if (err > errTol || i == 3) {
                if(i != 3) weight_adjusted = true;  // keep traning z untill done 

                for (int j=0; j<kernel_count; j++)
                {
                    weights[i][j] = weights[i][j] + deltaWeights[i][j];
                }
            }
        }
        


        // If we have itterated trough all samples, check if we are done training
        if (learning_iter == (probe.size()-1)) {
            learning_iter = 0;
            nEpochs++; 
            
            // Testing for the difference in error 
            double sumErr = errSum[0] + errSum[1] + errSum[2] + errSum[3];
            double oldSumErr = errSumOld[0] + errSumOld[1] + errSumOld[2] + errSumOld[3];

            if (!weight_adjusted || abs(oldSumErr-sumErr) < 0.005)
                learning_finished = true;
            else
                weight_adjusted = false;

            errSumOld = errSum;

            for (int i = 0; i < nOutputs; i++){
                rbfn_sse_file<< errSum[i] << ", "; 
            }
            rbfn_sse_file<<"\n";

            errSum = {0, 0 ,0, 0};

        }
        else 
        {
            learning_iter = learning_iter + 1;
        }
    }
    std::cout << "Training " << nEpochs << " epochs!" << std::endl;

    rbfn_sse_file.close();
    
    return true;
}

rbfn::~rbfn(){

}