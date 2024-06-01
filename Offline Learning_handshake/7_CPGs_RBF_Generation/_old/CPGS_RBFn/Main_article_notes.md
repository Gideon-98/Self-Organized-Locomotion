- [1. Self-organized stick insect-like locomotion under decentralized adaptive neural control: From biological investigation to robot simulation](#1-self-organized-stick-insect-like-locomotion-under-decentralized-adaptive-neural-control-from-biological-investigation-to-robot-simulation)
  - [1.1. Structure - decentralized adaptive neural control](#11-structure---decentralized-adaptive-neural-control)
    - [1.1.1. CPGs:](#111-cpgs)
    - [1.1.2. Premotor network (RBFn)](#112-premotor-network-rbfn)
  - [1.2. Setup](#12-setup)
  - [RBF: Premotor network for intralimb coordination](#rbf-premotor-network-for-intralimb-coordination)
  - [1.3. Tables](#13-tables)
  - [1.4. Images](#14-images)
  - [1.5. Bullet List](#15-bullet-list)


# 1. Self-organized stick insect-like locomotion under decentralized adaptive neural control: From biological investigation to robot simulation

## 1.1. Structure - decentralized adaptive neural control
Each leg is independently driven by a modular neural control system consisting of four sub-neural modules:
1. CPG-based control network
2. Premotor network (RBFn)
3. Forward model 
4. Dual-rate learner (or dual-rate learning mechanism)  


### 1.1.1. CPGs: 
The CPG is formed by:
- Two recurrent neurons $N_{1,2}$ 
- Modulatory input neuron $(MI)$

### 1.1.2. Premotor network (RBFn)
The premotor network is formed by multiple radial basis neurons (R1,...,n ). The outputs of the premotor network are projected to three <span style="color: red;">motor neurons (M1,...,3 )</span> and one <span style="color: red;">forward model neuron (FP)</span>. 

The __motor neurons__ transmit joint angle commands to finally control:
- the thoraco-coxal (TC-) joint
- coxo-trochanteral (CTr-) joint
- femoro-tibial (FTi-) joint 

for position control. 

The __output of FP__ predicts the foot contact signal which is further shaped by two postprocessing neurons (P1,2 ) before comparing it with the actual foot contact signal at a linear comparator neuron (E).

The foot contact sensory neuron (FC) receives a continuous foot contact signal. The signal is preprocessed at a sensory preprocessing neuron (SP) and compared with the predicted foot contact signal from the forward model. 

The difference in the comparison leads to an error which is then used in the <span style="color: green;">dual-rate learner </span>. for sensory feedback strength adaptation. All neurons of the control network are modeled as discrete-time non-spiking neurons.

## 1.2. Setup
Each leg is driven by an identical control system (i.e., one neural CPG-based control,
one premotor neural network, one forward model, and one dual-rate learning mechanism). As a consequence, controlling the stick insect robot with six legs requires six decentralized neural control systems.

To provide the flexibility of adaptive interlimb coordination, we do not define any connection or coupling between the neural control systems (i.e., they are decoupled or have no direct neural communication).
Instead, the coordination among them is mainly achieved by the interaction between the robot and the
environment

The model of the CPG-based control circuit is realized using the discrete-time dynamics of a simple
two-neuron recurrent network with neuromodulation.

It produces two periodic signals which are further shaped by the premotor network (RBF-n) to obtain final motor commands for driving the leg joints (TC-, CTr-, and FTi-joints).

We use an extrinsic modulatory input MI as neuromodulation to modulate the CPG frequency and project foot contact feedback FC to the CPG inputs S1,2 to automatically and continuously adjust the CPG phase __online__ for adaptive interlimb coordination. 

This technique can produce the appropriate <span style="color: red;">CPG phase shifts between the legs </span>, enabling the robot to achieve self-organized stable gaits.


The neurons (N1,2) of the circuit are modeled as non-spiking neurons. The activity of each neuron de-
velops according to:

$$
a_1(t+1)=w_{11} o_{N_1}(t)+w_{12} o_{N_2}(t)+\alpha S_1(t),
$$
$$
a_2(t+1)=w_{22} o_{N_2}(t)+w_{21} o_{N_1}(t)+\alpha S_2(t),
$$

where:
- $w_{11,22}$ are the self-connection weights of $\mathrm{N}_{1,2}$
- $w_{12,21}$ are the connection weights between $\mathrm{N}_{1,2}$. 
- $S_{1,2}$ are the CPG inputs
- $o_{N_i}$ are the CPG outputs. 
- $\alpha$ is a plastic CPG input connection (synaptic plasticity). It is automatically adjusted by dual-rate learning.
- $MI$ is the extrinsic modulatory input (neuromodulation) that alters the weights of the CPG, thereby modulating the CPG output frequency.
 
To obtain stable CPG periodic outputs, $MI$ is set within a range of $0.0-0.19$. Increasing $MI$ leads to a rise in CPG frequency (Figure 4A) resulting in a faster walking speed. The CPG inputs are defined as:

$$S_1(t+1)=-F C(t) \cos(a_1(t))$$
$$S_2(t+1)=-F C(t) \sin(a_2(t))$$

The functions are related to the phases of the CPG outputs $o_{N_{1,2}}$ which differ by $\pi/2$. 

The strength of the sensory feedback connection can be adapted to regulate the amount of sensory feedback to the CPG-based control. 

Through this connection, the foot contact sensory feedback can __slow down__ the leg speed when __highly loaded__ at the end of the stance phase, while increasing the speed of the leg trajectory when it is unloaded at the end of the swing phase. 

This allows the robot to adaptively adjust its leg movement to form stable gaits with good body weight distribution.

When implementing the neural control systems on different legs, the proper value of $\alpha$ needs to be used to achieve stable locomotion and this value might have to be changed with respect to certain conditions (e.g., different walking speeds). 

Predetermining an optimal value for all cases is time-consuming and impractical. Thus, we apply a dual-rate learning mechanism as an automatic process for continuously and dynamically adjusting the value online. 

Under this control scheme, there is no fixed and predefined interlimb coordination, but rather a flexible one, since the gaits obtained are derived from foot contact feedback, synaptic plasticity, neural activities, and body-environment interaction.

## RBF: Premotor network for intralimb coordination

For intralimb coordination (i.e., joint coordination) of each robot leg, we project the CPG outputs to the TC-, CTr-, and FTi-joints indirectly through the premotor neural network. 

The network shapes the periodic CPG patterns to finally generate robot foot trajectories following the stick insect foot trajectory data recorded during walking. A feedforward neural network with radial basis activation functions (i.e., radial basis function (RBF) network) is used as the premotor network. 

The RBF premotor network consists of three layers (input, hidden, and output). 

We integrate the CPG network with the RBF premotor network (called the CPG-RBF network). In other words, the two CPG outputs represent the two RBF inputs which are further transmitted to the RBF hidden neurons with 2 D radial basis or Gaussian activation functions $\left(R_{1, \ldots, n}\right.$). The neural activity of each RBF hidden neuron is given by:
$$
\phi_n(t)=e^{-}\left(\frac{\left.\left(o_{N_1}(t)-\mu_{n, 1}\right)^2+\left(o_{N_2}(t)-\mu_{n, 2}\right)^2\right)}{2 \sigma^2}\right), n \in 1, \ldots, N,
$$

where $\mu_{n, 0}$ and $\mu_{n, 1}$ are two means of the RBF neuron. 

$\sigma^2$ is the standard variance for the two means, empirically set to 0.04 for all neurons.

$\phi_n$ is the activity of the RBF neuron driven by the CPG output signals $o_{N_{1,2}}$. 

The two means are empirically set in such a way as to equally distribute the RBF neurons along one period of the CPG output and target signals.

$N$ is the total number of RBF hidden neurons (here, $N=40$, which is set to maintain sufficient signal complexity but not too large for computation.

The output signals from the RBF neurons are linearly combined at four linear output neurons as:
$$
o_{R B F_j}(t)=\sum_{n=1}^N w_{j n} \phi_n(t), j \in 1, \ldots, 4,
$$
$w_{j n}$ represents the weights used to shape and combine the transmitted signals between the RBF hidden and output neurons. 

The first three output neurons are used as three motor neurons $\left(\mathrm{M}_{1, \ldots, 3}\right.)$ for controlling the TC-, CTr-, and FTi-joints and the last output neuron as a forward model neuron FP for predicting foot contact feedback. 

This RBF premotor network structure is the same for all six neural control systems controlling the six legs of the robot. 

The output weights $w_{j n}$ are trained offline using a delta or error-based learning rule as follows:
$$
\Delta \mathrm{w}_{j n}=\eta\left(T_j(t)-\sum_{n=1}^N w_{j n} \phi_n(t)\right), j \in 1, \ldots, 4, n \in 1, \ldots, N .
$$

The $\mathrm{x}-\mathrm{z}$ plane trajectories of a stick insect (lateral view shown in Figure 2B), which express swing and stance durations, are the only ones used in this study for the sake of simplicity. The x-y plane trajectories (dorsal view shown in Figure 24) are simplified as straight lines. The foot trajectories are first preprocessed, and the preprocessed trajectories (Figure 5 A,B) are then applied to obtain the robot joint space. Here, the planned joint positions $\left(T_j\right)$ serve as the target signals (Figure $5 \mathrm{C}$-E) for training the RBF network. $\eta$ is the learning rate, empirically set to 0.1 . This training strategy follows our previous study [70].

## 1.3. Tables

Tables can be created using the pipe (|) and hyphen (-) characters to define the columns and rows. Here's an example:

| Column 1 | Column 2 |
| -------- | -------- |
| Cell     | Cell     |
| Cell     | Cell     |

## 1.4. Images

To insert an image in markdown, use the following syntax:

![Alt Text](image_url)

Replace `Alt Text` with the alternative text for the image and `image_url` with the URL or file path of the image.

## 1.5. Bullet List

To create a bullet list in markdown, use the asterisk (*) or hyphen (-) characters followed by a space. Here's an example:

* Item 1
* Item 2
* Item 3



<span style="color: red;">This text will be red.</span>