# MPC-Project
EPFL Project for the course [ME-425](https://edu.epfl.ch/coursebook/fr/model-predictive-control-ME-425) by [C.N. Jones](https://people.epfl.ch/colin.jones?lang=fr).
MPC (Model Predictive Control) controller for a rocket simulation in MATLAB.

Grade for the Project: 6/6 🥳

## Content of the project
During this project for the course ME-425: Model Predictive Control given by Dr. Colin Jones, we
will develop several different controllers for a rocket prototype. Our role is to implement our MPC
controller Matlab code into the provided simulation. First, we will simulate the rocket dynamics by
hands, in order to understand how it react to the different input commands, then using linearized
dynamics we will divide the rocket dynamics into 4 independent subsystems to develop MPC regulators
for each of them. Afterwards, we will implement a tracking controller allowing the rocket to
track a given point. This reference tracking MPC controller will be used to make the rocket track a
complex reference path. We will then implement an offset free tracking controller, which will be able
to track the same path reference despite a mass mismatch. Finally, we will design a MPC controller
for the Non Linear Dynamics and compare its performance against the linear controllers.

See the pdf report for more information: [here](MPC_Project_Group46_Report.pdf)
