
# Readme #

Here, you can find some resources related to the manuscript *Output maximization container loading problem with time availability constraints*. 

Feel free to use it, partially or completely. But mind the terms of license.

Also, note that there is no warrant for the code and __we do not provide support for it__. We recommend you to use the resources here as guidelines for your development. 
 
The code was developed using Julia 0.7 in Ubuntu 16.04. 

 
# Description of the folder #

  * *src*: Folder with source codes.
  
  * *datasets*: Folder with the instances. 

  * *results*: Folder with the results.

# Descriptions of the main files #

  *  *2017_CLP_CL_dynProg.jl*: Reads the cost of the states and runs the dynamic programming algorithm. Results are saved in the form CLTAC-dyn-x, x = {1, 2, 4}.
  
  *  *2017_CLP_CL_dynProgPreprocess.jl*: Solves the CLP for each instance in the thpack dataset. It solves 10 versions of the problem. Considering boxes available up until t=1, then t=2, ..., t=10.
  
  *  *2018_CLP_CL_dynProgAllStates.jl*: Solves each possible state to be used in the dynamic programming algorithm.
  
  * *2018_CLP_CL_dynPackingKnapsack.jl*: Runs the simulation for the arrival of the boxes using the solution from the dynamic programming to decided whether to load or wait for more boxes.
  
  * *2018_CLP_CL_dynPackingBoxSetManipulator.jl*: Auxiliary function to manipulate the boxes set that arrive.
