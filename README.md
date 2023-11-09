
This package contains scripts used in the article:

`Mullakkal-Babu, F. A., Wang, M., van Arem, B., & Happee, R. (2016, November). Design and analysis of full range adaptive cruise control with integrated collision a voidance strategy. In 2016 IEEE 19th International conference on intelligent transportation systems (ITSC) (pp. 308-315). IEEE.`

# Usage
+ Run the script `FR_ACC_code.m` from the matlab command window
+ Enter the three arguments when requested: test scenario, sensor delay and adaptation lag. 

# Remarks 
+ Please read the manuscript for details about the model and the test scenarios 
+ The details of the reference controller can be found in: S. Moon, I. Moon, and K. Yi, “Design, tuning, and evaluation of a 
full-range adaptive cruise control system with collision avoidance,” Control Eng. Pract., vol. 17, no. 4, pp. 442–455, 2009.
+ Add additional test scenarios in line 161 
+ You may choose the parameter values in line for better collision avoidance performance.
+ The sensor delay and adaptation lag should not be Zero 



