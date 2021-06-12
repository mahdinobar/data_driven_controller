# data_driven_controller
IfA FHNW ACCR-Automation PhD position application 
Mahdi Nobar

Bayesian Optimization (BO) result for the system with surrogate model (order 2):

![G2](https://user-images.githubusercontent.com/44223239/120083467-22383300-c0c9-11eb-8574-80f56e6bb58d.png)  


BO result for the system with full model (order 4):

![G4](https://user-images.githubusercontent.com/44223239/120083473-2b290480-c0c9-11eb-9b77-f6497735b759.png)


Data sampled from the hand joint of Panda arm for the reaching task simulated bellow:

![Panda_demo](https://user-images.githubusercontent.com/44223239/120083948-9cb68200-c0cc-11eb-9bb3-318d9d8ad674.gif)


BO result for system of order 8 without warm starting the search span:

![G8](https://user-images.githubusercontent.com/44223239/120890814-58177300-c605-11eb-8341-94cd3ed64c69.png)


BO result for system of order 8 with warm starting the PID gain search span using the optimum results for the surrogate model:

![G8_warmstart_withG2results](https://user-images.githubusercontent.com/44223239/120890827-70878d80-c605-11eb-8731-0b1723451407.png)

