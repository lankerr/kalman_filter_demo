# kalman_filter_demo
2022数学建模国赛代码  //获得国二



ang_gen 输入三个飞机的位置，输出他们的角度（理想情况下的）
ang_gen_v2 生成他们的角度（第三问实际情况下的）
angle_gen,angle_gen_v2分别是在两个情况下的
antianti,anticlock,clockclock,dist都是location的调用函数，用于loacte,根据角度
demo是第一题错误率的仿真，基本上是在0.009，说明我们的定位是有效的
demo2是第二题的仿真，错误率在0.01，也是合理的
demo3是第三题的仿真，我们可以将error转化为所有位置偏差之和，最后得到答案
hmodeneinitial.m和lmodeinitial都是kalmanopt的子函数，kalmanopt主要就是调用kalman算法来对无人机进行追踪定位，进行一次优化的算法，并且返回角度
kopt_demo用于仿真kalman函数的迭代次数与精度的计算
kalman.m是一个小小的kalman的追踪目标的实例，为了debug方便而使用的
location.m是根据三个角度和发射机来预测位置的函数
mytest是用于debug使用的便于图像可视化的代码


//2023-4-7更新-已经获得了国二
