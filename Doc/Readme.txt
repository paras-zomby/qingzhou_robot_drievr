1160 1510 1880
   350  370   
angle < 36

tand = tan(36/57.3) = 0.72647182173761126944500784757401
#define T 0.245f
#define L 0.29f
7200 = velocity*(1-T*Tand/2/L);

//程序留底
INSHEREHOLD(-5500, velocity, 5500)  //为了保证差速的有效性而进行的限幅
INSHEREHOLD(-36.0f, angle, 36.0f)   //同时限制了输入的电机速度和舵机角度。


前进时：        左 右
set_velocity    ++ --
   PWM          ++ --
   encoder      -- ++

在前进时，左侧编码器读数为负，右侧为正。