# Drone

##Summary
>MultiWii를 사용하지 않고 아두이노와 라즈베리파이를 이용하여 쿼드콥터를 제작하고 안드로이드 스마트 폰을 이용하여 와이파이로 쿼드콥터를 제어하는 프로젝트

##Environment
>* Flight board : Arduino Uno
>* Control board : Raspberry Pi b+
>* Controller : Anroid Smart Phone
>* Gyro+Aceel sensor : MPU-6050
>* ESC : Hobbyking 40A ESC 4A UBEC
>* BLDC motor : A2826-15

##Develop Tools
>* Arduino IDE
>* VIM
>* Android studio

##Language
>* Python2.7
>* Java
>* C++

##SDK
>* Android SDK v.23

##Methodology
###Complementary Filter
>* 자이로 가속도 센서를 이용해 각도를 추출할 시에는 필터를 이용해서 정확한 각도값을 필터링 해내는 기술이 필요한데. 대표적인 필터 두가지가 칼만필터(Kalman Filter)와 상보필터(Complementary Filter)이다. 본 프로젝트에서는 구현상의 편의를 따져 상보필터를 구현한다. 상보필터는 가속도 센서와 자이로 센서의 장점과 단점을 상호 보완하는 방식의 필터이다. 두 센서의 단점을 따져보자면

>>* 가속도센서의 단점 : 중력가속도를 기준으로 각도를 측정하기 때문에 센서가 이동하는 상황에서는 중력가속도 이외의 방향으로도 가속도가 적용되어 정확한 기울기 측정이 어려워진다.
>>* 자이로센서의 단점: 자이로센서는 각속도를 적분하는 방식으로 각도를 측정하기 때문에 적분상수에 의한 오차값이 시간이 갈수록 누적된다. 따라서 시간이 지날수록 부정확해진다.

>>>이다. 따라서 가속도 센서의 단점인 이동중에 각도 추출이 불가능 한 점을 자이로 센서를 이용해서 보완하고 자이로센서의 단점인 시간이 지남에 따라 오차가 커지는 문제를 가속도센서의 기울기값을 참고하여 정확한 값으로 수정, 보완하는 방식이 상보필터이다.

###PID Control
>* P(비례제어) I(적분제어) D(미분제어) 제어기를 말한다. 디지털 기기는 0과 1을 이용해서 하드웨어를 제어한다. 하지만 현실에서 어떠한 목표값에 알맞은 제어를 하기 위해서는 아날로그적인 제어가 필요하다. 디지털 신호를 이용해서 최대한 아날로그적인 목표값에 근접하게 도달 할 수는 있을 것이나 아날로그적인 목표값에 도달하는 과정에서 반드시 오차가 생겨나며 결국엔 목표값에 수렴하지 못한채 목표값 언저리에서 무한히 진동하는 상황에 이르거나 목표값에 도달한다고 해도 그 과정에서 수많은 진동과정이 생겨날 것이다. 만약 꺼져있는 선풍기가 50퍼센트의 출력을 내기 위해서 70->30->60->40->55->45->50 순서로 출력을 내면서 50퍼센트의 출력을 맞춘다고 생각한다면 PID제어가 왜 필요한 것인지 알 수 있을 것이다. 본 프로젝트에서는 PID제어를 이용하여 드론이 공중에서 별도의 제어가 없을 시에 수평을 유지하며 떠있을 수 있도록 (이를 호버링이라 한다.) 구현할 것이다.

>>* P제어: 비례제어는 (목표값-현재값)* pGain=제어량 으로 제어량을 정한다. 목표수치가 0이고 현재 수치가 -100이라면 +100를 더해서 목표값에 도달해야 할 것이다. 하지만 +100만큼 제어를 하게되면 목표값을 넘어가버릴수 있다(오버슈트). 따라서 적당한 비율 (예를들어 0.3)을 곱해줘서 +30의 제어만을 행해서 제어하는 대상이 목표값을 넘어가지 않도록 한다. 여기서 적당한 비율이 pGain을 말한다.
>>* I제어: I제어는 적분제어라고 하며 P제어에 계산한 (목표값-현재값) 을 누적한 뒤에 iGain을 곱하여 구한다. 적분제어는 P제어에서 발생하는 미세한 오차를 잡는 역할을 한다. 만약 P제어를 통해서 목표값에서 -10 +10 정도의 오차로 무한히 진동하는 상황이라면 I제어는 시간이 지날수록 이러한 미세한 오차를 잡아내어 점차 목표값에 더 정확하게 도달하게 한다.
>>* D제어: D제어는 미분제어라고 하며 순간적인 외란으로 발생하는 오차를 잡는 역할을 한다. 드론의 특성상 공중에서 바람의 영향을 많이 받게 된다. 이때 바람에 의해서 받는 힘은 드론 외부에서 오는것이므로 외란이라고 하는데. 호버링을 위해서는 이 외란에 대해서 신속하게 대처할 수 있어야 한다. D제어는 현재의 애러값(목표값-현재값)에서 이전 애러값(목표값-이전 현재값)을 뺸 다음 dGain값을 곱하여 구한다. D제어를 구현하여 튜닝하면 드론이 공중에서 호버링 중일때 손으로 충격을 주거나 바람을 일으켜 강제로 드론이 흔들리는 상황을 만들면 신속하게 드론이 신속하게 반응하여 호버링을 한다.

###HDR algorithm (Heuristic Drift Reduction algorithm)
>* gyro sensor는 하드웨어 특성상 바이어스가 생긴다. bias를 잡아주기 위해서 HDR algorithm이 필요하다. 

###DLPF
>* DLPF(Digital Low Pass Filter)

##Implementation

##Video
>* 2ch Test : https://youtu.be/YCUaSBQO8sU
