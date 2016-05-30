# Drone
##Summary
>MultiWii를 사용하지 않고 아두이노와 라즈베리파이를 이용하여 쿼드콥터를 제작하고 안드로이드 스마트 폰을 이용하여 와이파이로 쿼드콥터를 제어하는 프로젝트

##Methodology
>* 자이로 가속도 센서를 이용해 각도를 추출할 시에는 필터를 이용해서 정확한 각도값을 필터링 해내는 기술이 필요한데. 대표적인 필터 두가지가 칼만필터(Kalman Filter)와 상보필터(Complementary Filter)이다. 본 프로젝트에서는 구현상의 편의를 따져 상보필터를 구현한다. 상보필터는 가속도 센서와 자이로 센서의 장점과 단점을 상호 보완하는 방식의 필터이다. 두 센서의 단점을 따져보자면

>>* 가속도센서의 단점 : 중력가속도를 기준으로 각도를 측정하기 때문에 센서가 이동하는 상황에서는 중력가속도 이외의 방향으로도 가속도가 적용되어 정확한 기울기 측정이 어려워진다.
>>* 자이로센서의 단점: 자이로센서는 각속도를 적분하는 방식으로 각도를 측정하기 때문에 적분상수에 의한 오차값이 시간이 갈수록 누적된다. 따라서 시간이 지날수록 부정확해진다.

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

##Implementation

