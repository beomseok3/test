table Node
start_f_idx/ end_f_idx/ mission_state/ position (path_planning에 사용하진 않음)

table Path
f_idx/ x/ y/ yaw
f_idx/ x/ y/ yaw
f_idx/ x/ y/ yaw
f_idx/ x/ y/ yaw
...

다른 노드에 진입 --> mission_state check'


traffic_logic/ MISSION_LOGIC(parking, pickup, delivery, static or dynamic avoidance)


일단  path 분할 db에 알맞은 스트럭처로 저장 --> query --> controler and code 



-------------------
나중 아이디어:
path publish 한다음 db에 저장 
--> 저장되면
-->yaw값을 검사 차가 25도 보다 적게 나오는지 certify

---------------------------------------------------------------------------
자르고 parking avoidance까지만 state 전환해보자


parameter는 logic에서 sub 받아서 변수로 변형하자 int publish

 B2 parking_start :     position:
      x: 37.937705993652344
      y: 81.07661437988281
      z: 0.0
  C1 parking_end:     position:
      x: 18.148517608642578
      y: 43.508506774902344
      z: 0.0

A2 avoidance_start:     position:
      x: 18.148517608642578
      y: 43.508506774902344
      z: 0.0
B1 avoidance_end:     position:
      x: 18.148517608642578
      y: 43.508506774902344
      z: 0.0


---------------------------------------------------------------------
 좋은 db 구성하기
state machine logic 추가하기
