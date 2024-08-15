db_structures
------------------------------------------------------------------

table Node
start_f_idx/ end_f_idx/ mission_state/ position (path_planning에 사용하진 않음)

table Path
f_idx/ x/ y/ yaw
f_idx/ x/ y/ yaw
f_idx/ x/ y/ yaw
f_idx/ x/ y/ yaw
...

------------------------------------------------------------------
다른 노드에 진입 --> mission_state check'

일단  path 분할 db에 알맞은 스트럭처로 저장 --> query --> controler and code 

-------------------------------------------------------------------
자르고 parking avoidance까지만 state 전환해보자


parameter는 logic에서 sub 받아서 변수로 변형하자 int publish
일단 인덱스로 잘랐음
--> marker로 확인할 것 그리고 node에 좌표 추가할것, 또 lane 번호도 추가 할 것

---------------------------------------------------------------------
A1 Start_point: position 
	 index: 0
A2 avoidance_start:     position:
      x: 18.148517608642578
      y: 43.508506774902344
      z: 0.0
     index:400
B1 avoidance_end:     position:
      x: 18.148517608642578
      y: 43.508506774902344
      z: 0.0
      index:457
 B2 parking_start :     position:
      x: 37.937705993652344
      y: 81.07661437988281
      z: 0.0
      index:1270
  C1 parking_end:     position:
      x: 18.148517608642578
      y: 43.508506774902344
      z: 0.0
      index:1316
C2 End_point: positon :?
	index -1

path 분할을 idx 기준으로 해보자
---------------------------------------------------------------------
 좋은 db 구성하기
state machine logic 추가하기
이거 무조건 좌표 이상하다
---------------------------------------------------------------------
1. path  분할 idx changing
2. logic 추가 , parameter pub(근데 그냥 코드를 하나에 다 합치면 되기도 함)
3. node 추가
