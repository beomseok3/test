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
 좋은 db 구성하기
state machine logic 추가하기
