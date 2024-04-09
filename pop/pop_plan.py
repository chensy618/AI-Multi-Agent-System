from planning import *

# ac = air_cargo()
# print(ac.goal_test())
# ac.act(expr('Load(C2, P2, JFK)'))
# ac.act(expr('Load(C1, P1, SFO)'))
# ac.act(expr('Fly(P1, SFO, JFK)'))
# ac.act(expr('Fly(P2, JFK, SFO)'))
# ac.act(expr('Unload(C2, P2, SFO)'))
# print(ac.goal_test())
# ac.act(expr('Unload(C1, P1, JFK)'))
# print(ac.goal_test())
# for action in ac.actions:
#     print(action)

mw = mavis_world()
print(mw.goal_test())
mw.act(expr('Move(0,L3,L2)'))
print(mw.goal_test())
mw.act(expr('Pull(0,L2,L3,B1,L4)'))
mw.act(expr('Push(0,L3,B1,L2,L1)'))
print(mw.goal_test())
for action in mw.actions:
    print(action)
