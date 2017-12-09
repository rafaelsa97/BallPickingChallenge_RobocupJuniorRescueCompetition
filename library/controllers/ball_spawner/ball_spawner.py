from controller import Supervisor
import random

print('ball_spawner')

s = Supervisor()

y = 0.05

x = random.random() - 1.5
z = random.random() + 0.5
p = [x, y, z]
n = s.getFromDef("BALL1")
f = n.getField('translation')
f.setSFVec3f(p)

x = random.random() - 1.5
z = random.random() + 2.5
p = [x, y, z]
n = s.getFromDef("BALL2")
f = n.getField('translation')
f.setSFVec3f(p)

x = random.random() + 0.0
z = random.random() + 2.5
p = [x, y, z]
n = s.getFromDef("BALL3")
f = n.getField('translation')
f.setSFVec3f(p)

x = random.random() - 3.0
z = random.random() + 2.5
p = [x, y, z]
n = s.getFromDef("BALL4")
f = n.getField('translation')
f.setSFVec3f(p)

x = random.random() - 1.5
z = random.random() + 1.5
p = [x, y, z]
n = s.getFromDef("BALL5")
f = n.getField('translation')
f.setSFVec3f(p)
