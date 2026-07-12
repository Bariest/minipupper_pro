import math
# ---- legacy ESP IK (from main.c) ----
L1_L, L2_L = 50.0, 56.0
NZ = 70.0
def leg_neutral():
    ld=NZ
    th1=-math.acos((L1_L**2+ld**2-L2_L**2)/(2*L1_L*ld))
    th2=math.asin((ld**2-L1_L**2-L2_L**2)/(2*L1_L*L2_L))-th1
    return th1,th2
th1n,th2n=leg_neutral()
def legacy(x,z=NZ):
    zd=z; ld=math.sqrt(x*x+zd*zd); phi=math.atan2(x,zd)
    th1=phi-math.acos((L1_L**2+ld**2-L2_L**2)/(2*L1_L*ld))
    th2=math.asin((ld**2-L1_L**2-L2_L**2)/(2*L1_L*L2_L))-th1
    hip=(th1-th1n)*180/math.pi; knee=(th2-th2n)*180/math.pi
    return hip,knee  # before per-leg sign
# per-leg servo sign (hip,knee) applied in fRIK etc: FR(-,+) FL(+,-) RR(-,+) RL(+,-)
leg_sign_legacy={0:(-1,+1),1:(+1,-1),2:(-1,+1),3:(+1,-1)}

# ---- new exact Stanford IK ----
L1,L2,OFF=50.0,60.0,26.0
LEG_FB,LEG_LR=59.0,23.5
ORGX,ORGY=59.0,49.5
NH=80.0
abd_off=[-OFF,OFF,-OFF,OFF]
def clip(a): return max(-0.99,min(0.99,a))
def stan_leg(x,y,z,leg):
    Ryz=math.sqrt(y*y+z*z)
    t=Ryz*Ryz-OFF*OFF; t=max(0,t); Rhyz=math.sqrt(t)
    phi=math.acos(clip(abd_off[leg]/Ryz))
    hfa=math.atan2(z,y); abd=phi+hfa
    theta=math.atan2(-x,Rhyz)
    Rhf=math.sqrt(Rhyz*Rhyz+x*x)
    trident=math.acos(clip((L1*L1+Rhf*Rhf-L2*L2)/(2*L1*Rhf)))
    hip=theta+trident
    beta=math.acos(clip((L1*L1+L2*L2-Rhf*Rhf)/(2*L1*L2)))
    knee=hip-(math.pi-beta)
    return abd,hip,knee
def recon(fx,fy,fz,leg):
    sx=1 if leg in(0,1) else -1
    sy=1 if leg in(1,3) else -1
    bx=sx*ORGX+fx; by=sy*ORGY+fy; bz=-fz
    return bx-sx*LEG_FB, by-sy*LEG_LR, bz
neutral={}
for leg in range(4):
    lx,ly,lz=recon(0,0,NH,leg); neutral[leg]=stan_leg(lx,ly,lz,leg)
SK_SIGN={'hip':[1,-1,1,-1],'knee':[1,-1,1,-1]}
def new(fx,leg):
    lx,ly,lz=recon(fx,0,NH,leg); jr=stan_leg(lx,ly,lz,leg); n=neutral[leg]
    hip=(jr[1]-n[1])*180/math.pi*SK_SIGN['hip'][leg]
    knee=(jr[2]-n[2])*180/math.pi*SK_SIGN['knee'][leg]
    return hip,knee

print("leg | dHIP legacy/new  | dKNEE legacy/new   (sign as foot moves FORWARD +x=+20)")
for leg in range(4):
    h0,k0=legacy(0); h1,k1=legacy(20)
    hs,ks=leg_sign_legacy[leg]
    dHl=(h1-h0)*hs; dKl=(k1-k0)*ks
    nh0,nk0=new(0,leg); nh1,nk1=new(20,leg)
    dHn=nh1-nh0; dKn=nk1-nk0
    okH="MATCH" if (dHl>0)==(dHn>0) else "**FLIP**"
    okK="MATCH" if (dKl>0)==(dKn>0) else "**FLIP**"
    print(f" {leg}  | {dHl:+6.2f}/{dHn:+6.2f} {okH:8} | {dKl:+6.2f}/{dKn:+6.2f} {okK}")
