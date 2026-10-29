import numpy as np, math, random
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
SP='C:/Users/kgoku/AppData/Local/Temp/claude/C--Users-kgoku-Desktop-Remster-Peltier-Test-RIG-Data-28-10/697c7e07-25bd-432e-b0bd-99274cf68817/scratchpad/'
b=np.load(SP+'coef.npy')
def plant_rate(T,cool,heat):
    ce=min(b[2]+b[4]*T,-1e-6)            # cooling effect per duty, never positive
    return b[0]+b[1]*T+cool*ce+heat*b[3]
MAXC,MAXH=220,100
class Ctl:
    def __init__(s,mod):
        s.mod=mod
        s.Kp=60.0; s.Ki=0.6 if mod else 0.15; s.Kd=20.0 if mod else 150.0
        s.mincool=60;s.minheat=50
        s.I=0;s.sp_ramp=None;s.pvf=None;s.active=False;s.since=0;s.errsign=0
        s.u=0.0;s.target=0;s.lastdir=0;s.zero=0;s.pidlast=0;s.pvsm=None
    def decide(s,sp,pv,dt,now):
        if s.sp_ramp is None: s.sp_ramp=sp
        if abs(sp-s.sp_ramp)>1: s.sp_ramp=sp;
        else: s.sp_ramp=min(s.sp_ramp+0.5*dt,sp) if sp>s.sp_ramp else max(s.sp_ramp-0.5*dt,sp)
        e=s.sp_ramp-pv;ae=abs(e)
        if s.pvf is None:s.pvf=pv
        prev=s.pvf;s.pvf=0.7*s.pvf+0.3*pv;dm=(s.pvf-prev)/dt
        es=1 if e>0 else -1
        if (not s.mod) and s.errsign!=0 and es!=s.errsign:s.I=0
        s.errsign=es
        inst=now-s.since;far=ae>=1
        if not s.active:
            if ae>0.3 and (inst>=30 or far):s.active=True;s.since=now
        else:
            if ae<0.1 and inst>=30:s.active=False;s.since=now
        if not s.active:
            if not s.mod:s.I*=0.98
            return 0
        P=s.Kp*e;D=-s.Kd*dm
        u=max(-MAXC,min(MAXH,P+s.I+D))
        sath=u>=MAXH-1e-6 and e>0;satl=u<=-MAXC+1e-6 and e<0
        if not sath and not satl:s.I=max(-200,min(200,s.I+s.Ki*e*dt))
        elif not s.mod:s.I*=0.98
        if e<=-0.1 and u>0:u=0
        if e>=0.1 and u<0:u=0
        mc,mh=(s.mincool,s.minheat) if not s.mod else (40,35)
        if u<0:return -int(min(255,max(mc,-u)))
        if u>0:return int(min(255,max(mh,u)))
        return 0
    def step(s,sp,pv_raw,now):
        pv=pv_raw
        if s.mod:
            s.pvsm=pv_raw if s.pvsm is None else 0.95*s.pvsm+0.05*pv_raw;pv=s.pvsm
        if s.pidlast==0:s.pidlast=now
        elif now-s.pidlast>=1.0:
            dt=min(now-s.pidlast,3);s.pidlast=now
            t=s.decide(sp,pv,dt,now)
            # polarity interlock
            if s.u!=0:s.lastdir=-1 if s.u<0 else 1;s.zero=0
            elif s.zero==0:s.zero=now or 1e-3
            w=-1 if t<0 else (1 if t>0 else 0)
            if not(w==0 or s.lastdir==0 or w==s.lastdir):
                if s.u==0 and now-s.zero>=10:s.lastdir=w
                else:s.I=0;t=0
            s.target=t
        dtf=0.1
        mx=25*dtf
        s.u=min(s.u+mx,s.target) if s.target>s.u else max(s.u-mx,s.target)
        return s.u
def run(mod,sp,T0,minutes=120,seed=1,sigma=0.16,spike=0.01):
    rng=random.Random(seed);c=Ctl(mod);T=T0;n=int(minutes*600);out=[]
    for k in range(n):
        now=k*0.1
        meas=T+rng.gauss(0,sigma)+(rng.choice([-1,1])*1.2 if rng.random()<spike else 0)
        u=c.step(sp,meas,now)
        cool=-u if u<0 else 0;heat=u if u>0 else 0
        T+=plant_rate(T,cool,heat)*0.1
        if k%10==0:out.append((now/60,T,u))
    return np.array(out)
def metrics(r,sp):
    t,T=r[:,0],r[:,1];last=T[t>=t[-1]-40]
    reach=t[np.argmax(np.abs(T-sp)<=0.5)] if (np.abs(T-sp)<=0.5).any() else np.nan
    return dict(reach_min=reach,overshoot=max(0,(T.max()-sp) if T[0]>sp or True else 0) if sp>T[0] else max(0,sp-T.min()),
                last40_mean_err=last.mean()-sp,last40_std=last.std(),within05=(np.abs(last-sp)<=0.5).mean())
if __name__=='__main__':
    cases=[(20,24.5),(22.5,20),(25,22.5),(30,25),(35,30)]
    fig,ax=plt.subplots(len(cases),2,figsize=(16,3*len(cases)),sharex=True)
    print('%-5s %-5s %8s %9s %10s %9s %9s'%('SP','ctl','reach_m','overshoot','last40 err','last40 sd','within.5'))
    for i,(sp,T0) in enumerate(cases):
        for j,mod in enumerate([0,1]):
            r=run(mod,sp,T0);m=metrics(r,sp)
            print('%-5.1f %-5s %8.1f %9.2f %10.2f %9.2f %9.2f'%(sp,'NEW' if mod else 'OLD',m['reach_min'],m['overshoot'],m['last40_mean_err'],m['last40_std'],m['within05']))
            ax[i,j].plot(r[:,0],r[:,1],lw=.8);ax[i,j].axhline(sp,color='r');ax[i,j].set_ylabel('SP %.1f'%sp)
            a2=ax[i,j].twinx();a2.plot(r[:,0],-r[:,2]/2.55,color='gray',lw=.4,alpha=.6);a2.set_ylim(-40,100)
            if i==0:ax[i,j].set_title('NEW (fixes)' if mod else 'OLD (current code)')
    fig.tight_layout();fig.savefig('C:/Users/kgoku/Desktop/Remster/Peltier Test RIG/Data 28-10/sim_old_vs_new.png',dpi=80)
