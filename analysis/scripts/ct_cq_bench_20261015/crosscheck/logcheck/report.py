import json, os, numpy as np, sys
sys.path.insert(0,os.path.dirname(os.path.abspath(__file__)))
from scipy import stats
from common import RESULTS
OUT=RESULTS
T1=json.load(open(OUT+'/test1.json')); T2=json.load(open(OUT+'/test2.json')); T3=json.load(open(OUT+'/test3.json')); SAN=json.load(open(OUT+'/sanity.json'))
f=lambda x,n=3: 'n/a' if x is None or (isinstance(x,float) and np.isnan(x)) else f'{x:.{n}f}'
L=[]
P=L.append
P('# logcheck report: StampFly flight logs vs two motor-drive hypotheses\n')
P('Read-only analysis. No repository file was modified. Scripts and raw numbers are in this directory (`*.py`, `test1.json`, `test2.json`, `test3.json`, `sanity.json`). All 2026 logs use the same (current) propeller; date groups differ in firmware/gains only.\n')
P('H_chop: motor sees D*V (thrust depends on D*V only). H_rout: equivalent-circuit, I_m=(V-Ke*w)/R_out(D). "sils": current SILS plant (D*V-Km*w)/Rm.\n')
P('## 0. Method choices (stated, not dictated by the brief)\n')
P('''- Airborne: `flight_state==5` (FLYING; status is 1 Hz, held to the duty series). Altitude from posvel -z (ESKF fused; agrees with ToF within the logs checked: alt_max column). 
- Hover window: FLYING, alt > 0.3 m, t >= first-FLYING + 2 s, |vertical accel| < 0.5 m/s^2 (vz from posvel, 0.5 s moving average, derivative, 0.5 s average; nominal sample spacing used because imu timestamps contain duplicates), and additionally |vz| < 0.2 m/s (my addition, to avoid slow climbs). Local exclusion of +-1.5 s around any gyro-norm > 5 rad/s sample in every log; for the flip logs everything from 1 s before the first gyro-norm > 5 rad/s onward is excluded (no flight_phase column exists in these bundles; flight_state is only 1 Hz).
- Contiguous hover segments >= 1 s are cut into chunks of ~5 s (remainder merged, chunks >= 1 s kept). One chunk = one window: D-bar = mean of the four commanded duties, V = mean of linearly-interpolated 1 Hz status.voltage over the chunk. Duty source: ctrl_ref.motor_duty (50 Hz) for JSONL logs; 400 Hz `motor` stream for the native bundles (equal in the mean).
- Test 1 hypotheses, each normalised through the log's own mean point (mean V, mean D-bar): H_chop: D = c/V. H_rout: T_ref = Ct*w^2 at the mean point from the H_rout model, then D(V) solved so that T(D,V)=T_ref. RMS = RMS of (measured D-bar - curve) over windows. Slope CI = 95 % t-interval of an OLS fit on windows (windows are treated as independent: they are NOT -- consecutive chunks share the battery-sag trend; CIs are therefore optimistic).
- Test 2: for each 1-s status sample (cur = current_ma*10 mA, overhead subtracted), duty = per-motor mean over (t_status-1 s, t_status], V = that status voltage. Overhead = mean of current_ma*10 over status samples with flight_state in {0,1,2} and all four duties zero over the previous 3 s (n shown per log; some are 1-3 samples only). The ratio table uses samples whose predicted battery current is > 0.02 A. The alternative averaging conventions (mean of instantaneous predictions; centred window) are in test2.json.
- Test 3: kp mode, gains = status.pid_roll/pitch/yaw = [kp, ti, td] (constant within every log; verified against `git log -p` of sf_core/params.cpp: e.g. 2026-07-17 commit c3aec878 "td 0.02, kp x1.3" gives roll kp 1.2688e-3/td 0.02 as in the 07-17 late logs; 2026-07-20 commit c5633912 td 0.002 matches the 09-08 log; the first three 07-17 logs carry earlier live values from status). The `limit` is not in status: roll/pitch 5.2e-3 (rate_sysid DEFAULT_GAINS), yaw limit ASSUMED by date (2.2e-3 June, 1.83e-3 July, 1.226e-3 Sep/Oct) -- yaw is therefore the least certain axis. "air" span = FLYING, +2 s after takeoff, before flip; "whole" = the full log as `sf sysid rate-fit` would take it.
- FRF: H = Pyu/Puu (Welch, Hann, 50 % overlap, 2-s segments = 0.5 Hz resolution) from the duty differential u to the gyro rate y, over the air span. Differentials from actuator.cpp mixerCompute (M1=FR, M2=RR, M3=RL, M4=FL): roll = (M3+M4)-(M1+M2), pitch = (M1+M4)-(M2+M3), yaw = (M1+M3)-(M2+M4). 50 Hz logs: gyro = 8-sample boxcar, sampled at ctrl_ref times. Native bundles: 400 Hz duty and 400 Hz gyro. Data are closed-loop, so H is the biased closed-loop ETFE, not the plant. phase_model = -90 deg - atan(2 pi f T) - 360 f L with the (T,L) from the same log's air/kp fit.
''')
# ---- table of logs
P('## 1. Table of logs\n')
P('| log | date group | duration s | airborne s (status) | mode(s) | V min..max | span V | current_ma | hover s used (T1 windows) | imu dup ts | notes |')
P('|---|---|---|---|---|---|---|---|---|---|---|')
for k,s in SAN.items():
    if k=='sweep_steps': continue
    t1=T1.get(k); hs=f(t1.get('hover_s_in_windows'),1) if t1 else '-'
    note=''
    if k in ('flip_test1','flip_test2'): note='contains flip; native bundle, 400 Hz duty'
    if k=='motor_sweep_bench': note='bench, no flight'
    if k=='ground_spin_stab': note='ground, motors spinning'
    if k=='20260717T231940': note='gyro peak 23 rad/s (landing/crash?)'
    if k=='20260718T064222': note='gyro peak 48 rad/s; mostly ground/hand-held states'
    if k=='20260627T165713': note='gyro peak 6.1 rad/s, imu gaps up to 0.8 s'
    if k=='20260627T164611': note='imu gaps up to 0.54 s'
    mode=','.join({0:'ACRO',1:'STAB',2:'ALT',3:'POS'}.get(m,str(m)) for m in (s['modes'] or []))
    P(f"| {k} | {k[:8]} | {s['duration_s']:.0f} | {s['airborne_s']:.0f} | {mode} | {s['V_min']:.3f}..{s['V_max']:.3f} | {s['V_max']-s['V_min']:.2f} | {'yes (x10)' if s['has_current'] else 'no'} | {hs} | {s['imu_dup_ts']} | {note} |")
P('\nMode legend: ACRO=0, STAB=1, ALT_HOLD=2, POS_HOLD=3 (mode column of ctrl_ref; includes pre-flight values).\n')
# ---- Test 1
P('## 2. Test 1: hover duty versus battery voltage\n')
P('Pass/fail criteria as in the brief: H_chop predicts no trend of D-bar*V vs V (slope CI containing 0); H_rout predicts D-bar(V) following its curve. "Too small" = V span < 0.15 V.\n')
P('| log | group | #win | hover s | V span | D*V mean | slope d(DV)/dV [V/V] | 95 % CI | slope % per V | meas dD/dV | CI | dD/dV chop | dD/dV rout | d(DV)/dV implied by rout | RMS chop | RMS rout | note |')
P('|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|')
for k,r in T1.items():
    if r['n_win']==0:
        P(f"| {k} | {r['group']} | 0 | 0 | - | - | - | - | - | - | - | - | - | - | - | - | no hover window passes the gates (hover_total_s={r['diag']['hover_total_s']:.1f}; frac vz-ok={r['diag']['frac_vz']:.2f}) |"); continue
    note='SPAN TOO SMALL (<0.15 V)' if r['too_small_span'] else ''
    if r['n_win']<3: note+=' ; <3 windows: no slope CI'
    Vm=np.mean([w['V'] for w in r['rows']]);Dm=np.mean([w['Dbar'] for w in r['rows']]);imp=r['rout_dDdV']*Vm+Dm
    P(f"| {k} | {r['group']} | {r['n_win']} | {r['hover_s_in_windows']:.1f} | {r['Vspan']:.3f} | {r['DV_mean']:.3f} | {f(r.get('DV_slope'))} | +-{f(r.get('DV_slope_ci95'))} | {f(r.get('DV_slope_rel_pct_per_V'),1)} | {f(r.get('meas_dDdV'))} | +-{f(r.get('meas_dDdV_ci95'))} | {f(r.get('chop_dDdV'))} | {f(r.get('rout_dDdV'))} | {imp:.3f} | {f(r['rms_chop'],4)} | {f(r['rms_rout'],4)} | {note} |")
P('\n`_relaxed` rows: flip logs re-run with |vz|<0.6 m/s and |az|<3 m/s^2 because they are stick-flown (STABILIZE) with large altitude motion; this is a deviation from the stated hover gate and the V span is tiny, so it carries no discriminating power.\n')
# pooled by group
P('### Test 1 pooled per date group (within-log regression; each log normalised to its own mean point)\n')
P('| group | logs used | #win | pooled slope d(DV)/dV | 95 % CI | RMS chop | RMS rout | mean V span of logs |')
P('|---|---|---|---|---|---|---|---|')
groups={}
for k,r in T1.items():
    if r['n_win']>=1 and not k.endswith('_relaxed'): groups.setdefault(r['group'],[]).append(k)
for g,ks in groups.items():
    Vs=[];Ps=[];rc=[];rr=[];lab=0;spans=[]
    for k in ks:
        r=T1[k]
        if r['too_small_span']: continue
        V=np.array([w['V'] for w in r['rows']]);D=np.array([w['Dbar'] for w in r['rows']]);Pp=D*V
        Vs+=list(V-V.mean());Ps+=list(Pp-Pp.mean());lab+=1;spans.append(r['Vspan'])
        rc+=list((D-np.array(r['Dchop']))**2);rr+=list((D-np.array(r['Drout']))**2)
    used=[k for k in ks if not T1[k]['too_small_span']]
    if len(Vs)<4 or lab==0:
        P(f"| {g} | none usable (all spans < 0.15 V) | - | - | - | - | - | - |"); continue
    Vs=np.array(Vs);Ps=np.array(Ps);dof=len(Vs)-lab-1;b=(Vs@Ps)/(Vs@Vs);res=Ps-b*Vs;se=np.sqrt(res@res/dof/(Vs@Vs));ci=stats.t.ppf(.975,dof)*se
    P(f"| {g} | {len(used)} ({', '.join(u[9:] for u in used)}) | {len(Vs)} | {b:.3f} | +-{ci:.3f} | {np.sqrt(np.mean(rc)):.4f} | {np.sqrt(np.mean(rr)):.4f} | {np.mean(spans):.2f} |")
P('\nThe column "implied by rout" is d(DV)/dV = V*dD/dV|rout + D at the log mean point (H_chop implies 0).\n')
# ---- Test 2
P('## 3. Test 2: battery current\n')
P('Ratio = measured / predicted battery current (measured = current_ma*10 - overhead). Median [16th..84th percentile], n = 1-s samples with predicted I > 0.02 A. Pass/fail criterion from the brief: H_rout is "wrong" if its ratio is beyond 1 +- 0.08.\n')
P('| log | group | overhead mA (n) | n | H_rout med [p16..p84] | H_chop med [p16..p84] | SILS plant med [p16..p84] | H_rout within 1+-0.08? |')
P('|---|---|---|---|---|---|---|---|')
for k,r in T2.items():
    s=r['summary']['pred']
    if s['rout'] is None:
        P(f"| {k} | {r['group']} | {r['overhead_mA']:.0f} ({r['n_overhead_samples']}) | {r['n']} | no samples passed | | | |"); continue
    c=lambda h: f"{s[h]['median']:.3f} [{s[h]['p16']:.3f}..{s[h]['p84']:.3f}]" if s[h] else 'n/a'
    P(f"| {k} | {r['group']} | {r['overhead_mA']:.0f} ({r['n_overhead_samples']}) | {s['rout']['n']} | {c('rout')} | {c('chop')} | {c('sils')} | {'yes' if abs(s['rout']['median']-1)<=0.08 else 'NO'} |")
P('\n### Test 2 pooled per date group (hover logs only)\n')
P('| group | n | H_rout median [p16..p84] | H_chop | SILS plant |')
P('|---|---|---|---|---|')
gr={}
for k,r in T2.items():
    if r['mode']!='hover' or r.get('relaxed'): continue
    for row in r['rows']:
        if row['I_meas_A'] is None: continue
        gr.setdefault(r['group'],[]).append(row)
for g,rows in gr.items():
    out=[]
    for h in ('rout','chop','sils'):
        ra=np.array([x['I_meas_A']/x['pred'][h] for x in rows if x['pred'][h]>0.02])
        out.append(f"{np.median(ra):.3f} [{np.percentile(ra,16):.3f}..{np.percentile(ra,84):.3f}]")
    P(f"| {g} | {len(rows)} | "+' | '.join(out)+' |')
P('\nSensitivity to the averaging convention (median ratio, H_rout / H_chop / SILS) for the same samples is in `test2.json` (`summary.pred_inst`, `summary.pred_centred`); differences between conventions are <= 0.02 for H_rout in all hover logs except 20260718T064222 (mixed ground/flight, large duty swings).\n')
P('### Test 2 on the bench/ground logs (whole duty range available)\n')
P('**What motor_sweep_bench does** (read from the 400 Hz motor stream): flight_state 1 (idle), STABILIZE mode, props state unknown. One motor at a time is driven at a single constant duty of 0.100 for ~2.0 s each, with ~1 s pauses: FR 13.94-15.95 s, RR 16.95-18.95 s, RL 19.96-21.97 s, FL 22.97-24.99 s; all other motors 0. So the sweep covers exactly ONE duty level (0.10), not a range. Battery voltage 4.11-4.13 V throughout.\n')
P('**What ground_spin_stab does**: armed on the ground (flight_state 3), STABILIZE, four duties varying independently between 0 and 0.61 (stabilize controller reacting while the airframe is on the ground; motor-by-motor mix changes over 36 s). V 4.02-4.18 V. Whether it was restrained or free to tilt, and whether propellers were fitted, is not recorded in the files.\n')
for key in ('motor_sweep_bench','ground_spin_stab'):
    r=T2[key]
    P(f'**{key}**: per 1-s status sample (duty window = the preceding second). I_meas = current_ma*10 - {r["overhead_mA"]:.0f} mA overhead.\n')
    if key=='motor_sweep_bench':
        P('| t s | V | D4 (FR,RR,RL,FL) | I_meas A | pred H_rout A | pred H_chop A | pred SILS A | meas/rout | meas/chop | meas/SILS |')
        P('|---|---|---|---|---|---|---|---|---|---|')
    rows=[x for x in r['rows'] if x['Dbar']>0.0005]
    if key=='ground_spin_stab':
        # bin by mean duty of four motors
        edges=[0,0.05,0.1,0.15,0.2,0.25,0.3,0.35,0.4,0.5,0.7]
        P('\n(binned by mean of the four duties; each cell = median over the samples in the bin)')
        P('| Dbar bin | n | I_meas A | pred rout A | pred chop A | pred SILS A | meas/rout | meas/chop | meas/SILS |')
        P('|---|---|---|---|---|---|---|---|---|')
        out=[]
        for a,b in zip(edges[:-1],edges[1:]):
            rs=[x for x in rows if a<=x['Dbar']<b and x['I_meas_A'] is not None]
            if not rs: continue
            m=lambda g: np.median([g(x) for x in rs])
            im=m(lambda x:x['I_meas_A']); pr=m(lambda x:x['pred']['rout']);pc=m(lambda x:x['pred']['chop']);ps=m(lambda x:x['pred']['sils'])
            P(f"| {a:.2f}-{b:.2f} | {len(rs)} | {im:.3f} | {pr:.3f} | {pc:.3f} | {ps:.3f} | {im/pr:.2f} | {im/pc:.2f} | {im/ps:.2f} |")
        # drop the per-row table header already printed
        L[-len(out)-1:]=L[-len(out)-1:]
    else:
        for x in rows:
            D4=','.join(f'{d:.3f}' for d in x['D4']); p=x['pred']; im=x['I_meas_A']
            P(f"| {x['t']:.0f} | {x['V']:.3f} | {D4} | {im:.3f} | {p['rout']:.3f} | {p['chop']:.3f} | {p['sils']:.3f} | {im/p['rout'] if p['rout']>0 else float('nan'):.2f} | {im/p['chop'] if p['chop']>0 else float('nan'):.2f} | {im/p['sils'] if p['sils']>0 else float('nan'):.2f} |")
    P('')
P('Note: at duty 0.05-0.10 the net current is 20-70 mA while the status current resolution is ~8 mA after x10 (current_ma has ~0.8 mA steps), so single-sample ratios are noisy there; and the sweep has only the 0.10 level.\n')
# ---- Test 3
P('## 4. Test 3: small-signal rate-loop identification (kp mode)\n')
P('b [1/s^2 x torque normalised as in rate_sysid], T, L in ms, T+L, mean coherence over the fit band 0.8-30 Hz, and the standard deviation of rate_ref and of the gyro over the fitted span (excitation). Note the T/L split is degenerate (T often collapses to 0, as already documented in analysis/reports/rate_sysid_reference/README.md); T+L is the quantity to compare. Reference (2026-06-14 logs, README): roll T+L 14.1-14.7 ms, pitch 16.2-18.3 ms, yaw 10.6-11.0 ms.\n')
for axis in ('roll','pitch','yaw'):
    P(f'### {axis}\n')
    P('| log | gains kp/ti/td | span | mode | b | T ms | L ms | T+L ms | coh | rate_ref std rad/s | gyro std rad/s | n |')
    P('|---|---|---|---|---|---|---|---|---|---|---|---|')
    for k,r in T3.items():
        g=r['gains'][axis]
        for key,v in r['fits'].items():
            vn,mode,ax=key.split('/')
            if ax!=axis: continue
            if 'error' in v:
                P(f"| {k} | {g[0]:.3e}/{g[1]:.2f}/{g[2]:.3f} | {vn} | {mode} | error: {v['error'][:60]} | | | | | | | |"); continue
            P(f"| {k} | {g[0]:.3e}/{g[1]:.2f}/{g[2]:.3f} | {vn} | {mode} | {v['b']:.0f} | {v['T']*1e3:.2f} | {v['L']*1e3:.2f} | {v['T_plus_L']*1e3:.2f} | {v['coh_mean']:.2f} | {v['rate_ref_std']:.3f} | {v['gyro_std']:.3f} | {v['n']} |")
    P('')
P('### Group summary of T+L (air span, kp mode), ms: median over logs\n')
P('| group | roll | pitch | yaw |'); P('|---|---|---|---|')
gg={}
for k,r in T3.items():
    for ax in ('roll','pitch','yaw'):
        v=r['fits'].get(f'air/kp/{ax}')
        if v and 'b' in v: gg.setdefault(r['group'],{}).setdefault(ax,[]).append(v['T_plus_L']*1e3)
for g,d in gg.items():
    P(f"| {g} | "+' | '.join((f"{np.median(d[a]):.1f} (n={len(d[a])}, range {min(d[a]):.1f}-{max(d[a]):.1f})" if a in d else '-') for a in ('roll','pitch','yaw'))+' |')
P('\nSource of gains: status.pid_* (see section 0). Flip logs: both `kp` and `duty` shown above; flip_test2 has no usable airborne pre-flip span (FLYING lasts 3.6 s, flip starts 1.7 s after takeoff + 2 s skip) so every fit fails with "no excitation rows".\n')
P('### Frequency response from duty differential to gyro (phase deg, coherence)\n')
P('Phase of H=Pyu/Puu. Expected for the plant b e^{-Ls}/(s(Ts+1)): -90 deg minus lag; the "model" value uses the fitted T,L of the same log. 50 Hz logs: gyro = centred 8-sample boxcar sampled at the 50 Hz ctrl_ref times (no added delay); the 50 Hz duty is a held (ZOH) sample, which adds an unmodelled ~half-period (10 ms = 29 deg at 8 Hz) lag relative to a continuous input, so 50 Hz phases and 400 Hz phases are not directly comparable. Phase wraps at +-180.\n')
for axis in ('roll','pitch'):
    P(f'#### {axis}\n')
    P('| log | rate | n samples | '+' | '.join(f'{fq} Hz phase / model / coh' for fq in (2,4,6,8,10))+' |')
    P('|---|---|---|---|---|---|---|---|')
    for k,r in T3.items():
        for kk,v in r['frf'].items():
            if not kk.startswith(axis) or v is None: 
                if kk.startswith(axis): P(f'| {k} | {kk} | too short | | | | | |')
                continue
            cells=[f"{p['phase_deg']:.0f} / {f(p.get('phase_model_deg'),0)} / {p['coh']:.2f}" for p in v['points']]
            P(f"| {k} | {kk.split('_')[1]} | {v['n_samples']} | "+' | '.join(cells)+' |')
    P('')
P('(yaw FRF is in test3.json; not tabulated.)\n')
P('**Caution on reading these FRF numbers.** The brief expected low coherence; it is often high (0.8-0.99 at 2-6 Hz) and the measured phase sits near -160..-180 deg, i.e. 50-80 deg below the fitted-model phase (-100..-140 deg). In closed loop with little external excitation (rate_ref std only 0.1-0.2 rad/s, section Test 3 tables), u is itself driven by y through the controller, so Pyu/Puu is dominated by the feedback path (approximately the inverse controller, phase near +-180 deg) rather than the plant; high coherence here does not mean the plant is identified. The same direct-ETFE structure underlies rate_sysid.fit_from_df (u reconstructed from r and y, then S_uy/S_uu), so its coherence and T+L carry the same caveat. This is reported as observed; no inference is drawn about which motor model is right.\n')
# ---- caveats
P('## 5. Things not done / suspicious\n')
P('''- **Duplicated imu timestamps**: in the 2026-06/07/09 JSONL logs 6-13 % of consecutive imu samples carry identical `ts` (see `imu dup ts` column; consistent with the known "IMU reused in ~13 % of control cycles" firmware issue). In the native bundles `motor`/`ctrl_output` timestamps are duplicated for 273-1188 rows (aligned-table artefact of 400 Hz packet batching). The vertical-acceleration gate therefore uses nominal sample spacing; with real timestamps np.gradient divided by zero and the gate silently rejected everything in the first attempt (fixed).
- **Time gaps**: 20260627T164611 (imu gaps to 0.54 s, status dt up to 4 s), 20260627T165713 (gaps to 0.80 s, status dt to 6 s), 20260614 logs (gaps ~45 ms), ground_spin_stab (0.285 s gap). Voltage for windows is interpolated across such status gaps.
- **status.voltage and current are 1 Hz instantaneous samples under load**; V sag fluctuates with the commanded thrust itself. The firmware divides thrust by the same voltage when forming the duty (thrustToDuty uses live sensor_power voltage), so D*V constancy in Test 1 mixes the plant with the firmware's own V compensation; errors-in-variables (V quantisation 4 mV, ripple) bias the slope of D*V on V positively and D on V negatively.
- **Test 1 windows are not independent** (battery sag is monotone in time; thrust-hover drift, DOB/learning integrators, mass-independent but time-correlated), so CI are optimistic. Voltage spans of single 1-min logs are 0.15-0.2 V only; the 3-min logs (06-27 020137, 164611; 06-29) reach 0.4 V. 06-14 and 09-08 logs too small (< 0.15 V) to discriminate.
- 20260629T145526: only 7 % of the time passes the gate because the vehicle hovered at ~0.3-0.6 m (alt > 0.3 m gate fails 90 % of FLYING time); 4 windows.
- 20260718T064222: mostly ground/hand-held states (flight_state 1,2) and gyro peak 48 rad/s; hover windows are few and scattered.
- **current_ma x10**: applied to all logs from 07-17 on, including 09-08 and the 10-02 bundles, as instructed. The idle level (16 mA raw -> 160 mA) and hover plateau (330-355 mA raw -> 3.3-3.5 A) are mutually consistent across the logs, but the x10 factor itself is not independently verified here. Overhead estimates rely on 1-5 idle samples in several logs (09-08: 3 samples, 200 mA vs 146-172 mA elsewhere).
- The hover current is compared with a *steady-state* model, but duties fluctuate by +-0.1 within a second in some logs; see the `pred_inst` variant in test2.json.
- **Flip logs**: flight_state is only 1 Hz and there is no flight_phase column in the bundles, so the flip was located from gyro norm > 5 rad/s (flip_test1 at 15.9 s, flip_test2 at 6.7 s). They are stick-flown STABILIZE flights with large altitude changes: the stated hover gate rejects nearly everything (only a relaxed variant gives 2 windows with 0.004 V span). flip_test2 has < 2 s of pre-flip flight and yields nothing for Tests 1-3 except 400 Hz overhead.
- In STABILIZE mode (flip logs, motor sweep, ground spin) total thrust is the pilot throttle, so there is no altitude-hold feedback that would reveal a V-dependence of thrust (Test 1 inapplicable).
- Rate-sysid: T collapses to 0 on most axes (degenerate T/L split, as documented). Yaw limit assumed by date (see section 0). Fits in `whole` span include ground segments and takeoffs, and for 20260717T231940 and 20260718T064222 the landing/crash gyro peaks (coherence drops to 0.4-0.6): `air` span is the one to use.
- ground_spin_stab: not known whether propellers were fitted or the airframe restrained.
- Not done: no thrust measurement exists in any log (only weight inferred), so H_rout's thrust at the hover point is taken from the model itself (per the brief) rather than from independent weight; yaw FRF not tabulated; no check of the INA3221 averaging window (the (t-1,t] trailing window is assumed; the centred alternative is in test2.json).
''')
open(OUT+'/logcheck_report.md','w').write('\n'.join(L))
print(len(L))
