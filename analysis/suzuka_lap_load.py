#!/usr/bin/env python3
"""鈴鹿東コースの勾配プロファイルを作り、燃費優先の走り方で定常周回を再現して、
ダイナモの負荷装置が出すべき力・パワー（負荷プロファイル）を求める。

  負荷の指令  F_cmd = M·g·(Crr + 勾配) + M·k·v² + (M − M_eq)·a − F_loss(v)

使い方:
    python analysis/suzuka_lap_load.py                 # 乾いた路面、損失ゼロ・M_eq = M
    python analysis/suzuka_lap_load.py --rain          # 雨の走行抵抗と速度の上限
    python analysis/suzuka_lap_load.py --loss data/derived/dyno_LOG0650.json   # 現行ダイナモの損失を差し引く

走り方は config/suzuka_2027_strategy_params.json（UNOR4_Chibi-T_EFI の戦略ファイルの写し）による。
戦略シミュレータ（UNOR4_Chibi-T_EFI/tools/strategy_sim.py）とは独立に組み直した近似で、
加速の位置・頂上の速度・1 周のタイムは戦略書と数 m・1 km/h・2% 程度で一致する。
"""
import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.ndimage import gaussian_filter1d

G = 9.80665
ROOT = Path(__file__).resolve().parents[1]


def grade_profile(waypoint_csv, sigma_m=15.0, step=1.0):
    """ウェイポイント（緯度・経度・標高）を閉じた線として 1 m ごとに補間し、円周状にガウス平滑して勾配を求める。"""
    w = pd.read_csv(waypoint_csv)
    R = 6371000.0
    la, lo = np.radians(w.lat.values), np.radians(w.lng.values)
    hv = lambda a1, o1, a2, o2: 2 * R * np.arcsin(np.sqrt(np.sin((a2 - a1) / 2) ** 2 + np.cos(a1) * np.cos(a2) * np.sin((o2 - o1) / 2) ** 2))
    d = np.concatenate([[0], np.cumsum(hv(la[:-1], lo[:-1], la[1:], lo[1:]))])
    L = d[-1] + hv(la[-1], lo[-1], la[0], lo[0])
    x = np.arange(0, L, step)
    z = np.interp(x, np.append(d, L), np.append(w.alt.values, w.alt.values[0]))
    n = int(4 * sigma_m / step) + 50
    zs = gaussian_filter1d(np.concatenate([z[-n:], z, z[:n]]), sigma_m / step)[n:-n]
    lat = np.interp(x, np.append(d, L), np.append(w.lat.values, w.lat.values[0]))
    lng = np.interp(x, np.append(d, L), np.append(w.lng.values, w.lng.values[0]))
    return pd.DataFrame({'s_m': x, 'lat': lat, 'lng': lng, 'alt_m': z, 'alt_smooth_m': zs, 'grade': np.gradient(zs, step)}), L


def in_zones(x, zones, L):
    for a, b in zones:
        if (a <= b and a <= x < b) or (a > b and (x >= a or x < b)):
            return True
    return False


def simulate(prof, L, P, laps=3, dt=0.02, rain=False):
    veh = dict(P['vehicle']); crs = P['course']
    vmax_zones = crs['v_max_zones_m']
    if rain:
        veh.update(P['variants']['rain']['vehicle']); vmax_zones = P['variants']['rain']['course']['v_max_zones_m']
    CRR, K = veh['crr'], veh['k_aero']
    A_KMH, A_REAL = np.array(P['burn']['accel_real_kmh']), np.array(P['burn']['accel_real'])
    LAG, DELAY, GUARD = P['burn']['cut_lag_s'], P['restart']['delay_s'], crs['guard_kmh']
    crests, coast = crs['crest_cut'], crs['coast_only_zones_m']
    GR = prof.grade.values
    sl = lambda x: GR[int(x) % len(GR)]
    a_drv = lambda v: np.interp(v * 3.6, A_KMH, A_REAL)
    a_cst = lambda x, v: -(G * (CRR + sl(x)) + K * v * v)

    def vcap(x):
        for a, b, kmh in vmax_zones:
            if in_zones(x, [(a, b)], L):
                return kmh / 3.6
        return 1e9

    def reaches(x, v, c):
        t = 0.0
        while x < c['crest_m']:
            v += ((a_drv(v) if t < LAG else 0.0) + a_cst(x, v)) * 0.05; x += v * 0.05; t += 0.05
            if v * 3.6 < c['target_kmh']:
                return False
        return True

    x0 = 1914.0
    x, v, t = x0, 40 / 3.6, 0.0
    state, timer, crest, cut_until = 'coast', 0.0, None, None
    rec = []
    while x < x0 + laps * L and t < 4000:
        xl = x % L
        if cut_until is not None and (xl >= cut_until['crest_m'] or xl < cut_until['crest_m'] - 700):
            cut_until = None
        if state in ('starting', 'burn') and in_zones(xl, coast, L):
            state = 'coast'
        held = cut_until is not None and v * 3.6 >= cut_until['restart_kmh']
        if state == 'coast' and not held and not in_zones(xl, coast, L) and v * 3.6 <= GUARD:
            state, timer = 'starting', 0.0
            crest = next((c for c in crests if c['from_m'] - 400 <= xl < c['crest_m']), None)
        if state == 'starting':
            timer += dt
            if timer >= DELAY:
                state = 'burn'
        if state == 'burn':
            if crest and xl >= crest['from_m'] and reaches(xl, v, crest):
                state, timer, cut_until = 'lag', 0.0, crest
            elif crest is None and v * 3.6 >= P.get('v_off', 29.0):
                state, timer = 'lag', 0.0
        if state == 'lag':
            timer += dt
            if timer >= LAG:
                state = 'coast'
        on = state in ('burn', 'lag')
        ad = a_drv(v) if on else 0.0
        a = ad + a_cst(xl, v)
        brake = 0.0
        if v >= vcap(xl) and a > 0:
            brake, a = a, 0.0
        rec.append((t, x, xl, v, on, ad, a, brake, sl(xl)))
        v = max(v + a * dt, 0.0); x += v * dt; t += dt
    r = pd.DataFrame(rec, columns=['t', 'x', 's', 'v', 'on', 'a_drive', 'a', 'a_brake', 'grade'])
    r = r[r.x >= x0 + (laps - 1) * L].copy()
    r['t'] -= r.t.iloc[0]
    return r, CRR, K, veh['mass_kg']


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--waypoints', default=ROOT / 'data/course/suzuka_east_waypoint.csv')
    ap.add_argument('--params', default=ROOT / 'config/suzuka_2027_strategy_params.json')
    ap.add_argument('--rain', action='store_true')
    ap.add_argument('--meq', type=float, default=None, help='ダイナモの回転マス [kg]（既定は車両質量と同じ）')
    ap.add_argument('--loss', default=None, help='損失の係数 JSON（dyno_log_analysis.py の出力）。指定しなければ損失ゼロ')
    ap.add_argument('--outdir', default=ROOT / 'data/derived')
    ap.add_argument('--fig', default=ROOT / 'figures/suzuka_dyno_requirements.png')
    a = ap.parse_args()
    P = json.load(open(a.params))
    prof, L = grade_profile(a.waypoints)
    r, CRR, K, M = simulate(prof, L, P, rain=a.rain)
    Meq = a.meq or M
    c = json.load(open(a.loss))['loss_N'] if a.loss else {'c0': 0, 'c1': 0, 'c2': 0}
    loss = c['c0'] + c['c1'] * r.v + c['c2'] * r.v ** 2
    r['F_cmd'] = M * G * (CRR + r.grade) + M * K * r.v ** 2 + (M - Meq) * r.a - loss
    r['P_cmd'] = r.F_cmd * r.v
    r['F_drive'] = M * r.a_drive
    dt = 0.02
    fuel = (np.interp(r.v[r.on] * 3.6, P['burn']['speed_kmh'], P['burn']['fuel_ml_s']) * dt).sum()

    seg = (r.on != r.on.shift()).cumsum()
    burns = [dict(from_m=round(g.s.iloc[0]), to_m=round(g.s.iloc[-1]), v_from_kmh=round(g.v.iloc[0] * 3.6, 1),
                  v_to_kmh=round(g.v.max() * 3.6, 1), dur_s=round(len(g) * dt, 1), grade_mean_pct=round(g.grade.mean() * 100, 1))
             for _, g in r[r.on].groupby(seg[r.on])]
    crest_v = {c_['crest_m']: round(float(r.v[(r.s - c_['crest_m']).abs().idxmin()] * 3.6), 1) for c_ in P['course']['crest_cut']}
    E = lambda s: round(float(s.sum() * dt / 1000), 1)
    summary = dict(
        condition='rain' if a.rain else 'dry', mass_kg=M, meq_kg=Meq, loss=c, lap_m=round(L, 1),
        lap_s=round(float(r.t.iloc[-1]), 1), v_min_kmh=round(float(r.v.min() * 3.6), 1), v_max_kmh=round(float(r.v.max() * 3.6), 1),
        grade_max_pct=round(float(prof.grade.max() * 100), 2), grade_min_pct=round(float(prof.grade.min() * 100), 2),
        alt_range_m=[round(float(prof.alt_m.min()), 1), round(float(prof.alt_m.max()), 1)],
        burns=burns, crest_speed_kmh=crest_v, fuel_ecu_ml_per_lap=round(float(fuel + len(burns) * P['restart']['fuel_ml']), 2),
        F_cmd_N=[round(float(r.F_cmd.min()), 1), round(float(r.F_cmd.max()), 1)],
        P_cmd_W=[round(float(r.P_cmd.min())), round(float(r.P_cmd.max()))],
        absorb_time_pct=round(float((r.F_cmd > 0).mean() * 100)),
        energy_absorb_kJ=E(r.P_cmd.clip(lower=0)), energy_motoring_kJ=E(-r.P_cmd.clip(upper=0)),
        F_drive_max_N=round(float(r.F_drive.max()), 1), dFdt_max_N_per_s=round(float(np.abs(np.gradient(r.F_cmd.values, dt)).max()), 1))
    print(json.dumps(summary, ensure_ascii=False, indent=1))

    out = Path(a.outdir); out.mkdir(parents=True, exist_ok=True)
    tag = ('rain' if a.rain else 'dry') + ('_with_loss' if a.loss else '')
    if not a.loss and not a.rain:
        prof.round({'lat': 7, 'lng': 7, 'alt_m': 2, 'alt_smooth_m': 3, 'grade': 5}).to_csv(out / 'suzuka_east_grade_1m.csv', index=False)
    rr = r.iloc[::5]   # 0.1 秒刻み
    pd.DataFrame({'t_s': rr.t.round(2), 's_m': rr.s.round(1), 'v_kmh': (rr.v * 3.6).round(2), 'engine_on': rr.on.astype(int),
                  'grade': rr.grade.round(5), 'accel_mps2': rr.a.round(4), 'F_drive_N': rr.F_drive.round(2),
                  'F_cmd_N': rr.F_cmd.round(2), 'P_cmd_W': rr.P_cmd.round(1)}).to_csv(out / f'suzuka_lap_load_{tag}.csv', index=False)
    json.dump(summary, open(out / f'suzuka_lap_load_{tag}_summary.json', 'w'), ensure_ascii=False, indent=1)
    if a.fig and not a.loss and not a.rain:
        plot(prof, r, M, a.fig)


def plot(s, r, M, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    jp = [f.name for f in font_manager.fontManager.ttflist if 'CJK JP' in f.name or 'IPA' in f.name]
    if jp:
        plt.rcParams['font.family'] = jp[0]
    r = r.sort_values('s')
    fig, ax = plt.subplots(3, 1, figsize=(9, 8.4), dpi=150, sharex=True, gridspec_kw={'hspace': .12, 'height_ratios': [1, 1, 1.2]})
    a = ax[0]; a.plot(s.s_m, s.alt_smooth_m, color='#333', lw=1.8, label='標高'); a.set_ylabel('標高 [m]')
    b = a.twinx()
    b.fill_between(s.s_m, s.grade * 100, 0, where=s.grade > 0, color='#d0552b', alpha=.35, lw=0, label='上り')
    b.fill_between(s.s_m, s.grade * 100, 0, where=s.grade < 0, color='#1f6fb4', alpha=.3, lw=0, label='下り')
    b.set_ylim(-8, 8); b.set_ylabel('勾配 [%]'); b.axhline(0, color='#666', lw=.5)
    h1, l1 = a.get_legend_handles_labels(); h2, l2 = b.get_legend_handles_labels()
    a.legend(h1 + h2, l1 + l2, fontsize=8, loc='upper left', ncol=3, frameon=False); a.grid(alpha=.25)
    a.set_title(f'鈴鹿東コース 定常周回（燃費優先: 加速2回・ブレーキなし）とダイナモの負荷指令（{M:.0f}kg）', fontsize=10.5)
    c = ax[1]; c.plot(r.s, r.v * 3.6, color='#333', lw=1.6)
    c.fill_between(r.s, 0, 80, where=r.on.values, color='#f4b183', alpha=.5, lw=0, label='エンジン燃焼（加速）')
    for xc, lab in [(1533, 'S字の頂上 約14km/h'), (1916, 'ダンロップ頂上 約10km/h')]:
        c.axvline(xc, color='#888', lw=.7, ls=':'); c.text(xc + 8, 62, lab, fontsize=7.5, color='#555')
    c.set_ylim(0, 80); c.set_ylabel('車速 [km/h]'); c.legend(fontsize=8, loc='upper center', frameon=False); c.grid(alpha=.25)
    d = ax[2]; d.plot(r.s, r.F_cmd, color='#2e7d32', lw=1.8, label='負荷装置の指令 [N]（損失ゼロ・回転マス=車両質量）')
    d2 = d.twinx(); d2.plot(r.s, r.P_cmd, color='#7b1fa2', lw=1, alpha=.7, label='指令パワー [W]'); d2.set_ylim(-1000, 1200)
    d2.set_ylabel('パワー [W]', color='#7b1fa2')
    d.axhline(0, color='#333', lw=.8); d.set_ylim(-80, 100); d.set_ylabel('力 [N]　＋吸収 ／ −力行')
    d.set_xlabel('コース上の距離 [m]（ウェイポイント ID 1 から、右回り）')
    h1, l1 = d.get_legend_handles_labels(); h2, l2 = d2.get_legend_handles_labels()
    d.legend(h1 + h2, l1 + l2, fontsize=8, loc='lower left', frameon=False); d.grid(alpha=.25); d.set_xlim(0, s.s_m.max())
    fig.savefig(path, bbox_inches='tight')


if __name__ == '__main__':
    main()
