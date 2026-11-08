#!/usr/bin/env python3
"""現行の加速度型ダイナモ（ミノウラのローラ φ105・40:19 増速・回転マス 105 kg 相当）のログから、
① エンジン停止中の惰行減速でダイナモ自体の損失 F_loss(v) = c0 + c1·v + c2·v² を求め、
② 全開加速の区間でエンジン出力を推定する（F = M_eq·a + F_loss）。

使い方:
    python analysis/dyno_log_analysis.py data/logs/LOG0650.CSV --out data/derived/dyno_LOG0650.json

注意: 損失の当てはめは惰行 7〜29 km/h のデータからなので、それより高速の値は外挿になる。
"""
import argparse
import json
import numpy as np
import pandas as pd
from scipy.signal import savgol_filter

COLS = ['t', 'v', 'lap', 'tsec', 'rpm', 'inj', 'adv', 'injend', 'dist', 'fuel', 'kml', 'lat', 'lng', 'alt', 'place',
        'tmp', 'p', 'tair', 'hum', 'p1', 'p2', 'pf']


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('log')
    ap.add_argument('--meq', type=float, default=105.0, help='回転マス（ローラ表面での相当質量）[kg]')
    ap.add_argument('--eta', type=float, default=0.90)
    ap.add_argument('--out', default=None)
    a = ap.parse_args()

    df = pd.read_csv(a.log, encoding='utf-8-sig').iloc[:, :len(COLS)]
    df.columns = COLS
    t = pd.to_datetime(df.t, format='%Y/%m/%d %H:%M:%S.%f')
    d = (t - t.iloc[0]).dt.total_seconds().diff().fillna(0)
    d[d > 1] = 0.1                                   # GNSS の時刻合わせによる飛びを除く
    df['ts'] = d.cumsum()
    g = df.groupby(df.ts.round(1)).agg(v=('v', 'mean'), rpm=('rpm', 'max')).reset_index()
    u = pd.DataFrame({'ts': np.round(np.arange(0, g.ts.max(), 0.1), 1)}).merge(g, on='ts', how='left')
    u['v'] = u.v.interpolate(limit=30)
    u['rpm'] = u.rpm.fillna(0)
    vms = u.v.fillna(0) / 3.6
    u['vs'] = savgol_filter(vms, 15, 2)
    u['a'] = savgol_filter(vms, 15, 2, deriv=1, delta=0.1)

    # ① 惰行（前後 2 秒エンジンなし、2 m/s 超、減速中）
    run = (u.rpm > 0).astype(int).rolling(41, center=True, min_periods=1).max()
    c = u[(run == 0) & (u.vs > 2) & (u.a < 0)]
    X = np.column_stack([np.ones(len(c)), c.vs, c.vs ** 2])
    k = np.linalg.lstsq(X, -a.meq * c.a, rcond=None)[0]
    print(f'惰行 {len(c)} 点（{c.vs.min()*3.6:.1f}〜{c.vs.max()*3.6:.1f} km/h）: F_loss = {k[0]:.2f} + {k[1]:.3f}·v + {k[2]:.4f}·v² [N, v は m/s]')
    for kmh in (10, 20, 30, 40, 75):
        v = kmh / 3.6
        print(f'  {kmh:3d} km/h: {k[0] + k[1]*v + k[2]*v*v:6.1f} N' + ('（外挿）' if kmh > 29 else ''))

    # ② 出力（クラッチ直結区間）
    u['F'] = a.meq * u.a + np.clip(k[0] + k[1] * u.vs + k[2] * u.vs ** 2, 0, None)
    u['Pe'] = u.F * u.vs / a.eta
    u['ratio'] = u.rpm / (u.vs * 3.6)
    x = u[(u.rpm > 2400) & u.ratio.between(110, 130)].copy()
    x['rb'] = x.rpm // 250 * 250 + 125
    tab = x.groupby('rb').agg(n=('Pe', 'size'), PS=('Pe', lambda s: s.mean() / 735.5))
    tab = tab[tab.n >= 5]
    tab['T'] = tab.PS * 735.5 / (2 * np.pi * tab.index / 60)
    tab['PS_inertia_only'] = x.assign(Pi=a.meq * x.a * x.vs / a.eta).groupby('rb').Pi.mean().reindex(tab.index) / 735.5
    print(f'rpm/(km/h) 中央値 {x.ratio.median():.1f}')
    print(tab.round(3).to_string())
    if a.out:
        json.dump({'source': a.log, 'meq_kg': a.meq, 'eta': a.eta,
                   'loss_N': {'c0': round(k[0], 3), 'c1': round(k[1], 4), 'c2': round(k[2], 5),
                              'form': 'F = c0 + c1*v + c2*v^2 (v in m/s)', 'fit_range_kmh': [round(c.vs.min()*3.6, 1), round(c.vs.max()*3.6, 1)]},
                   'ratio_rpm_per_kmh': round(float(x.ratio.median()), 2),
                   'power_curve': tab.round(4).reset_index().to_dict(orient='records')},
                  open(a.out, 'w'), ensure_ascii=False, indent=1)


if __name__ == '__main__':
    main()
