#!/usr/bin/env python3
"""実走ログ（Chibi-T EFI ロガーの CSV）から、全開バーン区間のエンジン出力・トルクを推定する。

F = m·a + m·g·勾配 + Crr·m·g + ½·ρ·CdA·v²  →  Pw = F·v、Pe = Pw/η、T = Pe/ω

使い方:
    python analysis/road_power_estimate.py data/logs/LOG0672_燃料消費量燃費補正済.CSV \
        --out data/derived/power_curve_LOG0672.csv --fig figures/power_torque_curve_LOG0672.png

前提: スロットルバルブのないエンジン（バーン中は常に全開）、クラッチ直結区間のみ採用。
Crr・CdA は、LOG0672 のエンジン停止中の惰行区間を勾配補正したうえで中央値回帰した値（既定値）。
"""
import argparse
import numpy as np
import pandas as pd
from scipy.ndimage import gaussian_filter1d
from scipy.signal import savgol_filter

COLS = ['t_str', 'v', 'lap', 'tsec', 'rpm', 'inj', 'adv', 'injend', 'dist', 'fuel', 'kml', 'lat', 'lng', 'alt',
        'place', 'tmp', 'p', 'tair', 'hum', 'p1', 'p2', 'pf']
G = 9.80665


def load_grid(path, dt=0.1):
    """同一時刻の重複行をまとめ、dt 秒の格子に整形する（車速=平均、回転数=最大、欠測は 3 秒まで線形補間）。"""
    df = pd.read_csv(path, encoding='utf-8-sig')
    df = df.iloc[:, :len(COLS)]
    df.columns = COLS
    t = pd.to_datetime(df.t_str, format='%Y/%m/%d %H:%M:%S.%f')
    df['ts'] = (t - t.iloc[0]).dt.total_seconds()
    g = df.groupby('ts').agg(v=('v', 'mean'), rpm=('rpm', 'max'), inj=('inj', 'max'), dist=('dist', 'max'),
                             fuel=('fuel', 'max'), alt=('alt', 'mean'), p=('p', 'mean'), tair=('tair', 'mean'),
                             hum=('hum', 'mean'), lap=('lap', 'max')).reset_index()
    u = pd.DataFrame({'ts': np.round(np.arange(0, g.ts.max() + dt / 2, dt), 1)}).merge(g, on='ts', how='left')
    for c in ['v', 'dist', 'fuel', 'alt', 'p', 'tair', 'hum', 'lap']:
        u[c] = u[c].interpolate(limit=30).ffill()
    u[['rpm', 'inj']] = u[['rpm', 'inj']].fillna(0)
    u['vms'] = u.v / 3.6
    return u


def air_density(p_kpa, t_c, rh_pct):
    """乾燥空気＋水蒸気（飽和水蒸気圧は Magnus 式）。"""
    pv = rh_pct / 100 * 611.2 * np.exp(17.62 * t_c / (243.12 + t_c))
    T = t_c + 273.15
    return (p_kpa * 1000 - pv) / (287.05 * T) + pv / (461.5 * T)


def grade_profile(u, bin_m=10.0, sigma_bins=5, alt_band=7.0):
    """周回ごとの標高を周回内位置 bin_m ごとに集計 → 周平均を引いた偏差の中央値 → 円周ガウス平滑 → 中央差分。"""
    u = u.copy()
    u['vs21'] = savgol_filter(u.vms, 21, 2)
    u['lapi'] = u.lap.round().astype(int)
    u['ph'] = u.dist - u.lapi.map(u.groupby('lapi').dist.min())
    med_alt = u.alt.median()
    u['alt_c'] = u.alt.where(u.alt.between(med_alt - alt_band, med_alt + alt_band)).interpolate(limit=50)
    laps = sorted(u.lapi.unique())[1:-1]                      # 最初と最後の周は除く（不完全な周）
    full = u[u.lapi.isin(laps) & (u.vs21 > 2)].copy()
    full['b'] = (full.ph // bin_m).astype(int)
    nb = int(full.groupby('lapi').ph.max().median() // bin_m)
    prof = full.groupby(['lapi', 'b']).alt_c.mean().unstack(0).reindex(range(nb))
    med = prof.sub(prof.mean(axis=0), axis=1).median(axis=1).interpolate().bfill().ffill()
    pad = np.concatenate([med.values[-30:], med.values, med.values[:30]])
    sm = gaussian_filter1d(pad, sigma_bins)[30:-30]
    slope = pd.Series(np.gradient(sm, bin_m))
    b = (u.ph // bin_m).astype(int).clip(0, nb - 1)
    s = b.map(slope)
    s[~u.lapi.isin(laps)] = 0.0                                # 勾配の分からない周は 0
    return s.values, laps, nb


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('log')
    ap.add_argument('--mass', type=float, default=100.0, help='車両＋ドライバー [kg]')
    ap.add_argument('--crr', type=float, default=0.00215)
    ap.add_argument('--cda', type=float, default=0.071, help='[m²]')
    ap.add_argument('--eta', type=float, default=0.90, help='駆動効率')
    ap.add_argument('--sg', type=int, default=15, help='加速度の Savitzky-Golay 窓（点数、0.1 s 刻み）')
    ap.add_argument('--ratio', type=float, nargs=2, default=(112, 128), help='クラッチ直結とみなす rpm/(km/h)')
    ap.add_argument('--rpm', type=float, nargs=2, default=(2400, 4300))
    ap.add_argument('--out', default=None)
    ap.add_argument('--fig', default=None)
    a = ap.parse_args()

    u = load_grid(a.log)
    rho = air_density(u.p.mean(), u.tair.mean(), u.hum.mean())
    slope, laps, nb = grade_profile(u)
    u['vs'] = savgol_filter(u.vms, a.sg, 2)
    u['acc'] = savgol_filter(u.vms, a.sg, 2, deriv=1, delta=0.1)
    u['F'] = a.mass * u.acc + a.mass * G * slope + a.crr * a.mass * G + 0.5 * rho * a.cda * u.vs ** 2
    u['Pe'] = u.F * u.vs / a.eta
    u['T'] = np.where(u.rpm > 500, u.Pe / (2 * np.pi * u.rpm / 60), np.nan)

    on = (u.rpm > 500).astype(int).rolling(5, center=True, min_periods=1).max()
    seg = (on.diff().fillna(0) != 0).cumsum()
    b = u[(on == 1) & (u.rpm > 500)].copy()
    b['seg'] = seg[b.index]
    b['t_in'] = b.groupby('seg').ts.transform(lambda s: s - s.min())
    b['t_out'] = b.groupby('seg').ts.transform(lambda s: s.max() - s)
    b['ratio'] = b.rpm / (b.vs * 3.6)
    x = b[(b.t_in >= 1) & (b.t_out >= 1) & (b.vs > 1) & b.ratio.between(*a.ratio) & b.rpm.between(*a.rpm)].copy()
    x['PS'] = x.Pe / 735.5
    x['rb'] = x.rpm // 250 * 250 + 125
    K = x.ratio.median()
    q = lambda p: (lambda s: s.quantile(p))
    c = x.groupby('rb').agg(n=('PS', 'size'), PS=('PS', 'mean'), PS10=('PS', q(.1)), PS90=('PS', q(.9)),
                            T=('T', 'mean'), T10=('T', q(.1)), T90=('T', q(.9)))
    c = c[c.n >= 30]
    c['kW'] = c.PS * 0.7355
    c['v_kmh'] = c.index / K
    print(f'空気密度 {rho:.4f} kg/m³、勾配プロファイル {nb} ビン（周 {laps}）、採用点 {len(x)}、rpm/(km/h) 中央値 {K:.1f}')
    print(c.round(3).to_string())
    print(f'最大出力 {c.PS.max():.2f} PS @ {c.PS.idxmax():.0f} rpm、最大トルク {c["T"].max():.2f} N·m @ {c["T"].idxmax():.0f} rpm')
    if a.out:
        c.rename_axis('rpm_bin_center').round(4).to_csv(a.out)
    if a.fig:
        plot(x, c, K, a, a.fig)


def plot(x, c, K, a, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    jp = [f.name for f in font_manager.fontManager.ttflist if 'CJK JP' in f.name or 'IPA' in f.name]
    if jp:
        plt.rcParams['font.family'] = jp[0]
    c1, c2 = '#1f6fb4', '#d0552b'
    fig, (p1, p2) = plt.subplots(2, 1, figsize=(8.5, 7.2), dpi=150, sharex=True, gridspec_kw={'hspace': .08})
    p1.scatter(x.rpm, x.PS, s=5, color=c1, alpha=.18, lw=0)
    p1.fill_between(c.index, c.PS10, c.PS90, color=c1, alpha=.14, lw=0)
    p1.plot(c.index, c.PS, '-o', color=c1, lw=2.4, ms=5)
    p2.scatter(x.rpm, x['T'], s=5, color=c2, alpha=.18, lw=0)
    p2.fill_between(c.index, c.T10, c.T90, color=c2, alpha=.14, lw=0)
    p2.plot(c.index, c['T'], '-s', color=c2, lw=2.4, ms=5)
    p1.set_ylabel('出力 (PS)'); p2.set_ylabel('トルク (N·m)'); p2.set_xlabel('エンジン回転数 (rpm)')
    p1.set_ylim(0, 1.4); p2.set_ylim(0, 3.0); p2.set_xlim(*a.rpm)
    p1.secondary_yaxis('right', functions=(lambda v: v * 0.7355, lambda v: v / 0.7355)).set_ylabel('出力 (kW)')
    top = p1.secondary_xaxis('top', functions=(lambda r: r / K, lambda v: v * K))
    top.set_xlabel(f'車速の目安 (km/h)  クラッチ直結時は rpm を {K:.0f} で割った値')
    for ax in (p1, p2):
        ax.grid(alpha=.25)
    p1.set_title(f'推定エンジン出力・トルク（全開バーン・クラッチ直結区間、{a.mass:.0f} kg・η={a.eta}）', fontsize=11, pad=38)
    fig.savefig(path, bbox_inches='tight')


if __name__ == '__main__':
    main()
