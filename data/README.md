# データ

## logs/（元のログ、変更しない）

| ファイル | 内容 |
| --- | --- |
| `LOG0672_燃料消費量燃費補正済.CSV` | 2026 年全国大会 決勝（もてぎ、2026-10-25）の走行ログ。約 10 Hz、28,396 行。燃料消費量は燃費補正済み |
| `LOG0650.CSV` | 現行の加速度型ダイナモ（ミノウラのローラ φ105、40:19 増速のフライホイール、回転マス 105 kg 相当）での全開加速と惰行 |

どちらも Chibi-T EFI のロガーの CSV（UTF-8 BOM 付き）。解析で使う列（左から）:

| 列 | 名前 | 単位 |
| --- | --- | --- |
| 1 | 記録日時 | `YYYY/MM/DD hh:mm:ss.fff`（GNSS の時刻合わせで飛ぶことがある） |
| 2 | 速度 | km/h（GNSS、約 0.4 秒ごとに更新） |
| 3 | ラップ数 | |
| 5 | エンジン回転数 | rpm |
| 6 | 燃料噴射時間 | ms |
| 9 | 走行距離 | m |
| 10 | 積算燃料消費量 | ml |
| 12〜14 | 緯度・経度・標高 | °, °, m |
| 17〜19 | 気圧・気温・湿度 | kPa, °C, % |

## course/

| ファイル | 内容 |
| --- | --- |
| `suzuka_east_waypoint.csv` | 鈴鹿東コースのウェイポイント 300 点（`ID, lat, lng, alt, distance`）。右回り、ID 1 から。UNOR4_Chibi-T_EFI の戦略で使っているものと同じ |

## derived/（スクリプトの出力）

| ファイル | 作るスクリプト | 列・内容 |
| --- | --- | --- |
| `power_curve_LOG0672.csv` | `road_power_estimate.py` | `rpm_bin_center, n, PS, PS10, PS90, T, T10, T90, kW, v_kmh`（250 rpm ごと、P10/P90 は 10%/90% 点） |
| `dyno_LOG0650.json` | `dyno_log_analysis.py` | ダイナモ自体の損失 `loss_N`（c0 + c1·v + c2·v²、v は m/s）、rpm/(km/h)、出力カーブ |
| `suzuka_east_grade_1m.csv` | `suzuka_lap_load.py` | `s_m, lat, lng, alt_m, alt_smooth_m, grade`（1 m ごと、σ = 15 m で平滑、上り＋）。**ダイナモの制御器に入れる勾配表** |
| `suzuka_lap_load_<条件>.csv` | `suzuka_lap_load.py` | `t_s, s_m, v_kmh, engine_on, grade, accel_mps2, F_drive_N, F_cmd_N, P_cmd_W`（0.1 秒ごとの定常 1 周）。`F_cmd_N` は＋が吸収、−が力行 |
| `suzuka_lap_load_<条件>_summary.json` | `suzuka_lap_load.py` | 1 周のタイム、加速の位置、頂上の速度、燃料、力・パワーの範囲、エネルギー |

条件: `dry`（乾、損失ゼロ）、`rain`（雨）、`dry_with_loss`（乾、現行 φ105 の損失を差し引いた指令）。
