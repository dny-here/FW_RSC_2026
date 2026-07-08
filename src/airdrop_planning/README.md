# airdrop_planning

Node perencanaan dropping presisi KRTI 2026 Divisi FW. Desain mengikuti
Mathisen et al., *Autonomous Ballistic Airdrop of Objects from a Small
Fixed-Wing UAV* (referensi `prec_drop`), Sec. 2.2–2.3.

## Alur kerja

```
target_recognition ──TargetEstimate──▶ airdrop_planner ──AirdropPlan──▶ guidance/bringup
        ▲                                   │  ▲                            │
        │                                   │  └── NavSatFix / Odometry /   ▼
     kamera                                 │      wind_estimation      ArduPilot (FCU)
                                            └──UInt8 drop_command──▶ node mekanisme drop
```

State machine: `IDLE → PLANNING → TRANSIT → (loiter di s) → FINAL_APPROACH → RELEASED`,
dengan cabang `MISSED → PLANNING` bila wahana melewati release point tanpa
masuk *release proximity circle* (paper, Fig. 1).

Di titik entry `p`, release point **dihitung ulang** memakai *ground velocity*
aktual wahana (bukan prediksi airspeed+angin) dengan heading garis
dipertahankan — persis strategi paper Sec. 2.3.

## Topik & service

| Arah | Nama | Tipe |
|---|---|---|
| sub | `target_recognition/target_estimate` | `interfaces/TargetEstimate` |
| sub | `mavros/global_position/global` | `sensor_msgs/NavSatFix` |
| sub | `mavros/local_position/odom` | `nav_msgs/Odometry` (ENU→NED di node) |
| sub | `mavros/wind_estimation` | `geometry_msgs/TwistWithCovarianceStamped` |
| pub | `airdrop/plan` | `interfaces/AirdropPlan` |
| pub | `airdrop/status` | `interfaces/AirdropStatus` |
| pub | `airdrop/drop_command` | `std_msgs/UInt8` (bay index) |
| srv | `airdrop/start` | `interfaces/StartAirdrop` |

## Build & test

```bash
colcon build --packages-select interfaces airdrop_planning
source install/setup.bash
ros2 launch airdrop_planning airdrop_planning.launch.py
colcon test --packages-select airdrop_planning   # unit test model balistik
```

## Identifikasi C_D (WAJIB sebelum lomba)

Akurasi drop hampir sepenuhnya ditentukan `ballistic.drag_coeff` dan
`ballistic.ref_area_m2`:

1. Terbang lurus pada ketinggian & airspeed tetap, lepas payload manual di
   titik GPS tercatat, ukur titik jatuh.
2. Ulangi ≥ 5 kali, arah bervariasi terhadap angin.
3. Cari `C_D` yang meminimalkan error prediksi model (sweep sederhana
   dengan `BallisticModel::simulate` cukup).
4. Perhitungkan juga `release.mechanism_latency` (ukur delay servo dengan
   video slow-motion).

Paper mencapai mean error 5,5 m pada release 50 m AGL, Va 18 m/s —
target DZ KRTI 5 m × 5 m menuntut tuning yang lebih ketat + approach
distance `d` cukup panjang agar transien roll hilang sebelum release.
