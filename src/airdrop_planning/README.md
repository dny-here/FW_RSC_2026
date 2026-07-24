# airdrop_planning

Node perencanaan dropping presisi KRTI 2026 Divisi FW. Desain mengikuti
Mathisen et al., *Autonomous Ballistic Airdrop of Objects from a Small
Fixed-Wing UAV* (referensi `prec_drop`), Sec. 2.2–2.3.

## Alur kerja

Node **swakelola** (sesuai gambar arsitektur sistem): satu node yang
merencanakan **dan** menerbangkan approach drop, sehingga cukup
`airdrop_planning` + `target_recognition` yang berjalan di Raspberry Pi —
tanpa `mission_bridge` terpisah.

```
target_recognition ──action execute_airdrop (goal: TargetEstimate+bay_index)──▶ airdrop_planner
        ▲   kamera        ◀── feedback AirdropStatus @10Hz / result drop_successful ──┘
        │                                        │  ▲                    │
        │                                        │  └ NavSatFix/rel_alt/  │ DO_REPOSITION (tujuan
     (locked target)                             │    velocity_local/wind │ GUIDED) + DO_SET_SERVO
                                                 │      (MAVROS)          ▼   + set_mode AUTO → FCU
                                                 └──────────────────────────┘
```

Guidance memakai **L1/TECS ArduPilot**: node hanya mengirim *setpoint posisi*
lewat `MAV_CMD_DO_REPOSITION` (COMMAND_INT), bukan roll/pitch (kotak "LOS
Guidance" pada rancangan tidak diimplementasi). Perintah, bukan stream —
dikirim sekali per perubahan tujuan, di-retry hanya bila FCU menolak.
`setpoint_position/global` **tidak** dipakai: ArduPlane membuang lat/lon-nya
pada fixed-wing (lihat CLAUDE.md).

State machine: `IDLE → PLANNING → TRANSIT → LOITER (di s) → FINAL_APPROACH → RELEASED`,
dengan cabang `MISSED → PLANNING` (≤ `max_replans`) bila wahana melewati release
point tanpa masuk *release proximity circle* (paper, Fig. 1). Referensi terbang
pertama adalah **loiter center s**; `p` adalah titik singgung pada orbit CW di
sekitar s — di sanalah `entryCaptured()` terpenuhi. Di `p`, release point
**dihitung ulang** memakai *ground velocity* aktual (heading dikunci), lalu
tujuan pindah ke titik *overshoot* di luar release point agar lintasan lurus
(paper Sec. 2.3–2.4). Setelah selesai/abort, node mengembalikan wahana ke AUTO.

## Antarmuka

| Arah | Nama | Tipe |
|---|---|---|
| action server | `execute_airdrop` | `interfaces/action/TargetAirdrop` (goal: `gps_estimation`+`bay_index`, feedback: `AirdropStatus`, result: `drop_successful`) |
| sub | `mavros/global_position/global` | `sensor_msgs/NavSatFix` |
| sub | `mavros/global_position/rel_alt` | `std_msgs/Float64` |
| sub | `mavros/local_position/velocity_local` | `geometry_msgs/TwistStamped` (ENU→NED di node) |
| sub | `mavros/wind_estimation` | `geometry_msgs/TwistWithCovarianceStamped` |
| pub | `airdrop/plan` | `interfaces/AirdropPlan` (telemetri geometri) |
| pub | `airdrop/status` | `interfaces/AirdropStatus` |
| srv client | `mavros/cmd/command_int` | `mavros_msgs/srv/CommandInt` (DO_REPOSITION, tujuan GUIDED) |
| srv client | `mavros/cmd/command` | `mavros_msgs/srv/CommandLong` (DO_SET_SERVO, lepas payload) |
| srv client | `mavros/set_mode` | `mavros_msgs/srv/SetMode` (resume AUTO) |

## Build & test

```bash
# interfaces WAJIB dibangun sebelum konsumennya
colcon build --symlink-install --packages-select interfaces airdrop_planning
source install/setup.bash
ros2 launch airdrop_planning airdrop_planning.launch.py
colcon test --packages-select airdrop_planning   # ballistic + geo_utils + release_point_solver
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
