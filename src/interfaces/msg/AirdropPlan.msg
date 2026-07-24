# Rencana approach & release hasil node airdrop_planning.
# Geometri mengikuti "truncated Dubins path" (prec_drop, Fig. 5):
#   loiter_center (s)  ->  entry_point (p)  ->  release_point,
# dengan arah approach melawan angin (against the wind).
# Semua titik dalam frame lokal NED relatif terhadap datum (home).

std_msgs/Header header

uint8 target_id

geometry_msgs/Point release_point_ned   # titik pelepasan payload [m, NED]
geometry_msgs/Point entry_point_ned     # titik p, awal segmen lurus final
geometry_msgs/Point loiter_center_ned   # titik s, pusat loiter (clockwise)

float64 loiter_radius        # [m]
float64 approach_heading     # [rad] arah terbang saat release (dari North, CW)
float64 release_altitude_agl # [m]
float64 fall_time            # [s]  hasil simulasi balistik
float64 impact_offset_north  # [m]  perpindahan balistik (x pada Algorithm 1)
float64 impact_offset_east   # [m]  perpindahan balistik (y pada Algorithm 1)

# Duplikat titik kunci dalam WGS-84 (deg) untuk telemetri/logging GCS.
# Node airdrop_planning sendiri bekerja di NED dan memerintah setpoint GUIDED
# langsung ke FCU; field ini hanya memaparkan geometri rencana ke luar.
#
# loiter_center (s) adalah referensi PERTAMA yang diterbangkan (prec_drop
# Sec. 2.3: "Its center s will be the first reference sent to the guidance and
# control subsystem"). entry_point (p) TIDAK pernah menjadi tujuan terbang —
# ia titik singgung pada orbit di sekitar loiter_center.
float64 entry_point_lat
float64 entry_point_lon
float64 release_point_lat
float64 release_point_lon
float64 loiter_center_lat
float64 loiter_center_lon
