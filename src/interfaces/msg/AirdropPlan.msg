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

# Duplikat titik kunci dalam WGS-84 (deg) agar mission_bridge bisa
# memerintah setpoint GUIDED global tanpa mengetahui datum lokal.
float64 entry_point_lat
float64 entry_point_lon
float64 release_point_lat
float64 release_point_lon
