# Status internal state machine node airdrop_planning,
# dipublikasikan berkala untuk monitoring GCS / logging.

std_msgs/Header header

uint8 STATE_IDLE          = 0   # menunggu estimasi target
uint8 STATE_PLANNING      = 1   # menghitung release point + path
uint8 STATE_TRANSIT       = 2   # menuju loiter center (s)
uint8 STATE_LOITER        = 3   # loiter CW menunggu aligned ke entry point (p)
uint8 STATE_FINAL_APPROACH= 4   # segmen lurus p -> release point (heading fixed)
uint8 STATE_RELEASED      = 5   # payload dilepas
uint8 STATE_MISSED        = 6   # missed target -> replan

uint8 state
uint8 target_id
uint8 payloads_remaining

float64 distance_to_release   # [m] jarak horizontal ke release point
float64 wind_speed            # [m/s] estimasi angin yang dipakai perencana
float64 wind_direction        # [rad] arah DARI mana angin bertiup (meteorological)
