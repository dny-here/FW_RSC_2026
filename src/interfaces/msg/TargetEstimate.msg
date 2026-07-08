# Estimasi posisi target (Dropping Zone) hasil georeferencing
# dari node target_recognition. Mengikuti interface pada
# Mathisen et al. (prec_drop), Sec. 2: rata-rata posisi georeferenced
# dari sekumpulan deteksi dikirim ke airdrop planning.

std_msgs/Header header

uint8 target_id          # 0 = DZ-1 (tengah zona), 1 = DZ-2 (akhir zona)

float64 latitude         # [deg] WGS-84
float64 longitude        # [deg] WGS-84
float64 altitude_amsl    # [m]  elevasi tanah target (AMSL), untuk flat-earth assumption

float64[4] covariance    # [m^2] kovarians 2x2 posisi NE (row-major)
uint32 num_observations  # jumlah citra dengan deteksi yang dirata-rata
