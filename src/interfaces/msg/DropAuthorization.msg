# Otorisasi drop dari mission_bridge -> target_recognition.
# Dikirim (latched) setelah loiter 2x di DZ selesai, menandakan
# operasi airdrop (approach + release) boleh dimulai untuk DZ ini.
std_msgs/Header header

uint8 dz_index      # DZ mana yang diotorisasi (0 = DZ-1, 1 = DZ-2)
bool authorized     # true = loiter 2x selesai, drop boleh dieksekusi
