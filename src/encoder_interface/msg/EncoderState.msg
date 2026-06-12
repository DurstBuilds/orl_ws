std_msgs/Header header
uint32 position_counts       # raw 18-bit value (0..262143)
int32 turn_count             # raw 16-bit two's complement
float64 angle_rad            # position within current turn
float64 total_angle_rad      # multi-turn absolute angle
bool error_active            # true when E bit indicates fault
bool warning_active          # true when W bit indicates power-off motion warning
bool crc_valid               # true when CRC check passed this frame
