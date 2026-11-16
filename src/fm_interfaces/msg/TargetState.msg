# Etat de la cible suivie : publie par la perception, lu par le controleur.

uint8 STATE_SEARCHING = 0   # aucune cible verrouillee
uint8 STATE_LOCKED = 1      # cible verrouillee et vue dans l'image courante
uint8 STATE_LOST = 2        # cible verrouillee mais absente depuis peu (position = derniere connue)

std_msgs/Header header      # stamp = instant de capture de l'image
uint8 state
int32 track_id              # identifiant du tracker (-1 si aucune cible)

# Boite englobante normalisee entre 0 et 1 (origine en haut a gauche, v vers le bas)
float32 u                   # centre horizontal
float32 v                   # centre vertical
float32 width
float32 height
float32 confidence

bool touches_border         # boite au bord de l'image : hauteur non fiable pour estimer la distance
float32 time_since_seen     # secondes depuis la derniere observation (0 si LOCKED)