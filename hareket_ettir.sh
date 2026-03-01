# Robotun düz pozisyondan (0,0,0) hedef konuma (target) hareket etmesini sağlayan script

MODEL_NAME="rrr_robot"

gz joint -m $MODEL_NAME -j joint_1 --pos-t -0.407 --pos-p 50.0 --pos-i 0.1 --pos-d 5.0

gz joint -m $MODEL_NAME -j joint_2 --pos-t 1.875 --pos-p 50.0 --pos-i 0.1 --pos-d 5.0

gz joint -m $MODEL_NAME -j joint_3 --pos-t -1.468 --pos-p 50.0 --pos-i 0.1 --pos-d 5.0

