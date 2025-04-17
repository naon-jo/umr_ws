# defines a YOLOv8 detection result

# class probability
int32 class_id
string class_name
float64 confidence

# 2D bounding box surrounding the object in pixels
umr_interfaces/BoundingBox2D bbox

float32 distance