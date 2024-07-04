#pragma once

typedef struct box {
    float x, y, w, h;
} box;

typedef struct object_info {
    float x, y, w, h, distance, velocity;
    int class_id;
} object_info;

