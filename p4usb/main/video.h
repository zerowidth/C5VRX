#pragma once

#include <stddef.h>
#include <stdint.h>

#define VIDEO_WIDTH 720
#define VIDEO_HEIGHT 480
#define VIDEO_FPS 60

void video_init(void);
/* Hands out the newest finished JPEG, which stays valid until video_release. NULL if none yet. */
const uint8_t *video_acquire(size_t *len);
void video_release(void);
void video_command(int argc, char **argv);
